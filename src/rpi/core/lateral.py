from __future__ import annotations

from dataclasses import dataclass
import logging
from typing import TYPE_CHECKING

from core.config import AppConfiguration
from core.events import EventBus, EventKind
from core.shared_state import SharedState
from motion.homing_controller import (
    HomingController,
    HomingFaultError,
    HomingTimeoutError,
    SegmentSpec,
)
from motion import RampConfig
from motion.axis_state import AxisState
from transport.messages import MultiAxisSegment, MultiAxisSegmentBlockPayload, SpiMessageResult
from transport.spi_transport import Esp32SpiTransport

if TYPE_CHECKING:
    from motion.move_queue import MoveQueue


logger = logging.getLogger(__name__)


@dataclass(slots=True)
class _HomingSpiAdapter:
    transport: Esp32SpiTransport
    axis_id: int
    reverse_direction: bool
    poll_interval_s: float = 0.001
    block_sequence: int = 0

    def send_enable_endstop(self, axis_id: int, arm: int):
        sequence, send_status = self.transport.enable_endstop_request(axis_id, arm=bool(arm))
        return self._wait_for_request_result(sequence, send_status)

    def send_flush(self, flush_sequence: int):
        return self.transport.flush_until(flush_sequence)

    def send_segments(self, segments: list[SegmentSpec]):
        converted: list[MultiAxisSegment] = []
        for segment in segments:
            axis_steps = 0
            if len(segment.step_counts) > self.axis_id:
                axis_steps = int(segment.step_counts[self.axis_id])
            direction = 1 if (int(segment.direction_mask) & (1 << self.axis_id)) else 0
            if self.reverse_direction:
                direction ^= 1
            converted.append(
                MultiAxisSegment(
                    sequence=int(segment.motion_sequence) & 0xFFFF,
                    duration_us=int(segment.duration_us),
                    steps=[max(0, axis_steps)],
                    directions=[direction],
                )
            )

        payload = MultiAxisSegmentBlockPayload(
            axis_ids=[self.axis_id],
            block_seq=self.block_sequence & 0xFFFF,
            segments=converted,
        )
        self.block_sequence = (self.block_sequence + 1) & 0xFFFF

        sequence, send_status = self.transport.send_multi_axis_segment_block_request(payload)
        status = self._wait_for_request_result(sequence, send_status)
        if int(getattr(status, "last_result", int(SpiMessageResult.OK))) != int(SpiMessageResult.OK):
            return status
        return status

    def get_status(self):
        return self.transport.get_status()

    def send_estop(self, axis_id: int):
        return self.transport.emergency_stop(axis_id)

    def _wait_for_request_result(self, sequence: int, send_status):
        try:
            return self.transport.wait_for_request_result(
                sequence,
                hint_status=send_status,
                poll_interval_s=self.poll_interval_s,
            )
        except TypeError as exc:
            if "hint_status" not in str(exc):
                raise
            return self.transport.wait_for_request_result(
                sequence,
                poll_interval_s=self.poll_interval_s,
            )


class LateralAxisController:
    """Own lateral-axis-only rules so the engine stays orchestration-focused."""

    def __init__(
        self,
        *,
        transport: Esp32SpiTransport,
        shared_state: SharedState,
        move_queue: MoveQueue,
        event_bus: EventBus,
        config: AppConfiguration,
    ) -> None:
        self._transport = transport
        self._state = shared_state
        self._move_queue = move_queue
        self._events = event_bus
        self._config = config

    def home(
        self,
        *,
        axis_id: int,
        approach_rpm: float,
        search_rpm: float,
        backoff_steps: int,
    ) -> tuple[bool, str | None]:
        logger.info(
            "Starting lateral homing: axis_id=%s approach_rpm=%.3f search_rpm=%.3f backoff_steps=%s",
            axis_id,
            approach_rpm,
            search_rpm,
            backoff_steps,
        )
        self._events.publish(
            EventKind.HOMING_STARTED,
            axis_id=axis_id,
            approach_rpm=approach_rpm,
            search_rpm=search_rpm,
            backoff_steps=backoff_steps,
        )

        steps_per_rev = (
            self._config.lateral_steps_per_revolution
            * self._config.lateral_microstepping
        )
        segment_duration_us = 4000
        segment_duration_s = segment_duration_us / 1_000_000.0
        fast_steps_per_segment = max(
            1,
            int(round((float(approach_rpm) / 60.0) * float(steps_per_rev) * segment_duration_s)),
        )
        slow_steps_per_segment = max(
            1,
            int(round((float(search_rpm) / 60.0) * float(steps_per_rev) * segment_duration_s)),
        )

        # Homing must run on a physically enabled axis.
        self._transport.set_axis_enabled(axis_id, True)

        adapter = _HomingSpiAdapter(
            transport=self._transport,
            axis_id=axis_id,
            reverse_direction=bool(self._config.lateral_invert_direction),
        )
        controller = HomingController(spi_interface=adapter, axis_id=axis_id)

        try:
            try:
                _hit_sequence = controller.home(
                    fast_speed_steps_per_seg=fast_steps_per_segment,
                    slow_speed_steps_per_seg=slow_steps_per_segment,
                    backoff_distance_steps=int(backoff_steps),
                    segment_duration_us=segment_duration_us,
                    timeout_s=10.0,
                )
            finally:
                # Keep the axis enabled by default after homing attempt.
                # This is the central point where a future policy can disable it.
                logger.debug("Lateral homing finished; axis %s remains enabled", axis_id)

            axis_state = self.require_axis_state(axis_id)
            axis_state.mark_homed(0)
            self._events.publish(
                EventKind.HOMING_COMPLETED,
                axis_id=axis_id,
                axis_state=axis_state.snapshot(),
            )
            return True, None
        except (HomingTimeoutError, HomingFaultError) as exc:
            message = f"Homing failed: {exc}"
            self._state.set_fault(message)
            self._events.publish(
                EventKind.HOMING_FAILED,
                axis_id=axis_id,
                error=message,
            )
            return False, str(exc)
        except Exception as exc:
            message = f"Homing failed: {exc}"
            self._state.set_fault(message)
            self._events.publish(
                EventKind.HOMING_FAILED,
                axis_id=axis_id,
                error=message,
            )
            return False, str(exc)

    def require_axis_state(self, axis_id: int) -> AxisState:
        axis_state = self._state.axis_states.get(axis_id)
        if axis_state is None:
            raise RuntimeError(f"Unknown axis_id {axis_id}")
        return axis_state

    def refresh_home_state(self) -> None:
        axis_state = self._state.axis_states.get(self._config.lateral_axis_id)
        if axis_state is None or not axis_state.homed:
            return

        try:
            status = self._transport.get_status()
        except Exception:
            return

        enabled_mask = int(getattr(status, "enabled_mask", 0))
        if (enabled_mask & (1 << self._config.lateral_axis_id)) != 0:
            return

        logger.warning(
            "Lateral axis enable lost; invalidating homing state and position"
        )
        axis_state.invalidate_position()

    def require_homed(self, axis_id: int | None = None) -> AxisState:
        lateral_axis_id = self._config.lateral_axis_id if axis_id is None else axis_id
        if lateral_axis_id != self._config.lateral_axis_id:
            return self.require_axis_state(lateral_axis_id)

        self.refresh_home_state()
        axis_state = self.require_axis_state(lateral_axis_id)
        if not axis_state.homed or axis_state.position_steps is None:
            raise RuntimeError("Lateral axis must be homed before motion")
        return axis_state

    def ensure_delta_allowed(self, delta_steps: int) -> None:
        axis_state = self.require_homed()
        if axis_state.check_move(delta_steps):
            return

        target_steps = (axis_state.position_steps or 0) + delta_steps
        target_mm = self.steps_to_mm(target_steps)
        raise ValueError(
            f"Lateral target {target_mm:.3f} mm is outside configured soft limits"
        )

    def mm_to_steps(self, position_mm: float) -> int:
        return int(round(float(position_mm) * self._config.lateral_steps_per_mm))

    def steps_to_mm(self, position_steps: int) -> float:
        return float(position_steps) / float(self._config.lateral_steps_per_mm)

    @staticmethod
    def ramp_delta_steps(ramp: RampConfig) -> int:
        total_steps = int(round(ramp.steps_at(ramp.total_duration)))
        return -total_steps if ramp.reverse_direction else total_steps
