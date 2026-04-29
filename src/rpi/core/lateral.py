from __future__ import annotations

import logging
from typing import TYPE_CHECKING

from core.config import AppConfiguration
from core.events import EventBus, EventKind
from core.shared_state import SharedState
from motion import RampConfig
from motion.axis_state import AxisState
from motion.move import HomingMove, MoveState
from transport.spi_transport import Esp32SpiTransport

if TYPE_CHECKING:
    from motion.move_queue import MoveQueue


logger = logging.getLogger(__name__)


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
        """
        Execute lateral homing by enqueueing a HomingMove into MoveQueue.

        The MoveQueue handles the full event-driven sequence:
          1. Fast approach with endstop armed → waits for endstop_hit_mask event
          2. Backoff with endstop disarmed → waits for lateral_endstop_state OPEN
          3. Slow search with endstop armed → waits for endstop_hit_mask event
          4. mark_homed(0) on AxisState

        Returns (True, None) on success, (False, reason) on failure.
        """
        self.require_axis_state(axis_id)
        steps_per_rev = (
            self._config.lateral_steps_per_revolution
            * self._config.lateral_microstepping
        )
        # Borne de sécurité : max 20 tours d'approche avant timeout.
        max_approach_steps = steps_per_rev * 20

        self._events.publish(EventKind.HOMING_STARTED, axis_id=axis_id)

        move = HomingMove(
            name=f"home_axis_{axis_id}",
            axis_id=axis_id,
            steps_per_rev=steps_per_rev,
            approach_rpm=approach_rpm,
            search_rpm=search_rpm,
            backoff_steps=backoff_steps,
            max_approach_steps=max_approach_steps,
            home_position_steps=0,
            reverse_direction=self._config.lateral_invert_direction,
        )

        self._move_queue.enqueue(move)

        try:
            self._move_queue.wait_until_idle(timeout_s=120.0)
        except TimeoutError as exc:
            self._events.publish(
                EventKind.HOMING_FAILED,
                axis_id=axis_id,
                reason=str(exc),
            )
            return False, f"homing timeout: {exc}"

        if move.state == MoveState.COMPLETED:
            self._events.publish(EventKind.HOMING_COMPLETED, axis_id=axis_id)
            return True, None

        reason = move.error or f"homing ended in state {move.state.name}"
        self._events.publish(EventKind.HOMING_FAILED, axis_id=axis_id, reason=reason)
        return False, reason
    
    
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
