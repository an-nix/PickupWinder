from __future__ import annotations

import time
from typing import Any

from .config import AppConfiguration
from motion.axis_state import AxisLimits, AxisState
from motion.move import HomingMove, JogMove, RampMove, RampMoveConfig
from motion.move_queue import MoveQueue
from motion.ramp import AxisMotionConfig, RampConfig
from transport import Esp32SpiTransport


class WinderApp:
    def __init__(self, config: AppConfiguration | None = None) -> None:
        self.config = config or AppConfiguration()
        self.transport: Esp32SpiTransport | None = None
        self._axis_states: dict[int, AxisState] = {
            self.config.spindle_axis_id: AxisState(
                axis_id=self.config.spindle_axis_id,
                steps_per_rev=200 * 32,
            ),
            self.config.lateral_axis_id: AxisState(
                axis_id=self.config.lateral_axis_id,
                steps_per_rev=200 * 32,
            ),
        }
        self._move_queue: MoveQueue | None = None

    def start(self, bus: int = 0, device: int = 0) -> None:
        self.transport = Esp32SpiTransport(
            bus=bus,
            device=device,
            speed_hz=self.config.spi_speed_hz,
            mode=0,
        )
        self._move_queue = MoveQueue(
            self.transport,
            self._axis_states,
            poll_interval_s=0.001,
            print_every=1,
        )
        self._move_queue.start()

    def stop(self) -> None:
        if self._move_queue is not None:
            self._move_queue.stop(timeout_s=3.0)
            self._move_queue = None
        if self.transport is not None:
            self.transport.close()
            self.transport = None

    def status(self) -> dict[str, Any]:
        return {
            "transport_connected": self.transport is not None,
            "spi_device": self.config.spi_device,
            "spi_speed_hz": self.config.spi_speed_hz,
            "move_queue": self._move_queue.status() if self._move_queue else None,
            "axis_states": {
                ax_id: state.snapshot()
                for ax_id, state in self._axis_states.items()
            },
        }

    def stop_operation(self) -> dict[str, Any]:
        if self._move_queue is None:
            return {"status": "idle"}
        self._move_queue.clear()
        if self._move_queue.current_move is not None:
            self._move_queue.current_move.mark_aborted("stop_operation called")
        return self._move_queue.status()

    def run_spindle(self, duration_s: float, rpm: float) -> dict[str, Any]:
        if self._move_queue is None:
            raise RuntimeError("Application not started")
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if rpm <= 0.0:
            raise ValueError("rpm must be positive")
        move = RampMove(
            name="spindle",
            config=RampMoveConfig(
                axis_configs=[
                    AxisMotionConfig(
                        axis_id=self.config.spindle_axis_id,
                        ramp=RampConfig(
                            axis_id=self.config.spindle_axis_id,
                            target_rpm=min(rpm, float(self.config.spindle_max_speed_rpm)),
                            accel_s=min(0.5, duration_s * 0.25),
                            cruise_s=max(duration_s - 1.0, 0.0),
                            decel_s=min(0.5, duration_s * 0.25),
                        ),
                    )
                ],
            ),
        )
        self._move_queue.enqueue(move)
        return {"status": "queued", "move": move.snapshot()}

    def run_multi_axis(self, duration_s: float, spindle_rpm: float, lateral_rpm: float) -> dict[str, Any]:
        if self._move_queue is None:
            raise RuntimeError("Application not started")
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if spindle_rpm < 0.0 or lateral_rpm < 0.0:
            raise ValueError("rpm values must be non-negative")
        move = RampMove(
            name="multi_axis",
            config=RampMoveConfig(
                axis_configs=[
                    AxisMotionConfig(
                        axis_id=self.config.spindle_axis_id,
                        ramp=RampConfig(
                            axis_id=self.config.spindle_axis_id,
                            target_rpm=min(spindle_rpm, float(self.config.spindle_max_speed_rpm)),
                            accel_s=min(0.5, duration_s * 0.2),
                            cruise_s=max(duration_s - 1.0, 0.0),
                            decel_s=min(0.5, duration_s * 0.2),
                        ),
                    ),
                    AxisMotionConfig(
                        axis_id=self.config.lateral_axis_id,
                        ramp=RampConfig(
                            axis_id=self.config.lateral_axis_id,
                            target_rpm=min(lateral_rpm, float(self.config.lateral_max_rpm)),
                            accel_s=min(0.5, duration_s * 0.2),
                            cruise_s=max(duration_s - 1.0, 0.0),
                            decel_s=min(0.5, duration_s * 0.2),
                        ),
                    ),
                ],
            ),
        )
        self._move_queue.enqueue(move)
        return {"status": "queued", "move": move.snapshot()}

    def home_axis(
        self,
        axis_id: int,
        approach_rpm: float = 100.0,
        search_rpm: float = 20.0,
        backoff_steps: int = 3200,
        max_approach_steps: int = 200 * 32 * 10,
    ) -> dict[str, Any]:
        """
        Queue a homing sequence on axis_id.
        The host is responsible for ensuring the axis starts on the correct
        side of the endstop. If the axis is already at the endstop, call
        jog_axis() in the clearance direction first.
        """
        if self._move_queue is None:
            raise RuntimeError("Application not started")
        move = HomingMove(
            name=f"home_axis_{axis_id}",
            axis_id=axis_id,
            steps_per_rev=200 * 32,
            approach_rpm=approach_rpm,
            search_rpm=search_rpm,
            backoff_steps=backoff_steps,
            max_approach_steps=max_approach_steps,
        )
        self._move_queue.enqueue(move)
        return {"status": "queued", "move": move.snapshot()}

    def jog_axis(
        self,
        axis_id: int,
        steps: int,
        rpm: float,
        reverse: bool = False,
    ) -> dict[str, Any]:
        """
        Queue a fixed-step jog move. Useful for clearance moves before homing
        and for manual positioning.
        """
        if self._move_queue is None:
            raise RuntimeError("Application not started")
        move = JogMove(
            name=f"jog_axis_{axis_id}",
            axis_id=axis_id,
            steps_per_rev=200 * 32,
            steps=steps,
            rpm=rpm,
            reverse_direction=reverse,
        )
        self._move_queue.enqueue(move)
        return {"status": "queued", "move": move.snapshot()}

    def arm_endstop(self, axis_id: int | None = None) -> dict[str, Any]:
        """Arm the endstop on the given axis (default: lateral_axis_id).

        Sends ENABLE_ENDSTOP arm=True to firmware. The streamer will stop the
        move automatically when the endstop is triggered.
        """
        if self.transport is None:
            raise RuntimeError("Application transport is not started")
        axis = axis_id if axis_id is not None else self.config.lateral_axis_id
        seq, status = self.transport.enable_endstop_request(axis, arm=True)
        status = self.transport.wait_for_request_result(seq)
        return {"axis_id": axis, "armed": True, "result": status.last_result}

    def disarm_endstop(self, axis_id: int | None = None) -> dict[str, Any]:
        """Disarm the endstop on the given axis (default: lateral_axis_id).

        Sends ENABLE_ENDSTOP arm=False to firmware. Use before a clearance
        move that needs to pass through the endstop without stopping.
        """
        if self.transport is None:
            raise RuntimeError("Application transport is not started")
        axis = axis_id if axis_id is not None else self.config.lateral_axis_id
        seq, status = self.transport.enable_endstop_request(axis, arm=False)
        status = self.transport.wait_for_request_result(seq)
        return {"axis_id": axis, "armed": False, "result": status.last_result}

    def endstop_status(self) -> dict[str, Any]:
        """Return current endstop state from a live status poll."""
        if self.transport is None:
            raise RuntimeError("Application transport is not started")
        status = self.transport.get_status()
        return {
            "lateral_endstop_state": status.lateral_endstop_state,
            "endstop_armed_mask": status.endstop_armed_mask,
        }

    def config_snapshot(self) -> dict[str, Any]:
        return {
            "spi_device": self.config.spi_device,
            "spi_speed_hz": self.config.spi_speed_hz,
            "spindle_max_speed_rpm": self.config.spindle_max_speed_rpm,
            "lateral_max_rpm": self.config.lateral_max_rpm,
            "lateral_traverse_pitch_mm": self.config.lateral_traverse_pitch_mm,
        }
