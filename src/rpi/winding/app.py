from __future__ import annotations

from pathlib import Path
from typing import Any

from domain import AppConfiguration
from motion.axis import Axis
from motion.axis_controller import AxisController
from motion.multi_axis_controller import MultiAxisMotionController
from motion.streaming_manager import StreamingManager
from transport import Esp32SpiTransport


class WinderApp:
    def __init__(self, config: AppConfiguration | None = None) -> None:
        self.config = config or AppConfiguration()
        self.transport: Esp32SpiTransport | None = None
        self.streaming_manager = StreamingManager()
        self.spindle_axis = Axis(
            axis_id=self.config.spindle_axis_id,
            name="spindle",
            can_move_without_homing=True,
        )
        self.lateral_axis = Axis(
            axis_id=self.config.lateral_axis_id,
            name="lateral",
            can_move_without_homing=False,
        )
        self.spindle_controller: AxisController | None = None
        self.lateral_controller: AxisController | None = None
        self.motion_controller: MultiAxisMotionController | None = None

    def start(self, bus: int = 0, device: int = 0) -> None:
        self.transport = Esp32SpiTransport(
            bus=bus,
            device=device,
            speed_hz=self.config.spi_speed_hz,
            mode=0,
        )
        self.spindle_controller = AxisController(
            self.spindle_axis,
            self.transport,
            streaming_manager=self.streaming_manager,
        )
        self.lateral_controller = AxisController(
            self.lateral_axis,
            self.transport,
            streaming_manager=self.streaming_manager,
        )

    def stop(self) -> None:
        if self.transport is not None:
            self.transport.close()
            self.transport = None

    def status(self) -> dict[str, Any]:
        return {
            "spindle": {
                "axis_id": self.spindle_axis.axis_id,
                "name": self.spindle_axis.name,
                "can_move": self.spindle_axis.can_move,
            },
            "lateral": {
                "axis_id": self.lateral_axis.axis_id,
                "name": self.lateral_axis.name,
                "can_move": self.lateral_axis.can_move,
                "homed": self.lateral_axis.homed,
            },
            "transport_connected": self.transport is not None,
        }

    def run_spindle(self, duration_s: float, rpm: float) -> dict[str, Any]:
        if self.spindle_controller is None:
            raise RuntimeError("Application transport is not started")
        if rpm <= 0.0:
            raise ValueError("rpm must be positive")
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        rpm = min(rpm, float(self.config.spindle_max_speed_rpm))
        return self.spindle_controller.run_ramp(duration_s=duration_s, target_rpm=rpm)

    def run_multi_axis(
        self,
        duration_s: float,
        spindle_rpm: float,
        lateral_rpm: float,
    ) -> dict[str, Any]:
        if self.transport is None:
            raise RuntimeError("Application transport is not started")
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if spindle_rpm < 0.0 or lateral_rpm < 0.0:
            raise ValueError("rpm values must be non-negative")

        self.motion_controller = MultiAxisMotionController(
            self.transport,
            self.spindle_axis,
            self.lateral_axis,
            self.config,
            streaming_manager=self.streaming_manager,
        )
        return self.motion_controller.run(
            duration_s=duration_s,
            spindle_rpm=spindle_rpm,
            lateral_rpm=lateral_rpm,
        )

    def config_snapshot(self) -> dict[str, Any]:
        return {
            "spi_device": self.config.spi_device,
            "spi_speed_hz": self.config.spi_speed_hz,
            "spindle_max_speed_rpm": self.config.spindle_max_speed_rpm,
            "spindle_max_acceleration_rpm": self.config.spindle_max_acceleration_rpm,
            "lateral_max_rpm": self.config.lateral_max_rpm,
            "lateral_max_acceleration_mm_per_s2": self.config.lateral_max_acceleration_mm_per_s2,
            "lateral_traverse_pitch_mm": self.config.lateral_traverse_pitch_mm,
            "lateral_steps_per_mm": self.config.lateral_steps_per_mm,
        }
