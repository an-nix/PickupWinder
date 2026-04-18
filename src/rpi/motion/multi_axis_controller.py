from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from domain.config import AppConfiguration
from motion.axis import Axis
from motion.ramp import RampConfig
from motion.streaming_manager import StreamingManager
from transport import StreamAxisConfig, MultiAxisRampStreamer
from transport.spi_transport import Esp32SpiTransport


@dataclass(slots=True)
class MultiAxisMotionResult:
    spindle_axis_id: int
    lateral_axis_id: int
    duration_s: float
    spindle_rpm: float
    lateral_rpm: float
    blocks_sent: int


class MultiAxisMotionController:
    def __init__(
        self,
        transport: Esp32SpiTransport,
        spindle_axis: Axis,
        lateral_axis: Axis,
        config: AppConfiguration,
        streaming_manager: StreamingManager | None = None,
    ) -> None:
        self.transport = transport
        self.spindle_axis = spindle_axis
        self.lateral_axis = lateral_axis
        self.config = config
        self.streaming_manager = streaming_manager or StreamingManager()

    def run(
        self,
        duration_s: float,
        spindle_rpm: float,
        lateral_rpm: float,
    ) -> dict[str, Any]:
        if duration_s <= 0.0:
            raise ValueError("duration_s must be positive")
        if spindle_rpm < 0.0 or lateral_rpm < 0.0:
            raise ValueError("rpm values must be non-negative")

        spindle_rpm = min(spindle_rpm, float(self.config.spindle_max_speed_rpm))
        lateral_rpm = min(lateral_rpm, float(self.config.lateral_max_rpm))

        accel_s = min(0.5, duration_s * 0.2)
        decel_s = accel_s
        cruise_s = max(duration_s - accel_s - decel_s, 0.0)

        spindle_steps_per_rev = self.config.spindle_steps_per_revolution * self.config.spindle_microstepping
        lateral_steps_per_rev = self.config.lateral_steps_per_revolution * self.config.lateral_microstepping

        spindle_ramp = RampConfig(
            axis_id=self.spindle_axis.axis_id,
            steps_per_rev=spindle_steps_per_rev,
            target_rpm=spindle_rpm,
            accel_s=accel_s,
            cruise_s=cruise_s,
            decel_s=decel_s,
            reverse_direction=self.config.spindle_invert_direction,
        )

        lateral_ramp = RampConfig(
            axis_id=self.lateral_axis.axis_id,
            steps_per_rev=lateral_steps_per_rev,
            target_rpm=lateral_rpm,
            accel_s=accel_s,
            cruise_s=cruise_s,
            decel_s=decel_s,
            reverse_direction=self.config.lateral_invert_direction,
        )

        streamer = MultiAxisRampStreamer(
            self.transport,
            [
                StreamAxisConfig(axis_id=self.spindle_axis.axis_id, ramp=spindle_ramp),
                StreamAxisConfig(axis_id=self.lateral_axis.axis_id, ramp=lateral_ramp),
            ],
            poll_interval_s=0.001,
            print_every=1,
        )

        # Start streaming in background thread to avoid blocking the JSON-RPC handler
        session_id = self.streaming_manager.stream_async(
            streamer,
            name=f"multi_axis_{self.spindle_axis.axis_id}_{self.lateral_axis.axis_id}",
        )

        return {
            "spindle_axis_id": self.spindle_axis.axis_id,
            "lateral_axis_id": self.lateral_axis.axis_id,
            "duration_s": duration_s,
            "spindle_rpm": spindle_rpm,
            "lateral_rpm": lateral_rpm,
            "session_id": session_id,
            "status": "streaming_started",
        }
