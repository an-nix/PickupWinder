from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

from transport.messages import (
    LATERAL_ENDSTOP_ABSENT,
    LATERAL_ENDSTOP_PRESENT_CLOSED,
    LATERAL_ENDSTOP_PRESENT_OPEN,
    MultiAxisSegment,
    MultiAxisSegmentBlockPayload,
    SpiMessageResult,
)
from transport.spi_transport import Esp32SpiTransport
from motion.axis import Axis
from motion.axis_controller import AxisController, AxisControllerError


@dataclass(slots=True)
class LateralHomingConfig:
    axis: Axis
    steps_per_attempt: int = 32
    duration_us: int = 50_000
    max_attempts: int = 200
    max_time_s: float = 60.0
    poll_interval_s: float = 0.02
    reverse: bool = False


class LateralHomingError(RuntimeError):
    pass


def home_lateral_axis(transport: Esp32SpiTransport, config: LateralHomingConfig) -> None:
    """Prototype homing routine for the lateral axis using status feedback."""

    sequence, _ = transport.set_axis_enabled_request(config.axis.axis_id, True)
    status = transport.wait_for_request_result(sequence, poll_interval_s=config.poll_interval_s)
    if status.last_result != int(SpiMessageResult.OK):
        raise LateralHomingError(
            f"enable axis {config.axis.axis_id} failed with result=0x{status.last_result:02X}"
        )

    controller = AxisController(axis=config.axis, transport=transport, poll_interval_s=config.poll_interval_s)
    # Arm the hardware endstop ISR on the ESP32 so the motor stops within
    # microseconds of contact rather than waiting for the next SPI status poll.
    transport.arm_endstop(axis_id=config.axis.axis_id)
    try:
        controller.home(
            steps_per_attempt=config.steps_per_attempt,
            duration_us=config.duration_us,
            max_attempts=config.max_attempts,
            max_time_s=config.max_time_s,
            reverse=config.reverse,
        )
    except AxisControllerError as exc:
        raise LateralHomingError(str(exc)) from exc
    finally:
        # Always disarm after homing (success or failure) to prevent the ISR
        # from stopping unexpected future moves.
        transport.disarm_endstop(axis_id=config.axis.axis_id)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Prototype lateral homing over SPI")
    parser.add_argument("--bus", type=int, default=0)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--speed-hz", type=int, default=4_000_000)
    parser.add_argument("--steps-per-attempt", type=int, default=32)
    parser.add_argument(
        "--duration-us",
        type=int,
        default=50_000,
        help="Durée d'une tentative de homing en microsecondes (max 65535)",
    )
    parser.add_argument("--max-attempts", type=int, default=200)
    parser.add_argument("--poll-interval-s", type=float, default=0.02)
    parser.add_argument("--reverse", action="store_true")
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    config = LateralHomingConfig(
        axis=Axis(axis_id=1, name="lateral", can_move_without_homing=False),
        steps_per_attempt=args.steps_per_attempt,
        duration_us=args.duration_us,
        max_attempts=args.max_attempts,
        poll_interval_s=args.poll_interval_s,
        reverse=args.reverse,
    )

    with Esp32SpiTransport(bus=args.bus, device=args.device, speed_hz=args.speed_hz, mode=0) as transport:
        try:
            home_lateral_axis(transport, config)
        except LateralHomingError as exc:
            print(f"Homing abort: {exc}")


if __name__ == "__main__":
    main()
