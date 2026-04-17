from __future__ import annotations

import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from axis import Axis
from messages import (
    LATERAL_ENDSTOP_ABSENT,
    LATERAL_ENDSTOP_PRESENT_CLOSED,
    LATERAL_ENDSTOP_PRESENT_OPEN,
    MultiAxisSegment,
    MultiAxisSegmentBlockPayload,
    SpiMessageResult,
)
from spi_transport import Esp32SpiTransport


class AxisControllerError(RuntimeError):
    pass


@dataclass(slots=True)
class AxisController:
    """Prototype host-side controller for an axis.

    This class manages the axis state, waits for sensor feedback via SPI
    status messages, and drives the homing/motion protocol for the axis.
    """

    axis: Axis
    transport: Esp32SpiTransport
    poll_interval_s: float = 0.01

    def can_move(self) -> bool:
        return self.axis.can_move

    def requires_homing(self) -> bool:
        return self.axis.requires_homing

    def status(self):
        return self.transport.poll_status()

    def wait_for_endstop_state(self, expected_states: Iterable[int], timeout_s: float) -> "StatusPayload":
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            status = self.status()
            if status.lateral_endstop_state in expected_states:
                return status
            time.sleep(self.poll_interval_s)
        raise AxisControllerError(
            f"Timeout waiting for lateral endstop state in {expected_states}"
        )

    def home(self, steps_per_attempt: int = 20, duration_us: int = 200_000, max_attempts: int = 200) -> None:
        if self.axis.axis_id != 1:
            raise AxisControllerError("Homing prototype only supports the lateral axis (axis_id=1)")

        status = self.status()
        if status.lateral_endstop_state == LATERAL_ENDSTOP_ABSENT:
            raise AxisControllerError("Lateral endstop absent or fault")

        if status.lateral_endstop_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
            self.axis.mark_homed()
            return

        if duration_us < 1 or duration_us > 0xFFFF:
            raise AxisControllerError(
                f"duration_us must be between 1 and 65535, got {duration_us}"
            )

        for attempt in range(1, max_attempts + 1):
            segment = MultiAxisSegment(
                sequence=attempt,
                duration_us=duration_us,
                steps=[steps_per_attempt],
                directions=[False],
            )
            payload = MultiAxisSegmentBlockPayload(
                axis_ids=[self.axis.axis_id],
                block_seq=attempt,
                segments=[segment],
            )
            status = self.transport.send_multi_axis_segment_block(payload)
            if status.last_result != int(SpiMessageResult.OK):
                raise AxisControllerError(
                    f"SPI segment block rejected: 0x{status.last_result:02X}"
                )

            print(
                f"home attempt={attempt} status=0x{status.lateral_endstop_state:02X} "
                f"queue_free={status.queue_free_slots[self.axis.axis_id]}"
            )

            try:
                status = self.wait_for_endstop_state(
                    [LATERAL_ENDSTOP_PRESENT_CLOSED, LATERAL_ENDSTOP_ABSENT],
                    timeout_s=duration_us / 1_000_000.0 + 0.5,
                )
            except AxisControllerError:
                continue

            if status.lateral_endstop_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
                self.transport.flush_until(attempt)
                self.axis.mark_homed()
                print("Homing complete.")
                return

            if status.lateral_endstop_state == LATERAL_ENDSTOP_ABSENT:
                raise AxisControllerError("Lateral endstop fault detected during homing")

        raise AxisControllerError("Homing failed: endstop never closed")
