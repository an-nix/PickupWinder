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

    def home(
        self,
        steps_per_attempt: int = 20,
        duration_us: int = 200_000,
        max_attempts: int = 200,
        reverse: bool = False,
        max_time_s: float | None = None,
    ) -> None:
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

        # Ensure we send at least enough steps to seed the ESP32 RMT buffer
        # Firmware expects STEP_STREAM_START_FILL >= 2 * PART_SIZE; with
        # PART_SIZE=32 the minimum safe value is 64. Use 64 to avoid underruns.
        steps_per_attempt = max(steps_per_attempt, 64)
        direction_flag = reverse

        # Continuous streaming mode: keep emitting segments to rotate the
        # axis until the endstop is contacted. When contact occurs, flush
        # outstanding motion up to the last sent motion sequence, then
        # perform a small reverse/back-off until the endstop is released.
        motion_seq = 0
        attempts = 0
        start_time = time.monotonic()
        end_time = start_time + max_time_s if (max_time_s is not None and max_time_s > 0) else None
        while True:
            # enforce both time and attempt limits if provided
            if end_time is not None and time.monotonic() > end_time:
                raise AxisControllerError("Homing failed: time expired")
            if max_attempts is not None and attempts >= max_attempts:
                raise AxisControllerError("Homing failed: endstop never closed")
            attempts += 1

            motion_seq += 1
            segment = MultiAxisSegment(
                sequence=motion_seq,
                duration_us=duration_us,
                steps=[steps_per_attempt],
                directions=[direction_flag],
            )

            payload = MultiAxisSegmentBlockPayload(
                axis_ids=[self.axis.axis_id],
                block_seq=motion_seq,
                segments=[segment],
            )

            # Ensure MCU has space and send the segment
            status = self.transport.send_multi_axis_segment_block_with_backpressure(
                payload, minimum_free_blocks=1, poll_interval_s=self.poll_interval_s
            )

            print(
                f"home send_seq={motion_seq} status=0x{status.lateral_endstop_state:02X} "
                f"queue_free={status.queue_free_slots[self.axis.axis_id]}"
            )

            # Check immediately if endstop was triggered
            if status.lateral_endstop_state == LATERAL_ENDSTOP_PRESENT_CLOSED:
                # Stop any further queued motion beyond motion_seq
                self.transport.flush_until(motion_seq)

                # Back off a little in reverse direction until endstop opens
                backoff_steps = max(steps_per_attempt // 4, 8)
                backoff_duration_us = int(duration_us // 4)
                backoff_seq = motion_seq
                backoff_attempts = 0
                while backoff_attempts < 20:
                    backoff_attempts += 1
                    backoff_seq += 1
                    back_segment = MultiAxisSegment(
                        sequence=backoff_seq,
                        duration_us=backoff_duration_us,
                        steps=[backoff_steps],
                        directions=[not direction_flag],
                    )
                    back_payload = MultiAxisSegmentBlockPayload(
                        axis_ids=[self.axis.axis_id],
                        block_seq=backoff_seq,
                        segments=[back_segment],
                    )
                    self.transport.send_multi_axis_segment_block_with_backpressure(
                        back_payload, minimum_free_blocks=1, poll_interval_s=self.poll_interval_s
                    )

                    # Poll status; stop when endstop is OPEN
                    s = self.status()
                    print(f"backoff attempt={backoff_attempts} endstop=0x{s.lateral_endstop_state:02X}")
                    if s.lateral_endstop_state == LATERAL_ENDSTOP_PRESENT_OPEN:
                        self.axis.mark_homed()
                        print("Homing complete: endstop released after backoff.")
                        return
                    time.sleep(self.poll_interval_s)

                raise AxisControllerError("Backoff failed: endstop remained closed")

            # If endstop disappears or fault is detected, abort
            if status.lateral_endstop_state == LATERAL_ENDSTOP_ABSENT:
                raise AxisControllerError("Lateral endstop fault detected during homing")

            # Small sleep to allow status updates; tune if too slow/fast
            time.sleep(self.poll_interval_s)

        raise AxisControllerError("Homing failed: endstop never closed")
