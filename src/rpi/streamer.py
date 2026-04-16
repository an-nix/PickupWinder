from __future__ import annotations

from dataclasses import dataclass
import time
from typing import Iterator

try:  # pragma: no cover - import mode depends on how the script is started
    from .messages import SegmentBlockPayload, SpiMessageResult, SpiMessageType, StepBlockPayload
    from .ramp import HybridRampBlockGenerator, RampConfig
    from .spi_transport import Esp32SpiTransport
except ImportError:  # pragma: no cover - direct script execution fallback
    from messages import SegmentBlockPayload, SpiMessageResult, SpiMessageType, StepBlockPayload  # type: ignore
    from ramp import HybridRampBlockGenerator, RampConfig  # type: ignore
    from spi_transport import Esp32SpiTransport  # type: ignore


@dataclass(slots=True)
class StreamAxisConfig:
    axis_id: int
    ramp: RampConfig
    minimum_free_blocks: int = 1
    prefill_blocks: int | None = None
    low_watermark_blocks: int | None = None
    max_queued_blocks: int | None = None


@dataclass(slots=True)
class _AxisStreamState:
    config: StreamAxisConfig
    generator: Iterator
    finished: bool = False
    blocks_sent: int = 0
    queue_depth: int = 0


@dataclass(slots=True)
class _PendingBlock:
    sequence: int
    stream: _AxisStreamState
    block: StepBlockPayload | SegmentBlockPayload


class MultiAxisRampStreamer:
    """Queue-aware block streamer for one or more axes.

    The goal is to keep the ESP32 FreeRTOS queue comfortably primed instead of
    sending one block only when the previous one is almost consumed. This gives
    the executor more look-ahead and reduces host-side timing gaps that can end
    up as RMT underruns, especially during slow or mixed-speed motion.
    """

    def __init__(
        self,
        transport: Esp32SpiTransport,
        axis_streams: list[StreamAxisConfig],
        *,
        poll_interval_s: float = 0.001,
        print_every: int = 8,
    ):
        self._transport = transport
        self._poll_interval_s = poll_interval_s
        self._print_every = max(print_every, 1)
        self._stop_requested = False
        self._streams = [
            _AxisStreamState(
                config=stream,
                generator=iter(HybridRampBlockGenerator(stream.ramp)),
            )
            for stream in axis_streams
        ]

    def _update_queue_depths(self, status) -> None:
        for stream in self._streams:
            axis_id = stream.config.axis_id
            if axis_id >= len(status.queue_free_slots):
                continue
            stream.queue_depth = max(stream.queue_depth, int(status.queue_free_slots[axis_id]))

    def _target_fill(self, stream: _AxisStreamState) -> int:
        depth = max(stream.queue_depth, 1)
        if stream.config.prefill_blocks is not None:
            target = max(1, min(stream.config.prefill_blocks, depth))
        elif depth <= 2:
            target = 1
        else:
            target = depth - 2

        if stream.config.max_queued_blocks is not None:
            return min(target, stream.config.max_queued_blocks)
        return target

    def _low_watermark(self, stream: _AxisStreamState) -> int:
        target_fill = self._target_fill(stream)
        if stream.config.low_watermark_blocks is not None:
            return max(0, min(stream.config.low_watermark_blocks, target_fill))
        return max(1, target_fill // 3)

    def _queued_blocks(self, status, stream: _AxisStreamState) -> int:
        axis_id = stream.config.axis_id
        if axis_id >= len(status.queue_free_slots):
            return 0
        depth = max(stream.queue_depth, int(status.queue_free_slots[axis_id]), 1)
        stream.queue_depth = max(stream.queue_depth, depth)
        return max(0, depth - int(status.queue_free_slots[axis_id]))

    def _enable_axes(self) -> None:
        for stream in self._streams:
            sequence, _ = self._transport.set_axis_enabled_request(stream.config.axis_id, True)
            status = self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(
                    f"enable axis {stream.config.axis_id} failed with result=0x{status.last_result:02X}"
                )

    def _should_fill(self, status, stream: _AxisStreamState, *, initial_fill: bool) -> bool:
        queued_blocks = self._queued_blocks(status, stream)
        if initial_fill:
            return queued_blocks < self._target_fill(stream)
        return queued_blocks <= self._low_watermark(stream)

    def _next_block(self, stream: _AxisStreamState) -> StepBlockPayload | SegmentBlockPayload | None:
        if stream.finished:
            return None
        try:
            return next(stream.generator)
        except StopIteration:
            stream.finished = True
            return None

    def _choose_stream(self, status, *, initial_fill: bool) -> _AxisStreamState | None:
        eligible: list[tuple[int, _AxisStreamState]] = []
        for stream in self._streams:
            if stream.finished:
                continue
            axis_id = stream.config.axis_id
            if axis_id >= len(status.queue_free_slots):
                continue
            if int(status.queue_free_slots[axis_id]) < stream.config.minimum_free_blocks:
                continue
            if not self._should_fill(status, stream, initial_fill=initial_fill):
                continue
            eligible.append((self._queued_blocks(status, stream), stream))

        if not eligible:
            return None

        eligible.sort(key=lambda item: (item[0], item[1].config.axis_id))
        return eligible[0][1]

    def _confirm_pending(self, pending: _PendingBlock, status) -> None:
        if status.last_rx_sequence != (pending.sequence & 0xFFFF):
            raise RuntimeError(
                f"unexpected confirmation sequence {status.last_rx_sequence} for pending block seq={pending.sequence}"
            )

        expected_type = (
            SpiMessageType.STEP_BLOCK
            if isinstance(pending.block, StepBlockPayload)
            else SpiMessageType.SEGMENT_BLOCK
        )
        if status.last_rx_type != int(expected_type):
            raise RuntimeError(
                f"unexpected confirmed message type 0x{status.last_rx_type:02X} "
                f"for block seq={pending.sequence}"
            )

        if status.last_result == int(SpiMessageResult.OK):
            pending.stream.blocks_sent += 1
            return

        if status.last_result == int(SpiMessageResult.QUEUE_FULL):
            raise RuntimeError(
                "block was rejected with QUEUE_FULL despite host-side queue tracking; "
                "this indicates a transport synchronisation bug"
            )

        raise RuntimeError(
            f"block seq={pending.sequence} failed with result=0x{status.last_result:02X}"
        )

    def _flush_pending(self, pending: _PendingBlock):
        status = self._transport.get_status()
        self._update_queue_depths(status)
        self._confirm_pending(pending, status)
        return status

    def request_stop(self) -> None:
        """Request the streamer to stop sending new blocks."""
        self._stop_requested = True

    def has_stop_been_requested(self) -> bool:
        return self._stop_requested

    def stream_all(self) -> int:
        status = self._transport.get_status()
        self._update_queue_depths(status)

        self._enable_axes()
        status = self._transport.get_status()
        self._update_queue_depths(status)

        total_blocks = 0

        pending: _PendingBlock | None = None
        initial_fill = True

        while True:
            self._update_queue_depths(status)
            active_streams = any(not stream.finished for stream in self._streams)
            if self._stop_requested:
                if pending is not None:
                    status = self._flush_pending(pending)
                    total_blocks += 1
                break
            if not active_streams and pending is None:
                break

            stream = self._choose_stream(status, initial_fill=initial_fill)
            if stream is not None:
                block = self._next_block(stream)
                if block is not None:
                    if isinstance(block, SegmentBlockPayload):
                        sequence, next_status = self._transport.send_segment_block_request(block)
                    else:
                        sequence, next_status = self._transport.send_step_block_request(block)
                    if pending is not None:
                        self._confirm_pending(pending, next_status)
                        total_blocks += 1
                        if total_blocks % self._print_every == 0:
                            print(
                                f"block={total_blocks} axis={pending.stream.config.axis_id} enabled=0x{next_status.enabled_mask:02X} "
                                f"running=0x{next_status.running_mask:02X} queue_free={next_status.queue_free_slots} "
                                f"ring_free={next_status.ring_free_slots} underruns={next_status.underrun_count}"
                            )
                    pending = _PendingBlock(sequence=sequence, stream=stream, block=block)
                    status = next_status
                    self._update_queue_depths(status)
                    if initial_fill and all(
                        not self._should_fill(status, candidate, initial_fill=True)
                        for candidate in self._streams
                        if not candidate.finished
                    ):
                        initial_fill = False
                    continue

            if initial_fill and all(
                not self._should_fill(status, candidate, initial_fill=True)
                for candidate in self._streams
                if not candidate.finished
            ):
                initial_fill = False

            if pending is not None:
                status = self._flush_pending(pending)
                total_blocks += 1
                if total_blocks % self._print_every == 0:
                    print(
                        f"block={total_blocks} axis={pending.stream.config.axis_id} enabled=0x{status.enabled_mask:02X} "
                        f"running=0x{status.running_mask:02X} queue_free={status.queue_free_slots} "
                        f"ring_free={status.ring_free_slots} underruns={status.underrun_count}"
                    )
                pending = None
                continue

            if any(not stream.finished for stream in self._streams):
                time.sleep(self._poll_interval_s)
                status = self._transport.get_status()
                self._update_queue_depths(status)

        return total_blocks
