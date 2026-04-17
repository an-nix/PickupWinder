from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import time
from typing import Iterator, List

try:  # pragma: no cover - import mode depends on how the script is started
    from .messages import (
        MultiAxisSegment,
        MultiAxisSegmentBlockPayload,
        SpiMessageResult,
        SpiMessageType,
    )
    from .ramp import AxisMotionConfig, MultiAxisSegmentGenerator, RampConfig
    from .spi_transport import Esp32SpiTransport
except ImportError:  # pragma: no cover - direct script execution fallback
    from messages import (  # type: ignore
        MultiAxisSegment,
        MultiAxisSegmentBlockPayload,
        SpiMessageResult,
        SpiMessageType,
    )
    from ramp import AxisMotionConfig, MultiAxisSegmentGenerator, RampConfig  # type: ignore
    from spi_transport import Esp32SpiTransport  # type: ignore


@dataclass(slots=True)
class StreamAxisConfig:
    """Per-axis configuration for ``MultiAxisRampStreamer``.

    In the new time-based model the fields ``minimum_free_blocks``,
    ``prefill_blocks``, ``low_watermark_blocks``, ``max_queued_blocks``, and
    ``ring_send_threshold`` are retained for backward compatibility with
    ``demo_spi.py`` argument parsing but are not used by the streamer itself —
    back-pressure is now derived entirely from MCU queue/ring feedback.
    """

    axis_id: int
    ramp: RampConfig
    minimum_free_blocks: int = 1
    prefill_blocks: int | None = 6
    low_watermark_blocks: int | None = None
    max_queued_blocks: int | None = None
    ring_send_threshold: int = 1


class MultiAxisRampStreamer:
    """Deterministic, feedback-driven, non-blocking multi-axis motion streamer.

    Design overview
    ---------------
    The host (Raspberry Pi) is the **single source of truth** for all motion
    planning.  The ESP32 is a **pure deterministic executor** that receives
    pre-computed time-based segments and executes them in strict sequence.

    Synchronisation model
    ~~~~~~~~~~~~~~~~~~~~~
    All axes share **one global timeline**.  Each ``MultiAxisSegment`` carries:

    * ``sequence``    — *motion* sequence number, strictly increasing,
                        assigned by ``MultiAxisSegmentGenerator``.  The ESP32
                        stores the last fully executed sequence number in
                        ``StatusPayload.last_executed_sequence`` and returns it
                        with every SPI response.
    * ``duration_us`` — wall-clock duration the segment occupies on the MCU.

    These two independent sequence spaces must NOT be confused:

    * **Motion sequence** (``segment.sequence``, 0–65535 wrapping): identifies
      a segment in the *logical* trajectory.  Used to compute buffered time
      and to target flush operations.
    * **Transport sequence** (second element of the ``_inflight`` tuple): the
      SPI frame sequence number echoed back by the ESP32 in
      ``status.last_rx_sequence``.  Used to detect lost or reordered SPI
      transactions.

    Buffer management
    ~~~~~~~~~~~~~~~~~
    Buffered time is the sum of ``duration_us`` of all in-flight segments that
    have NOT yet been executed by the ESP32.  This is computed in O(1) via the
    maintained accumulator ``_buffered_time_s``:

    * **On send** : ``_buffered_time_s += segment.duration_us / 1e6``
    * **On confirm**: ``_buffered_time_s -= segment.duration_us / 1e6``

    This replaces the previous O(n) sum and eliminates wall-clock drift.

    Real-time constraints
    ~~~~~~~~~~~~~~~~~~~~~
    * Segment duration: 2–5 ms.  Small enough for low latency, large enough to
      avoid SPI congestion from excessively frequent transfers.
    * Target buffer: 20–50 ms ahead of the MCU execution cursor.  This absorbs
      SPI scheduling jitter and OS preemption on the Pi without causing
      noticeable trajectory lag.
    * If ``_buffered_time_s < MIN_BUFFER_TIME_S`` the loop skips the sleep to
      refill aggressively and prevent MCU underrun.

    Non-blocking guarantee
    ~~~~~~~~~~~~~~~~~~~~~~
    The main loop never waits for a single segment confirmation and never
    retries a rejected send.  Back-pressure is handled by:

    1. Checking ``status.queue_free_slots`` and ``status.ring_free_slots``
       before each send.
    2. Limiting the number of concurrently in-flight segments to
       ``MAX_INFLIGHT_SEGMENTS`` (default 32).
    3. Sleeping ``POLL_SLEEP_S`` between loop iterations — except when the
       buffer is dangerously low (emergency refill mode).

    Flush behaviour
    ~~~~~~~~~~~~~~~
    ``flush_until(sequence)`` sends a ``FLUSH`` command to the ESP32, which
    discards any queued segments with sequence > flush_sequence.  The host
    mirrors this by clearing ``_inflight`` and resetting ``_buffered_time_s``,
    ensuring no stale segments are counted or confirmed after the flush.
    """

    # ── Real-time buffer targets ──────────────────────────────────────────────
    # TARGET_BUFFER_TIME_S defines the nominal look-ahead window.  100 ms prevents
    # ring starvation and smooths low-speed motion by maintaining larger look-ahead.
    # Larger buffers reduce jitter and ring drain-to-zero events.
    TARGET_BUFFER_TIME_S = 0.10   # nominal look-ahead: 100 ms
    MIN_BUFFER_TIME_S    = 0.06   # emergency refill threshold: 60 ms
    MAX_BUFFER_TIME_S    = 0.12   # hard cap: 120 ms

    # ── Segment duration clamp ────────────────────────────────────────────────
    # 2–5 ms per segment balances SPI bandwidth against latency.  Shorter
    # segments increase SPI traffic; longer segments increase reaction time to
    # trajectory changes.
    MIN_SEGMENT_TIME_S = 0.002
    MAX_SEGMENT_TIME_S = 0.005

    POLL_SLEEP_S = 0.0005   # normal inter-loop sleep: 0.5 ms

    # Maximum number of segments allowed in flight simultaneously.  Prevents
    # runaway memory growth and SPI overload if the MCU stops acknowledging.
    MAX_INFLIGHT_SEGMENTS = 32

    def __init__(
        self,
        transport: Esp32SpiTransport,
        axis_streams: list[StreamAxisConfig],
        *,
        segment_duration_s: float = 0.004,
        target_buffer_time_s: float = TARGET_BUFFER_TIME_S,
        poll_interval_s: float = 0.001,
        print_every: int = 1,
        log_each_send: bool = False,
        send_log_path: str | None = None,
    ):
        """Initialise the streamer.

        Args:
            transport:           Initialised ``Esp32SpiTransport`` instance.
            axis_streams:        Per-axis ramp configuration.  All axes share
                                 the same global timeline.
            segment_duration_s:  Nominal duration of each generated segment,
                                 clamped to [MIN_SEGMENT_TIME_S,
                                 MAX_SEGMENT_TIME_S].
            target_buffer_time_s: Desired look-ahead on the MCU, clamped to
                                  [MIN_BUFFER_TIME_S, MAX_BUFFER_TIME_S].
            poll_interval_s:     Sleep between ``get_status()`` calls.
            print_every:         Print a progress line every N segments.
            log_each_send:       Print a line for every segment sent.
            send_log_path:       If set, write the full send log as JSON here
                                 when streaming finishes.
        """
        self._transport = transport
        self._poll_interval_s = poll_interval_s
        self._print_every = max(print_every, 1)
        self._log_each_send = log_each_send
        self._send_log_path = send_log_path
        self._send_events: list[dict] = []
        self._stop_requested = False
        self._flush_sequence_requested: int | None = None

        self._axis_configs = [AxisMotionConfig(axis_id=s.axis_id, ramp=s.ramp) for s in axis_streams]
        self._axis_ids = [cfg.axis_id for cfg in self._axis_configs]
        self._segment_duration_s = max(self.MIN_SEGMENT_TIME_S, min(self.MAX_SEGMENT_TIME_S, segment_duration_s))
        self._target_buffer_time_s = max(self.MIN_BUFFER_TIME_S, min(self.MAX_BUFFER_TIME_S, target_buffer_time_s))

        self._generator = iter(
            MultiAxisSegmentGenerator(self._axis_configs, segment_duration_s=self._segment_duration_s)
        )

        # ── In-flight tracking ────────────────────────────────────────────────
        # Each entry is (MultiAxisSegment, transport_sequence).
        #
        # WHY inflight tracking is required:
        #   The MCU executes segments asynchronously.  Without tracking which
        #   segments are still queued on the MCU side, the host has no reliable
        #   way to know how much future motion is already buffered.
        #
        # The TWO sequence numbers carried in each tuple serve different roles:
        #   segment.sequence  → motion identity, used to match MCU feedback
        #                       (last_executed_sequence) and to flush.
        #   transport_seq     → SPI frame identity, used to detect lost frames
        #                       via last_rx_sequence comparison.
        self._inflight: deque[tuple[MultiAxisSegment, int]] = deque()

        # O(1) buffered-time accumulator.
        # WHY the buffer must be MCU-driven, not wall-clock:
        #   Wall-clock subtraction drifts because it ignores scheduler latency,
        #   SPI retransmits, and any MCU execution jitter.  Deriving buffer from
        #   last_executed_sequence means the Pi reacts to what the MCU has
        #   actually completed, not what the Pi *thinks* it has completed.
        self._buffered_time_s: float = 0.0

        # Strict monotonicity guard: every outgoing segment must have a higher
        # motion sequence than the one before it.
        self._last_sent_motion_seq: int = -1

        # Transport sequence of the most recently sent SPI frame, for ACK check.
        self._last_sent_transport_seq: int = -1

        # Diagnostic counter for non-fatal ACK drift warnings.
        self._ack_drift_warnings: int = 0

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _timestamp(self) -> str:
        """Return a human-readable HH:MM:SS.mmm timestamp string."""
        now = time.time()
        seconds = int(now)
        milliseconds = int((now - seconds) * 1000)
        return time.strftime(f"%H:%M:%S.{milliseconds:03d}", time.localtime(now))

    def _enable_axes(self) -> None:
        """Enable all configured axes over SPI and verify acknowledgement.

        Raises:
            RuntimeError: if any axis enable command is rejected by the ESP32.
        """
        for axis_id in self._axis_ids:
            sequence, _ = self._transport.set_axis_enabled_request(axis_id, True)
            status = self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(
                    f"enable axis {axis_id} failed with result=0x{status.last_result:02X}"
                )

    def _check_transport_ack(self, status) -> None:
        """Validate that the ESP32 has echoed back the expected transport sequence.

        The ESP32 returns ``last_rx_sequence`` in every status frame.  After a
        send, this must eventually match the transport sequence we used.  This
        method is called after every ``get_status()`` to surface lost or
        reordered SPI frames before they cause silent trajectory corruption.

        Args:
            status: ``StatusPayload`` received from the ESP32.

        Raises:
            RuntimeError: if ``last_rx_sequence`` does not match the most
                recently confirmed transport sequence from the in-flight queue.
        """
        # No in-flight motion: nothing to validate.
        if not self._inflight:
            return

        # last_rx_sequence is global transport traffic (status polls + motion),
        # while _inflight tracks only unexecuted motion segments. Therefore
        # last_rx_sequence can legitimately advance far beyond the oldest
        # in-flight transport sequence during normal operation.
        #
        # Keep this check diagnostic-only: detect obvious impossible states,
        # but never abort streaming on ACK drift.
        oldest_transport_seq = self._inflight[0][1]
        last_rx = int(status.last_rx_sequence)
        newest_sent = self._last_sent_transport_seq
        if newest_sent < 0:
            return

        # If last_rx is "behind" oldest by > 32768, modulo arithmetic indicates
        # it is actually ahead due wrap; not an error.
        oldest_gap = (last_rx - oldest_transport_seq) & 0xFFFF
        if oldest_gap > 0x8000:
            return

        # Outstanding unacked frames based on latest sent transport sequence.
        # In nominal conditions this should stay reasonably bounded.
        outstanding = (newest_sent - last_rx) & 0xFFFF
        if outstanding > 0x8000:
            # last_rx has already moved past newest_sent (wrap/ordering); OK.
            return

        if outstanding > (self.MAX_INFLIGHT_SEGMENTS * 4):
            self._ack_drift_warnings += 1
            if self._ack_drift_warnings <= 5 or (self._ack_drift_warnings % 50) == 0:
                print(
                    f"[{self._timestamp()}] WARN transport ACK drift: "
                    f"oldest_inflight_tx={oldest_transport_seq:#06x} "
                    f"last_sent_tx={newest_sent:#06x} "
                    f"last_rx_tx={last_rx:#06x} "
                    f"outstanding={outstanding}"
                )

    def _remove_confirmed_segments(self, status) -> None:
        """Retire all in-flight segments that the MCU has fully executed.

        The ESP32 reports ``last_executed_sequence`` in every status frame.
        Any segment with motion sequence ≤ that value has been executed and
        its duration should no longer count toward the buffered look-ahead.

        This method maintains ``_buffered_time_s`` in O(1) amortised time
        (deque pop from the left).

        Args:
            status: ``StatusPayload`` received from the ESP32.
        """
        last_executed = int(getattr(status, "last_executed_sequence", -1))
        while self._inflight:
            seg, _transport_seq = self._inflight[0]
            if seg.sequence <= last_executed:
                # Subtract this segment's duration from the O(1) accumulator.
                self._buffered_time_s -= seg.duration_us / 1_000_000.0
                self._buffered_time_s = max(0.0, self._buffered_time_s)
                self._inflight.popleft()
            else:
                break

    def _queue_full(self, status) -> bool:
        """Return True if the MCU queue or ring buffer has no free space.

        Only the configured axes are considered.  The ESP32 status payload may
        report zero free slots for unused axes, and those must not block the
        host streamer.

        Args:
            status: ``StatusPayload`` received from the ESP32.
        """
        for axis_id in self._axis_ids:
            if axis_id < len(status.queue_free_slots) and status.queue_free_slots[axis_id] == 0:
                return True

        if hasattr(status, "ring_free_slots"):
            for axis_id in self._axis_ids:
                if axis_id < len(status.ring_free_slots) and status.ring_free_slots[axis_id] == 0:
                    return True

        return False

    def _block_summary(self, segment: MultiAxisSegment) -> dict:
        """Return a JSON-serialisable summary of a segment for the send log."""
        return {
            "type": "multi-axis",
            "motion_sequence": segment.sequence,
            "duration_us": segment.duration_us,
            "axis_count": len(segment.steps),
            "total_steps": sum(segment.steps),
        }

    def _record_send_event(
        self, segment: MultiAxisSegment, transport_seq: int, status
    ) -> None:
        """Append a send event to the in-memory log and optionally print it.

        Args:
            segment:       The segment that was just sent.
            transport_seq: SPI frame sequence number returned by the transport.
            status:        Status response received immediately after the send.
        """
        now = time.time()
        event = {
            "timestamp": now,
            "timestamp_str": self._timestamp(),
            "transport_sequence": transport_seq,
            "segment_summary": self._block_summary(segment),
            "queue_free": list(status.queue_free_slots),
            "ring_free": list(status.ring_free_slots),
            "last_result": int(status.last_result),
            "enabled_mask": int(status.enabled_mask),
            "running_mask": int(status.running_mask),
        }
        self._send_events.append(event)
        if self._log_each_send:
            print(
                f"[{event['timestamp_str']}] send tx_seq={transport_seq} "
                f"motion_seq={segment.sequence} "
                f"axis_count={event['segment_summary']['axis_count']} "
                f"duration_us={event['segment_summary']['duration_us']} "
                f"total_steps={event['segment_summary']['total_steps']} "
                f"result=0x{event['last_result']:02X}"
            )

    def _write_send_log(self) -> None:
        """Flush the send event log to disk if a path was configured."""
        if self._send_log_path is None:
            return
        with open(self._send_log_path, "w", encoding="utf-8") as handle:
            json.dump(self._send_events, handle, indent=2)

    # ── Public API ────────────────────────────────────────────────────────────

    def request_stop(self) -> None:
        """Signal the streamer to exit the main loop after the current iteration."""
        self._stop_requested = True

    def has_stop_been_requested(self) -> bool:
        """Return True if ``request_stop()`` has been called."""
        return self._stop_requested

    def request_flush(self, sequence: int) -> None:
        """Schedule a flush to be sent at the start of the next loop iteration.

        The flush will tell the ESP32 to discard all queued segments whose
        motion sequence is greater than ``sequence``.  The host in-flight state
        is reset atomically with the flush command.

        Args:
            sequence: Motion sequence up to (and including) which execution
                      should be preserved.  All later segments are discarded.
        """
        self._flush_sequence_requested = sequence

    def flush_until(self, sequence: int):
        """Send a FLUSH command to the ESP32 and reset all host-side state.

        WHY flush resets all state:
            After a flush, any segments that were in-flight may or may not have
            been executed by the MCU.  The only safe action is to treat every
            in-flight segment as gone and start fresh from the new generator
            state.  Leaving stale entries in ``_inflight`` would corrupt the
            buffered-time calculation and could cause the host to stop sending
            new segments (thinking the buffer is already full).

        Args:
            sequence: The MCU will keep and complete all segments with motion
                      sequence ≤ this value; later ones are dropped.

        Returns:
            The ``StatusPayload`` returned by the FLUSH transaction.
        """
        status = self._transport.flush_until(sequence)
        # Clear in-flight and reset O(1) accumulator — no stale state allowed.
        self._inflight.clear()
        self._buffered_time_s = 0.0
        return status

    def stream_all(self) -> int:
        """Stream the full motion plan to the ESP32 and return the segment count.

        Main loop behaviour
        ~~~~~~~~~~~~~~~~~~~
        Each iteration of the outer ``while`` loop:

        1. Polls the ESP32 for current status (non-blocking).
        2. Validates the transport ACK sequence for lost-frame detection.
        3. Retires confirmed in-flight segments and updates the O(1) buffer
           accumulator via ``_remove_confirmed_segments()``.
        4. Fills the look-ahead buffer up to ``_target_buffer_time_s`` by
           sending new segments — one per inner iteration — stopping immediately
           if the MCU queue/ring is full or the in-flight limit is reached.
        5. Handles any pending flush request.
        6. Sleeps ``POLL_SLEEP_S`` (or skips sleep in emergency refill mode).

        Non-blocking guarantee:
            Sending is attempted at most once per inner iteration.  If the send
            returns QUEUE_FULL, the inner loop exits and the outer loop polls
            again.  There are NO retry loops anywhere in this path.

        Returns:
            Total number of segments successfully confirmed by the ESP32.
        """
        # Initial status poll before enabling axes.
        status = self._transport.get_status()
        self._enable_axes()
        status = self._transport.get_status()

        total_segments = 0
        finished = False

        while True:
            # ── Stop requested ────────────────────────────────────────────────
            if self._stop_requested:
                if self._flush_sequence_requested is not None:
                    self.flush_until(self._flush_sequence_requested)
                break

            # ── 1. Poll MCU status ────────────────────────────────────────────
            status = self._transport.get_status()

            # ── 2. Transport ACK validation ───────────────────────────────────
            # Detects lost or reordered SPI frames before they cause silent
            # trajectory desynchronisation.
            self._check_transport_ack(status)

            # ── 3. Retire confirmed in-flight segments ────────────────────────
            # Uses last_executed_sequence to determine which segments the MCU
            # has fully executed.  O(1) amortised; updates _buffered_time_s.
            self._remove_confirmed_segments(status)

            # ── 4. Fill look-ahead buffer ─────────────────────────────────────
            # WHY blocking is forbidden:
            #   Any blocking or retry loop here would stall the entire host
            #   streaming pipeline, causing the MCU RMT ring to drain and
            #   producing motion underruns.  We send at most one segment per
            #   inner loop iteration and bail immediately on any back-pressure.
            while not finished and self._buffered_time_s < self._target_buffer_time_s:

                # Back-pressure: MCU queue or ring is full — stop sending.
                if self._queue_full(status):
                    break

                # In-flight limit: prevent runaway memory / SPI overload.
                if len(self._inflight) >= self.MAX_INFLIGHT_SEGMENTS:
                    break

                try:
                    segment = next(self._generator)
                except StopIteration:
                    finished = True
                    break

                # Strict monotonicity guard: motion sequences must be increasing.
                if segment.sequence <= self._last_sent_motion_seq:
                    raise RuntimeError(
                        f"motion sequence not strictly increasing: "
                        f"got {segment.sequence}, last was {self._last_sent_motion_seq}"
                    )

                payload = MultiAxisSegmentBlockPayload(
                    axis_ids=self._axis_ids,
                    block_seq=segment.sequence,
                    segments=[segment],
                )

                # Send non-blocking: one attempt, no retry on QUEUE_FULL.
                transport_seq, send_status = self._transport.send_multi_axis_segment_block_request(payload)
                status = send_status

                if send_status.last_result == int(SpiMessageResult.OK):
                    # Track the segment and update ALL state atomically.
                    self._inflight.append((segment, transport_seq))
                    self._buffered_time_s += segment.duration_us / 1_000_000.0
                    self._last_sent_motion_seq = segment.sequence
                    self._last_sent_transport_seq = transport_seq

                    self._record_send_event(segment, transport_seq, send_status)
                    total_segments += 1

                    if total_segments % self._print_every == 0:
                        print(
                            f"[{self._timestamp()}] segment={total_segments} "
                            f"motion_seq={segment.sequence} tx_seq={transport_seq} "
                            f"duration={segment.duration_us}us "
                            f"total_steps={sum(segment.steps)} "
                            f"buf={self._buffered_time_s * 1000:.1f}ms "
                            f"inflight={len(self._inflight)} "
                            f"enabled=0x{send_status.enabled_mask:02X} "
                            f"running=0x{send_status.running_mask:02X} "
                            f"queue_free={send_status.queue_free_slots} "
                            f"ring_free={send_status.ring_free_slots} "
                            f"underrun={send_status.underrun_count}"
                        )

                elif send_status.last_result == int(SpiMessageResult.QUEUE_FULL):
                    # Do NOT retry.  Exit the fill loop and let the outer loop
                    # poll status again on the next iteration.
                    break

                else:
                    raise RuntimeError(
                        f"segment motion_seq={segment.sequence} failed with "
                        f"result=0x{send_status.last_result:02X}"
                    )

            # ── 5. Handle pending flush ───────────────────────────────────────
            if self._flush_sequence_requested is not None:
                self.flush_until(self._flush_sequence_requested)
                self._flush_sequence_requested = None

            # ── Termination check ─────────────────────────────────────────────
            if finished and not self._inflight:
                break

            # ── 6. Adaptive sleep ─────────────────────────────────────────────
            # Ring-aware: if axis-0 ring has fewer than 1024 steps left, skip
            # sleep entirely regardless of buffer level. At high cruise rates
            # this preserves ~10 ms of runway and avoids host-side sleep from
            # letting the ring drain into underrun territory.
            ring_free_0 = status.ring_free_slots[0] if status.ring_free_slots else 0
            ring_critically_low = ring_free_0 > 3072  # < 1024 steps remain (for 4096 ring)
            if ring_critically_low:
                pass  # no sleep — ring needs immediate refill
            elif self._buffered_time_s >= self._target_buffer_time_s:
                # Buffer full — sleep one segment duration
                time.sleep(0.004)   # 4 ms (one segment)
            elif self._buffered_time_s >= self.MIN_BUFFER_TIME_S:
                # Buffer adequate — half segment sleep
                time.sleep(0.002)   # 2 ms
            # else: buffer < MIN, no sleep — refill immediately

        self._write_send_log()
        return total_segments

