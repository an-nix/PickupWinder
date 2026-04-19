from __future__ import annotations

from dataclasses import dataclass
from typing import Iterator, List, Union
import warnings

from transport.messages import (
        SEGMENT_BLOCK_SIZE,
        STEP_BLOCK_SIZE,
        MotionSegment,
        MultiAxisSegment,
        SegmentBlockPayload,
        StepBlockPayload,
        StepEntry,
    )


@dataclass(slots=True)
class RampConfig:
    axis_id: int = 0
    steps_per_rev: int = 200 * 32
    start_hz: float = 200.0
    target_rpm: float = 1000.0
    accel_s: float = 10.0
    cruise_s: float = 3.0
    decel_s: float = 10.0
    # Must match RMT_STEP_RESOLUTION_HZ in stepper_driver.h (80 MHz).
    # Used only by RampBlockGenerator / SegmentBlockGenerator for tick
    # calculations. MultiAxisSegmentGenerator works in steps, not ticks,
    # and is not affected by this value.
    resolution_hz: int = 80_000_000
    reverse_direction: bool = False
    phase_segments: int = 8
    segment_duration_s: float = 0.05

    @property
    def target_hz(self) -> float:
        return self.target_rpm / 60.0 * float(self.steps_per_rev)

    @property
    def total_duration(self) -> float:
        return self.accel_s + self.cruise_s + self.decel_s


class RampBlockGenerator:
    """
    .. deprecated::
        Use MultiAxisSegmentGenerator via MultiAxisRampStreamer instead.
        This class generates per-axis step blocks and is no longer
        used by the main streaming path.
    """

    def __init__(self, config: RampConfig):
        warnings.warn(
            "RampBlockGenerator is deprecated. "
            "Use MultiAxisSegmentGenerator instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        self.config = config
        self._time_cursor = 0.0
        self._tick_error = 0.0
        self._block_seq = 0

    def _hz_at_time(self, t: float) -> float:
        cfg = self.config
        if t < cfg.accel_s:
            accel_rate = (cfg.target_hz - cfg.start_hz) / cfg.accel_s
            return cfg.start_hz + accel_rate * t
        if t < cfg.accel_s + cfg.cruise_s:
            return cfg.target_hz
        decel_t = t - cfg.accel_s - cfg.cruise_s
        decel_rate = (cfg.target_hz - cfg.start_hz) / cfg.decel_s
        return cfg.target_hz - decel_rate * decel_t

    def _next_interval_ticks(self) -> int | None:
        cfg = self.config
        if self._time_cursor >= cfg.total_duration:
            return None

        hz = max(self._hz_at_time(self._time_cursor), 1.0)
        ideal_ticks = float(cfg.resolution_hz) / hz
        quantized = ideal_ticks + self._tick_error
        interval_ticks = max(int(quantized + 0.5), 12)
        self._tick_error += ideal_ticks - float(interval_ticks)
        self._time_cursor += interval_ticks / float(cfg.resolution_hz)
        return interval_ticks

    def __iter__(self) -> Iterator[StepBlockPayload]:
        while True:
            entries: List[StepEntry] = []
            for _ in range(STEP_BLOCK_SIZE):
                interval_ticks = self._next_interval_ticks()
                if interval_ticks is None:
                    break
                entries.append(
                    StepEntry(
                        interval_ticks=interval_ticks,
                        direction_reverse=self.config.reverse_direction,
                    )
                )

            if not entries:
                return

            yield StepBlockPayload(
                axis_id=self.config.axis_id,
                block_seq=self._block_seq & 0xFF,
                entries=entries,
            )
            self._block_seq += 1


class SegmentBlockGenerator:
    """
    .. deprecated::
        Use MultiAxisSegmentGenerator via MultiAxisRampStreamer instead.
        This class generates per-axis arithmetic segment blocks and is no longer
        used by the main streaming path.
    """

    def __init__(self, config: RampConfig):
        warnings.warn(
            "SegmentBlockGenerator is deprecated. "
            "Use MultiAxisSegmentGenerator instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        self.config = config
        self._block_seq = 0
        self._segments = iter(self._build_segments())

    def _ticks_from_hz(self, hz: float) -> int:
        hz = max(hz, 1.0)
        return max(12, int(round(float(self.config.resolution_hz) / hz)))

    def _append_split_segment(
        self,
        out: list[MotionSegment],
        *,
        step_count: int,
        start_ticks: int,
        add_ticks: int,
    ) -> None:
        remaining = max(step_count, 0)
        current_start = start_ticks
        while remaining > 0:
            chunk = min(remaining, 0xFFFF)
            out.append(
                MotionSegment(
                    step_count=chunk,
                    start_ticks=current_start,
                    add_ticks=add_ticks,
                    direction_reverse=self.config.reverse_direction,
                )
            )
            remaining -= chunk
            current_start += add_ticks * chunk

    def _phase_segment_count(self, duration_s: float) -> int:
        if duration_s <= 0.0:
            return 0
        base_count = max(1, int(round(duration_s / max(self.config.segment_duration_s, 1e-4))))
        return max(base_count, self.config.phase_segments)

    def _append_phase_segments(
        self,
        out: list[MotionSegment],
        *,
        duration_s: float,
        hz_fn,
    ) -> None:
        phase_segments = self._phase_segment_count(duration_s)
        if phase_segments == 0:
            return

        dt = duration_s / float(phase_segments)
        for index in range(phase_segments):
            t0 = index * dt
            t1 = duration_s if index == phase_segments - 1 else (index + 1) * dt
            hz0 = max(hz_fn(t0), 1.0)
            hz1 = max(hz_fn(t1), 1.0)
            avg_hz = max((hz0 + hz1) * 0.5, 1.0)
            step_count = max(1, int(round(avg_hz * (t1 - t0))))

            start_ticks = self._ticks_from_hz(hz0)
            end_ticks = self._ticks_from_hz(hz1)
            if step_count <= 1:
                add_ticks = 0
            else:
                add_ticks = int(round((end_ticks - start_ticks) / float(step_count - 1)))

            self._append_split_segment(
                out,
                step_count=step_count,
                start_ticks=start_ticks,
                add_ticks=add_ticks,
            )

    def _build_segments(self) -> list[MotionSegment]:
        cfg = self.config
        segments: list[MotionSegment] = []

        accel_rate = 0.0 if cfg.accel_s <= 0.0 else (cfg.target_hz - cfg.start_hz) / cfg.accel_s
        decel_rate = 0.0 if cfg.decel_s <= 0.0 else (cfg.target_hz - cfg.start_hz) / cfg.decel_s

        self._append_phase_segments(
            segments,
            duration_s=cfg.accel_s,
            hz_fn=lambda t: cfg.start_hz + accel_rate * t,
        )

        if cfg.cruise_s > 0.0:
            cruise_steps = max(1, int(round(cfg.target_hz * cfg.cruise_s)))
            self._append_split_segment(
                segments,
                step_count=cruise_steps,
                start_ticks=self._ticks_from_hz(cfg.target_hz),
                add_ticks=0,
            )

        self._append_phase_segments(
            segments,
            duration_s=cfg.decel_s,
            hz_fn=lambda t: cfg.target_hz - decel_rate * t,
        )
        return [segment for segment in segments if segment.step_count > 0]

    def _next_segment(self) -> MotionSegment | None:
        return next(self._segments, None)

    def __iter__(self) -> Iterator[SegmentBlockPayload]:
        while True:
            segments: List[MotionSegment] = []
            for _ in range(SEGMENT_BLOCK_SIZE):
                segment = self._next_segment()
                if segment is None:
                    break
                segments.append(segment)

            if not segments:
                return

            yield SegmentBlockPayload(
                axis_id=self.config.axis_id,
                block_seq=self._block_seq & 0xFF,
                segments=segments,
            )
            self._block_seq += 1


BlockPayload = Union[StepBlockPayload, SegmentBlockPayload]


class HybridRampBlockGenerator:
    """
    .. deprecated::
        Use MultiAxisSegmentGenerator via MultiAxisRampStreamer instead.
        This class generates a hybrid of step and segment blocks and is no
        longer used by the main streaming path.
    """

    def __init__(self, config: RampConfig, *, segment_threshold_hz: float = 8_000.0):
        warnings.warn(
            "HybridRampBlockGenerator is deprecated. "
            "Use MultiAxisSegmentGenerator instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        self.config = config
        self.segment_threshold_hz = max(segment_threshold_hz, 1.0)
        self._segments = SegmentBlockGenerator(config)._build_segments()
        self._block_seq = 0
        self._step_ticks_threshold = self._ticks_from_hz(self.segment_threshold_hz)

    def _ticks_from_hz(self, hz: float) -> int:
        return max(12, int(round(float(self.config.resolution_hz) / max(hz, 1.0))))

    def _should_send_as_step_blocks(self, segment: MotionSegment) -> bool:
        if segment.step_count <= 1:
            return True
        if segment.start_ticks >= self._step_ticks_threshold:
            return True
        if abs(segment.add_ticks) > 1 and segment.step_count <= STEP_BLOCK_SIZE:
            return True
        return False

    def _emit_segment_block(self, segments: List[MotionSegment]) -> SegmentBlockPayload:
        payload = SegmentBlockPayload(
            axis_id=self.config.axis_id,
            block_seq=self._block_seq & 0xFF,
            segments=segments,
        )
        self._block_seq += 1
        return payload

    def _emit_step_blocks(self, segment: MotionSegment) -> Iterator[StepBlockPayload]:
        entries: List[StepEntry] = []
        current_ticks = segment.start_ticks
        for _ in range(segment.step_count):
            entries.append(
                StepEntry(
                    interval_ticks=current_ticks,
                    direction_reverse=segment.direction_reverse,
                )
            )
            current_ticks += segment.add_ticks
            if len(entries) >= STEP_BLOCK_SIZE:
                yield StepBlockPayload(
                    axis_id=self.config.axis_id,
                    block_seq=self._block_seq & 0xFF,
                    entries=entries,
                )
                self._block_seq += 1
                entries = []

        if entries:
            yield StepBlockPayload(
                axis_id=self.config.axis_id,
                block_seq=self._block_seq & 0xFF,
                entries=entries,
            )
            self._block_seq += 1

    def __iter__(self) -> Iterator[BlockPayload]:
        buffered_segments: List[MotionSegment] = []
        for segment in self._segments:
            if self._should_send_as_step_blocks(segment):
                if buffered_segments:
                    yield self._emit_segment_block(buffered_segments)
                    buffered_segments = []
                yield from self._emit_step_blocks(segment)
                continue

            buffered_segments.append(segment)
            if len(buffered_segments) >= SEGMENT_BLOCK_SIZE:
                yield self._emit_segment_block(buffered_segments)
                buffered_segments = []

        if buffered_segments:
            yield self._emit_segment_block(buffered_segments)


@dataclass(slots=True)
class AxisMotionConfig:
    axis_id: int
    ramp: RampConfig


class MultiAxisSegmentGenerator:
    def __init__(self, axis_configs: list[AxisMotionConfig], *, segment_duration_s: float = 0.004):
        self.axis_configs = axis_configs
        self.segment_duration_s = max(0.002, min(0.005, segment_duration_s))
        self._sequence = 0
        self._axis_errors = [0.0 for _ in axis_configs]
        self._axis_total_durations = [config.ramp.total_duration for config in axis_configs]
        self._overall_duration = max(self._axis_total_durations) if axis_configs else 0.0

    def _hz_at_time(self, config: RampConfig, t: float) -> float:
        if t < 0.0 or t >= config.total_duration:
            return 0.0

        if t < config.accel_s:
            accel_rate = 0.0 if config.accel_s <= 0.0 else (config.target_hz - config.start_hz) / config.accel_s
            return config.start_hz + accel_rate * t

        cruise_end = config.accel_s + config.cruise_s
        if t < cruise_end:
            return config.target_hz

        decel_t = t - cruise_end
        decel_rate = 0.0 if config.decel_s <= 0.0 else (config.target_hz - config.start_hz) / config.decel_s
        return max(config.target_hz - decel_rate * decel_t, 0.0)

    def __iter__(self) -> Iterator[MultiAxisSegment]:
        time_cursor = 0.0
        while time_cursor < self._overall_duration:
            next_cursor = min(time_cursor + self.segment_duration_s, self._overall_duration)
            duration_s = next_cursor - time_cursor
            duration_us = int(round(duration_s * 1_000_000))

            steps: list[int] = []
            directions: list[int] = []
            for index, config in enumerate(self.axis_configs):
                hz0 = self._hz_at_time(config.ramp, time_cursor)
                hz1 = self._hz_at_time(config.ramp, next_cursor)
                delta_steps = ((hz0 + hz1) * 0.5) * duration_s
                count = int(round(self._axis_errors[index] + delta_steps))
                self._axis_errors[index] += delta_steps - float(count)
                if count < 0:
                    count = 0
                steps.append(count)
                directions.append(1 if config.ramp.reverse_direction else 0)

            yield MultiAxisSegment(
                sequence=self._sequence,
                duration_us=duration_us,
                steps=steps,
                directions=directions,
            )
            self._sequence = (self._sequence + 1) & 0xFFFF
            time_cursor = next_cursor
