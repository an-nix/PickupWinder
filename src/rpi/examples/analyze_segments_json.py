from __future__ import annotations

import argparse
import json
from pathlib import Path
from statistics import mean
from typing import Any


def load_segments(path: Path) -> list[dict[str, Any]]:
    with path.open("r", encoding="utf-8") as handle:
        document = json.load(handle)

    if int(document.get("version", 0)) != 1:
        raise ValueError(f"unsupported segment JSON version: {document.get('version')}")

    segments = document.get("segments")
    if not isinstance(segments, list):
        raise ValueError("segment JSON file is missing 'segments' list")

    for index, segment in enumerate(segments):
        if not isinstance(segment, dict):
            raise ValueError(f"segment {index} is not an object")
        if "steps" not in segment or "duration_us" not in segment or "directions" not in segment:
            raise ValueError(f"segment {index} is missing required fields")

    return segments


def analyze(segments: list[dict[str, Any]], steps_per_rev: float) -> dict[str, Any]:
    axis_count = len(segments[0]["steps"])
    axis_stats = []

    for axis_index in range(axis_count):
        steps = [int(segment["steps"][axis_index]) for segment in segments]
        durations = [int(segment["duration_us"]) for segment in segments]
        rates = [
            steps[i] / (durations[i] / 1_000_000) if durations[i] > 0 else 0.0
            for i in range(len(segments))
        ]
        rpms = [
            rates[i] / steps_per_rev if steps_per_rev > 0 else 0.0
            for i in range(len(segments))
        ]

        axis_stats.append(
            {
                "axis_index": axis_index,
                "segments": len(segments),
                "step_min": min(steps),
                "step_max": max(steps),
                "step_mean": mean(steps),
                "duration_min_us": min(durations),
                "duration_max_us": max(durations),
                "duration_mean_us": mean(durations),
                "rate_min_hz": min(rates),
                "rate_max_hz": max(rates),
                "rate_mean_hz": mean(rates),
                "rpm_min": min(rpms),
                "rpm_max": max(rpms),
                "rpm_mean": mean(rpms),
                "zero_step_segments": sum(1 for s in steps if s == 0),
            }
        )

    return {
        "count": len(segments),
        "axis_count": axis_count,
        "axis_stats": axis_stats,
    }


def print_summary(document: dict[str, Any], stats: dict[str, Any]) -> None:
    print("SEGMENT JSON ANALYSIS")
    print("======================")
    print(f"version: {document.get('version')}")
    print(f"axis_ids: {document.get('axis_ids')}")
    metadata = document.get("metadata", {})
    print(f"mode: {metadata.get('mode')}")
    print(f"generated_at: {metadata.get('generated_at')}")
    print(f"segment_duration_s: {metadata.get('segment_duration_s')}")
    print()
    print(f"total segments: {stats['count']}")
    print(f"axis count: {stats['axis_count']}")

    for axis in stats["axis_stats"]:
        print()
        print(f"Axis {axis['axis_index']}")
        print(f"  segments: {axis['segments']}")
        print(f"  steps: min={axis['step_min']} max={axis['step_max']} mean={axis['step_mean']:.2f}")
        print(
            f"  duration_us: min={axis['duration_min_us']} max={axis['duration_max_us']} mean={axis['duration_mean_us']:.1f}"
        )
        print(
            f"  rate: min={axis['rate_min_hz']:.1f} Hz max={axis['rate_max_hz']:.1f} Hz mean={axis['rate_mean_hz']:.1f} Hz"
        )
        print(
            f"  rpm: min={axis['rpm_min']:.1f} rpm max={axis['rpm_max']:.1f} rpm mean={axis['rpm_mean']:.1f} rpm"
        )
        print(f"  zero-step segments: {axis['zero_step_segments']}")


def print_detail(segments: list[dict[str, Any]], count: int = 10, steps_per_rev: float = 6400.0) -> None:
    print()
    print(f"First {min(count, len(segments))} segments:")
    for segment in segments[:count]:
        steps = segment["steps"]
        duration = int(segment["duration_us"])
        rate = sum(int(s) for s in steps) / (duration / 1_000_000) if duration > 0 else 0.0
        rpm = rate / steps_per_rev if duration > 0 else 0.0
        print(
            f"  seq={segment['sequence']} duration_us={duration} steps={steps} directions={segment['directions']} rate={rate:.1f} steps/s rpm={rpm:.1f}"
        )

    print()
    print(f"Last {min(count, len(segments))} segments:")
    for segment in segments[-count:]:
        steps = segment["steps"]
        duration = int(segment["duration_us"])
        rate = sum(int(s) for s in steps) / (duration / 1_000_000) if duration > 0 else 0.0
        rpm = rate / steps_per_rev if duration > 0 else 0.0
        print(
            f"  seq={segment['sequence']} duration_us={duration} steps={steps} directions={segment['directions']} rate={rate:.1f} steps/s rpm={rpm:.1f}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Analyze a PickupWinder segment JSON file and summarize steps/duration values"
    )
    parser.add_argument(
        "path",
        nargs="?",
        default="doc/generated/segments.json",
        help="Path to the segment JSON file (default: doc/generated/segments.json)",
    )
    parser.add_argument(
        "--detail",
        type=int,
        default=10,
        help="Number of first/last segments to display",
    )
    parser.add_argument(
        "--steps-per-rev",
        type=float,
        default=6400.0,
        help="Steps per revolution used to estimate RPM (default: 6400)",
    )
    args = parser.parse_args()

    path = Path(args.path)
    with path.open("r", encoding="utf-8") as handle:
        document = json.load(handle)

    segments = load_segments(path)
    stats = analyze(segments, steps_per_rev=args.steps_per_rev)
    print_summary(document, stats)
    print_detail(segments, count=args.detail, steps_per_rev=args.steps_per_rev)


if __name__ == "__main__":
    main()
