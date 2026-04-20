from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt


def load_segments(path: Path) -> tuple[list[int], list[int], list[int], list[int]]:
    with path.open("r", encoding="utf-8") as handle:
        document = json.load(handle)

    segments = document.get("segments")
    if not isinstance(segments, list):
        raise ValueError("segment JSON file is missing 'segments' list")

    sequence = []
    steps = []
    durations = []
    directions = []
    for segment in segments:
        sequence.append(int(segment["sequence"]))
        steps.append(int(segment["steps"][0]))
        durations.append(int(segment["duration_us"]))
        directions.append(int(segment["directions"][0]))

    return sequence, steps, durations, directions


def plot_segments(
    sequence: list[int],
    steps: list[int],
    durations: list[int],
    directions: list[int],
    steps_per_rev: float,
    output: Path,
    title: str,
) -> None:
    rates = [step / (duration / 1_000_000.0) if duration > 0 else 0.0 for step, duration in zip(steps, durations)]
    rpms = [rate / steps_per_rev for rate in rates]
    directions_flag = [bool(d) for d in directions]

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    fig.suptitle(title, fontsize=14)

    axes[0].plot(sequence, steps, marker=".", linestyle="-", color="#1f77b4", label="steps")
    axes[0].set_ylabel("Steps")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(sequence, durations, marker=".", linestyle="-", color="#ff7f0e", label="duration_us")
    axes[1].set_ylabel("Duration (µs)")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    axes[2].plot(sequence, rates, marker=".", linestyle="-", color="#2ca02c", label="rate (steps/s)")
    axes[2].set_ylabel("Rate (steps/s)")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend()

    axes[3].plot(sequence, rpms, marker=".", linestyle="-", color="#d62728", label="rpm")
    axes[3].set_ylabel("RPM")
    axes[3].set_xlabel("Segment sequence")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend()

    fig.tight_layout(rect=[0, 0.03, 1, 0.97])
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot PickupWinder segment JSON metrics.")
    parser.add_argument("path", nargs="?", default="doc/generated/segments.json", help="Path to the segment JSON file")
    parser.add_argument("--output", default="doc/generated/segments_plot.png", help="Output image path")
    parser.add_argument("--steps-per-rev", type=float, default=6400.0, help="Steps per revolution for RPM calculation")
    parser.add_argument("--title", default="Segments JSON Metrics", help="Plot title")
    args = parser.parse_args()

    path = Path(args.path)
    sequence, steps, durations, directions = load_segments(path)
    plot_segments(
        sequence=sequence,
        steps=steps,
        durations=durations,
        directions=directions,
        steps_per_rev=args.steps_per_rev,
        output=Path(args.output),
        title=args.title,
    )
    print(f"Saved plot to {args.output}")


if __name__ == "__main__":
    main()
