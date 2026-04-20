from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import matplotlib.pyplot as plt
from motion.segment_json import dump_segment_json, load_segment_json
from transport import MockSpiTransport, MultiAxisRampStreamer


def load_segments(path: Path) -> tuple[list[int], dict[str, Any], list[Any]]:
    axis_ids, metadata, segments = load_segment_json(path)
    return axis_ids, metadata, list(segments)


def flatten_received_segments(mock_transport: MockSpiTransport) -> list[Any]:
    segments = []
    for payload in mock_transport.sent_payloads:
        segments.extend(payload.segments)
    return segments


def write_segments(path: Path, axis_ids: list[int], metadata: dict[str, Any], segments: list[Any], source: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    output_metadata = dict(metadata)
    output_metadata["source"] = source
    dump_segment_json(path, iter(segments), axis_ids=axis_ids, metadata=output_metadata)


def plot_comparison(
    original_segments: list[Any],
    received_segments: list[Any],
    output_path: Path,
    steps_per_rev: float,
    title: str,
) -> None:
    def compute_values(segments: list[Any]) -> tuple[list[int], list[int], list[float], list[float]]:
        seq = [seg.sequence for seg in segments]
        steps = [seg.steps[0] for seg in segments]
        durations = [seg.duration_us for seg in segments]
        rate = [steps[i] / (durations[i] / 1_000_000.0) if durations[i] > 0 else 0.0 for i in range(len(segments))]
        rpm = [r / steps_per_rev for r in rate]
        return seq, steps, durations, rpm

    seq_o, steps_o, durations_o, rpm_o = compute_values(original_segments)
    seq_r, steps_r, durations_r, rpm_r = compute_values(received_segments)

    fig, axes = plt.subplots(4, 1, figsize=(14, 14), sharex=False)
    fig.suptitle(title, fontsize=16)

    axes[0].plot(seq_o, steps_o, label="generated", color="#1f77b4")
    axes[0].plot(seq_r, steps_r, label="received", color="#ff7f0e", linestyle="--")
    axes[0].set_ylabel("steps")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(seq_o, durations_o, label="generated", color="#1f77b4")
    axes[1].plot(seq_r, durations_r, label="received", color="#ff7f0e", linestyle="--")
    axes[1].set_ylabel("duration_us")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(seq_o, [s / (d / 1_000_000.0) if d > 0 else 0.0 for s, d in zip(steps_o, durations_o)], label="generated", color="#1f77b4")
    axes[2].plot(seq_r, [s / (d / 1_000_000.0) if d > 0 else 0.0 for s, d in zip(steps_r, durations_r)], label="received", color="#ff7f0e", linestyle="--")
    axes[2].set_ylabel("rate (steps/s)")
    axes[2].legend()
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(seq_o, rpm_o, label="generated", color="#1f77b4")
    axes[3].plot(seq_r, rpm_r, label="received", color="#ff7f0e", linestyle="--")
    axes[3].set_ylabel("rpm")
    axes[3].set_xlabel("segment sequence")
    axes[3].legend()
    axes[3].grid(True, alpha=0.3)

    fig.tight_layout(rect=[0, 0.03, 1, 0.97])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=150)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare generated and mock-received segment JSON data")
    parser.add_argument("input", help="Input generated segment JSON file")
    parser.add_argument("--output-plot", default="doc/generated/segments_compare_plot.png", help="Output plot image path")
    parser.add_argument("--generated-json", default="doc/generated/generated_segments.json", help="Output generated segments JSON copy")
    parser.add_argument("--received-json", default="doc/generated/received_segments.json", help="Output received segments JSON from mock")
    parser.add_argument("--steps-per-rev", type=float, default=6400.0, help="Steps per revolution for RPM calculation")
    parser.add_argument("--print-every", type=int, default=100, help="Print progress every N segments")
    args = parser.parse_args()

    axis_ids, metadata, original_segments = load_segments(Path(args.input))

    mock_transport = MockSpiTransport()
    streamer = MultiAxisRampStreamer.from_axis_ids(
        transport=mock_transport,
        axis_ids=axis_ids,
        target_hz=max(
            sum(seg.steps) / (seg.duration_us / 1_000_000.0)
            for seg in original_segments
            if seg.duration_us > 0
        ),
        segment_duration_s=float(metadata.get("segment_duration_s", 0.004)),
        print_every=args.print_every,
    )
    streamer.set_generator(iter(original_segments))
    streamer.stream_all()

    received_segments = flatten_received_segments(mock_transport)

    write_segments(Path(args.generated_json), axis_ids, metadata, original_segments, source="generated")
    write_segments(Path(args.received_json), axis_ids, metadata, received_segments, source="received")

    plot_comparison(
        original_segments,
        received_segments,
        Path(args.output_plot),
        steps_per_rev=args.steps_per_rev,
        title="Generated vs Received Segment Comparison",
    )

    print(f"Wrote generated JSON to {args.generated_json}")
    print(f"Wrote received JSON to {args.received_json}")
    print(f"Wrote comparison plot to {args.output_plot}")


if __name__ == "__main__":
    main()
