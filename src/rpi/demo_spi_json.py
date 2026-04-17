from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
import sys

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))

try:
    from .ramp import HybridRampBlockGenerator, RampConfig
except ImportError:  # pragma: no cover
    from ramp import HybridRampBlockGenerator, RampConfig  # type: ignore


@dataclass(slots=True)
class DemoSummary:
    axis_count: int
    block_count: int
    step_count: int
    segment_count: int


def _build_single_axis_config(args: argparse.Namespace) -> RampConfig:
    return RampConfig(
        axis_id=args.axis,
        target_rpm=args.target_rpm,
        accel_s=args.accel_s,
        cruise_s=args.cruise_s,
        decel_s=args.decel_s,
        reverse_direction=args.reverse,
    )


def _build_axis_configs(args: argparse.Namespace) -> list[RampConfig]:
    if not args.axis_config:
        return [_build_single_axis_config(args)]

    streams: list[RampConfig] = []
    for raw in args.axis_config:
        axis_text, rpm_text = raw.split(":", 1)
        axis_id = int(axis_text)
        target_rpm = float(rpm_text)
        streams.append(
            RampConfig(
                axis_id=axis_id,
                target_rpm=target_rpm,
                accel_s=args.accel_s,
                cruise_s=args.cruise_s,
                decel_s=args.decel_s,
                reverse_direction=args.reverse,
            )
        )
    return streams


def _block_to_json(block) -> dict:
    data: dict = {
        "axis_id": block.axis_id,
        "block_seq": block.block_seq,
    }
    if hasattr(block, "segments"):
        data["type"] = "segment"
        data["segments"] = [
            {
                "step_count": int(segment.step_count),
                "start_ticks": int(segment.start_ticks),
                "add_ticks": int(segment.add_ticks),
                "direction_reverse": bool(segment.direction_reverse),
            }
            for segment in block.segments
        ]
    else:
        data["type"] = "step"
        data["entries"] = [
            {
                "interval_ticks": int(entry.interval_ticks),
                "direction_reverse": bool(entry.direction_reverse),
            }
            for entry in block.entries
        ]
    return data


def run_demo(args: argparse.Namespace) -> DemoSummary:
    axis_configs = _build_axis_configs(args)
    output: dict[str, object] = {
        "metadata": {
            "target_rpm": args.target_rpm,
            "accel_s": args.accel_s,
            "cruise_s": args.cruise_s,
            "decel_s": args.decel_s,
            "reverse": args.reverse,
        },
        "axes": [],
    }

    total_blocks = 0
    total_steps = 0
    total_segments = 0

    for config in axis_configs:
        blocks = []
        generator = HybridRampBlockGenerator(config)
        for block in generator:
            block_json = _block_to_json(block)
            blocks.append(block_json)
            total_blocks += 1
            if block_json["type"] == "step":
                total_steps += len(block_json["entries"])
            else:
                total_segments += len(block_json["segments"])

        output["axes"].append(
            {
                "axis_id": config.axis_id,
                "target_rpm": config.target_rpm,
                "blocks": blocks,
            }
        )

    Path(args.output).write_text(json.dumps(output, indent=2))
    print(f"Wrote {total_blocks} blocks for {len(axis_configs)} axis(es) to {args.output}")
    return DemoSummary(
        axis_count=len(axis_configs),
        block_count=total_blocks,
        step_count=total_steps,
        segment_count=total_segments,
    )


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PickupWinder JSON block dump demo")
    parser.add_argument("--axis", type=int, default=0)
    parser.add_argument(
        "--axis-config",
        action="append",
        default=[],
        metavar="AXIS:RPM",
        help="dump multiple axes, for example --axis-config 0:1000 --axis-config 1:120",
    )
    parser.add_argument("--target-rpm", type=float, default=1000.0)
    parser.add_argument("--accel-s", type=float, default=10.0)
    parser.add_argument("--cruise-s", type=float, default=3.0)
    parser.add_argument("--decel-s", type=float, default=10.0)
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument(
        "--output",
        type=str,
        default="demo_spi_dump.json",
        help="output JSON file path",
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    summary = run_demo(args)
    print(
        f"Summary: axes={summary.axis_count} blocks={summary.block_count} "
        f"steps={summary.step_count} segments={summary.segment_count}"
    )


if __name__ == "__main__":
    main()
