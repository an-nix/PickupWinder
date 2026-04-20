from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from motion import (
    AxisMotionConfig,
    MultiAxisSegmentGenerator,
    SpindleKinematics,
    RampConfig,
)
from winding import (
    SynchronizedSegmentGenerator,
    WindingPattern,
    ScatterEngine,
    SyncAxisConfig,
)
from motion.segment_json import dump_segment_json, load_segment_json
from transport import Esp32SpiTransport, MockSpiTransport, MultiAxisRampStreamer


def build_manual_generator(args: argparse.Namespace):
    ramp = RampConfig(
        axis_id=args.axis,
        target_rpm=args.target_rpm,
        accel_s=args.accel_s,
        cruise_s=args.cruise_s,
        decel_s=args.decel_s,
        reverse_direction=args.reverse,
    )
    axis_configs = [AxisMotionConfig(axis_id=args.axis, ramp=ramp)]
    return MultiAxisSegmentGenerator(
        axis_configs,
        segment_duration_s=args.segment_duration_s,
        start_sequence=args.start_sequence,
    )


def build_wound_generator(args: argparse.Namespace):
    kinematics = SpindleKinematics(
        target_rpm=args.target_rpm,
        accel_s=args.accel_s,
        cruise_s=args.cruise_s,
        decel_s=args.decel_s,
    )
    pattern = WindingPattern(
        bobbin_width_mm=args.bobbin_width_mm,
        turns_per_mm=args.turns_per_mm,
    )
    scatter = ScatterEngine(
        amplitude_mm=args.scatter_amplitude_mm,
        damping_margin_mm=args.damping_margin_mm,
    )
    spindle_cfg = SyncAxisConfig(
        axis_index=args.spindle_axis,
        steps_per_unit=args.spindle_steps_per_unit,
        reverse_direction=args.reverse_spindle,
    )
    traverse_cfg = SyncAxisConfig(
        axis_index=args.traverse_axis,
        steps_per_unit=args.traverse_steps_per_unit,
        reverse_direction=args.reverse_traverse,
    )
    return SynchronizedSegmentGenerator(
        kinematics,
        pattern,
        scatter,
        spindle_cfg,
        traverse_cfg,
        segment_duration_s=args.segment_duration_s,
        start_sequence=args.start_sequence,
    )


def infer_target_hz(segments: list):
    if not segments:
        return 1.0
    max_hz = 0.0
    for segment in segments:
        if segment.duration_us <= 0:
            continue
        rate = sum(segment.steps) / (segment.duration_us / 1_000_000.0)
        max_hz = max(max_hz, rate)
    return max_hz if max_hz > 0.0 else 1.0


def write_segment_json(args: argparse.Namespace) -> None:
    if args.mode == "manual":
        generator = build_manual_generator(args)
    else:
        generator = build_wound_generator(args)

    metadata = {
        "mode": args.mode,
        "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        "segment_duration_s": args.segment_duration_s,
    }

    dump_segment_json(
        args.output,
        generator,
        axis_ids=args.axis_ids,
        metadata=metadata,
    )
    print(f"Wrote segment JSON to {args.output}")


def run_segment_json(args: argparse.Namespace) -> None:
    axis_ids, metadata, segments = load_segment_json(args.input)
    segment_list = list(segments)
    if not segment_list:
        raise SystemExit("No segments found in JSON file")

    target_hz = args.target_hz or infer_target_hz(segment_list)
    segment_duration_s = args.segment_duration_s
    if segment_duration_s is None:
        segment_duration_s = float(metadata.get("segment_duration_s", 0.004))

    print(
        f"Streaming {len(segment_list)} segments from {args.input} "
        f"with target_hz={target_hz:.1f} segment_duration_s={segment_duration_s:.6f}"
    )
    if args.simulate:
        transport = MockSpiTransport()
        print("Simulation mode enabled: no real SPI access")
    else:
        transport = Esp32SpiTransport(bus=args.bus, device=args.device, speed_hz=args.speed_hz, mode=0)

    with transport:
        streamer = MultiAxisRampStreamer.from_axis_ids(
            transport=transport,
            axis_ids=axis_ids,
            target_hz=target_hz,
            segment_duration_s=segment_duration_s,
            poll_interval_s=args.poll_interval_s,
            print_every=args.print_every,
        )
        streamer.set_generator(iter(segment_list))
        total = streamer.stream_all()
        print(f"Streamed {total} segments")
        if args.disable_all and not args.simulate:
            print("Disabling all axes")
            transport.disable_all()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Generate and replay PickupWinder segment JSON files")
    subparsers = parser.add_subparsers(dest="command", required=True)

    gen_parser = subparsers.add_parser("generate", help="Generate a segment JSON file from an existing motion generator")
    gen_parser.add_argument("--output", required=True, help="Path to write JSON segment file")
    gen_parser.add_argument("--mode", choices=("manual", "wound"), required=True)
    gen_parser.add_argument("--segment-duration-s", type=float, default=0.004)
    gen_parser.add_argument("--start-sequence", type=int, default=0)
    gen_parser.add_argument("--axis", type=int, default=0, help="Axis id for manual mode")
    gen_parser.add_argument("--target-rpm", type=float, default=1000.0)
    gen_parser.add_argument("--accel-s", type=float, default=1.0)
    gen_parser.add_argument("--cruise-s", type=float, default=1.0)
    gen_parser.add_argument("--decel-s", type=float, default=1.0)
    gen_parser.add_argument("--reverse", action="store_true")
    gen_parser.add_argument("--axis-ids", type=int, nargs="*", default=None, help="Explicit axis ids for the generated segments")
    gen_parser.add_argument("--bobbin-width-mm", type=float, default=15.0)
    gen_parser.add_argument("--turns-per-mm", type=float, default=10.0)
    gen_parser.add_argument("--scatter-amplitude-mm", type=float, default=0.5)
    gen_parser.add_argument("--damping-margin-mm", type=float, default=2.0)
    gen_parser.add_argument("--spindle-axis", type=int, default=0)
    gen_parser.add_argument("--traverse-axis", type=int, default=1)
    gen_parser.add_argument("--spindle-steps-per-unit", type=float, default=6400.0)
    gen_parser.add_argument("--traverse-steps-per-unit", type=float, default=3072.0)
    gen_parser.add_argument("--reverse-spindle", action="store_true")
    gen_parser.add_argument("--reverse-traverse", action="store_true")

    run_parser = subparsers.add_parser("run", help="Replay a segment JSON file via SPI")
    run_parser.add_argument("--input", required=True, help="Path to read JSON segment file")
    run_parser.add_argument("--bus", type=int, default=0)
    run_parser.add_argument("--device", type=int, default=0)
    run_parser.add_argument("--speed-hz", type=int, default=1_000_000)
    run_parser.add_argument("--segment-duration-s", type=float, default=None)
    run_parser.add_argument("--target-hz", type=float, default=0.0)
    run_parser.add_argument("--poll-interval-s", type=float, default=0.001)
    run_parser.add_argument("--print-every", type=int, default=8)
    run_parser.add_argument("--disable-all", action="store_true")
    run_parser.add_argument("--simulate", action="store_true", help="Run without real SPI using a mock transport")

    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if args.command == "generate":
        write_segment_json(args)
    elif args.command == "run":
        run_segment_json(args)
    else:
        parser.error("unknown command")


if __name__ == "__main__":
    main()
