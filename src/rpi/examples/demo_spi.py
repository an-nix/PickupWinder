from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from motion import AxisMotionConfig, RampConfig, MultiAxisSegmentGenerator, SpindleKinematics
from winding import WindingPattern, ScatterEngine, SyncAxisConfig, SynchronizedSegmentGenerator
from transport import Esp32SpiTransport
from transport import MultiAxisRampStreamer, StreamAxisConfig


@dataclass(slots=True)
class DemoSummary:
    block_count: int
    axis_count: int


def _build_single_axis_config(args: argparse.Namespace) -> RampConfig:
    return RampConfig(
        axis_id=args.axis,
        target_rpm=args.target_rpm,
        accel_s=args.accel_s,
        cruise_s=args.cruise_s,
        decel_s=args.decel_s,
        reverse_direction=args.reverse,
    )


def run_demo_manual(args: argparse.Namespace, transport: Esp32SpiTransport) -> DemoSummary:
    print("Running Manual/Jog Mode")
    cfg = _build_single_axis_config(args)
    # the older simple API via MultiAxisSegmentGenerator
    generator = MultiAxisSegmentGenerator(
        [AxisMotionConfig(axis_id=cfg.axis_id, ramp=cfg)],
        start_sequence=0
    )
    
    streamer = MultiAxisRampStreamer(
        transport,
        poll_interval_s=args.poll_interval_s,
        print_every=args.print_every,
        log_each_send=args.log_each_send,
    )
    block_count = streamer.stream_all(generator)
    return DemoSummary(block_count=block_count, axis_count=1)

def run_demo_wound(args: argparse.Namespace, transport: Esp32SpiTransport) -> DemoSummary:
    print("Running Electronic Gearing Mode (Synchronized Winding)")
    kinematics = SpindleKinematics(
        target_rpm=args.target_rpm,
        accel_s=args.accel_s,
        cruise_s=args.cruise_s,
        decel_s=args.decel_s,
    )
    pattern = WindingPattern(bobbin_width_mm=15.0, turns_per_mm=10.0)
    scatter = ScatterEngine(amplitude_mm=0.5, damping_margin_mm=2.0)
    
    spindle_cfg = SyncAxisConfig(axis_index=0, steps_per_unit=6400)
    traverse_cfg = SyncAxisConfig(axis_index=1, steps_per_unit=3072)
    
    generator = SynchronizedSegmentGenerator(
        kinematics, pattern, scatter, spindle_cfg, traverse_cfg
    )
    
    streamer = MultiAxisRampStreamer(
        transport,
        poll_interval_s=args.poll_interval_s,
        print_every=args.print_every,
        log_each_send=args.log_each_send,
    )
    block_count = streamer.stream_all(generator)
    return DemoSummary(block_count=block_count, axis_count=2)


def run_demo(args: argparse.Namespace) -> DemoSummary:
    with Esp32SpiTransport(
        bus=args.bus,
        device=args.device,
        speed_hz=args.speed_hz,
        mode=0,
    ) as transport:
        print("Initial status:", transport.get_status())
        print("Reset stats:", transport.reset_stats())

        if args.sync_mode:
            summary = run_demo_wound(args, transport)
        else:
            summary = run_demo_manual(args, transport)

        print(f"Finished streaming {summary.block_count} motion blocks across {summary.axis_count} axis stream(s)")
        for _ in range(args.final_polls):
            status = transport.get_status()
            print("Final status:", status)
            time.sleep(args.poll_interval_s)

        if args.disable_all:
            print("Disable all:", transport.disable_all())

        return summary


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PickupWinder SPI ramp streaming demo")
    parser.add_argument("--bus", type=int, default=0)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--speed-hz", type=int, default=1_000_000)
    parser.add_argument("--axis", type=int, default=0)
    parser.add_argument(
        "--sync-mode",
        action="store_true",
        help="Use the Electronic Gearing mode instead of single-axis manual config"
    )
    parser.add_argument("--target-rpm", type=float, default=1000.0)
    parser.add_argument("--accel-s", type=float, default=10.0)
    parser.add_argument("--cruise-s", type=float, default=3.0)
    parser.add_argument("--decel-s", type=float, default=10.0)
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--poll-interval-s", type=float, default=0.001)
    parser.add_argument("--print-every", type=int, default=8)
    parser.add_argument("--log-each-send", action="store_true",
        help="print a log line for each block send request"
    )
    parser.add_argument("--final-polls", type=int, default=5)
    parser.add_argument("--disable-all", action="store_true")
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    run_demo(args)


if __name__ == "__main__":
    main()

