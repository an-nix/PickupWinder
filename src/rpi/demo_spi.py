from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path
import sys
import time

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from motion import RampConfig
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


def _build_axis_streams(args: argparse.Namespace) -> list[StreamAxisConfig]:
    if not args.axis_config:
        cfg = _build_single_axis_config(args)
        return [
            StreamAxisConfig(
                axis_id=cfg.axis_id,
                ramp=cfg,
                minimum_free_blocks=args.min_free_blocks,
                prefill_blocks=args.queue_prefill_blocks,
                low_watermark_blocks=args.queue_low_watermark_blocks,
                max_queued_blocks=args.max_queued_blocks,
            )
        ]

    streams: list[StreamAxisConfig] = []
    for raw in args.axis_config:
        axis_text, rpm_text = raw.split(":", 1)
        axis_id = int(axis_text)
        target_rpm = float(rpm_text)
        streams.append(
            StreamAxisConfig(
                axis_id=axis_id,
                ramp=RampConfig(
                    axis_id=axis_id,
                    target_rpm=target_rpm,
                    accel_s=args.accel_s,
                    cruise_s=args.cruise_s,
                    decel_s=args.decel_s,
                    reverse_direction=args.reverse,
                ),
                minimum_free_blocks=args.min_free_blocks,
                prefill_blocks=args.queue_prefill_blocks,
                low_watermark_blocks=args.queue_low_watermark_blocks,
                max_queued_blocks=args.max_queued_blocks,
            )
        )
    return streams


def run_demo(args: argparse.Namespace) -> DemoSummary:
    with Esp32SpiTransport(
        bus=args.bus,
        device=args.device,
        speed_hz=args.speed_hz,
        mode=0,
    ) as transport:
        print("Initial status:", transport.get_status())
        print("Reset stats:", transport.reset_stats())

        axis_streams = _build_axis_streams(args)
        streamer = MultiAxisRampStreamer(
            transport,
            axis_streams,
            poll_interval_s=args.poll_interval_s,
            print_every=args.print_every,
            log_each_send=args.log_each_send,
            send_log_path=args.send_log_path,
        )
        block_count = streamer.stream_all()

        print(f"Finished streaming {block_count} motion blocks across {len(axis_streams)} axis stream(s)")
        for _ in range(args.final_polls):
            status = transport.get_status()
            print("Final status:", status)
            time.sleep(args.poll_interval_s)

        if args.disable_all:
            print("Disable all:", transport.disable_all())

        return DemoSummary(block_count=block_count, axis_count=len(axis_streams))


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="PickupWinder SPI ramp streaming demo")
    parser.add_argument("--bus", type=int, default=0)
    parser.add_argument("--device", type=int, default=0)
    parser.add_argument("--speed-hz", type=int, default=4_000_000)
    parser.add_argument("--axis", type=int, default=0)
    parser.add_argument(
        "--axis-config",
        action="append",
        default=[],
        metavar="AXIS:RPM",
        help="stream multiple axes, for example --axis-config 0:1000 --axis-config 1:120",
    )
    parser.add_argument("--target-rpm", type=float, default=1000.0)
    parser.add_argument("--accel-s", type=float, default=10.0)
    parser.add_argument("--cruise-s", type=float, default=3.0)
    parser.add_argument("--decel-s", type=float, default=10.0)
    parser.add_argument("--reverse", action="store_true")
    parser.add_argument("--min-free-blocks", type=int, default=1)
    parser.add_argument(
        "--queue-prefill-blocks",
        type=int,
        default=None,
        help="target queued block count per axis; default keeps the ESP32 queue almost full",
    )
    parser.add_argument(
        "--queue-low-watermark-blocks",
        type=int,
        default=None,
        help="refill when queued blocks drop to this level; default is about one third of the target fill",
    )
    parser.add_argument("--max-queued-blocks", type=int, default=None,
        help="limit the number of queued blocks per axis for faster response to stop/change requests",
    )
    parser.add_argument("--poll-interval-s", type=float, default=0.001)
    parser.add_argument("--print-every", type=int, default=8)
    parser.add_argument("--log-each-send", action="store_true",
        help="print a log line for each block send request"
    )
    parser.add_argument("--send-log-path", type=str, default=None,
        help="write send history as JSON to this path when streaming finishes"
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
