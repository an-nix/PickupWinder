#!/usr/bin/env python3
"""Simple SPI status poller for PickupWinder.

Usage examples:
  python src/rpi/tools/read_last_executed.py --device-path /dev/spidev0.0
  python src/rpi/tools/read_last_executed.py --bus 0 --dev 0

Prints a timestamped line with `last_executed_sequence` and other helpful fields.
"""
from __future__ import annotations

import argparse
import time
from datetime import datetime

from transport import Esp32SpiTransport


def fmt(ts: float) -> str:
    return datetime.fromtimestamp(ts).strftime("%H:%M:%S.%f")[:-3]


def main() -> int:
    p = argparse.ArgumentParser(description="Poll ESP32 status and show last_executed_sequence")
    group = p.add_mutually_exclusive_group()
    group.add_argument("--device-path", help="Full SPI device path (e.g. /dev/spidev0.0)")
    group.add_argument("--bus", type=int, help="SPI bus number (e.g. 0)")
    p.add_argument("--dev", type=int, help="SPI device number (use with --bus)")
    p.add_argument("--interval", type=float, default=0.25, help="Poll interval in seconds")
    args = p.parse_args()

    if args.device_path is None and args.bus is None:
        device_path = "/dev/spidev0.0"
        spi = Esp32SpiTransport(device_path=device_path)
    elif args.device_path is not None:
        spi = Esp32SpiTransport(device_path=args.device_path)
    else:
        if args.dev is None:
            raise SystemExit("--dev is required when using --bus")
        spi = Esp32SpiTransport(bus=args.bus, device=args.dev)

    try:
        print("Connected to SPI; pressing Ctrl-C to exit")
        while True:
            status = spi.get_status()
            now = time.time()
            le = int(getattr(status, "last_executed_sequence", -1))
            qfree = getattr(status, "queue_free_slots", ())
            rfree = getattr(status, "ring_free_slots", ())
            underrun = getattr(status, "underrun_count", ())
            last_rx_seq = getattr(status, "last_rx_sequence", -1)
            last_result = getattr(status, "last_result", -1)
            enabled = getattr(status, "enabled_mask", 0)
            running = getattr(status, "running_mask", 0)
            lateral = getattr(status, "lateral_endstop_state", 0xFF)
            planner_free = getattr(status, "planner_queue_free", -1)

            print(
                f"[{fmt(now)}] last_executed={le} last_rx_seq={last_rx_seq} result=0x{last_result:02X} "
                f"queue_free={list(qfree)} ring_free={list(rfree)} underrun={list(underrun)} "
                f"planner_free={planner_free} enabled=0x{enabled:02X} running=0x{running:02X} lateral=0x{lateral:02X}"
            )
            time.sleep(max(0.01, args.interval))
    except KeyboardInterrupt:
        print("\nExiting")
        return 0
    finally:
        try:
            spi.close()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
