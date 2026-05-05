from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from transport.spi_transport import Esp32SpiTransport
from jsonrpc.client import UnixJsonRpcClient


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Debug SPI motor enable / jog behavior for PickupWinder"
    )
    parser.add_argument("--spi-device", default="/dev/spidev0.0")
    parser.add_argument("--bus", type=int, default=None)
    parser.add_argument("--device", type=int, default=None)
    parser.add_argument("--speed-hz", type=int, default=1_000_000)
    parser.add_argument("--poll-interval", type=float, default=0.1)
    parser.add_argument("--poll-duration", type=float, default=10.0)
    parser.add_argument("--enable-axis", type=int, default=None,
                        help="Enable the given axis before polling")
    parser.add_argument("--disable-axis", type=int, default=None,
                        help="Disable the given axis before polling")
    parser.add_argument("--jog-steps", type=int, default=None,
                        help="Send a short jog via JSON-RPC")
    parser.add_argument("--jog-rpm", type=float, default=100.0,
                        help="RPM for the jog command")
    parser.add_argument("--reverse", action="store_true",
                        help="Send the jog in reverse direction")
    parser.add_argument("--rpc-socket", default="/tmp/winding.sock",
                        help="JSON-RPC socket path")
    parser.add_argument("--log-path", default=None,
                        help="Optional path to write status log as JSON lines")
    return parser.parse_args()


def build_transport(args: argparse.Namespace) -> Esp32SpiTransport:
    if args.bus is not None and args.device is not None:
        return Esp32SpiTransport(bus=args.bus, device=args.device, speed_hz=args.spi_speed_hz, mode=1)
    return Esp32SpiTransport(device_path=args.spi_device, speed_hz=args.spi_speed_hz, mode=1)


def send_enable(transport: Esp32SpiTransport, axis_id: int, enable: bool, poll_interval: float) -> None:
    print(f"Sending {'ENABLE' if enable else 'DISABLE'} for axis {axis_id}...")
    sequence, status = transport.set_axis_enabled_request(axis_id, enable)
    status = transport.wait_for_request_result(sequence, poll_interval_s=poll_interval)
    print(f"  result={status.last_result} enabled_mask=0x{status.enabled_mask:02X} running_mask=0x{status.running_mask:02X}")


def send_jog(socket_path: str, axis_id: int, steps: int, rpm: float, reverse: bool) -> dict:
    client = UnixJsonRpcClient(socket_path, timeout_s=10.0)
    print(f"Sending jog via JSON-RPC {socket_path}: axis={axis_id} steps={steps} rpm={rpm} reverse={reverse}")
    return client.call(
        "winding.jog",
        params={"axis_id": axis_id, "steps": steps, "rpm": rpm, "reverse": reverse},
        request_id=1,
    )


def format_status(status) -> str:
    return (
        f"enabled_mask=0x{status.enabled_mask:02X} "
        f"running_mask=0x{status.running_mask:02X} "
        f"queue_free={list(status.queue_free_slots)} "
        f"ring_free={list(status.ring_free_slots)} "
        f"underrun={list(status.underrun_count)} "
        f"last_result=0x{status.last_result:02X} "
        f"last_rx_type=0x{status.last_rx_type:02X} "
        f"last_executed={status.last_executed_sequence}"
    )


def poll_status(transport: Esp32SpiTransport, interval: float, duration: float, log_path: str | None) -> None:
    deadline = time.time() + duration
    lines = []
    try:
        while time.time() < deadline:
            status = transport.get_status()
            timestamp = time.time()
            line = {
                "timestamp": timestamp,
                "enabled_mask": status.enabled_mask,
                "running_mask": status.running_mask,
                "queue_free_slots": list(status.queue_free_slots),
                "ring_free_slots": list(status.ring_free_slots),
                "underrun_count": list(status.underrun_count),
                "last_result": status.last_result,
                "last_rx_type": status.last_rx_type,
                "last_executed_sequence": status.last_executed_sequence,
            }
            text = f"{timestamp:.3f} {format_status(status)}"
            print(text)
            if log_path is not None:
                lines.append(line)
            time.sleep(interval)
    except KeyboardInterrupt:
        print("Polling interrupted by user")
    finally:
        if log_path is not None:
            with open(log_path, "w", encoding="utf-8") as handle:
                for entry in lines:
                    handle.write(json.dumps(entry) + "\n")
            print(f"Status log written to {log_path}")


def main() -> int:
    args = parse_args()
    transport = build_transport(args)
    with transport:
        if args.enable_axis is not None:
            send_enable(transport, args.enable_axis, True, args.poll_interval)
        if args.disable_axis is not None:
            send_enable(transport, args.disable_axis, False, args.poll_interval)
        if args.jog_steps is not None:
            try:
                response = send_jog(args.rpc_socket, 0 if args.enable_axis is None else args.enable_axis, args.jog_steps, args.jog_rpm, args.reverse)
                print("Jog response:", json.dumps(response, indent=2))
            except Exception as exc:
                print("JSON-RPC jog failed:", exc)
        print(f"Polling SPI status for {args.poll_duration:.1f} seconds...")
        poll_status(transport, args.poll_interval, args.poll_duration, args.log_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
