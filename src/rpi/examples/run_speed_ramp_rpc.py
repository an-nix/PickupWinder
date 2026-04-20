from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

# Add src/rpi to path so the local jsonrpc client package can be imported.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from jsonrpc.client import UnixJsonRpcClient


def send_jsonrpc_request(socket_path: str, method: str, params: dict | None = None, request_id: int = 1) -> dict[str, Any]:
    client = UnixJsonRpcClient(socket_path, timeout_s=10.0)
    return client.call(method, params=params, request_id=request_id)


def wait_for_completion(socket_path: str, timeout_s: float = 120.0, poll_interval_s: float = 1.0) -> bool:
    start = time.time()
    request_id = 100
    while True:
        status = send_jsonrpc_request(socket_path, "winding.status", None, request_id=request_id)
        request_id += 1
        move_queue = status.get("move_queue", {})
        current_move = move_queue.get("current_move")
        pending_moves = move_queue.get("pending_moves", [])
        if current_move is None and not pending_moves:
            return True
        if time.time() - start > timeout_s:
            print("Timeout waiting for operation completion")
            return False
        time.sleep(poll_interval_s)


def run_sequence(socket_path: str, axis_id: int, profiles: list[tuple[int, float]]) -> int:
    request_id = 1
    for rpm, duration_s in profiles:
        print(f"\n--- Starting ramp: axis={axis_id} rpm={rpm} duration={duration_s}s ---")
        try:
            response = send_jsonrpc_request(
                socket_path,
                "winding.run_axis",
                {
                    "duration_s": duration_s,
                    "targets": [{"axis_id": axis_id, "rpm": rpm}],
                },
                request_id=request_id,
            )
        except Exception as exc:
            print(f"Failed to send run_axis command: {exc}")
            return 1

        print("Command response:", json.dumps(response, indent=2))
        request_id += 1

        completed = wait_for_completion(socket_path, timeout_s=duration_s + 30.0)
        if not completed:
            print("Run did not complete cleanly, aborting sequence.")
            return 1

        print(f"Ramp {rpm} RPM finished.")
        time.sleep(2.0)

    print("All ramp stages completed.")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Execute a speed ramp sequence via winding.run_axis JSON-RPC."
    )
    parser.add_argument(
        "--socket",
        default="/tmp/winding.sock",
        help="Path to the JSON-RPC socket.",
    )
    parser.add_argument(
        "--axis-id",
        type=int,
        default=0,
        help="Axis ID to drive.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the planned sequence without sending commands.",
    )
    args = parser.parse_args()

    sequence = [
        (100, 30.0),
        (90,10),
        (10,10), 
        (500, 60.0),
        (1000, 60.0),
        (1500, 10.0),
    ]

    print(f"Using JSON-RPC socket: {args.socket}")
    print(f"Running axis {args.axis_id} speed profile:")
    for rpm, duration_s in sequence:
        print(f"  - {rpm} RPM for {duration_s}s")

    if args.dry_run:
        return 0

    return run_sequence(args.socket, args.axis_id, sequence)


if __name__ == "__main__":
    raise SystemExit(main())
