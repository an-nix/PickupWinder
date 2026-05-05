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


def send_jsonrpc_request(socket_path: str, method: str, params: dict | None = None, request_id: int = 1) -> dict:
    client = UnixJsonRpcClient(socket_path, timeout_s=10.0)
    result = client.call(method, params=params, request_id=request_id)
    return result


def parse_axis_target(value: str) -> dict[str, Any]:
    if ":" not in value:
        raise argparse.ArgumentTypeError(
            "axis target must be in the form axis_id:rpm"
        )

    axis_id_str, rpm_str = value.split(":", 1)
    try:
        axis_id = int(axis_id_str)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid axis_id: {axis_id_str}") from exc

    try:
        rpm = float(rpm_str)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid rpm: {rpm_str}") from exc

    return {"axis_id": axis_id, "rpm": rpm}


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send a run_axis request to the winding JSON-RPC server."
    )
    parser.add_argument(
        "--socket",
        default="/tmp/winding.sock",
        help="Path to the JSON-RPC socket.",
    )
    parser.add_argument(
        "duration_s",
        type=float,
        help="Duration for the ramp move in seconds.",
    )
    parser.add_argument(
        "targets",
        nargs="+",
        type=parse_axis_target,
        help="Axis target pairs in the form axis_id:rpm. Example: 0:500 1:100",
    )
    args = parser.parse_args()

    if len(args.targets) > 2:
        print("Error: at most two axis targets are supported.")
        return 1

    print(f"Connecting to JSON-RPC server at {args.socket}")
    print(f"Starting run_axis for duration={args.duration_s}s targets={args.targets}")

    try:
        response = send_jsonrpc_request(
            args.socket,
            "winding.run_axis",
            {
                "duration_s": args.duration_s,
                "targets": args.targets,
            },
            request_id=1,
        )
    except Exception as exc:
        print(f"Failed to send run_axis command: {exc}")
        return 1

    print("Command response:", json.dumps(response, indent=2))
    print("Polling controller status until completion...")

    start_time = time.time()
    request_id = 2
    while True:
        try:
            status = send_jsonrpc_request(args.socket, "winding.status", None, request_id=request_id)
        except Exception as exc:
            print(f"Failed to read controller status: {exc}")
            return 1

        print(f"Status: {json.dumps(status, indent=2)}")
        request_id += 1

        move_queue = status.get("move_queue")
        if move_queue is not None:
            current_move = move_queue.get("current_move")
            pending_moves = move_queue.get("pending_moves", [])
            if current_move is None and not pending_moves:
                print("Operation finished")
                break
        else:
            engine_state = status.get("shared_state", {}).get("engine_state")
            if engine_state in ("IDLE", "FAULT"):
                print("Operation finished")
                break

        if time.time() - start_time > args.duration_s + 10.0:
            print("Timeout waiting for operation completion")
            return 1

        time.sleep(1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
