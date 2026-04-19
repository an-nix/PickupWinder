from __future__ import annotations

import json
import socket
import sys
import time
from pathlib import Path

# Add src/rpi to path so the local jsonrpc client package can be imported.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from jsonrpc.client import UnixJsonRpcClient


def send_jsonrpc_request(socket_path: str, method: str, params: dict | None = None, request_id: int = 1) -> dict:
    client = UnixJsonRpcClient(socket_path, timeout_s=10.0)
    result = client.call(method, params=params, request_id=request_id)
    return result


def main() -> int:
    socket_path = "/tmp/winding.sock"

    # Parse RPM from command line argument, default to 500 RPM
    rpm = 500.0
    if len(sys.argv) > 1:
        try:
            rpm = float(sys.argv[1])
        except ValueError:
            print(f"Error: invalid RPM '{sys.argv[1]}', must be a number")
            return 1

    duration_s = 30.0
    steps_per_rev = 200 * 32
    steps = int(rpm / 60.0 * steps_per_rev * duration_s)

    print(f"Connecting to JSON-RPC server at {socket_path}")
    print(f"Starting spindle jog at {rpm:.1f} RPM for {duration_s:.1f} seconds ({steps} steps)")

    try:
        response = send_jsonrpc_request(
            socket_path,
            "winding.jog",
            {
                "axis_id": 0,
                "steps": steps,
                "rpm": rpm,
                "reverse": False,
            },
            request_id=1,
        )
    except Exception as exc:
        print(f"Failed to send spindle command: {exc}")
        return 1

    print("Command response:", json.dumps(response, indent=2))

    print("Polling controller status until completion...")
    start_time = time.time()
    request_id = 2
    while True:
        try:
            status = send_jsonrpc_request(socket_path, "winding.status", None, request_id=request_id)
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

        if time.time() - start_time > duration_s + 10.0:
            print("Timeout waiting for operation completion")
            return 1

        time.sleep(1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
