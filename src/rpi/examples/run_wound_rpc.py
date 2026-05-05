```python
from __future__ import annotations

import json
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

    # Default parameters for the Electronic Gearing winding demonstrator
    target_rpm = 1000.0
    if len(sys.argv) > 1:
        try:
            target_rpm = float(sys.argv[1])
        except ValueError:
            print(f"Error: invalid RPM '{sys.argv[1]}', must be a number")
            return 1

    print(f"Connecting to JSON-RPC server at {socket_path}")
    print(f"Starting synchronized winding (Electronic Gearing) at {target_rpm:.1f} RPM")

    # Command details outlining Spindle as master, Traverse as slave + Scattering
    params = {
        "spindle_axis_id": 0,
        "traverse_axis_id": 1,
        "target_rpm": target_rpm,
        "accel_s": 2.0,
        "cruise_s": 10.0,
        "decel_s": 2.0,
        "bobbin_width_mm": 15.0,            # 15mm wide coil
        "turns_per_mm": 10.0,               # Slave translation ratio
        "scatter_amplitude_mm": 0.5,        # +/- 0.5mm randomized shifting
        "scatter_damping_margin_mm": 2.0,   # smoothly decay scatter 2mm from the edges
        "spindle_reverse": False,
        "traverse_reverse": False
    }

    try:
        response = send_jsonrpc_request(
            socket_path,
            "winding.wound_run",
            params,
            request_id=1,
        )
    except Exception as exc:
        print(f"Failed to send winding command: {exc}")
        return 1

    print("Command response:", json.dumps(response, indent=2))

    print("Polling controller status until completion...")
    start_time = time.time()
    request_id = 2
    
    # We estimate the expected duration
    expected_duration_s = params["accel_s"] + params["cruise_s"] + params["decel_s"]

    while True:
        try:
            status = send_jsonrpc_request(socket_path, "winding.status", None, request_id=request_id)
        except Exception as exc:
            print(f"Failed to read controller status: {exc}")
            return 1

        print(f"\nStatus at {time.time() - start_time:.1f}s: {json.dumps(status, indent=2)}")
        request_id += 1

        move_queue = status.get("move_queue")
        if move_queue is not None:
            current_move = move_queue.get("current_move")
            pending_moves = move_queue.get("pending_moves", [])
            # If nothing is running and this isn't the very first split-second
            if current_move is None and not pending_moves and (time.time() - start_time) > 1.0:
                print("Operation finished successfully!")
                break
        else:
            engine_state = status.get("shared_state", {}).get("engine_state")
            if engine_state in ("IDLE", "FAULT") and (time.time() - start_time) > 1.0:
                print(f"Operation finished with state {engine_state}")
                break

        if time.time() - start_time > expected_duration_s + 10.0:
            print("Timeout waiting for operation completion")
            return 1

        time.sleep(1.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```