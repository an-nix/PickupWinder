from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from jsonrpc.client import UnixJsonRpcClient


def call_rpc(
    client: UnixJsonRpcClient,
    method: str,
    params: dict[str, Any] | None = None,
    *,
    request_id: int,
) -> Any:
    return client.call(method, params=params, request_id=request_id)


def wait_until_idle(
    client: UnixJsonRpcClient,
    *,
    timeout_s: float,
    poll_interval_s: float,
    request_id_start: int,
) -> tuple[dict[str, Any], int]:
    deadline = time.time() + timeout_s
    request_id = request_id_start

    while time.time() < deadline:
        status = call_rpc(client, "winding.status", None, request_id=request_id)
        request_id += 1

        shared_state = status.get("shared_state", {})
        move_queue = status.get("move_queue", {})
        engine_state = shared_state.get("engine_state")
        current_move = move_queue.get("current_move")
        pending_moves = move_queue.get("pending_moves", [])

        if engine_state == "FAULT":
            fault = shared_state.get("fault_message", "unknown fault")
            raise RuntimeError(f"Controller entered FAULT: {fault}")

        if engine_state == "IDLE" and current_move is None and not pending_moves:
            return status, request_id

        time.sleep(poll_interval_s)

    raise TimeoutError(f"Timeout waiting for controller idle after {timeout_s:.1f}s")


def read_axis_state(client: UnixJsonRpcClient, axis_id: int, request_id: int) -> dict[str, Any]:
    return call_rpc(
        client,
        "winding.axis_state",
        {"axis_id": axis_id},
        request_id=request_id,
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Home lateral if needed, then oscillate between two absolute mm positions."
    )
    parser.add_argument("--socket", default="/tmp/winding.sock", help="Path to the JSON-RPC socket")
    parser.add_argument("--axis-id", type=int, default=1, help="Lateral axis id")
    parser.add_argument("--position-a-mm", type=float, default=0.0, help="First absolute position in mm from home zero")
    parser.add_argument("--position-b-mm", type=float, default=10.0, help="Second absolute position in mm from home zero")
    parser.add_argument("--rpm", type=float, default=60.0, help="Jog RPM for the lateral moves")
    parser.add_argument("--cycles", type=int, default=3, help="Number of A→B→A cycles")
    parser.add_argument("--poll-interval", type=float, default=0.1, help="Status polling interval in seconds")
    parser.add_argument("--timeout", type=float, default=30.0, help="Timeout per move in seconds")
    args = parser.parse_args()

    if args.cycles < 1:
        print("cycles must be >= 1")
        return 1
    if args.rpm <= 0.0:
        print("rpm must be positive")
        return 1

    client = UnixJsonRpcClient(args.socket, timeout_s=max(args.timeout, 5.0))
    request_id = 1

    try:
        axis_state = read_axis_state(client, args.axis_id, request_id)
        request_id += 1
        print("Initial axis state:", json.dumps(axis_state, indent=2))

        if not axis_state.get("homed", False):
            print("Lateral axis is not homed. Starting homing...")
            result = call_rpc(client, "winding.home_lateral", None, request_id=request_id)
            request_id += 1
            print("Homing result:", json.dumps(result, indent=2))

        for cycle_index in range(args.cycles):
            for target_mm in (args.position_b_mm, args.position_a_mm):
                print(
                    f"Cycle {cycle_index + 1}/{args.cycles}: moving lateral axis to {target_mm:.3f} mm"
                )
                result = call_rpc(
                    client,
                    "winding.move_lateral_mm",
                    {"position_mm": target_mm, "rpm": args.rpm},
                    request_id=request_id,
                )
                request_id += 1
                print("Move command:", json.dumps(result, indent=2))

                status, request_id = wait_until_idle(
                    client,
                    timeout_s=args.timeout,
                    poll_interval_s=args.poll_interval,
                    request_id_start=request_id,
                )
                axis_states = status.get("shared_state", {}).get("axis_states", {})
                axis_state = axis_states.get(args.axis_id)
                if axis_state is None:
                    axis_state = axis_states.get(str(args.axis_id))
                if axis_state is None:
                    axis_state = read_axis_state(client, args.axis_id, request_id)
                    request_id += 1
                print("Axis state:", json.dumps(axis_state, indent=2))

    except Exception as exc:
        print(f"Simulation failed: {exc}")
        return 1
    finally:
        client.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())