from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

# Add src/rpi to path so the local jsonrpc client package can be imported.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from jsonrpc.client import UnixJsonRpcClient


def send_jsonrpc_request(socket_path: str, method: str, params: dict | None = None, request_id: int = 1) -> dict:
    client = UnixJsonRpcClient(socket_path, timeout_s=10.0)
    return client.call(method, params=params, request_id=request_id)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send a winding.flush_until request to the JSON-RPC server."
    )
    parser.add_argument(
        "--socket",
        default="/tmp/winding.sock",
        help="Path to the JSON-RPC socket.",
    )
    parser.add_argument(
        "sequence",
        type=int,
        help="Motion sequence number to flush until (0-65535).",
    )
    args = parser.parse_args()

    if args.sequence < 0 or args.sequence > 0xFFFF:
        print("Error: sequence must be between 0 and 65535.")
        return 1

    print(f"Connecting to JSON-RPC server at {args.socket}")
    print(f"Requesting flush until motion sequence {args.sequence}")

    try:
        response = send_jsonrpc_request(
            args.socket,
            "winding.flush_until",
            {"sequence": args.sequence},
            request_id=1,
        )
    except Exception as exc:
        print(f"RPC request failed: {exc}")
        return 1

    print("Response:")
    print(json.dumps(response, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
