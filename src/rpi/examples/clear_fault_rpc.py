from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from jsonrpc.client import UnixJsonRpcClient


def send_jsonrpc_request(socket_path: str, method: str, params: Any | None = None, request_id: int = 1) -> Any:
    client = UnixJsonRpcClient(socket_path, timeout_s=10.0)
    try:
        return client.call(method, params=params, request_id=request_id)
    finally:
        client.close()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Send a winding.clear_fault JSON-RPC request to the winding controller."
    )
    parser.add_argument(
        "--socket",
        default="/tmp/winding.sock",
        help="Path to the JSON-RPC socket.",
    )
    args = parser.parse_args()

    try:
        result = send_jsonrpc_request(args.socket, "winding.clear_fault", None, request_id=1)
    except Exception as exc:
        print(f"Failed to clear fault: {exc}")
        return 1

    print("clear_fault response:", json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
