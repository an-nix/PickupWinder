#!/usr/bin/env python3
"""
Diagnostic script pour vérifier le status d'une session de streaming.
"""

import json
import socket
import sys
import time
from pathlib import Path

def send_jsonrpc_request(socket_path: str, method: str, params: dict | None = None, req_id: int = 1) -> dict:
    """Send a JSON-RPC request and receive response."""
    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        sock.connect(socket_path)
        request = {
            "jsonrpc": "2.0",
            "id": req_id,
            "method": method,
        }
        if params is not None:
            request["params"] = params
        
        sock.sendall((json.dumps(request) + "\n").encode("utf-8"))
        
        # Receive response
        file = sock.makefile(mode="r", encoding="utf-8")
        response_line = file.readline().strip()
        if response_line:
            return json.loads(response_line)
        return {}
    finally:
        sock.close()


def main() -> int:
    socket_path = "/tmp/pickup_winder_rpc.sock"
    
    if len(sys.argv) < 2:
        print("Usage: python diag_session.py <session_id>")
        print("Example: python diag_session.py 1")
        return 1
    
    session_id = int(sys.argv[1])
    
    print(f"Querying session {session_id} status...")
    print(f"Socket: {socket_path}\n")
    
    # Query status
    response = send_jsonrpc_request(
        socket_path,
        "winder.session.status",
        {"session_id": session_id},
        req_id=100,
    )
    
    if "error" in response:
        print(f"❌ Error: {response['error']}")
        return 1
    
    result = response.get("result", {})
    
    print("=" * 70)
    print("SESSION STATUS")
    print("=" * 70)
    print(f"Session ID:    {result.get('session_id')}")
    print(f"Status:        {result.get('status')}")
    print(f"Block Count:   {result.get('block_count')}")
    print(f"Error:         {result.get('error')}")
    print(f"Started at:    {result.get('started_at')}")
    print(f"Completed at:  {result.get('completed_at')}")
    print()
    
    status = result.get('status')
    if status == "queued":
        print("⏳ Status: Waiting to start (queued)")
    elif status == "running":
        print("▶️  Status: Currently streaming")
    elif status == "completed":
        print("✅ Status: Completed successfully")
    elif status == "failed":
        print(f"❌ Status: FAILED")
        print(f"   Error: {result.get('error')}")
        return 1
    else:
        print(f"❓ Unknown status: {status}")
        return 1
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
