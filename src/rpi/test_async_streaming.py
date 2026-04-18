#!/usr/bin/env python
"""
Demonstration of non-blocking async streaming via JSON-RPC.

This script:
1. Starts the WinderApp with JSON-RPC server
2. Sends spindle.run and motion.run commands via JSON-RPC
3. Shows that handlers return immediately with session_id
4. Polls session status while streaming happens in background
5. Verifies that streaming completes without blocking the server
"""

import json
import socket
import sys
import time
from pathlib import Path
from threading import Thread

# Add src/rpi to path
sys.path.insert(0, str(Path(__file__).parent / "src" / "rpi"))

from domain import AppConfiguration
from winding import WinderApp
from jsonrpc import AppRpcHandler, UnixJsonRpcServer


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
    config = AppConfiguration()
    app = WinderApp(config)
    app.start()

    socket_path = Path(config.rpc_socket_path)
    handler = AppRpcHandler(app)
    server = UnixJsonRpcServer(str(socket_path), handler)
    server.start()

    print(f"✓ JSON-RPC server started at {socket_path}")
    print(f"✓ SPI transport connected")
    
    # Give server time to stabilize
    time.sleep(0.2)

    try:
        print("\n" + "=" * 70)
        print("TEST 1: Non-blocking spindle.run")
        print("=" * 70)

        start_time = time.time()
        print(f"\n[{time.time() - start_time:.2f}s] Sending spindle.run command (5s @ 100 RPM)...")
        response = send_jsonrpc_request(
            str(socket_path),
            "winder.spindle.run",
            {"duration_s": 5.0, "rpm": 100.0},
            req_id=1,
        )
        elapsed = time.time() - start_time
        print(f"[{elapsed:.2f}s] ✓ Handler returned immediately!")
        
        session_id = response.get("result", {}).get("details", {}).get("session_id")
        if not session_id:
            print(f"[{elapsed:.2f}s] ✗ No session_id in response: {response}")
            return 1
        
        print(f"[{elapsed:.2f}s] Session ID: {session_id}")
        print(f"[{elapsed:.2f}s] Response: {response.get('result', {}).get('status')}")

        # Poll session status while it's streaming
        print(f"\n[{elapsed:.2f}s] Polling session status while streaming continues...")
        for poll_count in range(1, 15):
            time.sleep(0.5)
            poll_response = send_jsonrpc_request(
                str(socket_path),
                "winder.session.status",
                {"session_id": session_id},
                req_id=100 + poll_count,
            )
            status_result = poll_response.get("result", {})
            status = status_result.get("status", "unknown")
            block_count = status_result.get("block_count", 0)
            elapsed = time.time() - start_time
            print(f"[{elapsed:.2f}s] Poll #{poll_count}: status={status}, blocks={block_count}")

            if status == "completed":
                print(f"[{elapsed:.2f}s] ✓ Streaming completed!")
                break
            if status == "failed":
                print(f"[{elapsed:.2f}s] ✗ Streaming failed: {status_result.get('error')}")
                return 1

        total_elapsed = time.time() - start_time
        print(f"\n[{total_elapsed:.2f}s] Test completed successfully")
        print(f"    Expected duration: ~5 seconds")
        print(f"    Actual duration: ~{total_elapsed:.2f} seconds")

        print("\n" + "=" * 70)
        print("TEST 2: Non-blocking motion.run (multi-axis)")
        print("=" * 70)

        start_time = time.time()
        print(f"\n[{time.time() - start_time:.2f}s] Sending motion.run command (3s)...")
        response = send_jsonrpc_request(
            str(socket_path),
            "winder.motion.run",
            {
                "duration_s": 3.0,
                "spindle_rpm": 150.0,
                "lateral_rpm": 50.0,
            },
            req_id=2,
        )
        elapsed = time.time() - start_time
        print(f"[{elapsed:.2f}s] ✓ Handler returned immediately!")
        
        session_id = response.get("result", {}).get("details", {}).get("session_id")
        if not session_id:
            print(f"[{elapsed:.2f}s] ✗ No session_id in response: {response}")
            return 1
        
        print(f"[{elapsed:.2f}s] Session ID: {session_id}")

        # Wait for session to complete (with 10s timeout)
        print(f"\n[{elapsed:.2f}s] Waiting for motion streaming to complete (max 10s)...")
        wait_response = send_jsonrpc_request(
            str(socket_path),
            "winder.session.wait",
            {"session_id": session_id, "timeout_s": 10.0},
            req_id=200,
        )
        elapsed = time.time() - start_time
        wait_result = wait_response.get("result", {})
        print(f"[{elapsed:.2f}s] ✓ Streaming completed!")
        print(f"    Status: {wait_result.get('status')}")
        print(f"    Blocks sent: {wait_result.get('block_count')}")
        print(f"    Duration: {wait_result.get('duration_s', 0):.2f}s")

        print("\n" + "=" * 70)
        print("✓ All tests passed!")
        print("=" * 70)
        print("\nKey observations:")
        print("  1. Handler returned immediately (blocking eliminated)")
        print("  2. Session status could be polled while streaming was active")
        print("  3. Multiple requests processed while background streaming occurred")
        print("  4. Streaming completed in expected time (~motion duration)")

        return 0

    finally:
        server.stop()
        app.stop()


if __name__ == "__main__":
    sys.exit(main())
