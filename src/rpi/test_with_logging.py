#!/usr/bin/env python3
"""
Test script pour vérifier le streaming avec logs détaillés.
"""

import json
import logging
import socket
import sys
import time
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Add src/rpi to path
sys.path.insert(0, str(Path(__file__).parent))

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
    logger.info("Starting WinderApp with logging enabled...")
    
    config = AppConfiguration()
    app = WinderApp(config)
    
    try:
        app.start()
        logger.info("✓ WinderApp started")
    except Exception as e:
        logger.error(f"Failed to start app: {e}", exc_info=True)
        return 1

    socket_path = Path(config.rpc_socket_path)
    handler = AppRpcHandler(app)
    server = UnixJsonRpcServer(str(socket_path), handler)
    server.start()

    logger.info(f"✓ JSON-RPC server started at {socket_path}")
    
    # Give server time to stabilize
    time.sleep(0.2)

    try:
        print("\n" + "=" * 70)
        print("TEST: Spindle motion with detailed logging")
        print("=" * 70)

        # Send motion command
        print("\nSending spindle.run command...")
        response = send_jsonrpc_request(
            str(socket_path),
            "winder.spindle.run",
            {"duration_s": 5.0, "rpm": 100.0},
            req_id=1,
        )
        
        result = response.get("result", {})
        session_id = result.get("details", {}).get("session_id")
        
        print(f"✓ Handler returned with session_id={session_id}")
        print(f"  Status: {result.get('details', {}).get('status')}")
        
        # Poll session status
        print("\nPolling session status...")
        for i in range(15):
            time.sleep(0.5)
            status_resp = send_jsonrpc_request(
                str(socket_path),
                "winder.session.status",
                {"session_id": session_id},
                req_id=100 + i,
            )
            
            status_result = status_resp.get("result", {})
            status = status_result.get("status")
            blocks = status_result.get("block_count", 0)
            error = status_result.get("error")
            
            elapsed = (i + 1) * 0.5
            print(f"[{elapsed:.1f}s] Status={status}, Blocks={blocks}, Error={error}")
            
            if status in ("completed", "failed"):
                if status == "failed":
                    print(f"\n❌ STREAMING FAILED: {error}")
                    return 1
                else:
                    print(f"\n✅ STREAMING COMPLETED")
                    return 0
        
        print(f"\n❓ Session still running after 7.5s (expected ~5s)")
        print("Check logs above for details")
        
        return 1

    finally:
        server.stop()
        app.stop()
        logger.info("Shutdown complete")


if __name__ == "__main__":
    sys.exit(main())
