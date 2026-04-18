

import sys
import time
from pathlib import Path

from domain import AppConfiguration
from jsonrpc import AppRpcHandler, UnixJsonRpcServer


def main() -> int:
    config = AppConfiguration()

    socket_path = Path(config.rpc_socket_path)
    server = UnixJsonRpcServer(str(socket_path), AppRpcHandler(config))
    server.start()

    print(f"JSON-RPC server listening on {socket_path}")
    print("Press Ctrl-C to stop.")

    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("Shutting down JSON-RPC server...")
    finally:
        server.stop()

    return 0




if __name__ == "__main__":
    sys.exit(main())