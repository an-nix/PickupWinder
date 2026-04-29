from __future__ import annotations

import argparse
import json
import logging
import sys
import threading
from pathlib import Path
from typing import Any



import tornado.ioloop
import tornado.web
import tornado.websocket

try:
    from .handlers import (
        JsonRpcHttpHandler,
        JsonRpcWebSocketHandler,
        OpenApiHandler,
        ReDocHandler,
        make_application,
    )
    from .rpc import UnixJsonRpcClient
except ImportError:
    from handlers import (
        JsonRpcHttpHandler,
        JsonRpcWebSocketHandler,
        OpenApiHandler,
        ReDocHandler,
        make_application,
    )
    from rpc import UnixJsonRpcClient

logger = logging.getLogger(__name__)


class WebRpcServer:
    def __init__(
        self,
        rpc_client: UnixJsonRpcClient,
        host: str = "0.0.0.0",
        port: int = 8080,
        supported_methods: list[str] | None = None,
    ) -> None:
        self._rpc_client = rpc_client
        self._host = host
        self._port = port
        self._app = make_application(self._rpc_client, supported_methods)
        self._ioloop = tornado.ioloop.IOLoop.current()
        self._stop_event = threading.Event()

    def start(self) -> None:
        self._rpc_client.set_notification_callback(self._broadcast_notification)
        self._rpc_client.connect()
        self._app.listen(self._port, address=self._host)
        logger.info("Wendy web JSON-RPC server listening on http://%s:%d", self._host, self._port)
        self._ioloop.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._ioloop.add_callback(self._ioloop.stop)
        self._rpc_client.close()

    def _broadcast_notification(self, notification: dict[str, Any]) -> None:
        message = json.dumps(notification)
        for client in tuple(self._app.ws_clients):
            try:
                client.write_message(message)
            except tornado.websocket.WebSocketClosedError:
                self._app.ws_clients.discard(client)


def run(
    rpc_client: UnixJsonRpcClient,
    host: str = "0.0.0.0",
    port: int = 8080,
    supported_methods: list[str] | None = None,
) -> WebRpcServer:
    """Create and start the Wendy web JSON-RPC gateway."""
    server = WebRpcServer(
        rpc_client=rpc_client,
        host=host,
        port=port,
        supported_methods=supported_methods,
    )
    server.start()
    return server


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    parser = argparse.ArgumentParser(description="Wendy HTTP/WebSocket JSON-RPC gateway")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--backend-socket", default="/tmp/winding.sock")
    parser.add_argument(
        "--supported-method",
        action="append",
        help="Optional JSON-RPC method name exposed by the backend",
    )
    args = parser.parse_args()

    rpc_client = UnixJsonRpcClient(socket_path=args.backend_socket)
    run(
        rpc_client=rpc_client,
        host=args.host,
        port=args.port,
        supported_methods=args.supported_method,
    )


if __name__ == "__main__":
    main()
