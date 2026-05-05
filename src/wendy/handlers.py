from __future__ import annotations

import json
import time
from typing import Any

import tornado.web
import tornado.websocket

from rpc import (
    JsonRpcError,
    JsonRpcInvalidRequestError,
    JsonRpcParseError,
    UnixJsonRpcClient,
    make_error_response,
    make_response,
    make_request,
    parse_json_rpc,
)


class JsonRpcHandlerMixin:
    def write_jsonrpc_response(self, response: dict[str, Any]) -> None:
        self.set_header("Content-Type", "application/json")
        self.set_status(200)
        self.write(json.dumps(response))

    def error_response(self, request_id: Any | None, error: JsonRpcError) -> dict[str, Any]:
        return json.loads(make_error_response(error, request_id))

    def dispatch_rpc(self, raw_payload: str) -> dict[str, Any] | None:
        request_id = None
        try:
            request = parse_json_rpc(raw_payload)
            request_id = request.get("id")
            response = self.application.rpc_client.send_raw(raw_payload)
            if response is None:
                return None
            return response
        except JsonRpcParseError as exc:
            return self.error_response(request_id, exc)
        except JsonRpcInvalidRequestError as exc:
            return self.error_response(request_id, exc)
        except JsonRpcError as exc:
            return self.error_response(request_id, exc)
        except Exception as exc:
            return self.error_response(request_id, JsonRpcError(-32000, str(exc)))


class JsonRpcHttpHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    def post(self) -> None:
        response = self.dispatch_rpc(self.request.body.decode("utf-8"))
        if response is None:
            self.set_status(204)
            return
        self.write_jsonrpc_response(response)


class JsonRpcWebSocketHandler(tornado.websocket.WebSocketHandler, JsonRpcHandlerMixin):
    def open(self) -> None:
        self.application.ws_clients.add(self)

    def on_message(self, message: str) -> None:
        response = self.dispatch_rpc(message)
        if response is not None:
            self.write_message(response)

    def on_close(self) -> None:
        self.application.ws_clients.discard(self)

    def check_origin(self, origin: str) -> bool:
        return True


class OpenApiHandler(tornado.web.RequestHandler):
    def get(self) -> None:
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps(self.application.openapi_schema))


class WindingRunAxisHandler(tornado.web.RequestHandler):
    def get(self) -> None:
        try:
            axis_id = int(self.get_query_argument("axis_id"))
            rpm = float(self.get_query_argument("rpm"))
            duration_s = float(self.get_query_argument("duration_s"))
            #reverse = bool(self.get_query_argument("reverse", default="0"))
            reverse = self.get_query_argument("reverse", default=False)
        except tornado.web.MissingArgumentError as exc:
            self.set_status(400)
            self.write(json.dumps({"error": str(exc)}))
            return
        except ValueError as exc:
            self.set_status(400)
            self.write(json.dumps({"error": f"Invalid parameter: {exc}"}))
            return
        print(f"Received run_axis command: axis_id={axis_id}, rpm={rpm}, duration_s={duration_s}, reverse={reverse}")
        request_id = int(time.time() * 1000)
        request_payload = make_request(
            "winding.run_axis",
            params={
                "duration_s": duration_s,
                "targets": [
                    {"axis_id": axis_id, "rpm": rpm,"reverse": reverse},
                ],
            },
            request_id=request_id,
        )
        response = self.application.rpc_client.send_raw(request_payload)
        if response is None:
            self.set_status(204)
            return
        if "error" in response:
            self.set_status(502)
            self.write(json.dumps(response))
            return
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps(response.get("result", response)))


class WindingHomeHandler(tornado.web.RequestHandler):
    def get(self) -> None:
        request_id = int(time.time() * 1000)
        request_payload = make_request(
            "winding.home_lateral",
            params=None,
            request_id=request_id,
        )
        response = self.application.rpc_client.send_raw(request_payload)
        if response is None:
            self.set_status(204)
            return
        if "error" in response:
            self.set_status(502)
            self.write(json.dumps(response))
            return
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps(response.get("result", response)))


class WindingStatusHandler(tornado.web.RequestHandler):
    def get(self) -> None:
        request_payload = make_request("winding.status", params=None, request_id=1)
        response = self.application.rpc_client.send_raw(request_payload)
        if response is None:
            self.set_status(204)
            return
        if "error" in response:
            self.set_status(502)
            self.write(json.dumps(response))
            return
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps(response.get("result", response)))


class WindingClearFaultHandler(tornado.web.RequestHandler):
    def get(self) -> None:
        request_payload = make_request("winding.clear_fault", params=None, request_id=1)
        response = self.application.rpc_client.send_raw(request_payload)
        if response is None:
            self.set_status(204)
            return
        if "error" in response:
            self.set_status(502)
            self.write(json.dumps(response))
            return
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps(response.get("result", response)))


class ReDocHandler(tornado.web.RequestHandler):
    def get(self) -> None:
        self.set_header("Content-Type", "text/html")
        self.write(
            """
            <!DOCTYPE html>
            <html>
            <head>
              <title>Wendy JSON-RPC API</title>
              <meta charset="utf-8" />
            </head>
            <body>
              <redoc spec-url='/openapi.json'></redoc>
              <script src='https://cdn.redoc.ly/redoc/latest/bundles/redoc.standalone.js'></script>
            </body>
            </html>
            """
        )


from docs import make_openapi_schema, SwaggerUIHandler


def make_application(
    rpc_client: UnixJsonRpcClient,
    supported_methods: list[str] | None = None,
) -> tornado.web.Application:
    application = tornado.web.Application(
        [
            (r"/rpc", JsonRpcHttpHandler, dict(rpc_client=rpc_client)),
            (r"/ws", JsonRpcWebSocketHandler),
            (r"/run_axis", WindingRunAxisHandler),
            (r"/home", WindingHomeHandler),
            (r"/status", WindingStatusHandler),
            (r"/clear_fault", WindingClearFaultHandler),
            (r"/openapi.json", OpenApiHandler),
            (r"/docs", ReDocHandler),
            (r"/swagger", SwaggerUIHandler),
        ],
        debug=False,
    )
    application.rpc_client = rpc_client
    application.ws_clients = set()
    application.openapi_schema = make_openapi_schema(supported_methods)
    return application
