from __future__ import annotations

import json
import time
from typing import Any
from urllib.parse import unquote

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

    def write_json(self, payload: Any, *, status: int = 200) -> None:
        self.set_header("Content-Type", "application/json")
        self.set_status(status)
        self.write(json.dumps(payload))

    def rpc_result(
        self,
        method: str,
        *,
        params: Any | None = None,
        success_status: int = 200,
    ) -> bool:
        request_id = int(time.time() * 1000)
        request_payload = make_request(method, params=params, request_id=request_id)
        response = self.application.rpc_client.send_raw(request_payload)
        if response is None:
            self.set_status(204)
            return False
        if "error" in response:
            self.write_json(response, status=502)
            return False
        self.write_json(response.get("result", response), status=success_status)
        return True

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


class ProgramCollectionHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    @staticmethod
    def _extract_program_body(body: dict[str, Any]) -> dict[str, Any]:
        if "program" in body and isinstance(body["program"], dict):
            return body["program"]
        return {
            key: value
            for key, value in body.items()
            if key not in {"load", "program_id", "id", "changes"}
        }

    def get(self) -> None:
        include_content = self.get_query_argument("include_content", default="0")
        self.rpc_result(
            "program.list",
            params={"include_content": include_content in {"1", "true", "True"}},
        )

    def post(self) -> None:
        try:
            body = json.loads(self.request.body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.write_json({"error": f"Invalid JSON: {exc}"}, status=400)
            return

        if not isinstance(body, dict):
            self.write_json({"error": "Request body must be a JSON object"}, status=400)
            return

        program = self._extract_program_body(body)
        params = {
            "program": program,
            "program_id": body.get("program_id") or body.get("id"),
            "load": bool(body.get("load", False)),
        }
        self.rpc_result("program.save", params=params, success_status=201)


class ProgramItemHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    def get(self, program_id: str) -> None:
        self.rpc_result("program.get", params={"program_id": unquote(program_id)})

    def put(self, program_id: str) -> None:
        try:
            body = json.loads(self.request.body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.write_json({"error": f"Invalid JSON: {exc}"}, status=400)
            return

        if not isinstance(body, dict):
            self.write_json({"error": "Request body must be a JSON object"}, status=400)
            return

        if "changes" in body and isinstance(body["changes"], dict):
            changes = body["changes"]
        else:
            changes = ProgramCollectionHandler._extract_program_body(body)
        self.rpc_result(
            "program.update",
            params={
                "program_id": unquote(program_id),
                "changes": changes,
                "load": bool(body.get("load", False)),
            },
        )

    def delete(self, program_id: str) -> None:
        self.rpc_result(
            "program.delete",
            params={"program_id": unquote(program_id)},
        )


class ProgramLoadHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    def post(self, program_id: str) -> None:
        self.rpc_result(
            "program.load",
            params={"program_id": unquote(program_id)},
        )


class ProgramQueueHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    def post(self, program_id: str) -> None:
        self.rpc_result(
            "winding.submit_program",
            params={"program_id": unquote(program_id), "load": True},
            success_status=202,
        )


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


class WindingWoundRunHandler(tornado.web.RequestHandler):
    """POST /wound_run — launch a synchronized two-axis winding run.

    Accepts a JSON body with the same fields as ``winding.wound_run``.
    Required: spindle_axis_id, traverse_axis_id, target_rpm,
              bobbin_width_mm, turns_per_mm.
    Optional: accel_s, cruise_s, decel_s, scatter_amplitude_mm,
              scatter_damping_margin_mm, scatter_freq1, scatter_freq2,
              spindle_reverse, traverse_reverse.
    """

    def post(self) -> None:
        try:
            body = json.loads(self.request.body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.set_status(400)
            self.write(json.dumps({"error": f"Invalid JSON: {exc}"}))
            return

        required = {"spindle_axis_id", "traverse_axis_id", "target_rpm",
                    "bobbin_width_mm", "turns_per_mm"}
        missing = required - body.keys()
        if missing:
            self.set_status(400)
            self.write(json.dumps({"error": f"Missing required fields: {sorted(missing)}"}))
            return

        params: dict = {
            "spindle_axis_id": int(body["spindle_axis_id"]),
            "traverse_axis_id": int(body["traverse_axis_id"]),
            "target_rpm": float(body["target_rpm"]),
            "bobbin_width_mm": float(body["bobbin_width_mm"]),
            "turns_per_mm": float(body["turns_per_mm"]),
        }
        for opt_float in ("accel_s", "cruise_s", "decel_s",
                           "scatter_amplitude_mm", "scatter_damping_margin_mm",
                           "scatter_freq1", "scatter_freq2"):
            if opt_float in body:
                params[opt_float] = float(body[opt_float])
        for opt_bool in ("spindle_reverse", "traverse_reverse"):
            if opt_bool in body:
                params[opt_bool] = bool(body[opt_bool])

        request_id = int(time.time() * 1000)
        request_payload = make_request(
            "winding.wound_run",
            params=params,
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


class WindingStopHandler(tornado.web.RequestHandler):
    """GET /stop[?mode=stop|pause|emergency_stop] — stop or pause motion."""

    def get(self) -> None:
        mode = self.get_query_argument("mode", default="stop")
        request_id = int(time.time() * 1000)
        request_payload = make_request(
            "winding.stop",
            params={"mode": mode},
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
            (r"/api/programs", ProgramCollectionHandler),
            (r"/api/programs/([^/]+)", ProgramItemHandler),
            (r"/api/programs/([^/]+)/load", ProgramLoadHandler),
            (r"/api/programs/([^/]+)/queue", ProgramQueueHandler),
            (r"/run_axis", WindingRunAxisHandler),
            (r"/wound_run", WindingWoundRunHandler),
            (r"/stop", WindingStopHandler),
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
