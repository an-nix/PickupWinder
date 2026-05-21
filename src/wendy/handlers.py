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


_RPC_CLIENT_ERROR_CODES: frozenset[int] = frozenset((-32700, -32600, -32601, -32602))


def _rpc_error_to_http(code: int) -> int:
    """Map a JSON-RPC error code to an HTTP status code.

    Client errors (-32700/-32600/-32601/-32602) → 400/404.
    Server errors (-32000 to -32099) and application errors → 502.
    """
    if code == -32601:
        return 404
    if code in _RPC_CLIENT_ERROR_CODES:
        return 400
    return 502


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
        try:
            response = self.application.rpc_client.send_raw(request_payload)
        except RuntimeError as exc:
            self.write_json({"error": f"RPC transport error: {exc}"}, status=503)
            return False
        if response is None:
            self.set_status(204)
            return False
        if "error" in response:
            _code = response["error"].get("code", -32000) if isinstance(response.get("error"), dict) else -32000
            self.write_json(response, status=_rpc_error_to_http(_code))
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


class WindingRunAxisHandler(tornado.web.RequestHandler):
    def get(self) -> None:
        try:
            axis_id = int(self.get_query_argument("axis_id"))
            rpm = float(self.get_query_argument("rpm"))
            duration_s = float(self.get_query_argument("duration_s"))
            _reverse_raw = self.get_query_argument("reverse", default="0")
            reverse = _reverse_raw.lower() not in {"0", "false", "no", ""}
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


class ProgramRevisionsHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """GET /api/programs/{id}/revisions — list backup revisions."""

    def get(self, program_id: str) -> None:
        self.rpc_result(
            "program.list_revisions",
            params={"program_id": unquote(program_id)},
        )


class ProgramRestoreHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/programs/{id}/restore/{revision} — restore a backup revision."""

    def post(self, program_id: str, revision: str) -> None:
        self.rpc_result(
            "program.restore_revision",
            params={"program_id": unquote(program_id), "revision": int(revision)},
        )


class SessionHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """GET/POST/DELETE/PATCH /api/session — adaptive winding session control."""

    def get(self) -> None:
        self.rpc_result("winding.session_status")

    def post(self) -> None:
        try:
            body = json.loads(self.request.body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.write_json({"error": f"Invalid JSON: {exc}"}, status=400)
            return
        if not isinstance(body, dict):
            self.write_json({"error": "Request body must be a JSON object"}, status=400)
            return
        session = body.get("session", body)
        self.rpc_result("winding.start_session", params={"session": session}, success_status=202)

    def delete(self) -> None:
        mode = self.get_query_argument("mode", default="stop")
        self.rpc_result("winding.stop", params={"mode": mode})

    def patch(self) -> None:
        try:
            body = json.loads(self.request.body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.write_json({"error": f"Invalid JSON: {exc}"}, status=400)
            return
        if not isinstance(body, dict) or not body:
            self.write_json({"error": "Body must be a non-empty JSON object"}, status=400)
            return
        self.rpc_result("winding.update_session", params=body)


class SessionFromProgramHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/session/from-program — start adaptive mode from a stored program."""

    def post(self) -> None:
        body: dict[str, Any] = {}
        if self.request.body:
            try:
                body = json.loads(self.request.body.decode("utf-8"))
            except json.JSONDecodeError as exc:
                self.write_json({"error": f"Invalid JSON: {exc}"}, status=400)
                return
        if not isinstance(body, dict):
            self.write_json({"error": "Request body must be a JSON object"}, status=400)
            return

        params: dict[str, Any] = {}
        if "program_id" in body:
            params["program_id"] = body["program_id"]
        if "program" in body:
            params["program"] = body["program"]
        if "load" in body:
            params["load"] = bool(body["load"])
        if "total_turns" in body:
            params["total_turns"] = float(body["total_turns"])
        if "chunk_time_s" in body:
            params["chunk_time_s"] = float(body["chunk_time_s"])

        self.rpc_result(
            "winding.start_session_from_program",
            params=params,
            success_status=202,
        )


class SessionPauseHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/session/pause"""

    def post(self) -> None:
        body: dict[str, Any] = {}
        if self.request.body:
            try:
                body = json.loads(self.request.body.decode("utf-8"))
            except json.JSONDecodeError:
                pass
        self.rpc_result(
            "winding.pause",
            params={"pause_at_turn": body.get("pause_at_turn")},
        )


class SessionResumeHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/session/resume"""

    def post(self) -> None:
        self.rpc_result("winding.resume_session")


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


class WindingWoundRunHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /wound_run — launch a synchronized two-axis winding run (legacy).

    .. deprecated::
        Use ``POST /api/machine/wound-run`` instead.
    """

    def post(self) -> None:
        try:
            body = json.loads(self.request.body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.write_json({"error": f"Invalid JSON: {exc}"}, status=400)
            return
        if not isinstance(body, dict):
            self.write_json({"error": "Request body must be a JSON object"}, status=400)
            return
        self.rpc_result("winding.wound_run", params=body, success_status=202)


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


# MachineHomeHandler, MachineClearFaultHandler, MachineStopHandler,
# MachineStatusHandler, MachineRunAxisHandler, MachineWoundRunHandler
# live in the /api/machine/ namespace — proper REST verbs, no side-effects on GET.


class MachineStatusHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """GET /api/machine/status — machine state, move-queue depth, axis positions."""

    def get(self) -> None:
        self.rpc_result("winding.status")


class MachineHomeHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/machine/home — start the lateral homing sequence."""

    def post(self) -> None:
        self.rpc_result("winding.home_lateral", success_status=202)


class MachineClearFaultHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/machine/clear-fault — clear the fault state."""

    def post(self) -> None:
        self.rpc_result("winding.clear_fault")


class MachineStopHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/machine/stop — stop or pause motion."""

    def post(self) -> None:
        body: dict[str, Any] = {}
        if self.request.body:
            try:
                body = json.loads(self.request.body.decode("utf-8"))
            except json.JSONDecodeError:
                pass
        mode = body.get("mode", "stop")
        self.rpc_result("winding.stop", params={"mode": mode})


class MachineRunAxisHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/machine/run-axis — run one or two axes for duration_s."""

    def post(self) -> None:
        try:
            body = json.loads(self.request.body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.write_json({"error": f"Invalid JSON: {exc}"}, status=400)
            return
        if not isinstance(body, dict):
            self.write_json({"error": "Request body must be a JSON object"}, status=400)
            return
        self.rpc_result(
            "winding.run_axis",
            params={
                "duration_s": body.get("duration_s"),
                "targets": body.get("targets", []),
            },
            success_status=202,
        )


class MachineWoundRunHandler(tornado.web.RequestHandler, JsonRpcHandlerMixin):
    """POST /api/machine/wound-run — diagnostic/dev synchronized winding."""

    def post(self) -> None:
        try:
            body = json.loads(self.request.body.decode("utf-8"))
        except json.JSONDecodeError as exc:
            self.write_json({"error": f"Invalid JSON: {exc}"}, status=400)
            return
        if not isinstance(body, dict):
            self.write_json({"error": "Request body must be a JSON object"}, status=400)
            return
        self.rpc_result("winding.wound_run", params=body, success_status=202)


from docs import make_openapi_schema, OpenApiHandler, ReDocHandler, SwaggerUIHandler


class WinderApp(tornado.web.Application):
    """Tornado application with typed RPC client and WebSocket registry."""

    def __init__(
        self,
        handlers: list,
        rpc_client: UnixJsonRpcClient,
        openapi_schema: dict,
        **kwargs: Any,
    ) -> None:
        super().__init__(handlers, **kwargs)
        self.rpc_client: UnixJsonRpcClient = rpc_client
        self.ws_clients: set[JsonRpcWebSocketHandler] = set()
        self.openapi_schema: dict = openapi_schema


def make_application(
    rpc_client: UnixJsonRpcClient,
    supported_methods: list[str] | None = None,
) -> WinderApp:
    openapi_schema = make_openapi_schema(supported_methods)
    return WinderApp(
        handlers=[
            (r"/rpc", JsonRpcHttpHandler),
            (r"/ws", JsonRpcWebSocketHandler),
            # REST API — /api/
            (r"/api/programs", ProgramCollectionHandler),
            (r"/api/programs/([^/]+)/revisions", ProgramRevisionsHandler),
            (r"/api/programs/([^/]+)/restore/(\d+)", ProgramRestoreHandler),
            (r"/api/programs/([^/]+)/load", ProgramLoadHandler),
            (r"/api/programs/([^/]+)/queue", ProgramQueueHandler),
            (r"/api/programs/([^/]+)", ProgramItemHandler),
            (r"/api/session/from-program", SessionFromProgramHandler),
            (r"/api/session/pause", SessionPauseHandler),
            (r"/api/session/resume", SessionResumeHandler),
            (r"/api/session", SessionHandler),
            # Machine control — POST for side-effects, GET for status
            (r"/api/machine/status", MachineStatusHandler),
            (r"/api/machine/home", MachineHomeHandler),
            (r"/api/machine/clear-fault", MachineClearFaultHandler),
            (r"/api/machine/stop", MachineStopHandler),
            (r"/api/machine/run-axis", MachineRunAxisHandler),
            (r"/api/machine/wound-run", MachineWoundRunHandler),
            # Legacy routes — kept for backward compatibility
            (r"/run_axis", WindingRunAxisHandler),
            (r"/wound_run", WindingWoundRunHandler),
            (r"/stop", WindingStopHandler),
            (r"/home", WindingHomeHandler),
            (r"/status", WindingStatusHandler),
            (r"/clear_fault", WindingClearFaultHandler),
            # Documentation
            (r"/openapi.json", OpenApiHandler),
            (r"/docs", ReDocHandler),
            (r"/swagger", SwaggerUIHandler),
        ],
        rpc_client=rpc_client,
        openapi_schema=openapi_schema,
        debug=False,
    )
