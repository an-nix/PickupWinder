"""WebInterface - HTTP + WebSocket server for PickupWinder on BBB.

Uses aiohttp (pip3 install aiohttp).
Replaces ESP32 AsyncWebServer+WiFi with a plain LAN server on port 80.

WebSocket message format matches ESP32 exactly:
  Incoming: {"cmd": "...", "val": "..."}
  Outgoing: compact JSON status (see sendUpdate)

HTTP endpoints:
  GET /              -> data/index.html
  GET /style.css     -> data/style.css
  GET /script.js     -> data/script.js
  GET /recipe.json   -> current recipe JSON (download)
  GET /protocol.json -> version info
  GET /capabilities.json -> command catalog
"""
import asyncio
import json
import os
import pathlib
from typing import Callable, Set

try:
    from aiohttp import web
    _AIOHTTP = True
except ImportError:
    _AIOHTTP = False
    print("[Web] aiohttp not installed - web interface disabled (pip3 install aiohttp)")

from command_registry import CommandRegistry
from config import (
    PICKUP_WS_PROTOCOL_VERSION, PICKUP_UART_PROTOCOL_VERSION,
    PICKUP_RECIPE_FORMAT_VERSION, WEB_PORT,
)

# ── Static files relative to this script ──────────────────────────────────────
_HERE = pathlib.Path(__file__).parent
_DATA_DIR = _HERE.parent.parent.parent / "data"   # ../../data from python/


class WebInterface:
    """Async HTTP + WebSocket server."""

    def __init__(self, command_controller, winder_app, host: str = "0.0.0.0", port: int = None):
        self._cmd_ctrl = command_controller
        self._app_ref = winder_app
        self._host = host
        self._port = port or WEB_PORT
        self._ws_clients: Set = set()
        self._registry = CommandRegistry()
        self._site = None
        self._runner = None

    # ── Startup ───────────────────────────────────────────────────────────────
    async def start(self):
        if not _AIOHTTP:
            print("[Web] aiohttp unavailable - skipping web server start")
            return

        app = web.Application()
        app.router.add_get("/", self._handle_root)
        app.router.add_get("/style.css", self._handle_static("style.css", "text/css"))
        app.router.add_get("/script.js", self._handle_static("script.js", "application/javascript"))
        app.router.add_get("/recipe.json", self._handle_recipe)
        app.router.add_get("/protocol.json", self._handle_protocol)
        app.router.add_get("/capabilities.json", self._handle_capabilities)
        app.router.add_get("/ws", self._handle_ws)

        self._runner = web.AppRunner(app)
        await self._runner.setup()
        self._site = web.TCPSite(self._runner, self._host, self._port)
        await self._site.start()
        print(f"[Web] Server listening on http://{self._host}:{self._port}/")

    async def stop(self):
        for ws in list(self._ws_clients):
            await ws.close()
        if self._runner:
            await self._runner.cleanup()

    # ── Broadcast status ─────────────────────────────────────────────────────
    async def send_update(self, status: dict):
        if not self._ws_clients:
            return
        try:
            payload = json.dumps(status, separators=(',', ':'))
        except Exception as e:
            print(f"[Web] JSON serialization error: {e}")
            return
        dead = set()
        for ws in list(self._ws_clients):
            try:
                await ws.send_str(payload)
            except Exception:
                dead.add(ws)
        self._ws_clients -= dead

    # ── HTTP handlers ─────────────────────────────────────────────────────────
    async def _handle_root(self, request):
        return await self._serve_file("index.html", "text/html")

    def _handle_static(self, filename: str, ctype: str):
        async def handler(request):
            return await self._serve_file(filename, ctype)
        return handler

    async def _serve_file(self, filename: str, ctype: str):
        path = _DATA_DIR / filename
        if not path.exists():
            return web.Response(status=404, text=f"Not found: {filename}")
        return web.FileResponse(path, headers={"Content-Type": ctype})

    async def _handle_recipe(self, request):
        if not self._app_ref:
            return web.Response(status=503, content_type="application/json",
                                text='{"error":"recipe unavailable"}')
        try:
            recipe_json = self._app_ref.recipe_json()
        except Exception as e:
            return web.Response(status=500, content_type="application/json",
                                text=f'{{"error":"{e}"}}')
        return web.Response(
            content_type="application/json",
            text=recipe_json,
            headers={"Content-Disposition": "attachment; filename=pickup-winder-recipe.json"},
        )

    async def _handle_protocol(self, request):
        payload = json.dumps({
            "ws": PICKUP_WS_PROTOCOL_VERSION,
            "uart": PICKUP_UART_PROTOCOL_VERSION,
            "recipe": PICKUP_RECIPE_FORMAT_VERSION,
        }, separators=(',', ':'))
        return web.Response(content_type="application/json", text=payload)

    async def _handle_capabilities(self, request):
        return web.Response(content_type="application/json",
                            text=self._registry.capabilities_json())

    # ── WebSocket handler ─────────────────────────────────────────────────────
    async def _handle_ws(self, request):
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ws_clients.add(ws)
        print(f"[WS] Client connected ({len(self._ws_clients)} total)")

        async for msg in ws:
            from aiohttp import WSMsgType
            if msg.type == WSMsgType.TEXT:
                await self._on_ws_message(msg.data)
            elif msg.type in (WSMsgType.ERROR, WSMsgType.CLOSE):
                break

        self._ws_clients.discard(ws)
        print(f"[WS] Client disconnected ({len(self._ws_clients)} remaining)")
        return ws

    async def _on_ws_message(self, raw: str):
        try:
            doc = json.loads(raw)
        except json.JSONDecodeError:
            return
        cmd = doc.get("cmd", "")
        val = doc.get("val", "")
        if cmd:
            self._cmd_ctrl.push_command(str(cmd), str(val))
