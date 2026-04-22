from __future__ import annotations

import json
from typing import Any

import tornado.web


def make_openapi_schema(
    supported_methods: list[str] | None = None,
    title: str = "Wendy JSON-RPC",
) -> dict[str, Any]:
    schema: dict[str, Any] = {
        "openapi": "3.0.3",
        "info": {
            "title": title,
            "version": "1.0.0",
            "description": "JSON-RPC 2.0 HTTP/WebSocket gateway for Wendy.",
        },
        "paths": {
            "/rpc": {
                "post": {
                    "summary": "JSON-RPC 2.0 endpoint",
                    "description": "Send JSON-RPC requests and notifications to Wendy.",
                    "requestBody": {
                        "required": True,
                        "content": {
                            "application/json": {
                                "schema": {
                                    "$ref": "#/components/schemas/JsonRpcRequest"
                                }
                            }
                        }
                    },
                    "responses": {
                        "200": {
                            "description": "JSON-RPC response",
                            "content": {
                                "application/json": {
                                    "schema": {
                                        "$ref": "#/components/schemas/JsonRpcResponse"
                                    }
                                }
                            },
                        }
                    },
                }
            },
            "/status": {
                "get": {
                    "summary": "Winding status",
                    "description": "Return the current winding engine status using winding.status.",
                    "responses": {
                        "200": {
                            "description": "Winding engine status result",
                            "content": {
                                "application/json": {
                                    "schema": {
                                        "type": "object"
                                    }
                                }
                            }
                        }
                    }
                }
            },
            "/run_axis": {
                "get": {
                    "summary": "Run axis for a duration",
                    "description": "Compute steps from RPM and duration, then call winding.run_axis on the backend.",
                    "parameters": [
                        {
                            "name": "axis_id",
                            "in": "query",
                            "required": True,
                            "schema": {"type": "integer"},
                            "description": "Axis index to drive.",
                        },
                        {
                            "name": "rpm",
                            "in": "query",
                            "required": True,
                            "schema": {"type": "number"},
                            "description": "Target speed in RPM.",
                        },
                        {
                            "name": "duration_s",
                            "in": "query",
                            "required": True,
                            "schema": {"type": "number"},
                            "description": "Duration of the motion in seconds.",
                        },
                    ],
                    "responses": {
                        "200": {
                            "description": "Run axis command result",
                            "content": {
                                "application/json": {
                                    "schema": {
                                        "type": "object"
                                    }
                                }
                            }
                        },
                        "400": {
                            "description": "Invalid request parameters"
                        },
                        "502": {
                            "description": "Backend RPC error"
                        }
                    }
                }
            },
            "/home": {
                "get": {
                    "summary": "Home the lateral axis",
                    "description": "Invoke winding.home_lateral on the backend to run the lateral homing procedure.",
                    "parameters": [
                        {
                            "name": "approach_rpm",
                            "in": "query",
                            "required": False,
                            "schema": {"type": "number", "default": 100.0},
                            "description": "Approach speed in RPM.",
                        },
                        {
                            "name": "search_rpm",
                            "in": "query",
                            "required": False,
                            "schema": {"type": "number", "default": 20.0},
                            "description": "Search speed in RPM.",
                        },
                        {
                            "name": "backoff_steps",
                            "in": "query",
                            "required": False,
                            "schema": {"type": "integer", "default": 3200},
                            "description": "Backoff steps after endstop trigger.",
                        },
                    ],
                    "responses": {
                        "200": {
                            "description": "Homing command result",
                            "content": {
                                "application/json": {
                                    "schema": {
                                        "type": "object"
                                    }
                                }
                            }
                        },
                        "400": {
                            "description": "Invalid request parameters"
                        },
                        "502": {
                            "description": "Backend RPC error"
                        }
                    }
                }
            }
        },
        "components": {
            "schemas": {
                "JsonRpcRequest": {
                    "type": "object",
                    "required": ["jsonrpc", "method"],
                    "properties": {
                        "jsonrpc": {"type": "string", "enum": ["2.0"]},
                        "method": {"type": "string"},
                        "params": {"type": ["object", "array", "null"]},
                        "id": {"type": ["string", "number", "null"]},
                    },
                },
                "JsonRpcResponse": {
                    "type": "object",
                    "required": ["jsonrpc", "id"],
                    "properties": {
                        "jsonrpc": {"type": "string", "enum": ["2.0"]},
                        "result": {},
                        "error": {
                            "$ref": "#/components/schemas/JsonRpcError"
                        },
                        "id": {"type": ["string", "number", "null"]},
                    },
                },
                "JsonRpcError": {
                    "type": "object",
                    "required": ["code", "message"],
                    "properties": {
                        "code": {"type": "integer"},
                        "message": {"type": "string"},
                        "data": {},
                    },
                },
            }
        },
    }
    if supported_methods is not None:
        schema["x-jsonrpc-methods"] = sorted(supported_methods)
    return schema


class OpenApiHandler(tornado.web.RequestHandler):
    def get(self) -> None:
        self.set_header("Content-Type", "application/json")
        self.write(json.dumps(self.application.openapi_schema))


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


class SwaggerUIHandler(tornado.web.RequestHandler):
    def get(self) -> None:
        self.set_header("Content-Type", "text/html")
        self.write(
            """
            <!DOCTYPE html>
            <html>
            <head>
              <title>Wendy Swagger UI</title>
              <meta charset="utf-8" />
              <link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/swagger-ui-dist/swagger-ui.css" />
            </head>
            <body>
              <div id="swagger-ui"></div>
              <script src="https://cdn.jsdelivr.net/npm/swagger-ui-dist/swagger-ui-bundle.js"></script>
              <script src="https://cdn.jsdelivr.net/npm/swagger-ui-dist/swagger-ui-standalone-preset.js"></script>
              <script>
                window.onload = function() {
                  SwaggerUIBundle({
                    url: '/openapi.json',
                    dom_id: '#swagger-ui',
                    presets: [
                      SwaggerUIBundle.presets.apis,
                      SwaggerUIStandalonePreset,
                    ],
                    layout: 'StandaloneLayout',
                  });
                };
              </script>
            </body>
            </html>
            """
        )
