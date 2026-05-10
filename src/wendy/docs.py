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
            "/api/programs": {
                "get": {
                    "summary": "List saved programs",
                    "description": "Return saved winding programs from the Raspberry Pi program library.",
                    "parameters": [
                        {
                            "name": "include_content",
                            "in": "query",
                            "required": False,
                            "schema": {"type": "boolean", "default": False},
                            "description": "Include the full program payload for each entry.",
                        }
                    ],
                    "responses": {
                        "200": {
                            "description": "Saved program list",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        }
                    }
                },
                "post": {
                    "summary": "Save a winding program",
                    "description": "Upload a program to the Raspberry Pi and persist it in the local library.",
                    "requestBody": {
                        "required": True,
                        "content": {
                            "application/json": {
                                "schema": {
                                    "type": "object",
                                    "properties": {
                                        "program_id": {"type": "string"},
                                        "id": {"type": "string"},
                                        "load": {"type": "boolean", "default": False},
                                        "program": {"$ref": "#/components/schemas/WindingProgram"},
                                        "name": {"type": "string"},
                                        "num_layers": {"type": "integer"},
                                        "spindle_rpm": {"type": "number"},
                                        "layer_pitch_mm": {"type": "number"},
                                        "wire_diameter_mm": {"type": "number"}
                                    }
                                }
                            }
                        }
                    },
                    "responses": {
                        "201": {
                            "description": "Saved program",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        }
                    }
                }
            },
            "/api/programs/{program_id}": {
                "get": {
                    "summary": "Read a saved program",
                    "parameters": [
                        {
                            "name": "program_id",
                            "in": "path",
                            "required": True,
                            "schema": {"type": "string"}
                        }
                    ],
                    "responses": {
                        "200": {
                            "description": "Saved program payload",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        }
                    }
                },
                "put": {
                    "summary": "Update a saved program",
                    "parameters": [
                        {
                            "name": "program_id",
                            "in": "path",
                            "required": True,
                            "schema": {"type": "string"}
                        }
                    ],
                    "requestBody": {
                        "required": True,
                        "content": {
                            "application/json": {
                                "schema": {
                                    "type": "object",
                                    "properties": {
                                        "load": {"type": "boolean", "default": False},
                                        "changes": {"$ref": "#/components/schemas/WindingProgram"},
                                        "program": {"$ref": "#/components/schemas/WindingProgram"}
                                    }
                                }
                            }
                        }
                    },
                    "responses": {
                        "200": {
                            "description": "Updated program",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        }
                    }
                },
                "delete": {
                    "summary": "Delete a saved program",
                    "parameters": [
                        {
                            "name": "program_id",
                            "in": "path",
                            "required": True,
                            "schema": {"type": "string"}
                        }
                    ],
                    "responses": {
                        "200": {
                            "description": "Deletion status",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        }
                    }
                }
            },
            "/api/programs/{program_id}/load": {
                "post": {
                    "summary": "Load a saved program",
                    "description": "Mark a saved program as the currently loaded program on the Raspberry Pi runtime.",
                    "parameters": [
                        {
                            "name": "program_id",
                            "in": "path",
                            "required": True,
                            "schema": {"type": "string"}
                        }
                    ],
                    "responses": {
                        "200": {
                            "description": "Loaded program",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        }
                    }
                }
            },
            "/api/programs/{program_id}/queue": {
                "post": {
                    "summary": "Queue a saved program",
                    "description": "Submit a saved program to the winding engine by program id.",
                    "parameters": [
                        {
                            "name": "program_id",
                            "in": "path",
                            "required": True,
                            "schema": {"type": "string"}
                        }
                    ],
                    "responses": {
                        "202": {
                            "description": "Queued program",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        }
                    }
                }
            },
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
            "/clear_fault": {
                "get": {
                    "summary": "Clear winding faults",
                    "description": "Invoke winding.clear_fault on the backend to acknowledge and clear the current fault.",
                    "responses": {
                        "200": {
                            "description": "Clear fault command result",
                            "content": {
                                "application/json": {
                                    "schema": {
                                        "type": "object"
                                    }
                                }
                            }
                        },
                        "502": {
                            "description": "Backend RPC error"
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
                        {
                            "name": "reverse",
                            "in": "query",
                            "required": False,
                            "schema": {"type": "boolean", "default": False},
                            "description": "Whether to reverse the direction of the axis.",
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
            "/wound_run": {
                "post": {
                    "summary": "Launch synchronized winding run",
                    "description": "Start a two-axis synchronized winding operation (Electronic Gearing) by invoking winding.wound_run on the backend.",
                    "requestBody": {
                        "required": True,
                        "content": {
                            "application/json": {
                                "schema": {
                                    "type": "object",
                                    "required": ["spindle_axis_id", "traverse_axis_id", "target_rpm", "bobbin_width_mm", "turns_per_mm"],
                                    "properties": {
                                        "spindle_axis_id": {"type": "integer", "description": "Axis index of the spindle (coil winding motor)."},
                                        "traverse_axis_id": {"type": "integer", "description": "Axis index of the traverse (lateral displacement motor)."},
                                        "target_rpm": {"type": "number", "description": "Spindle target speed in RPM."},
                                        "bobbin_width_mm": {"type": "number", "description": "Winding width of the bobbin in mm."},
                                        "turns_per_mm": {"type": "number", "description": "Wire turns per mm of traverse travel."},
                                        "accel_s": {"type": "number", "nullable": True, "description": "Acceleration ramp duration in seconds (auto-computed if omitted)."},
                                        "cruise_s": {"type": "number", "nullable": True, "description": "Cruise phase duration in seconds (auto-computed if omitted)."},
                                        "decel_s": {"type": "number", "nullable": True, "description": "Deceleration ramp duration in seconds (auto-computed if omitted)."},
                                        "scatter_amplitude_mm": {"type": "number", "default": 0.0, "description": "Scatter winding amplitude in mm (0 disables scatter)."},
                                        "scatter_damping_margin_mm": {"type": "number", "default": 0.0, "description": "Scatter damping margin near bobbin edges in mm."},
                                        "scatter_freq1": {"type": "number", "default": 1.0, "description": "Primary scatter frequency multiplier."},
                                        "scatter_freq2": {"type": "number", "default": 1.618, "description": "Secondary scatter frequency multiplier."},
                                        "spindle_reverse": {"type": "boolean", "default": False, "description": "Reverse spindle direction."},
                                        "traverse_reverse": {"type": "boolean", "default": False, "description": "Reverse traverse direction."},
                                    },
                                }
                            }
                        },
                    },
                    "responses": {
                        "200": {
                            "description": "Winding run queued",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        },
                        "400": {"description": "Missing or invalid parameters"},
                        "502": {"description": "Backend RPC error"},
                    },
                }
            },
            "/stop": {
                "get": {
                    "summary": "Stop or pause motion",
                    "description": "Invoke winding.stop on the backend. Use ?mode=stop (default), pause, or emergency_stop.",
                    "parameters": [
                        {
                            "name": "mode",
                            "in": "query",
                            "required": False,
                            "schema": {"type": "string", "enum": ["stop", "pause", "emergency_stop"], "default": "stop"},
                            "description": "Stop mode.",
                        }
                    ],
                    "responses": {
                        "200": {
                            "description": "Stop command result",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        },
                        "502": {"description": "Backend RPC error"},
                    },
                }
            },
            "/home": {
                "get": {
                    "summary": "Home the lateral axis",
                    "description": "Invoke winding.home_lateral on the backend to run the lateral homing procedure using the configured defaults.",
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
                "WindingProgram": {
                    "type": "object",
                    "required": [
                        "name",
                        "num_layers",
                        "spindle_rpm",
                        "layer_pitch_mm",
                        "wire_diameter_mm"
                    ],
                    "properties": {
                        "id": {"type": ["string", "null"]},
                        "program_id": {"type": ["string", "null"]},
                        "name": {"type": "string"},
                        "num_layers": {"type": "integer"},
                        "spindle_rpm": {"type": "number"},
                        "layer_pitch_mm": {"type": "number"},
                        "wire_diameter_mm": {"type": "number"},
                        "bobbin_width_mm": {"type": "number"},
                        "scatter_amplitude_mm": {"type": "number"},
                        "scatter_damping_margin_mm": {"type": "number"},
                        "scatter_freq1": {"type": "number"},
                        "scatter_freq2": {"type": "number"},
                        "accel_s": {"type": "number"},
                        "decel_s": {"type": "number"},
                        "spindle_axis_id": {"type": "integer"},
                        "lateral_axis_id": {"type": "integer"},
                        "lateral_steps_per_mm": {"type": "number"},
                        "home_before_start": {"type": "boolean"},
                        "home_approach_rpm": {"type": "number"},
                        "home_search_rpm": {"type": "number"},
                        "home_backoff_steps": {"type": "integer"},
                        "revision": {"type": "integer"},
                        "created_at": {"type": ["string", "null"]},
                        "updated_at": {"type": ["string", "null"]}
                    }
                }
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
