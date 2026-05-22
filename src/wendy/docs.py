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
            "/api/session/from-program": {
                "post": {
                    "summary": "Start adaptive session from a classic program",
                    "description": "Resolve a saved or loaded WindingProgram, translate it on the host into an AdaptiveWindingSessionConfig, then start the adaptive session.",
                    "requestBody": {
                        "required": False,
                        "content": {
                            "application/json": {
                                "schema": {
                                    "type": "object",
                                    "properties": {
                                        "program_id": {
                                            "type": "string",
                                            "description": "Stored program identifier. If omitted, the currently loaded program is used."
                                        },
                                        "program": {
                                            "$ref": "#/components/schemas/WindingProgram"
                                        },
                                        "load": {
                                            "type": "boolean",
                                            "default": True,
                                            "description": "Also mark the resolved program as the currently loaded program."
                                        },
                                        "total_turns": {
                                            "type": "number",
                                            "nullable": True,
                                            "description": "Optional override for the adaptive session total turns. Defaults to the full classic program total."
                                        },
                                        "chunk_time_s": {
                                            "type": "number",
                                            "nullable": True,
                                            "description": "Optional adaptive planner chunk duration override."
                                        }
                                    }
                                }
                            }
                        }
                    },
                    "responses": {
                        "202": {
                            "description": "Adaptive session started from program",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        },
                        "400": {"description": "Invalid request payload"},
                        "502": {"description": "Backend RPC error"}
                    }
                }
            },
            "/api/session": {
                "get": {
                    "summary": "Adaptive session status",
                    "description": "Return the current adaptive winding session snapshot (winding.session_status).",
                    "responses": {
                        "200": {
                            "description": "Session snapshot",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        }
                    }
                },
                "post": {
                    "summary": "Start adaptive winding session",
                    "description": "Start a live-controllable adaptive winding session (winding.start_session). Wire diameter is derived from wire_diameter_mm or wire_awg.",
                    "requestBody": {
                        "required": True,
                        "content": {
                            "application/json": {
                                "schema": {
                                    "type": "object",
                                    "required": ["name", "total_turns", "target_rpm", "window_low_mm", "window_high_mm"],
                                    "properties": {
                                        "name": {"type": "string"},
                                        "total_turns": {"type": "number", "description": "Total spindle turns for the full winding."},
                                        "target_rpm": {"type": "number", "description": "Initial spindle target speed in RPM."},
                                        "window_low_mm": {"type": "number", "description": "Lower edge of the winding window in mm from home."},
                                        "window_high_mm": {"type": "number", "description": "Upper edge of the winding window in mm from home."},
                                        "wire_diameter_mm": {"type": "number", "nullable": True, "description": "Wire diameter in mm. Required if wire_awg not set."},
                                        "wire_awg": {"type": "integer", "nullable": True, "description": "Wire gauge (AWG). Required if wire_diameter_mm not set."},
                                        "turns_per_mm_override": {"type": "number", "nullable": True, "description": "Override computed turns/mm."},
                                        "pitch_factor": {"type": "number", "default": 1.0, "description": "Pitch multiplier applied on top of wire_diameter_mm."},
                                        "scatter_amplitude_mm": {"type": "number", "default": 0.0},
                                        "scatter_damping_margin_mm": {"type": "number", "default": 0.0},
                                        "scatter_freq1": {"type": "number", "default": 1.0},
                                        "scatter_freq2": {"type": "number", "default": 1.618},
                                        "spindle_axis_id": {"type": "integer", "default": 0},
                                        "lateral_axis_id": {"type": "integer", "default": 1},
                                        "home_before_start": {"type": "boolean", "default": True},
                                        "home_approach_rpm": {"type": "number", "default": 100.0},
                                        "home_search_rpm": {"type": "number", "default": 20.0},
                                        "home_backoff_steps": {"type": "integer", "default": 3200},
                                        "chunk_time_s": {"type": "number", "default": 0.25, "description": "Chunk duration in seconds for adaptive planning."}
                                    }
                                }
                            }
                        }
                    },
                    "responses": {
                        "202": {
                            "description": "Session started",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        },
                        "400": {"description": "Invalid session config"},
                        "502": {"description": "Backend RPC error"}
                    }
                },
                "patch": {
                    "summary": "Update adaptive session controls",
                    "description": "Adjust live-controllable parameters of the running adaptive session (winding.update_session).",
                    "requestBody": {
                        "required": True,
                        "content": {
                            "application/json": {
                                "schema": {
                                    "type": "object",
                                    "properties": {
                                        "target_rpm": {"type": "number", "description": "New target RPM (0 triggers a pause)."},
                                        "window_low_mm": {"type": "number"},
                                        "window_high_mm": {"type": "number"},
                                        "wire_diameter_mm": {"type": "number", "nullable": True},
                                        "wire_awg": {"type": "integer", "nullable": True},
                                        "turns_per_mm": {"type": "number", "nullable": True},
                                        "pitch_factor": {"type": "number"}
                                    }
                                }
                            }
                        }
                    },
                    "responses": {
                        "200": {
                            "description": "Updated session snapshot",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        },
                        "502": {"description": "Backend RPC error"}
                    }
                },
                "delete": {
                    "summary": "Stop adaptive session",
                    "description": "Stop the running adaptive winding session (winding.stop). Use ?mode=stop (default), pause, or emergency_stop.",
                    "parameters": [
                        {
                            "name": "mode",
                            "in": "query",
                            "required": False,
                            "schema": {"type": "string", "enum": ["stop", "pause", "emergency_stop"], "default": "stop"}
                        }
                    ],
                    "responses": {
                        "200": {
                            "description": "Stop command result",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        },
                        "502": {"description": "Backend RPC error"}
                    }
                }
            },
            "/api/session/pause": {
                "post": {
                    "summary": "Pause adaptive session",
                    "description": "Request a controlled, resumable pause of the running session (winding.pause). Optionally provide pause_at_turn to defer the pause to a specific turn count.",
                    "requestBody": {
                        "required": False,
                        "content": {
                            "application/json": {
                                "schema": {
                                    "type": "object",
                                    "properties": {
                                        "pause_at_turn": {"type": "number", "nullable": True, "description": "Turn count at which to pause. Pauses immediately if omitted."}
                                    }
                                }
                            }
                        }
                    },
                    "responses": {
                        "200": {
                            "description": "Pause command result",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        },
                        "502": {"description": "Backend RPC error"}
                    }
                }
            },
            "/api/session/resume": {
                "post": {
                    "summary": "Resume adaptive session",
                    "description": "Resume a paused adaptive winding session (winding.resume_session).",
                    "responses": {
                        "200": {
                            "description": "Session snapshot after resume",
                            "content": {"application/json": {"schema": {"type": "object"}}}
                        },
                        "502": {"description": "Backend RPC error"}
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
                    "deprecated": True,
                    "summary": "Clear winding faults (deprecated — use POST /api/machine/clear-fault)",
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
                    "deprecated": True,
                    "summary": "Run axis for a duration (deprecated — use POST /api/machine/run-axis)",
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
                    "deprecated": True,
                    "summary": "Launch synchronized winding run (deprecated — use POST /api/machine/wound-run)",
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
                    "deprecated": True,
                    "summary": "Stop or pause motion (deprecated — use POST /api/machine/stop)",
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
                    "deprecated": True,
                    "summary": "Home the lateral axis (deprecated — use POST /api/machine/home)",
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
            # ── New REST routes ────────────────────────────────────────────
            "/api/programs/{program_id}/revisions": {
                "get": {
                    "summary": "List backup revisions of a program",
                    "parameters": [
                        {"name": "program_id", "in": "path", "required": True, "schema": {"type": "string"}}
                    ],
                    "responses": {"200": {"description": "Revision list", "content": {"application/json": {"schema": {"type": "object"}}}}}
                }
            },
            "/api/programs/{program_id}/restore/{revision}": {
                "post": {
                    "summary": "Restore a backup revision",
                    "parameters": [
                        {"name": "program_id", "in": "path", "required": True, "schema": {"type": "string"}},
                        {"name": "revision", "in": "path", "required": True, "schema": {"type": "integer"}},
                    ],
                    "responses": {"200": {"description": "Restored program snapshot", "content": {"application/json": {"schema": {"type": "object"}}}}}
                }
            },
            "/api/session/pause": {
                "post": {
                    "summary": "Pause the adaptive session",
                    "requestBody": {
                        "required": False,
                        "content": {"application/json": {"schema": {
                            "type": "object",
                            "properties": {"pause_at_turn": {"type": "number", "nullable": True}}
                        }}}
                    },
                    "responses": {"200": {"description": "Pause result", "content": {"application/json": {"schema": {"type": "object"}}}}}
                }
            },
            "/api/session/resume": {
                "post": {
                    "summary": "Resume the adaptive session",
                    "responses": {"200": {"description": "Session snapshot after resume", "content": {"application/json": {"schema": {"type": "object"}}}}}
                }
            },
            "/api/machine/status": {
                "get": {
                    "summary": "Machine status",
                    "description": "Engine state, move-queue depth, axis positions, transport diagnostics.",
                    "responses": {"200": {"description": "Machine status snapshot", "content": {"application/json": {"schema": {"type": "object"}}}}}
                }
            },
            "/api/machine/home": {
                "post": {
                    "summary": "Home the lateral axis",
                    "description": "Start the lateral homing sequence (winding.home_lateral).",
                    "responses": {
                        "202": {"description": "Homing started"},
                        "502": {"description": "Backend RPC error"}
                    }
                }
            },
            "/api/machine/clear-fault": {
                "post": {
                    "summary": "Clear winding fault",
                    "description": "Acknowledge and clear the current fault (winding.clear_fault).",
                    "responses": {
                        "200": {"description": "Fault cleared"},
                        "502": {"description": "Backend RPC error"}
                    }
                }
            },
            "/api/machine/stop": {
                "post": {
                    "summary": "Stop or pause motion",
                    "requestBody": {
                        "required": False,
                        "content": {"application/json": {"schema": {
                            "type": "object",
                            "properties": {"mode": {"type": "string", "enum": ["stop", "pause", "emergency_stop"], "default": "stop"}}
                        }}}
                    },
                    "responses": {
                        "200": {"description": "Stop command result"},
                        "502": {"description": "Backend RPC error"}
                    }
                }
            },
            "/api/machine/run-axis": {
                "post": {
                    "summary": "Run one or two axes for a duration",
                    "requestBody": {
                        "required": True,
                        "content": {"application/json": {"schema": {
                            "type": "object",
                            "required": ["duration_s", "targets"],
                            "properties": {
                                "duration_s": {"type": "number", "description": "Duration in seconds."},
                                "targets": {
                                    "type": "array",
                                    "items": {
                                        "type": "object",
                                        "required": ["axis_id", "rpm"],
                                        "properties": {
                                            "axis_id": {"type": "integer"},
                                            "rpm": {"type": "number"},
                                            "reverse": {"type": "boolean", "default": False}
                                        }
                                    }
                                }
                            }
                        }}}
                    },
                    "responses": {
                        "202": {"description": "Run-axis started"},
                        "400": {"description": "Invalid parameters"},
                        "502": {"description": "Backend RPC error"}
                    }
                }
            },
            "/api/machine/wound-run": {
                "post": {
                    "summary": "Diagnostic synchronized winding run",
                    "description": "Low-level two-axis Electronic Gearing winding (winding.wound_run). Intended for development and diagnostics; bypasses state machine.",
                    "requestBody": {
                        "required": True,
                        "content": {"application/json": {"schema": {"$ref": "#/components/schemas/WoundRunParams"}}}
                    },
                    "responses": {
                        "202": {"description": "Winding run queued"},
                        "400": {"description": "Invalid parameters"},
                        "502": {"description": "Backend RPC error"}
                    }
                }
            },
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
                        "home_before_start": {"type": "boolean"},
                        "home_approach_rpm": {"type": "number"},
                        "home_search_rpm": {"type": "number"},
                        "home_backoff_steps": {"type": "integer"},
                        "revision": {"type": "integer"},
                        "created_at": {"type": ["string", "null"]},
                        "updated_at": {"type": ["string", "null"]}
                    }
                },
                "WoundRunParams": {
                    "type": "object",
                    "required": ["spindle_axis_id", "traverse_axis_id", "target_rpm", "bobbin_width_mm", "turns_per_mm"],
                    "properties": {
                        "spindle_axis_id": {"type": "integer"},
                        "traverse_axis_id": {"type": "integer"},
                        "target_rpm": {"type": "number"},
                        "bobbin_width_mm": {"type": "number"},
                        "turns_per_mm": {"type": "number"},
                        "accel_s": {"type": "number", "nullable": True},
                        "cruise_s": {"type": "number", "nullable": True},
                        "decel_s": {"type": "number", "nullable": True},
                        "scatter_amplitude_mm": {"type": "number", "default": 0.0},
                        "scatter_damping_margin_mm": {"type": "number", "default": 0.0},
                        "scatter_freq1": {"type": "number", "default": 1.0},
                        "scatter_freq2": {"type": "number", "default": 1.618},
                        "spindle_reverse": {"type": "boolean", "default": False},
                        "traverse_reverse": {"type": "boolean", "default": False}
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
