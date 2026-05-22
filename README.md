# PickupWinder — Raspberry Pi + ESP32 Automated Winding Controller

An automated guitar pickup winding system driven by a Raspberry Pi host and an ESP32 real-time motion controller. The RPi handles planning, geometry, and user-facing APIs. The ESP32 drives the stepper motors in hard real-time using the RMT peripheral.

---

## Table of contents

1. [What it does](#what-it-does)
2. [Hardware overview](#hardware-overview)
3. [Repository layout](#repository-layout)
4. [Key concepts](#key-concepts)
5. [Getting started](#getting-started)
6. [Configuration reference](#configuration-reference)
7. [Winding a pickup — step by step](#winding-a-pickup--step-by-step)
8. [JSON-RPC API quick reference](#json-rpc-api-quick-reference)
9. [Further reading](#further-reading)

---

## What it does

PickupWinder automates or semi-automates the process of winding guitar pickup coils.
The machine moves two stepper axes in synchrony:

- **Spindle axis (axis 0)**: rotates the pickup bobbin.
- **Lateral axis (axis 1)**: moves the wire guide back and forth across the winding window.

The host computes exactly how far the guide must travel for each spindle revolution
(the *layer pitch*) and streams that motion to the ESP32 as pre-computed multi-axis
segment blocks. The ESP32 expands those segments into step pulses and streams them
to the motor drivers via the RMT peripheral.

Two winding modes are supported:

| Mode | Class | Description |
|------|-------|-------------|
| **Classic** | `WindingEngine` | Fixed program: N turns, fixed window, optional scatter, one shot. |
| **Adaptive** | `AdaptiveWindingService` | Live session: target RPM, window, and pitch can be changed while running. Progress is tracked in turns. |

---

## Hardware overview

```
  Raspberry Pi                     ESP32
  ─────────────────────────────────────────────────────
  spidev (master, mode 1) ──SPI──> slave (mode 1, DMA)
  GPIO poll / sysfs       <──────  GPIO17 READY output
  (optional shutdown)     <──────  GPIO16 RPI_SHUTDOWN_REQ

  ESP32 ──STEP/DIR──> Spindle stepper driver ──> Bobbin
  ESP32 ──STEP/DIR──> Lateral stepper driver ──> Wire guide
  ESP32 <── HX711 (tension load cell)
  ESP32 <── Rotary encoder (optional spindle feedback)
  ESP32 <── Lateral endstop (2-contact homing sensor)
```

- SPI bus: fixed 512-byte frames, mode 1 (CPOL=0, CPHA=1), CRC16-CCITT-FALSE.
- The ESP32 signals that it is ready to accept a new frame by asserting GPIO17 (active high by default).
- Do **not** drive GPIO12 high at boot — it straps the ESP32 flash voltage.

---

## Repository layout

```
src/
  windy/              Python host application
    winding_main.py   Process entry point and signal handling
    app/runtime.py    Runtime composition (wires all components together)
    core/
      config.py       AppConfiguration dataclass — all machine parameters
      engine.py       WindingEngine — classic program execution
      lateral.py      Lateral homing, soft-limit enforcement
      command_service.py  Motion command building and queuing
      coordinator.py  Centralized stop/fault coordination
      shared_state.py Thread-safe state shared between engine and RPC
      events.py       EventBus (EventKind enum + async dispatch)
      status.py       Snapshot builders for RPC status responses
    motion/
      move_queue.py   Move serialization and firmware flush coordination
      move.py         Move types: RampMove, HomingMove, CompositeMove
      move_builders.py  Factories for jog, homing, wound moves
      segment_generator.py   Host-side segment generation utilities
      multi_axis_segment_generator.py  General multi-axis move iterator
      spindle_kinematics.py  Bobbin turn profile over time (trapezoidal)
      trapezoidal_profile.py Trapezoidal speed ramp
      ramp_config.py  Low-level ramp parameters
      axis_state.py   Per-axis runtime state (position, home flag, limits)
    transport/
      messages.py     Python protocol mirror + wrap-aware sequence helpers
      spi_transport.py  SPI framing, polling, ACK confirmation
      streamer.py     MultiAxisRampStreamer — buffered segment streaming
      mock_spi_transport.py  In-process mock for testing
    winding/
      program.py      WindingProgram — persistent winding recipe
      session.py      SessionParams — transient execution context
      service.py      AdaptiveWindingService — live winding session
      adaptive.py     AdaptiveWindingRuntime — chunk planner
      wound_move.py   WoundMove / SynchronizedMove — electronic gearing move
      winding_pattern.py  Triangular traverse wave from spindle turns
      scatter_engine.py   Non-harmonic scatter with edge damping
      synchronized_segment_generator.py  Multi-axis segment iterator for winding
      program_store.py  Persistent program library (JSON files on disk)
      program.py      WindingProgram dataclass
    jsonrpc/
      rpc_server.py   Unix-socket JSON-RPC 2.0 server
      winding_handler.py  Facade that wires the four domain handlers
      execution_handler.py  winding.submit_program, wound_run, stop, status
      machine_handler.py   winding.jog, home_lateral, endstop, fault_clear
      session_handler.py   winding.start_session, pause, resume, update
      program_handler.py   program.list/get/save/update/load/delete
      handlers.py     Base RpcHandler dispatcher
      protocol.py     JSON-RPC 2.0 framing and error types

  esp32/              ESP-IDF firmware
    src/
      main.cpp        Pin map and app_main()
      messages.h      Packed protocol structs (must stay in sync with messages.py)
      comm/
        comm_interface.cpp    SPI slave task, CRC, dedup, dispatch
        comm_request_dispatcher.cpp  Route requests to handlers
        comm_status_builder.cpp     Build StatusPayload responses
      motion/
        motion_planner.cpp    Monotonic motion queue, flush handling
        multi_axis_executor.cpp  Segment expansion, ring fill, kick-start
        stepper_queue.cpp     Step ring management
        stepper_driver.cpp    RMT streaming, coast mode, underrun handling
      sensors/
        sensor_task.cpp       HX711, potentiometer, encoder acquisition
        endstop.cpp           2-contact lateral endstop handling

  wendy/              Lightweight HTTP bridge (Python)
    server.py         HTTP server → JSON-RPC forwarding
    handlers.py       REST endpoint handlers
    rpc.py            JSON-RPC client to Windy socket

data/                 Runtime data (created at startup if absent)
  config.json         Machine configuration (see Configuration reference)
  programs/           Saved winding programs (JSON files)

doc/                  Technical documentation
  architecture.md     Full system architecture
  spi_protocol.md     SPI frame format and ACK semantics
  sequencing.md       Sequence spaces, wrap-around safety, dedup rules
  stepper_engine.md   ESP32 RMT pipeline and ring buffer design
```

---

## Key concepts

### Program (`WindingProgram`)

A persistent recipe that describes **what** to wind, independent of the machine it runs on.
Saved to `data/programs/<id>.json`. Key fields:

| Field | Description |
|-------|-------------|
| `target_turns` | Total spindle turns to wind. Layer count is computed automatically. |
| `layer_pitch_mm` | Lateral advance per spindle revolution (mm). Equals wire diameter for tight winding. |
| `wire_diameter_mm` | Wire gauge in mm (informational, used in estimates). |
| `bobbin_width_mm` | Interior winding window width (flatwork to flatwork, mm). |
| `flatwork_thickness_mm` | Bobbin cheek/flatwork thickness (mm). Used to compute the start position. |
| `scatter_amplitude_mm` | Peak-to-peak scatter offset on the traverse (mm). `0` = no scatter. |
| `window_start_clearance_mm` | Per-program override for the safety gap at the winding start. `null` → use config default. |
| `window_end_clearance_mm` | Per-program override for the safety reduction at the far end. `null` → use config default. |

`num_layers` is **not stored** — it is derived:
```python
num_layers = ceil(target_turns / (2 * bobbin_width_mm * turns_per_mm))
# where turns_per_mm = 1 / layer_pitch_mm
```

### Session (`SessionParams`)

Transient execution context for one run. Not persisted. Key fields:

| Field | Description |
|-------|-------------|
| `spindle_rpm` | Target spindle speed. |
| `total_turns` | Optional override for `program.target_turns`. |
| `window_low_mm` / `window_high_mm` | Optional explicit window bounds override. |
| `chunk_time_s` | Adaptive planner chunk duration (default 0.25 s). |

### Configuration (`AppConfiguration`)

Machine-level parameters stored in `data/config.json`. They describe the hardware, not the pickup.
See [Configuration reference](#configuration-reference) for the full list.

### Winding window geometry

The lateral axis start position is built up from a chain of machine and program parameters:

```
window_low  = lateral_soft_limit_min_mm    ← plateau / homing reference edge
            + lateral_axis_offset_mm       ← machine fine-tuning (usually 0.0)
            + flatwork_thickness_mm        ← bobbin cheek thickness (per program)
            + window_start_clearance_mm    ← safety gap before first wire turn

window_high = window_low + bobbin_width_mm - window_end_clearance_mm
```

`window_start_clearance_mm` and `window_end_clearance_mm` have machine-level defaults
in `AppConfiguration` (`0.3 mm` and `0.0 mm`) that can be overridden per program.

### Electronic gearing model

The spindle and lateral axes are never driven independently during winding.
For every spindle revolution the guide must advance exactly `layer_pitch_mm`.

`WindingPattern.guide_pos_mm(spindle_turns)` implements the triangular wave:

```python
def guide_pos_mm(self, spindle_turns: float) -> float:
    total_dist = spindle_turns / self.turns_per_mm
    cycle_length = 2.0 * self.bobbin_width_mm
    mod_dist = total_dist % cycle_length
    return mod_dist if mod_dist <= self.bobbin_width_mm else cycle_length - mod_dist
```

`ScatterEngine` adds a small, non-repeating offset to break inter-layer resonance.

---

## Getting started

### 1. Build and flash ESP32 firmware

```bash
cd src/esp32
pio run -t upload          # PlatformIO
# or: idf.py build flash   # ESP-IDF native
```

### 2. Create a configuration file

Create `~/data/config.json` (or let the application create it with defaults on first run).
Minimal configuration for a typical build:

```json
{
  "spi_device": "/dev/spidev0.0",
  "spi_speed_hz": 4000000,
  "lateral_soft_limit_min_mm": 30.0,
  "lateral_axis_length_mm": 130.0,
  "window_start_clearance_mm": 0.3,
  "window_end_clearance_mm": 0.0
}
```

### 3. Start the host application

```bash
cd src/windy
python3 winding_main.py
```

The application opens the JSON-RPC socket at `/tmp/winding.sock` and starts the
winding engine and SPI transport. The lateral axis must be homed before any winding
can begin (this happens automatically when a program is submitted if `home_before_start`
is `true` in the config).

### 4. Create and run a winding program

```bash
# Save a program
echo '{"method":"program.save","params":{"name":"Strat bridge","target_turns":8000,
  "layer_pitch_mm":0.065,"wire_diameter_mm":0.063,"bobbin_width_mm":13.2,
  "flatwork_thickness_mm":3.2},"id":1,"jsonrpc":"2.0"}' | nc -U /tmp/winding.sock

# Run it (returns immediately; winding happens in background)
echo '{"method":"winding.submit_program","params":{"spindle_rpm":500,
  "program_id":"<id from above>"},"id":2,"jsonrpc":"2.0"}' | nc -U /tmp/winding.sock

# Poll status
echo '{"method":"winding.status","params":null,"id":3,"jsonrpc":"2.0"}' | nc -U /tmp/winding.sock
```

A sample program is included at `data/programs/programme-type-single-coil-42awg.json`.

---

## Configuration reference

All fields live in `src/windy/core/config.py` as the `AppConfiguration` dataclass.
Default values are shown. Override any field in `data/config.json`.

### SPI transport

| Field | Default | Description |
|-------|---------|-------------|
| `spi_device` | `"/dev/spidev0.0"` | Linux SPI device path. |
| `spi_speed_hz` | `4000000` | SPI clock frequency in Hz. |
| `spi_ready_gpio_chip` | `"/dev/gpiochip0"` | GPIO chip for the READY sideband. |
| `spi_ready_gpio_line` | `17` | GPIO line for the READY signal (GPIO17). |

### Spindle axis (axis 0)

| Field | Default | Description |
|-------|---------|-------------|
| `spindle_steps_per_revolution` | `200` | Full-step count for the spindle motor. |
| `spindle_microstepping` | `32` | Microstepping divisor. |
| `spindle_max_speed_rpm` | `1750` | Speed cap. |
| `spindle_accel_s` | `0.5` | Ramp-up time per layer (s). |
| `spindle_decel_s` | `0.5` | Ramp-down time per layer (s). |

### Lateral axis (axis 1)

| Field | Default | Description |
|-------|---------|-------------|
| `lateral_steps_per_revolution` | `96` | Full-step count for the lateral motor. |
| `lateral_microstepping` | `32` | Microstepping divisor. |
| `lateral_traverse_pitch_mm` | `1.0` | Lead screw pitch in mm/rev. |
| `lateral_soft_limit_min_mm` | `30.0` | Hard reference: plateau/homing edge in machine coordinates (mm). |
| `lateral_soft_limit_max_mm` | `null` | Optional upper travel limit (mm). |
| `lateral_axis_length_mm` | `130` | Physical travel; sizes the homing approach move. |
| `lateral_axis_offset_mm` | `0.0` | Machine fine-tuning offset added to `soft_limit_min_mm`. |
| `lateral_target_speed` | `120.0` | Speed for post-homing and start-position moves (RPM). |

### Winding window defaults

| Field | Default | Description |
|-------|---------|-------------|
| `window_start_clearance_mm` | `0.3` | Safety gap at the start of the winding window. Overridable per program. |
| `window_end_clearance_mm` | `0.0` | Safety reduction at the far end. Overridable per program. |

### Homing

| Field | Default | Description |
|-------|---------|-------------|
| `home_before_start` | `true` | Automatically home before executing a program. |
| `lateral_homing_approach_rpm` | `120.0` | Fast approach speed. |
| `lateral_homing_search_rpm` | `20.0` | Slow creep speed for precise trigger. |
| `lateral_homing_backoff_steps` | `6144` | Steps to back away from endstop before slow search. |

---

## Winding a pickup — step by step

This is the typical operator workflow:

1. **Load a saved program** (`program.load` or inline payload in `winding.submit_program`).
2. **Submit the program** with a target RPM via `winding.submit_program`.
3. The engine transitions `IDLE → HOMING → RUNNING`.
4. After homing, the lateral axis moves to `window_low`:
   ```
   window_low = soft_limit_min + axis_offset + flatwork_thickness + start_clearance
   ```
5. For each of `num_layers` passes the engine enqueues one `WoundMove` (spindle + lateral in sync).
6. On completion the engine returns to `IDLE` and publishes a `PROGRAM_COMPLETED` event.

For a live adjustable session, use `winding.start_session` with a `SessionParams` payload.
The adaptive planner breaks the full run into short chunks (~250 ms) so RPM, window, and
pitch ratio can be changed between chunks without stopping.

---

## JSON-RPC API quick reference

The socket is at `/tmp/winding.sock` (configurable via `rpc_socket_path`).
All calls follow JSON-RPC 2.0. Parameters are always a single object (`{}`).

### System

| Method | Description |
|--------|-------------|
| `winder.ping` | Health check. Returns `{"pong": true}`. |
| `winder.status` | Full runtime snapshot (engine state, layer progress, fault message). |
| `winder.config` | Returns the active `AppConfiguration` as JSON. |
| `winder.shutdown` | Graceful process stop. |

### Programs

| Method | Key params | Description |
|--------|-----------|-------------|
| `program.list` | — | List all saved programs. |
| `program.get` | `program_id` | Get one program by ID. |
| `program.save` | program fields | Create a new program. Returns the assigned `program_id`. |
| `program.update` | `program_id` + fields | Update fields; bumps `revision`. |
| `program.load` | `program_id` | Mark a program as loaded (does not start winding). |
| `program.delete` | `program_id` | Delete a saved program. |

### Winding execution

| Method | Key params | Description |
|--------|-----------|-------------|
| `winding.submit_program` | `spindle_rpm`, optional `program_id` or inline fields | Queue a classic winding run. |
| `winding.wound_run` | `spindle_rpm`, `bobbin_width_mm`, `layer_pitch_mm`, `target_turns` | One-shot run without saving a program. |
| `winding.status` | — | Current engine state, layer index, turns wound. |
| `winding.stop` | — | Request a controlled stop. |

### Live session

| Method | Key params | Description |
|--------|-----------|-------------|
| `winding.start_session` | `spindle_rpm`, optional window/pitch/turns | Start an adaptive session. |
| `winding.update_session` | `spindle_rpm`, `window_low_mm`, `window_high_mm` | Change parameters mid-session. |
| `winding.pause` | — | Pause after current chunk. |
| `winding.resume_session` | — | Resume a paused session. |
| `winding.session_status` | — | Turns completed, RPM, guide position, window. |

### Machine control

| Method | Key params | Description |
|--------|-----------|-------------|
| `winding.home_lateral` | — | Start a homing cycle. |
| `winding.jog` | `axis_id`, `steps`, `rpm`, `reverse` | Single-axis jog move. |
| `winding.move_lateral_mm` | `position_mm`, `rpm` | Move lateral to absolute position. |
| `winding.set_axis_offset` | `offset_mm` | Update `lateral_axis_offset_mm` live. |
| `winding.clear_fault` | — | Clear fault state to return to IDLE. |
| `winding.arm_endstop` | `axis_id` | Arm the endstop for homing. |
| `winding.disarm_endstop` | `axis_id` | Disarm the endstop. |

---

## Further reading

| Document | Contents |
|----------|----------|
| [doc/architecture.md](doc/architecture.md) | Full software architecture, runtime flow, state machine, wiring details. |
| [doc/spi_protocol.md](doc/spi_protocol.md) | SPI frame format, message types, ACK pipelining, adding new messages. |
| [doc/sequencing.md](doc/sequencing.md) | Three sequence spaces, wrap-around safety, deduplication layers. |
| [doc/stepper_engine.md](doc/stepper_engine.md) | ESP32 RMT pipeline, ring buffer design, coast mode, key constants. |
