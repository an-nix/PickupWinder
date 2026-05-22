# PickupWinder — System Architecture

This document describes the full software architecture of the PickupWinder system,
intended as a guide for developers new to the project.

---

## 1. System overview

PickupWinder is a **two-processor system**:

```
  ┌───────────────────────────────────────────────────────┐
  │  Raspberry Pi (host)                                  │
  │  ─────────────────────────────────────────────────    │
  │  Python application (src/windy/)                      │
  │    • Winding geometry computation                     │
  │    • Program and session management                   │
  │    • JSON-RPC API for operator UI                     │
  │    • SPI master transport                             │
  └────────────────┬──────────────────────────────────────┘
                   │  SPI (512-byte frames, mode 1, CRC16)
                   │  GPIO17 READY sideband
                   ▼
  ┌───────────────────────────────────────────────────────┐
  │  ESP32 (firmware, src/esp32/)                         │
  │  ─────────────────────────────────────────────────    │
  │    • SPI slave, frame validation, request dedup       │
  │    • Monotonic motion planner queue                   │
  │    • Step ring fill + RMT pulse emission              │
  │    • Sensor acquisition (tension, encoder, endstop)   │
  └──────────┬───────────────────────────┬────────────────┘
             │ STEP/DIR                  │ STEP/DIR
             ▼                           ▼
       Spindle driver             Lateral driver
       (bobbin rotation)          (wire guide traverse)
```

The **RPi owns all planning logic**: winding geometry, scatter, session state, retries,
and program persistence. The **ESP32 owns all real-time execution**: step pulse generation,
RMT streaming, and sensor sampling.

The ESP32 firmware does **not** compute winding geometry. It only validates, queues,
expands, and executes what the host planned.

---

## 2. Startup sequence

```
winding_main.py
  └─ WinderApplication.__init__()          # app/runtime.py
       ├─ ConfigurationManager.load()      # reads data/config.json
       ├─ Esp32SpiTransport(...)           # opens /dev/spidevX.Y
       ├─ SharedState(axis_states)         # thread-safe runtime state
       ├─ EventBus()                       # async event queue
       ├─ MoveQueue(transport, ...)        # move serializer thread
       ├─ LateralAxisController(...)       # homing and soft-limits
       ├─ MotionCommandService(...)        # move building helpers
       ├─ MotionCoordinator(...)           # centralized stop/fault
       ├─ WindingEngine(...)               # classic program executor
       ├─ AdaptiveWindingService(...)      # live session executor
       ├─ ProgramStore(...)               # JSON program library
       ├─ RuntimeStatusService(...)        # RPC snapshot builder
       └─ JsonRpcServer(socket_path)       # Unix-socket RPC listener
            └─ WindingRpcHandler           # wires 4 domain handlers

winding_main.py registers SIGINT/SIGTERM → app.stop()
then blocks in time.sleep(1) loop
```

All components are constructed in `app/runtime.py`. The entry point `winding_main.py`
is intentionally thin: it only handles signals and the blocking main loop.

---

## 3. Module responsibilities

### Host — core

| Module | Responsibility |
|--------|----------------|
| `core/config.py` | `AppConfiguration` dataclass. All machine parameters. Loaded from `data/config.json`. |
| `core/shared_state.py` | `SharedState` — thread-safe store for engine state, active program, layer progress, and session snapshot. Written by engine/service, read by RPC handlers. |
| `core/engine.py` | `WindingEngine` — executes classic winding programs. Owns the daemon thread, state machine, and layer loop. |
| `core/lateral.py` | `LateralAxisController` — homing sequence, `move_to_start_position()`, soft-limit checks, home-state invalidation. |
| `core/command_service.py` | `MotionCommandService` — convenience move builders used by RPC handlers (jog, endstop arm/disarm, manual axis moves). |
| `core/coordinator.py` | `MotionCoordinator` — centralized stop and fault coordination across engine and session. |
| `core/events.py` | `EventBus` — non-blocking multi-producer queue. Publishes `EventKind` events consumed by the RPC/UI layer. |
| `core/status.py` | `RuntimeStatusService` — builds explicit status/config snapshots for `winder.status` and `winding.status`. |

### Host — motion

| Module | Responsibility |
|--------|----------------|
| `motion/move_queue.py` | `MoveQueue` — serializes moves into the SPI pipeline. Runs a dedicated thread. Handles `HomingMove` phase-by-phase, `RampMove` via `MultiAxisRampStreamer`, and `WoundMove`/`AdaptiveWindingMove` via the synchronized generator. |
| `motion/move.py` | Move hierarchy: `Move` (base), `RampMove` (single trapezoidal axis), `HomingMove` (endstop-driven multi-phase), `CompositeMove`. |
| `motion/move_builders.py` | Factory functions for standard move types (`build_jog_move`, etc.). |
| `motion/segment_generator.py` | Host-side per-axis step profile generator. |
| `motion/multi_axis_segment_generator.py` | Interleaves per-axis profiles into `MultiAxisSegment` objects with monotonic `motion_sequence` values. |
| `motion/spindle_kinematics.py` | `SpindleKinematics` — computes spindle turns over time for a trapezoidal speed profile. |
| `motion/axis_state.py` | `AxisState` — per-axis runtime state: current position in steps, homed flag, soft limits. |

### Host — transport

| Module | Responsibility |
|--------|----------------|
| `transport/messages.py` | Python mirror of the firmware's packed structs. `MultiAxisSegment`, `SpiMessageHeader`, `StatusPayload`, etc. Wrap-aware 16-bit sequence helpers. |
| `transport/spi_transport.py` | `Esp32SpiTransport` — builds SPI frames, polls the READY GPIO, sends 512-byte transfers via `spidev`, and confirms requests via `wait_for_request_result()`. |
| `transport/streamer.py` | `MultiAxisRampStreamer` — batches `MultiAxisSegment` objects into `MULTI_AXIS_SEGMENT_BLOCK` SPI requests, tracks in-flight segments, and applies backpressure. |

### Host — winding

| Module | Responsibility |
|--------|----------------|
| `winding/program.py` | `WindingProgram` — persistent recipe (target turns, geometry, scatter params). `num_layers` is a computed property, not stored. |
| `winding/session.py` | `SessionParams` — transient execution parameters (RPM, window overrides, chunk time). Not persisted. |
| `winding/program_store.py` | `ProgramStore` — JSON library on disk under `data/programs/`. CRUD with stable `program_id` and monotonic `revision`. |
| `winding/service.py` | `AdaptiveWindingService` — adaptive winding thread. Resolves window geometry, plans chunks, streams motion, tracks progress, handles pause/resume. |
| `winding/adaptive.py` | `AdaptiveWindingRuntime` + `plan_next_chunk()` — chunk planner: accelerates spindle, winds, decelerates at edges before reversing. |
| `winding/wound_move.py` | `WoundMove` / `SynchronizedMove` — a move that streams spindle + lateral in lock-step using electronic gearing. |
| `winding/winding_pattern.py` | `WindingPattern.guide_pos_mm(spindle_turns)` — triangular traverse wave. |
| `winding/scatter_engine.py` | `ScatterEngine` — adds a small, non-harmonic perturbation to the guide position; damped near the flanges. |
| `winding/synchronized_segment_generator.py` | `SynchronizedSegmentGenerator` — samples the time domain and emits `MultiAxisSegment` objects for winding moves. |

### Host — JSON-RPC

| Module | Responsibility |
|--------|----------------|
| `jsonrpc/rpc_server.py` | Unix-socket JSON-RPC 2.0 server. Accepts connections, deserializes requests, dispatches to handlers. |
| `jsonrpc/winding_handler.py` | `WindingRpcHandler` — composition facade. Instantiates the four domain handlers and wires them to the dispatcher. |
| `jsonrpc/execution_handler.py` | `winding.submit_program`, `wound_run`, `flush_until`, `status`, `axis_state`, `stop`. |
| `jsonrpc/machine_handler.py` | `winding.jog`, `run_axis`, `home_lateral`, `move_lateral_mm`, `set_axis_offset`, `clear_fault`, `arm/disarm_endstop`. |
| `jsonrpc/session_handler.py` | `winding.start_session`, `update_session`, `pause`, `resume_session`, `session_status`. |
| `jsonrpc/program_handler.py` | `program.list`, `program.get`, `program.save`, `program.update`, `program.load`, `program.delete`. |

### Firmware (ESP32)

The firmware is split by CPU core:

| Core | Responsibilities |
|------|-----------------|
| Core 0 | SPI slave task (`comm_interface.cpp`), sensor task (`sensor_task.cpp`), endstop event publication. |
| Core 1 | Motion planner (`motion_planner.cpp`), ring fill and kick-start decisions (`multi_axis_executor.cpp`), RMT pulse streaming (`stepper_driver.cpp`). |

---

## 4. Motion pipeline — end to end

A winding move goes through the following stages:

```
[Host] WindingProgram + SessionParams
    │
    ▼
WindingEngine._run_layer()  or  AdaptiveWindingService
    │  creates a WoundMove / AdaptiveWindingMove
    ▼
MoveQueue.enqueue(move)
    │  move picked up by MoveQueue thread
    ▼
MultiAxisRampStreamer (transport/streamer.py)
    │  pulls MultiAxisSegment objects from the move iterator
    │  batches them into MULTI_AXIS_SEGMENT_BLOCK SPI requests
    ▼
Esp32SpiTransport.send_multi_axis_block()
    │  builds a 512-byte frame with CRC16
    │  polls GPIO17 READY
    │  calls spidev.xfer2()
    │  calls wait_for_request_result() → waits for pipelined ACK
    ▼
[ESP32] CommInterface (comm_interface.cpp)
    │  validates magic, version, CRC
    │  deduplicates exact retries (by sequence + type + len + CRC)
    │  deduplicates already-accepted blocks (by block_seq)
    ▼
MotionPlanner (motion_planner.cpp)
    │  enforces monotonic motion_sequence order
    │  drops stale or out-of-order segments
    │  feeds the executor queue
    ▼
MultiAxisExecutor (multi_axis_executor.cpp)
    │  expands segments into constant-rate step entries
    │  writes entries into the SPSC step ring
    │  calls kickStart() once per drain batch
    ▼
StepperDriver / RMT ISR (stepper_driver.cpp)
    │  encode_steps() emits STEP symbols from the ring
    │  coast mode emits pause symbols on transient starvation
    └─► motor pulses → STEP/DIR to stepper drivers
```

### Pipelined ACK rule

SPI is full-duplex: the status frame returned **during transfer N** reflects the
processing result of transfer **N-1**. The host must therefore always call
`wait_for_request_result(sequence)` after sending a control request, rather than
trusting the immediate return value.

```python
seq = transport.send_multi_axis_block(segments)
# seq is valid, but the ACK is in the NEXT transfer's response
transport.wait_for_request_result(seq)   # blocks until confirmed
```

---

## 5. Winding geometry model

### Window computation chain

The lateral axis traversal window for a winding run is computed as:

```
window_low  = lateral_soft_limit_min_mm    ← machine reference (plateau edge)
            + lateral_axis_offset_mm       ← machine fine-tuning offset (usually 0.0)
            + flatwork_thickness_mm        ← pickup-specific (per program)
            + window_start_clearance_mm    ← safety gap (config default, per-program override)

window_high = window_low
            + bobbin_width_mm              ← interior window width
            - window_end_clearance_mm      ← safety reduction at far end
```

Implemented in `WindingProgram.effective_window()`:

```python
def effective_window(self, *, soft_limit_min_mm, machine_offset_mm,
                     default_start_clearance_mm, default_end_clearance_mm):
    start_clearance = self.window_start_clearance_mm or default_start_clearance_mm
    window_low = soft_limit_min_mm + machine_offset_mm + self.flatwork_thickness_mm + start_clearance
    window_high = window_low + self.effective_winding_width_mm(default_end_clearance_mm)
    return window_low, window_high
```

### Layer count computation

```python
# num_layers is a computed property, NOT a stored field
@property
def num_layers(self) -> int:
    turns_per_pass = 2.0 * self.bobbin_width_mm * self.turns_per_mm
    return max(1, math.ceil(self.target_turns / turns_per_pass))
```

### Electronic gearing

The guide position is a triangular wave over spindle turns:

```python
def guide_pos_mm(self, spindle_turns: float) -> float:
    total_dist = spindle_turns / self.turns_per_mm
    cycle_length = 2.0 * self.bobbin_width_mm
    mod_dist = total_dist % cycle_length
    return mod_dist if mod_dist <= self.bobbin_width_mm else cycle_length - mod_dist
```

Scatter adds a small, damped, non-harmonic offset from `ScatterEngine` to avoid
inter-layer wire resonance.

---

## 6. State machine

`WindingEngine` transitions through the following states (stored in `SharedState.engine_state`):

```
         submit_program()
IDLE ─────────────────────► HOMING
  ▲                            │  homing success
  │                            ▼
  │                         RUNNING ──── stop/fault ──► FAULT
  │                            │                          │
  │         program complete   │                  clear_fault()
  └────────────────────────────┘◄─────────────────────────┘
```

| State | Description |
|-------|-------------|
| `IDLE` | No program running. Ready to accept commands. |
| `HOMING` | Lateral homing sequence in progress. |
| `RUNNING` | Executing winding layers. |
| `PAUSED` | Adaptive session paused between chunks. |
| `STOPPING` | Controlled stop requested; finishing current move. |
| `FAULT` | Error or endstop triggered. Requires `winding.clear_fault` to recover. |

---

## 7. Classic vs adaptive winding

### Classic — `WindingEngine`

Used via `winding.submit_program`. Parameters are fixed at the start.

```
submit_program(program, params)
  │
  ├─ [if home_before_start] home lateral → reposition to window_low
  │
  └─ for layer_index in range(program.num_layers):
       WoundMove(spindle + lateral, full traversal, scatter)
       → enqueue to MoveQueue
       → wait for drain
       → reverse direction for next layer
```

### Adaptive — `AdaptiveWindingService`

Used via `winding.start_session`. Supports live parameter updates.

```
start_session(program, params)
  │
  └─ worker thread:
       resolve window via program.effective_window()
       while turns_remaining > 0 and not stopped:
         chunk = plan_next_chunk(runtime)   ← ~250 ms of motion
         enqueue AdaptiveWindingMove
         wait for drain
         update progress
         [apply pending window / RPM changes]
```

The planner brakes the spindle to zero at each traverse edge so the reversal time
is deterministic rather than implicit.

---

## 8. Program library

Saved programs are stored on the Raspberry Pi filesystem under `data/programs/`.
Each file is a JSON object produced by `WindingProgram.to_dict()`.

```json
{
  "program_id": "strat-bridge-42awg",
  "name": "Stratocaster bridge — 42 AWG",
  "revision": 1,
  "target_turns": 8000,
  "layer_pitch_mm": 0.065,
  "wire_diameter_mm": 0.063,
  "bobbin_width_mm": 13.2,
  "flatwork_thickness_mm": 3.2,
  "scatter_amplitude_mm": 0.03,
  "scatter_freq1": 1.0,
  "scatter_freq2": 1.618
}
```

`ProgramStore` provides CRUD with stable `program_id` slugs, monotonic `revision`
numbers, and `created_at` / `updated_at` timestamps.

---

## 9. Data directory layout

```
~/data/                         Default root (configurable via WinderApplication)
  config.json                   AppConfiguration — machine-level parameters
  programs/
    <program_id>.json           One file per saved WindingProgram
```

The data directory is created automatically on first run.

---

## 10. Thread model

| Thread | Owner | Role |
|--------|-------|------|
| Main | `winding_main.py` | Blocking sleep loop; handles signals. |
| MoveQueue | `MoveQueue` | Serializes and streams moves to the firmware. |
| WindingEngine | `WindingEngine` | Executes classic programs in a daemon thread. |
| AdaptiveWinding | `AdaptiveWindingService` | Executes adaptive sessions in a daemon thread. |
| JsonRpcServer | `JsonRpcServer` | Accepts and dispatches incoming RPC connections. |

`SharedState` is protected by a single `threading.RLock` (reentrant so the engine
can call multiple setters without risk of deadlock).

---

## 11. Cross-reference to other docs

| Topic | Document |
|-------|----------|
| SPI frame format, message types, ACK semantics | [spi_protocol.md](spi_protocol.md) |
| Sequence spaces, wrap-around, deduplication | [sequencing.md](sequencing.md) |
| ESP32 RMT ring, coast mode, key constants | [stepper_engine.md](stepper_engine.md) |
