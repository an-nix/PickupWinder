# Copilot Instructions  PickupWinder (BeagleBone Black)

> Branch `beagle`  migration vers BeagleBone Black (AM335x) + PRU.
> Sources ESP32 originales dans `archive/original_esp32/` (référence uniquement).

---

## 1. Project Identity & Goals

- **Purpose**: Automated/assisted guitar pickup coil winding  precise lateral
  guide traversal, real-time speed control, recipe persistence.
- **Hardware**: BeagleBone Black (AM335x), two A4988/DRV8825 stepper drivers
  (spindle + lateral), rotary encoder, analog potentiometer, footswitch,
  dual-contact home sensor, WebSocket UI.
- **Core constraint**: Firmware pilots physical motors under wire tension.
  Correctness and determinism always outweigh elegance.

---

## 2. Architecture Overview

Two layers:

```
+-------------------------------------------------+
|         Linux userspace  (ARM Cortex-A8)        |
|  WinderApp . Session . Recipe . WebUI . Diag    |
|  StepperPRU . LateralPRU  (IPC wrappers)        |
|  MoveQueue (pre-computes move triples host-side) |
|              IpcChannel (linux/src/)             |
+----------------------+--------------------------+
|  PRU0 - Motor RT IO          |  PRU1 - Orchestration/Supervision |
|  200 MHz - IEP owner         |  200 MHz - IEP reader             |
|  Spindle + Lateral consumers |  Host-link + low-level supervisor |
|  STEP/DIR/EN + Endstops      |  No motor GPIO ownership          |
|  Edge timing IEP             |  Shared-memory sync only          |
|  No ramp on PRU              |  No move-ring consumption         |
+----------------------+--------------------------+
```

Full architecture: see `doc/beaglebone_architecture.md`.

### 2.1 Linux Modules

| Module               | File(s)                              | Responsibility |
|----------------------|--------------------------------------|----------------|
| `IpcChannel`         | `port/beaglebone/linux/src/IpcChannel.cpp`    | PRU shared RAM map; sendMove(), moveQueueFreeSlots() |
| `MoveQueue`          | `port/beaglebone/linux/src/MoveQueue.cpp`     | Host-side Klipper move planner (ramp math) |
| `StepperPRU`         | `port/beaglebone/linux/src/StepperPRU.cpp`    | Spindle API + tick() for ring refill |
| `LateralPRU`         | `port/beaglebone/linux/src/LateralPRU.cpp`    | Lateral API + _pushTraverse() |
| `WinderApp.Core`     | `src/WinderApp.Core.cpp`             | State machine |
| `WinderApp.Commands` | `src/WinderApp.Commands.cpp`         | Command dispatch |
| `WinderApp.Geometry` | `src/WinderApp.GeometryPattern.cpp`  | Geometry + pattern |
| `WinderApp.Telemetry`| `src/WinderApp.Telemetry.cpp`        | getStatus() builder |
| `WinderApp.Recipe`   | `src/WinderApp.Recipe.cpp`           | Recipe helpers |
| `CommandController`  | `src/CommandController.cpp`          | Queue-based bridge |
| `CommandRegistry`    | `src/CommandRegistry.cpp`            | Catalog+validation |
| `SessionController`  | `src/SessionController.cpp`          | Arbitration |
| `WebInterface`       | `src/WebInterface.cpp`               | HTTP+WebSocket |
| `WindingPattern`     | `src/WindingPattern.cpp`             | STRAIGHT/SCATTER/HUMAN |
| `WindingRecipeStore` | `src/WindingRecipeStore.cpp`         | JSON persistence |
| `Diag`               | `src/Diag.cpp`                       | Lightweight logger |

### 2.2 PRU Firmware

| File                                   | Responsibility |
|----------------------------------------|----------------|
| `pru/include/pru_ipc.h`               | IPC shared memory layout; pru_move_t; 6 CMD opcodes; fault flags |
| `pru/include/pru_stepper.h`           | IEP timer macros; stepper_t (with underrun); move_ring_t; step engine (inline, Klipper add pre-apply) |
| `pru/pru0_motor_control/main.c`       | Dual-motor IEP step generation + endstops (authoritative) |
| `pru/pru1_orchestration/main.c`       | PRU1 orchestration/supervision (no motor control) |

### 2.3 Key Headers

| Header                        | Content |
|-------------------------------|---------|
| `pru/include/pru_ipc.h`      | pru_cmd_t, pru_move_t, pru_axis_telem_t, pru_sync_t, memory offsets |
| `pru/include/pru_stepper.h`  | stepper_t (with underrun flag), move_ring_t, IEP_NOW(), stepper_edge(), stepper_try_load_next() (with Klipper add pre-apply) |
| `linux/include/IpcChannel.h` | IpcChannel class - sendMove(), moveQueueFreeSlots(), read/write |
| `linux/include/MoveQueue.h`  | MoveQueue - pushAccelSegment(), pushConstantMs(), pushDecelToStop() |
| `linux/include/StepperPRU.h` | StepperPRU - spindle API + tick() |
| `linux/include/LateralPRU.h` | LateralPRU - lateral API + _pushTraverse() |
| `include/Types.h`             | CommandEntry, WinderStatus, WindingState |
| `include/Protocol.h`          | Version constants |
| `include/CommandRegistry.h`   | CommandId enum, registry API |
| `include/WindingGeometry.h`   | Bobbin geometry, presets |
| `include/WindingPattern.h`    | WindingRecipe, TraversePlan, WindingPatternPlanner |
| `include/SessionController.h` | TickInput, SessionState, ControlIntent |

---

## 3. Hard Rules (Must-follow)

### 3.1 PRU Safety

- **NEVER** use dynamic memory, floats, or OS calls in PRU firmware.
- **NEVER** use division in the inner PRU step loop (intervals pre-computed host-side).
- The PRU step loop polls `IEP_NOW()` continuously — no `__delay_cycles` anywhere.
- Emergency stop (`CMD_EMERGENCY_STOP`) clears all STEP outputs and flushes ring in <= 2 PRU instructions.
- `CMD_QUEUE_FLUSH` empties the move ring without emergency stop (for speed transitions).
- PRU0 owns and initializes the IEP timer; PRU1 reads only — never resets it.
- All motor-related real-time code must target **PRU0**.
- PRU1 is reserved for orchestration, supervision, and host communication.
- All PRU shared memory accesses must use `volatile` pointers.
- Shared memory structs must be `__attribute__((packed, aligned(4)))`.

### 3.1.1 Canonical pin layout (MUST NOT change implicitly)

Motor A (Group 1):
- `P9_25` → `EN_A` (`PRU0 R30[7]`, active-low)
- `P9_27` → `DIR_A` (`PRU0 R30[5]`)
- `P9_29` → `STEP_A` (`PRU0 R30[1]`) — reserved for future `pru_uart` TX_1

Motor B (Group 2):
- `P9_28` → `EN_B` (`PRU0 R30[3]`, active-low)
- `P9_31` → `DIR_B` (`PRU0 R30[0]`) — reserved for future `pru_uart` TX_2
- `P9_42` → `STEP_B` (`PRU0 R30[4]`)

Endstops (PRU0 inputs):
- `P8_15` → `ENDSTOP_1` (`PRU0 R31[15]`)
- `P8_16` → `ENDSTOP_2` (`PRU0 R31[14]`)

Additional board IO:
- Encoder1: `P8_11` (A), `P8_12` (B)
- Encoder2: `P8_33` (A), `P8_35` (B)
- HX711: `P9_12` (SCK), `P9_14` (DOUT)
- Footswitch: `P9_23`

Rules:
- Do not remap these pins unless the user explicitly requests it.
- Keep `P9_29` and `P9_31` reserved for future TMC2209 UART migration.
- Any PRU pin change must update firmware + DTS + documentation in one commit.

### 3.2 Linux Daemon Safety

- **NEVER** use heap allocation in the hot control loop or telemetry path.
  All runtime buffers are stack-local with bounded sizes or static.
- **NEVER** call `serializeJsonPretty()`  compact JSON only.
- Control loop tick: <= 10 ms. Log `control_worst_loop_us` every 5 s.
- Ring buffer full: drop + log error. Never block the control loop.

### 3.3 IPC Protocol

- Move triples `(interval, count, add, direction)` pushed via `IpcChannel::sendMove(axis, ...)`.
- Lifecycle commands (ENABLE, EMERGENCY_STOP, HOME_START, QUEUE_FLUSH, RESET_POSITION) via `sendCommand()`.
- `pru_ipc.h` is C-compatible: included from both PRU C and Linux C++.
- Do not poll telemetry at step frequency; read at control loop cadence (<= 10 ms).
- Move ring: 128 slots per axis; `moveQueueFreeSlots()` to check before pushing.
- `StepperPRU::tick()` must be called from the control loop to refill the spindle ring.

### 3.4 Command Protocol (unchanged)

- All commands validated through `CommandRegistry` before reaching the domain.
- Aliases normalized in `normalizeKeyImpl()`.
- Value validation in `validateValue()`.
- Never bypass `CommandController::onTransportCommand()`.

### 3.5 Recipe & Persistence

- Recipe format version: `PICKUP_RECIPE_FORMAT_VERSION` in `Protocol.h`.
- `fromJson()` rejects recipes with `version > PICKUP_RECIPE_FORMAT_VERSION`.
- Float fields clamped after parsing. Bump version only for schema changes.
- Storage: JSON files on BBB eMMC (replaces ESP32 NVS).

### 3.6 Sensor Safety

- Home sensor: NO=LOW + NC=HIGH -> home. NO=HIGH + NC=HIGH -> FAULT.
- Debounce: IEP-based 500 µs window (100,000 IEP cycles).
- All `LateralPRU` methods guard `if (!_channel) return;`.

### 3.7 Build

- PRU: `pru-gcc` or TI `clpru`. Makefile: `port/beaglebone/pru/Makefile`.
- Linux: CMake >= 3.16. `port/beaglebone/linux/CMakeLists.txt`.
- Unit tests: native x86-64 host, **gtest** (replaces Unity/PlatformIO).
  Tests in `test/test_*/` must compile and pass on host without hardware.
- CI: `.github/workflows/beaglebone-build.yml`.

 - PRU cross-compile: prefer using `crosstool-NG` to build the PRU-capable
   cross-toolchain used to produce a `pru-gcc` toolchain. Ensure the PRU
   toolchain is available on the build host or installed/cached by CI.

### 3.8 Logging

- Use `Diag::info()` / `warn()` / `error()` / `infof()` / `warnf()` / `errorf()`.
- High-frequency logs wrapped in `#if DIAG_VERBOSE`.
- Never log credentials, recipe blobs, or raw IPC buffers in production.

---

## 4. Domain Knowledge

### 4.1 Winding State Machine

```
IDLE --(start)--> PAUSED (positioning)
                    |
                    +--(positioned+pot/resume)--> WINDING
                    |                               |
                    |                         (pot=0)--> PAUSED
                    |                  (turns>=target)--> TARGET_REACHED
                    |                               |
                    +----------(stop)---------------> IDLE

IDLE --(rodage)--> RODAGE --(rodage_stop/complete)--> IDLE
```

### 4.2 Session Arbitration (Last-Event-Wins)

- Sources: Pot (analog), IHM (WebSocket/UART), Footswitch.
- IDLE: only IHM can produce a Start intent.
- Pot-lock: when another source takes control while pot > 0, pot locked until it returns to 0.
- Pot -> RunMode::Pot (proportional). UI/Footswitch -> RunMode::Max.

### 4.3 Lateral Traverse Synchronization

- Lateral speed: `effWidthMm x windingHz / (tpp x STEPS_PER_REV)`.
- Scaled by pattern `speedScale` (0.55-1.60).
- Compensation applied **host-side**: WinderApp calls `setSpeedHz(compensatedHz)` each tick; `StepperPRU::tick()` detects change and requeues via MoveQueue.
- One-shot flags: `stop_next_high`, `stop_next_low`, `armPauseOnNextReversal`.

### 4.4 Winding Patterns

- **STRAIGHT**: constant traverse.
- **SCATTER**: per-layer random TPP + speed jitter.
- **HUMAN**: smooth Perlin-like noise on traverse and speed.
- All patterns deterministic per `seed`.

### 4.5 Hardware Constants

| Parameter         | Value           | Notes |
|-------------------|-----------------|-------|
| Spindle steps/rev | 6400            | 200-step x 32 ustep |
| Speed range       | ~1067 - 160000 Hz | ~10 - 1500 RPM |
| Lateral steps/mm  | 3072            | 96-step x 32 ustep, M6 1mm pitch |
| PRU clock         | 200 MHz         | 5 ns/cycle, 1 cycle/instruction |

---

## 5. Code Style Conventions

- **Language**: C++ for Linux, pure C for PRU. No STL containers, no exceptions in hot paths.
- **Naming**: PascalCase classes/enums, camelCase methods/members, _prefix private, UPPER_SNAKE macros.
- **Fixed buffers**: `char buf[N]` + `snprintf(buf, sizeof(buf), ...)`. Never `sprintf`.
- **String comparison**: `strcmp()` / `strncmp()` on `const char*`.
- **Numeric parsing**: `strtol()` / `strtof()` + `constrain()`.
- **PRU integers only**: speeds in Hz (uint32_t), positions in steps (int32_t). No floats.
- **Comments**: French acceptable in log messages. English for code comments.
- **Include order**: matching .h first, then system headers, then project headers.

---

## 6. Testing & Validation

- Unit tests: `test/test_<suite>/test_main.cpp` (gtest, host-compilable).
- Coverage priorities:
  1. Command normalization + validation (`test_command_registry`)
  2. Geometry math (`test_winding_geometry`)
  3. Recipe version gating (`test_recipe_format`)
  4. Session arbitration (`test_session_rules`)
  5. Move underrun detection (`test_pru_underrun` - planned)
- Add/update test when adding a command or changing validation.

---

## 7. Common Pitfalls

| Pitfall | Why it matters |
|---------|----------------|
| Float or division in PRU loop | PRU has no FPU; crashes or extreme slowdown |
| OS call in PRU firmware | PRU has no OS; link error or crash |
| Missing `volatile` on shared RAM pointer | Compiler may cache stale value |
| Full IPC ring buffer -> blocking | Breaks control loop determinism |
| `sprintf()` without size | Buffer overflow |
| `atof`/`strtol` without `constrain` | Unbounded value reaches motor parameters |
| No `if (!_channel)` guard | Null deref if IpcChannel failed to init |
| Logging credentials / blobs | Security violation |
| Mismatched `pru_cmd_t` size PRU vs Linux | Silent corrupt IPC messages |
| Integer ratio for effective winding width | Wires stack on top of each other |
| Missing add pre-apply at move boundary | Ramp timing skew vs Klipper (stepper_try_load_next) |
| pushDecelToStop for speed decrease | Decels to Hz=1 instead of target; use pushAccelSegment |
| Ignoring FAULT_MOVE_UNDERRUN | Ring drained while running; host didn't push fast enough |

---

## 8. Winding Uniformity & Spindle-Lateral Synchronization (CRITICAL)

### Core Principle

```
Ratio = Spindle_Hz / Lateral_Hz = CONSTANT at all times
```

On BBB, compensation runs **host-side** in `StepperPRU::tick()` every control loop tick:

```cpp
uint32_t compHz = StepperPRU::calculateCompensatedSpindleHz(
    nominalSpindleHz, currentLatHz, nominalLatHz);
stepper.setSpeedHz(compHz);   /* sets _speedChanged flag */
/* tick() detects _speedChanged -> CMD_QUEUE_FLUSH + repush via MoveQueue */
```

### Expected results

- Nominal: both Hz constant, ratio stable +-5%.
- Decel (reversal): spindle Hz decreases proportionally with lateral.
- Accel: spindle Hz increases proportionally.
- Visual: regular cross-hatch (losange) pattern across full bobbin width.


### Winding ratio must be non-integer

`effWidthMm / wire_diameter_mm` must not be a whole number (avoid layer stacking).
Verify after every geometry change.

---

## 9. Quick Reference: Adding a New Feature

1. **New command**: add `CommandId` + `CommandDefinition` in `CommandRegistry.cpp`,
   handle in `WinderApp.Commands.cpp`, add unit test.
2. **New geometry parameter**: add to `WindingGeometry`, serialize in
   `WindingRecipeStore`, add to `WinderStatus`, update `WebInterface::sendUpdate()`.
3. **New recipe field**: add to `WindingRecipe`, update toJson/fromJson,
   bump `PICKUP_RECIPE_FORMAT_VERSION` if breaking.
4. **New PRU command**: add `CMD_*` in `pru_ipc.h`, handle in PRU `main.c`,
   add `IpcChannel` helper, update `StepperPRU`/`LateralPRU`.
5. **New REST endpoint**: add `_server.on(...)` in `WebInterface::begin()`.

---

## 10. File Organization

```
port/beaglebone/            <- BBB port (active development)
  pru/include/              <- pru_ipc.h, pru_stepper.h
  pru/pru0_motor_control/   <- PRU0 firmware (motor control + endstops)
  pru/pru1_orchestration/   <- PRU1 firmware (orchestration + supervision)
  pru/Makefile
  linux/include/            <- IpcChannel.h, StepperPRU.h, LateralPRU.h
  linux/src/                <- IpcChannel.cpp, StepperPRU.cpp, LateralPRU.cpp
  linux/CMakeLists.txt
  dts/                      <- Device-tree overlay
  scripts/                  <- load_pru.sh, deploy.sh
src/                        <- Linux application logic (WinderApp.*, Session, ...)
include/                    <- Application headers
data/                       <- Web assets
test/test_*/                <- Unit tests (gtest, host-compilable)
doc/                        <- Architecture docs, checklists
archive/original_esp32/     <- ESP32 sources (reference only, do NOT modify)
```

Do not modify `archive/original_esp32/`.
Do not create new source files without clear domain justification.
Keep the WinderApp split (Core, Commands, GeometryPattern, Telemetry, Recipe).
