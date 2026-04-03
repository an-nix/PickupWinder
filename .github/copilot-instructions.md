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
|              IpcChannel (linux/src/)             |
+----------------------+--------------------------+
|  PRU0 - Spindle      |  PRU1 - Lateral+Sensors  |
|  200 MHz determinism |  200 MHz determinism     |
|  STEP/DIR/EN GPIO    |  STEP/DIR/EN GPIO        |
|  Accel/decel ramp    |  Accel/decel ramp        |
|  Compensation RPM    |  Home sensor NO+NC       |
+----------------------+--------------------------+
```

Full architecture: see `doc/beaglebone_architecture.md`.

### 2.1 Linux Modules

| Module               | File(s)                              | Responsibility |
|----------------------|--------------------------------------|----------------|
| `IpcChannel`         | `port/beaglebone/linux/src/IpcChannel.cpp`    | PRU shared RAM map |
| `StepperPRU`         | `port/beaglebone/linux/src/StepperPRU.cpp`    | Spindle API -> IPC |
| `LateralPRU`         | `port/beaglebone/linux/src/LateralPRU.cpp`    | Lateral API -> IPC |
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
| `pru/include/pru_ipc.h`               | IPC shared memory layout (C only) |
| `pru/include/pru_ramp.h`              | Fixed-point ramp engine (inline) |
| `pru/pru0_spindle/main.c`             | Spindle STEP generation + compensation |
| `pru/pru1_lateral/main.c`             | Lateral STEP + home sensor + reversal |

### 2.3 Key Headers

| Header                        | Content |
|-------------------------------|---------|
| `pru/include/pru_ipc.h`      | pru_cmd_t, pru_axis_telem_t, pru_sync_t, memory offsets |
| `pru/include/pru_ramp.h`     | axis_state_t, ramp_tick(), hz_to_halfperiod() |
| `linux/include/IpcChannel.h` | IpcChannel class - shared RAM open/map/read/write |
| `linux/include/StepperPRU.h` | StepperPRU - spindle API |
| `linux/include/LateralPRU.h` | LateralPRU - lateral API |
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
- **NEVER** use division in the inner PRU step loop (precompute periods).
- Emergency stop (`CMD_EMERGENCY_STOP`) clears all STEP outputs in <= 2 PRU instructions.
- All PRU shared memory accesses must use `volatile` pointers.
- Shared memory structs must be `__attribute__((packed, aligned(4)))`.

### 3.2 Linux Daemon Safety

- **NEVER** use heap allocation in the hot control loop or telemetry path.
  All runtime buffers are stack-local with bounded sizes or static.
- **NEVER** call `serializeJsonPretty()`  compact JSON only.
- Control loop tick: <= 10 ms. Log `control_worst_loop_us` every 5 s.
- Ring buffer full: drop + log error. Never block the control loop.

### 3.3 IPC Protocol

- Commands go through `IpcChannel::sendCommand()` -> ring buffer (32 slots).
- `pru_ipc.h` is C-compatible: included from both PRU C and Linux C++.
- Do not poll telemetry at step frequency; read at control loop cadence (<= 10 ms).

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

### 3.6 Sensor Safety (PRU1)

- Home sensor: NO=LOW + NC=HIGH -> home. NO=HIGH + NC=HIGH -> FAULT.
- Debounce: 3 consecutive stable reads at 10 us interval.
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
- Reversal compensation applied in PRU0 by reading `pru_sync.lateral_hz`.
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
  5. Ramp engine math (`test_pru_ramp` - planned)
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

---

## 8. Winding Uniformity & Spindle-Lateral Synchronization (CRITICAL)

### Core Principle

```
Ratio = Spindle_Hz / Lateral_Hz = CONSTANT at all times
```

On BBB, compensation runs in PRU0 at every ramp tick (1 ms):

```c
uint32_t lat = pru_sync->lateral_hz;
if (lat >= LAT_MIN_HZ && nominal_lat_hz > 0) {
    spindle_state.target_hz =
        (uint64_t)nominal_spindle_hz * lat / nominal_lat_hz;
    spindle_state.target_hz =
        clamp32(spindle_state.target_hz, SPEED_HZ_MIN, SPEED_HZ_MAX);
}
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
  pru/include/              <- pru_ipc.h, pru_ramp.h
  pru/pru0_spindle/         <- PRU0 firmware (spindle)
  pru/pru1_lateral/         <- PRU1 firmware (lateral + sensors)
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
