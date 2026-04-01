# Copilot Instructions — PickupWinder Firmware

> ESP32 guitar pickup winding machine controller.
> Dual-core FreeRTOS, PlatformIO + Arduino framework.

---

## 1. Project Identity & Goals

- **Purpose**: Automated/assisted guitar pickup coil winding with precise
  lateral guide traversal, real-time speed control, and recipe persistence.
- **Hardware**: ESP32-WROOM-32, two A4988/DRV8825 stepper drivers (spindle +
  lateral), rotary encoder, analog potentiometer, footswitch, dual-contact
  home sensor, UART2 display link, WiFi + WebSocket UI.
- **Core constraint**: The firmware pilots physical motors and wire under
  tension. Any software fault may damage the pickup being wound. Correctness
  and determinism always outweigh feature elegance.

## 2. Architecture Overview

### 2.1 FreeRTOS Tasks

| Task | Core | Priority | Period | Stack | Role |
|------|------|----------|--------|-------|------|
| `controlTask` | 1 | 5 | 10 ms | 8 192 B | Sensors → commands → session → motor |
| `commsTask` | 0 | 2 | 20 ms | 8 192 B | UART poll, WS/UART status broadcast |

- **Inter-task comms**:
  - Commands flow comms→control via a FreeRTOS queue (`CommandController`).
  - Status flows control→comms via a mutex-protected `WinderStatus` snapshot.
  - No other shared mutable state exists between tasks.

### 2.2 Source File Map

| File(s) | Responsibility |
|---------|----------------|
| `main.cpp` | Task creation, RTOS metrics, boot diagnostics |
| `ControlHardware.cpp` | ISR-based encoder, filtered pot, footswitch debounce |
| `SessionController.cpp` | Last-event-wins intent arbitration (pot/UI/footswitch), speed output |
| `CommandController.cpp` | Queue-based command bridge, schema validation via `CommandRegistry` |
| `CommandRegistry.cpp` | Canonical command catalog, alias normalization, value validation, `/capabilities.json` |
| `WinderApp.Core.cpp` | State machine (IDLE/WINDING/PAUSED/TARGET_REACHED/RODAGE), lateral events, motor control |
| `WinderApp.Commands.cpp` | Command dispatch & immediate handlers (start/stop/pause/resume/…) |
| `WinderApp.GeometryPattern.cpp` | Geometry (trims, presets, dimensions), pattern (style/seed/jitter) |
| `WinderApp.Telemetry.cpp` | `getStatus()` builder, encoder trim during pause |
| `WinderApp.Recipe.cpp` | Recipe apply/capture/save helpers |
| `StepperController.cpp` | Non-blocking spindle stepper (deferred enable, soft-start) |
| `LateralController.cpp` | 10-state lateral axis: homing, positioning, synchronized traversal, overrun correction |
| `WindingPattern.cpp` | Deterministic pseudo-random traverse variation (STRAIGHT/SCATTER/HUMAN) |
| `WindingRecipeStore.cpp` | NVS recipe persistence with `StaticJsonDocument<1024>`, version gating |
| `WebInterface.cpp` | HTTP (embedded assets) + WebSocket server, REST endpoints |
| `WifiManager.cpp` | WiFi credential management (NVS + compile-time fallback) |
| `LinkSerial.cpp` | UART2 protocol (banner + pipe-separated status + `cmd:val\n` RX) |
| `Diag.cpp` | Lightweight logger: Serial + up to 4 registered sinks, fixed 512-byte format buffer |
| `NvsCompat.cpp` | C-string NVS wrapper (no Arduino String) |

### 2.3 Key Headers

| Header | Content |
|--------|---------|
| `Config.h` | Pin assignments, motor tuning, pot/encoder params, WiFi, winding defaults |
| `Types.h` | `CommandEntry` (cmd[32]+val[48]), `WinderStatus`, `WindingState` enum |
| `Protocol.h` | Version constants: WS, UART, Capabilities, Recipe format |
| `CommandRegistry.h` | `CommandId` enum, `CommandDefinition`, `CommandValueKind`, registry API |
| `WindingGeometry.h` | Bobbin geometry math, presets, turns-per-pass |
| `WindingPattern.h` | `WindingRecipe`, `TraversePlan`, `WindingPatternPlanner`, `WindingEndPos` |
| `SessionController.h` | `TickInput` (MAX_CMDS=16), `SessionState`, `ControlIntent`, `InputSource` |

## 3. Hard Rules (Must-follow)

### 3.1 Real-Time Safety

- **NEVER** call `delay()` inside `controlTask`, `commsTask`, or any function
  reachable from them at runtime. The only acceptable `delay()` locations are
  `setup()` / `begin()` one-shots before tasks start.
- **NEVER** use heap allocation (`new`, `malloc`, `String`, `std::string`,
  `std::vector`, `DynamicJsonDocument`) in any runtime path. All buffers must be
  stack-local with bounded sizes or static.
- **NEVER** call `serializeJsonPretty()` in firmware — compact JSON only.
- Control tick must complete well under 10 ms worst-case. Log
  `control_worst_loop_us` every 5 s to catch regressions.

### 3.2 Memory / Stack

- `StaticJsonDocument` sizes must be computed from payload, not guessed.
  Current sizing: recipe = 1024 B, WS command parse = 192 B.
- Recipe `load()`/`save()` use a 2048-byte stack buffer — these functions run
  only at startup (`begin()`) or from `controlTask` context. Never call them
  from the ISR or comms task.
- WebSocket status `sendUpdate()` uses an 832-byte stack buffer on commsTask
  (8 KB stack). Keep total commsTask stack pressure below ~3 KB.
- Diag format buffer is 512 bytes. Keep log format strings short; if truncation
  risk exists, split into multiple calls.

### 3.3 Logging

- Use `Diag::info()` / `warn()` / `error()` and `infof()` / `warnf()` / `errorf()`.
- High-frequency logs (per-tick, per-sample) **must** be wrapped in
  `#if DIAG_VERBOSE` (compile-time) or `if (DIAG_VERBOSE)` (runtime check for
  state-change guards).
- Production build (`esp32dev`) compiles with `DIAG_VERBOSE=0`.
- Lab build (`esp32dev-lab`) compiles with `DIAG_VERBOSE=1`, `-O0`.
- Never log WiFi credentials, NVS blobs, or recipe JSON in production.

### 3.4 Command Protocol

- All commands go through `CommandRegistry` before reaching the domain.
- To add a command: add entry to `kCommands[]` in `CommandRegistry.cpp`, add
  `CommandId` enum value, handle in the appropriate WinderApp handler.
- Aliases (`max-rpm` → `max_rpm`, `windows_shift` → `window_shift`) are in
  `normalizeKeyImpl()`.
- Value validation (type, range, enum set) happens in `validateValue()`.
- Never bypass `CommandController::onTransportCommand()` for transport input.

### 3.5 Recipe & NVS

- Recipe format version is `PICKUP_RECIPE_FORMAT_VERSION` in `Protocol.h`.
- `fromJson()` rejects recipes with `version > PICKUP_RECIPE_FORMAT_VERSION`.
- All float fields are clamped to safe ranges after parsing.
- Bump the version only for actual schema changes (new fields, removed fields,
  changed semantics).

### 3.6 Lateral Controller Safety

- All public methods on `LateralController` must guard `if (!_stepper) return;`
  to survive a failed `begin()`.
- The home sensor uses a dual-contact (NO+NC) validation to detect wire faults.
- Overrun correction in `updateWinding()` snaps the carriage to the moved bound
  and reverses immediately.

### 3.7 Build & CI

- Two PlatformIO environments: `esp32dev` (production), `esp32dev-lab` (debug).
- CI builds both environments + compiles unit tests.
- Unit tests use the Unity framework and live under `test/test_*/test_main.cpp`.
- The build must compile with zero warnings at `-Os -flto` (production) and
  `-O0` (lab).

## 4. Domain Knowledge

### 4.1 Winding State Machine

```
IDLE ──(start)──► PAUSED (positioning to low bound)
                    │
                    ├──(positioned + pot/resume)──► WINDING
                    │                                  │
                    │                            (pot→0)├──► PAUSED
                    │                     (turns≥target)├──► TARGET_REACHED
                    │                                  │
                    └──────────(stop)──────────────────► IDLE

IDLE ──(rodage)──► RODAGE ──(rodage_stop / complete)──► IDLE
```

### 4.2 Session Arbitration (Last-Event-Wins)

- Three input sources: **Pot** (analog), **IHM** (WebSocket/UART UI), **Footswitch**.
- While IDLE, only IHM can produce a `Start` intent (prevents accidental pot/footswitch starts).
- When another source takes control while pot is above zero, pot is locked until
  it returns to zero ("re-arm" mechanism).
- Pot → `RunMode::Pot` (proportional), UI/Footswitch → `RunMode::Max` (ceiling speed).

### 4.3 Lateral Traverse Synchronization

- Lateral speed is computed from: `effWidthMm × windingHz / (tpp × STEPS_PER_REV)`.
- Speed is scaled by a pattern-computed `speedScale` (0.55–1.60).
- Reversal window triggers spindle slow-down via real-time compensation (proportional to actual carriage velocity).
- One-shot stop flags: `stop_next_high`, `stop_next_low`, `armPauseOnNextReversal`.

### 4.4 Winding Patterns

- **STRAIGHT**: constant traverse, no variation.
- **SCATTER**: per-layer random TPP jitter + speed jitter, stepped.
- **HUMAN**: smooth Perlin-like noise on traverse and speed.
- All patterns are deterministic per `seed`.

### 4.5 Hardware Constants (from Config.h)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Spindle steps/rev | 6400 | 200-step motor × 32 µstep |
| Spindle speed range | 9600/9 – 160000 Hz | ~90 – 1500 RPM |
| Lateral steps/mm | 3072 | 96-step motor × 32 µstep, M6 1mm pitch |
| Pot filter | 32 samples × 20 ms = 640 ms | Circular average + exponential curve |
| Encoder debounce | 1000 µs | ISR-level, IRAM_ATTR |
| Home sensor | NO (GPIO 23) + NC (GPIO 22) | Dual-contact fault detection |

## 5. Code Style Conventions

- **Language**: C++ (Arduino/ESP-IDF subset). No STL containers, no exceptions,
  no RTTI.
- **Naming**: `PascalCase` for classes/enums, `camelCase` for methods/variables,
  `_prefixed` for private members, `UPPER_SNAKE` for macros/constants.
- **Fixed buffers**: always use `char buf[N]` with `snprintf(buf, sizeof(buf), …)`.
  Never use `sprintf()`.
- **String comparison**: `strcmp()` / `strncmp()` on `const char*`. No `String`.
- **Numeric parsing**: `strtol()` / `strtof()` / `atof()` with post-parse
  `constrain()`.
- **Comments**: French in user-facing log messages is acceptable (existing
  convention). English for code comments and documentation.
- **Include order**: matching `.h` first, then `<Arduino.h>`, then library
  headers, then project headers alphabetically.

## 6. Testing & Validation

- Unit tests live in `test/test_<suite>/test_main.cpp` using Unity.
- Test coverage priorities:
  1. Command normalization and validation (`test_command_registry`)
  2. Geometry math (`test_winding_geometry`)
  3. Recipe version gating (`test_recipe_format`)
  4. Session start/pause/stop arbitration (`test_session_rules` — planned)
- When adding a new command or changing validation rules, add or update the
  corresponding test.

## 7. Common Pitfalls to Avoid

| Pitfall | Why it matters |
|---------|----------------|
| `delay()` in task body | Blocks the entire core, breaks timing guarantees |
| `String` / `std::string` | Heap fragmentation over hours of continuous winding |
| `DynamicJsonDocument` | Same heap concern; always use `StaticJsonDocument<N>` |
| `serializeJsonPretty` | Wastes RAM and flash for no embedded benefit |
| Shared mutable state without mutex | Race condition between Core 0 and Core 1 |
| Calling recipe save from commsTask | 2 KB stack buffer on 8 KB stack = overflow risk |
| Forgetting `if (!_stepper)` guard | Null dereference crash if stepper init failed |
| Logging passwords/credentials | Security violation; use flags like `hasNvs`/`compileTimeDefault` |
| `sprintf()` without size limit | Buffer overflow; always `snprintf()` |
| atof/strtol without constrain | Unbounded user input reaching motor parameters |

## 8. Winding Uniformity & Carriage-Spindle Synchronization (CRITICAL)

### 8.1 Core Concept: Constant Turns-per-Millimeter Ratio

The fundamental principle for uniform winding is that the number of spindle turns 
deposited per millimeter of lateral carriage travel must remain **CONSTANT at all times**, 
including during carriage reversal phases.

```
Ratio = Spindle_RPM / Carriage_velocity_mm_per_s = CONSTANT
```

If this ratio varies, wire density varies → bulges or gaps emerge.

**Problem scenario during carriage reversal:**
- Carriage decelerates → velocity decreases
- If spindle RPM remains constant → ratio increases → too much wire deposited 
  over short distance → bulge at edges (or gap at center if spindle also slows)

**Compensation formula:**
```
RPM_spindle_corrected = RPM_nominal × (Carriage_velocity_actual / Carriage_velocity_nominal)
```

### 8.2 Code Audit Checklist

#### 1. Base turns-per-mm ratio calculation
- ✅ Verify nominal carriage velocity is computed as:
  ```
  Carriage_velocity_mm_s = (Spindle_RPM × Wire_diameter_mm) / 60
  ```
- ✅ Verify that the winding/traverse ratio is a **non-integer** (e.g., 3.7, 4.3...) 
  to avoid layer-upon-layer wire stacking and create a uniform cross-hatch pattern.
- ✅ If ratio is integer (3, 4, 5...), wires align exactly on top of each other 
  at every pass → **MUST CORRECT**.

#### 2. Carriage reversal handling
- ✅ Identify all code sections that manage carriage turnaround (end-of-travel, 
  limit sensors, step counting).
- ✅ Check if deceleration/acceleration ramp is applied to carriage.
- ✅ If yes: is spindle speed compensated proportionally?
- ✅ If no: does spindle also slow down during reversal (current problematic behavior)?

#### 3. Dynamic compensation implementation
Implement real-time compensation function:

```cpp
float ratio_nominal = RPM_nominal / Carriage_velocity_nominal;

// Called at every carriage velocity update:
float calculateCompensatedRPM(float carriage_velocity_actual) {
    if (carriage_velocity_actual < MIN_VELOCITY_THRESHOLD) {
        // Avoid division by zero during complete stop
        return RPM_nominal;  // or 0 depending on chosen behavior
    }
    return ratio_nominal * carriage_velocity_actual;
}
```

Must be invoked:
- At every step of carriage deceleration ramp
- At every step of carriage acceleration ramp
- In nominal state (verify ratio remains constant)

#### 4. Update frequency
- Compensation must apply as frequently as possible 
  (ideally at every stepper step or main loop tick).
- No delay or lag between carriage velocity change and spindle RPM adjustment.

#### 5. End-of-travel boundary margins
- System has intentional margin between carriage reversal point and physical 
  bobbin edge. This is **CORRECT by design** — do not modify.
- **BUT**: verify margin is **SYMMETRIC** on both sides (left/right edges). 
  Asymmetric margin shifts winding center → unequal distribution.
- Verify margin is defined in ONE place only (constant or parameter), 
  not duplicated with differing values.
- Effective winding width is:
  ```
  Effective_width = Total_width - (2 × Border_margin)
  ```
  All velocity and ratio calculations must use effective_width, not total_width.

#### 6. Carriage complete stop handling
During brief instant when carriage is at zero velocity (between deceleration 
and re-acceleration):
- **Option A:** Spindle also stops (ratio 0/0 → neutral, but mechanical irregularity)
- **Option B:** Spindle continues at nominal RPM (a few extra turns at reversal point 
  → slight edge bulge, acceptable if duration is very short)
- **Option C:** Carriage ramp so fast no hard stop occurs → **IDEAL**

Verify which option is implemented and consistency with observations.

#### 7. Complete velocity profile logging
- Log carriage velocity AND spindle RPM over one complete pass (forward + reversal).
- Verify graphically that both curves are proportional at all times 
  (same shape, just scaled differently).
- Any deviation = location of poor wire distribution.
- Visualization via:
  - Option A: Lightweight live dashboard on ESP web server (if not too heavy)
  - Option B: Separate Python app polling WebSocket every 100ms for offline curve analysis

---

### 8.3 Expected Results After Correction

- **Nominal regime:** Spindle RPM and carriage velocity constant and proportional
- **Deceleration phase:** Spindle RPM decreases proportionally with carriage velocity
- **Acceleration phase:** Spindle RPM increases proportionally with carriage velocity
- **Ratio stability:** Turns/mm varies ±5% max across entire winding width
- **Visual pattern:** Regular cross-hatch (losange) pattern across full surface 
  (evidence of non-integer traverse ratio properly applied)
- **UI simplification:** Winding style parameters must be minimal to avoid cluttering 
  interface with low-impact parameters.

## 9. Quick Reference: Adding a New Feature

1. **New command**: Add `CommandId`, `CommandDefinition` in `CommandRegistry.cpp`,
   handle in the appropriate `_handle*Command()` method, add unit test.
2. **New geometry parameter**: Add field to `WindingGeometry`, serialize in
   `WindingRecipeStore::toJson`/`fromJson`, add to `WinderStatus`, update
   `WebInterface::sendUpdate()` format string, update `script.js` UI.
3. **New recipe field**: Add to `WindingRecipe`, update toJson/fromJson with
   safe default, bump `PICKUP_RECIPE_FORMAT_VERSION` if breaking.
4. **New REST endpoint**: Add `_server.on(...)` in `WebInterface::begin()`,
   document in `WebInterface.h`.
5. **New diagnostic**: Use `Diag::infof()`, gate with `#if DIAG_VERBOSE` if
   high-frequency.

## 10. File Organization

```
include/          — All .h headers (no .cpp in include/)
src/              — All .cpp source files
data/             — Embedded web assets (HTML/CSS/JS) linked at compile time
test/test_*/      — Unity unit test suites
doc/              — Engineering docs, checklists, benchmarks
.github/workflows — CI configuration
```

Do not create new source files without a clear domain justification.
Keep the WinderApp split as-is; do not re-merge or add more split files
unless the domain genuinely requires a new responsibility boundary.
