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
- `P9_29` → `DIR_A` (`PRU0 R30[1]`)
- `P9_31` → `STEP_A` (`PRU0 R30[0]`)

Motor B (Group 2):
- `P9_41` → `EN_B` (`PRU0 R30[6]`, active-low)
- `P9_28` → `DIR_B` (`PRU0 R30[3]`)
- `P9_30` → `STEP_B` (`PRU0 R30[2]`)

Endstops (PRU1 inputs — sampled by PRU1 and published to PRU0):
- `P8_15` → `ENDSTOP_1` (`PRU1 R31[15]`)
- `P8_16` → `ENDSTOP_2` (`PRU1 R31[14]`)

Additional board IO:
- Encoder1: `P8_11` (A), `P8_12` (B)
- Encoder2: `P8_33` (A), `P8_35` (B)
- HX711: `P9_12` (SCK), `P9_14` (DOUT)
- Footswitch: `P9_23`

Rules:
- Do not remap these pins unless the user explicitly requests it.
- Keep `P9_29` and `P9_31` reserved for future TMC2209 UART migration.
- Any PRU pin change must update firmware + DTS + documentation in one commit.

Selected / Retained pins (final):
- Motor A: `P9_31` (STEP_A), `P9_29` (DIR_A), `P9_25` (EN_A, active-low)
- Motor B: `P9_30` (STEP_B), `P9_28` (DIR_B), `P9_41` (EN_B, active-low)
- Endstops: `P8_15`, `P8_16` (sampled by PRU1 and published to PRU0)
- Encoders (eQEP): `P9_42`/`P9_27` and `P8_33`/`P8_35`
- UART TMC config: `P9_24` (UART1_TXD), `P9_26` (UART1_RXD)

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
| 2-cell `pinctrl-single,pins` on kernel 6.12 | `#pinctrl-cells=<2>` expects 3-cell format; only first pin applied, rest silently skipped |
| `fragment@ {}` syntax in `/plugin/` DTS | Works on older kernels, unreliable on 6.12; use direct `&node {}` syntax |
| `pinctrl-0` on counter/eQEP driver node only | `ti-eqep-cnt` may claim only one pin during probe; prefer `bone-pinmux-helper` if needed |
| `bone-pinmux-helper` on kernel 6.12 | Driver not compiled in kernel 6.12; node appears in sysfs with `waiting_for_supplier` but no `driver/` symlink → pinmux never applied. Use `&gpio1` / `&gpio0` target instead |
| Wrong pad offset for GPIO input pin | P9_23 is NOT `gpmc_ad4` (0x010). Derive offset from the pinctrl debugfs dump: `cat /sys/kernel/debug/pinctrl/44e10800.pinmux-pinctrl-single/pins` — identify the pin by its `gpio-XX-YY` label, then offset = `register_address - 0x44e10800` |

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

---

## 11. Device Tree Overlay Authoring (BBB / kernel 6.12)

### 11.1 pinctrl-single,pins format — CRITICAL

The AM335x pinmux node on Debian 12 / kernel 6.12 declares **`#pinctrl-cells = <2>`**.
This switches `pinctrl-single,pins` to **3-cell format per pin**:

```
<pad_offset   config_flags   mux_mode>
```

The driver writes `config_flags | mux_mode` to the pad register at
`pinmux_base + pad_offset`.

> ⚠️ **Using the old 2-cell format `<offset value>` silently applies only the
> first pin.** The second pin's offset is misinterpreted as a config value and
> skipped. This is the root cause of "one pin claimed, the other stays GPIO".

#### Evidence from the running DTB (kernel 6.12.76-bone50)

| Group | Raw cells | Format | Interpretation |
|---|---|---|---|
| `i2c0-pins` | `<0x188 0x30 0x00  0x18c 0x30 0x00>` | 3-cell | 2 pins: conf_i2c0_sda + conf_i2c0_scl |
| `i2c2-pins` | `<0x178 0x30 0x03  0x17c 0x30 0x03>` | 3-cell | 2 pins: I2C2 SDA/SCL at MODE3 |
| `user-leds-s0` | `<0x54 0x00 0x07  0x58 0x10 0x07  0x5c 0x00 0x07  0x60 0x10 0x07>` | 3-cell | 4 LED GPIO pins at MODE7 |
| `pinmux_comms_can` | `<0x184 0x32  0x180 0x12>` | **2-cell** | Old BBORG cape overlay, pre-6.12 format |
| `pinmux_comms_rs485` | `<0x74 0x0e  0x70 0x2e>` | **2-cell** | Old BBORG cape overlay, pre-6.12 format |

The 2-cell entries come from older BeagleBone cape overlays compiled against the
legacy binding; they still load but must not be used as templates.
**All new overlays targeting kernel 6.12 must use 3-cell format.**

Note: `&am33xx_pinmux` and `&bone_pinmux` are aliases for the same node
(`/ocp/interconnect@44c00000/segment@200000/target-module@10000/scm@0/pinmux@800`);
both labels work in overlays.

### 11.2 Pad register config_flags bits (AM335x)

| Bit | Name | 0 | 1 |
|-----|------|---|---|
| 6 | SLEWCTRL | fast | slow |
| 5 | RXACTIVE | input disabled | input enabled |
| 4 | PUTYPESEL | pull-down | pull-up |
| 3 | PUDEN | pull **enabled** | pull disabled |
| 2:0 | MUXMODE | — | 0–7 function select |

Common `config_flags` values:

| Value | Binary | Meaning | Typical use |
|-------|--------|---------|-------------|
| `0x30` | `00110000` | fast, rx-EN, pull-UP, pull-EN | digital input with pull-up (encoder, eQEP) |
| `0x10` | `00010000` | fast, rx-OFF, pull-UP, pull-EN | GPIO output with pull-up |
| `0x00` | `00000000` | fast, rx-OFF, pull-DOWN, pull-EN | GPIO output, pull-down |
| `0x08` | `00001000` | fast, rx-OFF, pull-DISABLED | PRU output (STEP/DIR/EN — no pull load) |
| `0x20` | `00100000` | fast, rx-EN, pull-DOWN, pull-EN | input with pull-down |

Final register value = `config_flags | mux_mode`.
Example: pull-up input at MODE4 → `0x30 | 0x04 = 0x34`.

### 11.3 Known pad offsets (confirmed in this project)

| BBB Pin | Pad name | Offset | Function at MODE | Config | Final reg |
|---------|----------|--------|------------------|--------|-----------|
| P8_35 | conf_mcasp0_ahclkr | `0x0D0` | MODE4 = EQEP1A_in | `0x30 0x04` | `0x34` |
| P8_33 | conf_mcasp0_fsr    | `0x0D4` | MODE4 = EQEP1B_in | `0x30 0x04` | `0x34` |
| — | conf_i2c0_sda | `0x188` | MODE0 = I2C0_SDA  | `0x30 0x00` | `0x30` |
| — | conf_i2c0_scl | `0x18C` | MODE0 = I2C0_SCL  | `0x30 0x00` | `0x30` |

For PRU stepper and endstop pins (P9 header, P8_15/16), see
`src/pru/dts/pickup-winder-p8-steppers.dts` — confirmed working with the same
3-cell syntax.

### 11.4 Minimal working overlay template

```dts
/dts-v1/;
/plugin/;

&am33xx_pinmux {
    my_pins: my_pins {
        pinctrl-single,pins = <
            /* offset  config  mux  -- final = config | mux */
            0x0D0  0x30  0x04   /* P8_35: input, pull-up, MODE4 */
            0x0D4  0x30  0x04   /* P8_33: input, pull-up, MODE4 */
        >;
    };
};

&my_device {
    pinctrl-names = "default";
    pinctrl-0 = <&my_pins>;
    status = "okay";
};
```

Build:  `dtc -O dtb -o MY-OVERLAY-00A0.dtbo -b 0 -@ overlay.dts`

Deploy:
```bash
sudo cp MY-OVERLAY-00A0.dtbo /lib/firmware/
# In /boot/uEnv.txt:
uboot_overlay_addr4=/lib/firmware/MY-OVERLAY-00A0.dtbo
```

### 11.5 Overlay-specific pitfalls

| Pitfall | Effect |
|---------|--------|
| 2-cell format on kernel 6.12 (`<offset value>`) | Only first pin applied; rest silently skipped |
| `fragment@ {}` syntax inside `/plugin/` | Unreliable on kernel 6.12; use direct `&node {}` |
| Attaching `pinctrl-0` to a counter/eQEP driver node | `ti-eqep-cnt` may claim only one pin at probe; `bone-pinmux-helper` under `&ocp` avoids this |
| `0x32` / `0x34` used as combined value in 2-cell | Worked on old kernels; misleading on 6.12 where cells are split |
| Missing `-@` flag in `dtc` command | Overlay symbols not emitted; `dtbo` loads but references unresolved |
| `bone-pinmux-helper` on kernel 6.12 | Not compiled; node appears in sysfs (`waiting_for_supplier`) but pinmux is never applied. Use `&gpio0` / `&gpio1` as target instead |
| Wrong pad offset derived from pin name | Do NOT guess offsets from pad names (e.g. `gpmc_ad4` ≠ P9_23). Always derive from the pinctrl debugfs dump (see §11.7) |


### 11.6 Debugging tips

### 11.7 Comment trouver le bon pad offset — méthode fiable

Ne jamais deviner l'offset depuis le nom du pad ou la documentation générique AM335x :
les noms (`gpmc_ad4`, `gpmc_a1`, etc.) ne correspondent pas directement au numéro
de broche BBB (P8/P9).

**Procédure à suivre :**

1. Lire le dump pinctrl réel sur la cible :
   ```bash
   cat /sys/kernel/debug/pinctrl/44e10800.pinmux-pinctrl-single/pins
   ```

2. Chaque ligne a la forme :
   ```
   pin NN (PINNN)  GPIO_BANK:GPIO_NUM  REGISTER_ADDR  VALUE  pinctrl-single
   ```

3. Identifier la pin par son GPIO connu :
   - P9_23 = GPIO1[17] → cherche `17:gpio-32-63` (GPIO1 = bank 32–63)
   - P9_24 = GPIO0[15] → cherche `15:gpio-0-31`

4. Calculer l'offset :
   ```
   offset = REGISTER_ADDR - 0x44e10800
   ```
   Exemple : `44e10844 - 44e10800 = 0x044`

5. Vérifier après application de l'overlay que la valeur passe bien à `0x37`
   (pull-up + input + MODE7) :
   ```bash
   grep '44e10844' /sys/kernel/debug/pinctrl/44e10800.pinmux-pinctrl-single/pins
   ```

**Valeurs de référence confirmées sur ce projet :**

| BBB Pin | GPIO | Registre | Offset DTS | Valeur repos |
|---------|------|----------|------------|--------------|
| P9_23 | GPIO1[17] = gpio-32-63 #17 | `44e10844` | `0x044` | `0x2f` (pull-down) |
| P8_33 | GPIO0[11] = gpio-0-31 #11  | `44e108d4` | `0x0D4` | — |
| P8_35 | GPIO0[8]  = gpio-0-31 #8   | `44e108d0` | `0x0D0` | — |

**Pour les GPIO input sur kernel 6.12 :** utiliser `&gpio0` ou `&gpio1` comme
target du fragment au lieu de `bone-pinmux-helper` (non compilé dans kernel 6.12) :

```dts
fragment@1 {
    target = <&gpio1>;   /* GPIO1 pour P9_23 */
    __overlay__ {
        pinctrl-names = "default";
        pinctrl-0 = <&pinctrl_gpio_input>;
    };
};
```

# BeagleBone eQEP (Kernel 6.6+)

Purpose: Concise checklist for using the modern Linux `counter` framework
with the AM335x eQEP peripheral and avoiding common "busy" or zero-count
failures.

- Hardware & overlay:
  - eQEP1 is part of EPWMSS1 (physical base `0x48302000`).
  - eQEP1 device address (counter child): `0x48302180` (appears as `48302180.counter`).
  - Pins to configure for eQEP1: **P8.33** and **P8.35** must be set to the QEP
    function (MODE2 on some images) — use the pad offsets and muxmode required by your image.
  - Always use 3-cell pin entries in overlays for kernel 6.x: `<offset config_flags mux_mode>`
    (example: `0x0D4 0x30 0x02`).

- Driver & sysfs location:
  - Modern driver: `ti-eqep-cnt` (counter framework). Do not rely on legacy
    `/sys/devices/.../eqep*` paths.
  - The device shows up on the counter bus: `/sys/bus/counter/devices/counterX/`.
  - The driver node binds to the device at the physical address: `48302180.counter` — verify with
    `readlink -f /sys/bus/counter/devices/counter0/of_node`.

- Runtime configuration (critical):
  - `count0/ceiling`: set to `4294967295` (32-bit max). If `ceiling` is left at `0`,
    the counter will remain at zero.
    - `echo 4294967295 | sudo tee /sys/bus/counter/devices/counter0/count0/ceiling`
  - `count0/function`: set to `quadrature x4` for full quadrature counting.
    - `echo 'quadrature x4' | sudo tee /sys/bus/counter/devices/counter0/count0/function`
  - `count0/enable`: set to `1` to start counting.
    - `echo 1 | sudo tee /sys/bus/counter/devices/counter0/count0/enable`

- Diagnostics & sanity checks:
  - Confirm pinmux: `show-pins | grep -E 'P8\.(33|35|31|32)'` (or inspect DT overlay produced values / devmem2 pad registers).
  - Confirm counter device: `ls /sys/bus/counter/devices/` should list `counter0` (or similar).
  - Confirm driver binding: `ls -l /sys/bus/platform/devices/48302180.counter/driver` (should point to `ti-eqep-cnt`).
  - Confirm runtime config:
    - `cat /sys/bus/counter/devices/counter0/count0/ceiling`
    - `cat /sys/bus/counter/devices/counter0/count0/function`
    - `cat /sys/bus/counter/devices/counter0/count0/enable`
  - Check `signal0_action` / `signal1_action` — for quadrature both signals should be active
    (e.g. `both edges`). If one signal is `none` the counter will not increment.

- Common failure modes:
  - `ceiling=0` → counter stays at zero even with signals present.
  - `function` not set to `quadrature x4` → wrong counting mode.
  - `enable=0` → counter disabled.
  - Pinmux still in old 2-cell format → only first pin applied; second pin remains GPIO.
  - `pinctrl-0` attached to `&eqep1` may cause only one pin to be claimed on some kernels; prefer `bone-pinmux-helper` under `&ocp` to apply pinctrl.

Add this document as a companion to `.github/copilot-instructions.md` or paste the section into that file when updating the central instructions.
