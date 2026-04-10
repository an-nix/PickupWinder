# Copilot Instructions — PickupWinder (BeagleBone Black)

> Branch `beagle` — migration to BeagleBone Black (AM335x) + PRU.
> ESP32 sources in `resources/esp32/` (reference only).

---

## 1. Project Identity & Goals

- **Purpose**: Automated/assisted guitar pickup coil winding — precise lateral
  guide traversal, real-time speed control, recipe persistence.
- **Hardware**: BeagleBone Black (AM335x), two A4988/DRV8825 stepper drivers
  (spindle + lateral), rotary encoder, analog potentiometer, footswitch,
  dual-contact home sensor, WebSocket UI.
- **Core constraint**: Firmware pilots physical motors under wire tension.
  Correctness and determinism always outweigh elegance.

---

## 2. Architecture Overview — Option A (Continuous Shared Parameters)

4-layer architecture. **No move rings.** Host writes target speeds;
PRU0 orchestrates; PRU1 generates pulses from continuous parameters.

```
┌───────────────────────────────────────────────────────────────────┐
│  Layer 4 — Python application (asyncio, runs on ARM Linux)        │
│  PruClient · pickup_test.py · (future: WinderApp, WebUI, ...)     │
│  Sends JSON commands to daemon, receives telem + events           │
│                      │ Unix socket /run/pickup-winder.sock        │
├──────────────────────▼────────────────────────────────────────────┤
│  Layer 3 — C hardware daemon  (pickup_daemon, runs on ARM Linux)  │
│  Maps PRU Shared RAM via /dev/mem                                 │
│  Writes host_cmd_t   (commands: set_speed, enable, estop, home)   │
│  Reads pru_status_t  (aggregated status from PRU0)                │
│  Polls at 10 ms, broadcasts telem + events to Python              │
│  ONLY talks to PRU0 — never accesses motor_params or motor_telem  │
│                      │ /dev/mem mmap (PRU Shared RAM 0x4A310000)  │
├──────────────────────▼────────────────────────────────────────────┤
│  Layer 2 — PRU0 orchestration  (200 MHz, 5 ns/cycle)              │
│  Reads host_cmd_t from daemon (via shared RAM)                    │
│  Owns homing state machine (IDLE→APPROACH→HIT)                    │
│  Writes motor_params_t (intervals, dirs, enable, run flags)       │
│  Reads motor_telem_t from PRU1                                    │
│  Publishes pru_status_t (aggregated status for daemon)            │
│  Detects events (endstop, home_complete, fault) → sets event flags│
│  NO motor pin ownership. NO STEP/DIR/EN control.                  │
│                      │ PRU Shared RAM (host_cmd / motor_params)   │
├──────────────────────▼────────────────────────────────────────────┤
│  Layer 1 — PRU1 motor control  (200 MHz, IEP owner)               │
│  Reads motor_params_t from PRU0 continuously                      │
│  IEP-based pulse generation: pulse_gen_t per axis                 │
│  STEP/DIR/EN GPIO for spindle + lateral                           │
│  Reads R31 endstops → unconditional lateral safety stop           │
│  Publishes motor_telem_t (step counts, positions, faults, endstop)│
│  DUMB motor driver — no homing logic, no host commands            │
└───────────────────────────────────────────────────────────────────┘
```

**Communication flow** (total shared RAM: 224 bytes):
```
Host ──host_cmd_t──→ PRU0 ──motor_params_t──→ PRU1
                     PRU0 ←──motor_telem_t── PRU1
Host ←─pru_status_t─ PRU0
```

Key files per layer:
- Layer 1: `src/pru/motor_control/main.c`
- Layer 2: `src/pru/orchestrator/main.c`
- Layer 3: `src/linux/daemon/pickup_daemon.c`
- Layer 4: `src/python/pickup_test.py`, `src/python/pru_client.py`

Full architecture: see `doc/beaglebone_architecture.md`.

### 2.1 PRU Firmware

| File                                   | Responsibility |
|----------------------------------------|----------------|
| `pru/include/pru_ipc.h`               | IPC shared memory layout: host_cmd_t, motor_params_t, motor_telem_t, pru_status_t; HOST_CMD_* opcodes; fault/state/event flags |
| `pru/include/pru_stepper.h`           | IEP timer macros; pulse_gen_t continuous engine; pulse_update() / pulse_stop() |
| `pru/include/pru_regs.h`              | R30/R31 register aliases for STEP/DIR/EN/ENDSTOP pins |
| `pru/motor_control/main.c`       | Motor firmware (PRU1 at runtime): IEP owner, dual-axis pulse generation, trapezoidal move_to profile FSM, motor_telem_t publisher |
| `pru/orchestrator/main.c`        | Orchestrator firmware (PRU0 at runtime): host_cmd_t processor, homing FSM, software limits, move_to arming, R31 endstop reading, motor_params_t writer, pru_status_t publisher |

### 2.2 Layer 3 — C Daemon (`pickup_daemon`)

| File                                          | Responsibility |
|-----------------------------------------------|----------------|
| `src/linux/daemon/pickup_daemon.c`            | C hardware daemon: mmap PRU shared RAM, write host_cmd_t, read pru_status_t, Unix socket server |

Socket path: `/run/pickup-winder.sock`
Protocol: newline-delimited JSON.

Commands from Python:
- `set_speed` — set spindle Hz (and optionally lateral Hz for continuous mode)
- `enable` — enable/disable stepper drivers
- `e_stop` — immediate all-axis stop
- `home_start` — start lateral homing sequence
- `set_limits` — configure axis software position limits (steps)
- `move_to` — move lateral to absolute position with trapezoidal profile
- `ack_event` — acknowledge pending event and release limit locks
- `reset_pos` — reset step counters and positions

Events to Python: `endstop_hit`, `home_complete`, `fault`, `limit_hit`, `move_complete`, `telem`

### 2.3 Layer 4 — Python Application

| Module               | File(s)                              | Responsibility |
|----------------------|--------------------------------------|----------------|
| `PruClient`          | `src/python/pru_client.py`           | Async socket client for pickup_daemon; JSON command/event protocol |
| `pickup_test.py`     | `src/python/pickup_test.py`          | Test sketch: DaemonClient class + 11 test functions for validating the full stack |

> Python must NOT access /dev/mem directly. All PRU interaction goes through
> `PruClient` → Unix socket → `pickup_daemon`.

### 2.4 Key Header (C layer)

| Header                        | Content |
|-------------------------------|---------|
| `pru/include/pru_ipc.h`      | `host_cmd_t` (64B), `motor_params_t` (32B), `motor_telem_t` (64B), `pru_status_t` (64B); HOST_CMD_* opcodes; MOTOR_STATE_*, FAULT_*, EVENT_* flags |
| `pru/include/pru_stepper.h`  | `pulse_gen_t` (continuous IEP engine), `pulse_update()`, `IEP_NOW()`, `IEP_INIT()` |
| `pru/include/pru_regs.h`     | R30/R31 hardware register aliases — **`register volatile` is mandatory**; without `register` writes go to RAM, not pins |

### 2.5 IPC Shared RAM Layout

```
Offset   Size    Struct              Direction       Description
0x0000   64 B    host_cmd_t          Host → PRU0     Commands (set_speed, enable, estop, home, set_limits, move_to, ...)
0x0040   32 B    motor_params_t      PRU0 → PRU1     Motor intervals, dirs, enable/run flags, move_to profile params
0x0060   64 B    motor_telem_t       PRU1 → PRU0     Step counts, positions, faults, endstop mask, lat_move_done
0x00A0   64 B    pru_status_t        PRU0 → Host     Aggregated status for daemon broadcast
Total:  224 bytes (well within 12 KB PRU shared RAM)
```

### 2.6 HAL Contract — Strict Separation of Concerns

The C daemon is the **sole Hardware Abstraction Layer** between the physical
world (PRU, /dev/mem, registers) and the application world (Python, recipes,
UI). This separation is intentional and must be maintained.

**Why the boundary exists:**

The hardware world is unstable by nature:
- PRU shared RAM layout (`pru_ipc.h` offsets) may change with firmware updates.
- Stepper driver replacement (A4988 → TMC2209 UART) changes the low-level protocol.
- PRU firmware opcode changes, struct sizes, flag bits — all hardware details.

The application world is unstable by nature:
- New winding recipes, patterns, geometry parameters.
- UI changes, new WebSocket events, REST endpoints.
- Session arbitration rules, turn counting, target logic.

**The contract between them is intentionally small and stable:**

```
Commands (Python → C):  set_speed / enable / e_stop / home_start / set_limits / move_to / ack_event / reset_pos
Events   (C → Python):  endstop_hit / home_complete / fault / limit_hit / move_complete / telem
```

This contract exposes **no hardware detail**: no memory addresses, no register
values, no PRU opcodes. Python does not know and must never know what
`motor_params_t`, `IEP_NOW()`, or `pulse_gen_t` are.

**Discipline rules (enforced in code review):**

In the C daemon — never let application logic leak in:
```c
/* ❌ wrong: application logic in C */
if (step_count >= recipe_target) { send_event("target_reached"); }

/* ✅ correct: raw hardware fact only */
send_event("telem", step_count, position, faults);
/* Python decides whether target_reached based on telem */
```

In Python — never reference hardware concepts:
```python
# ❌ wrong: hardware detail leaking into Python
motor_params.sp_interval = 31250

# ✅ correct: socket contract only
await client.set_speed(sp_hz=6400)
await client.move_to(pos=3072, start_hz=200, max_hz=4000, accel_steps=300)
```

**Consequence:** If the hardware changes (new PRU firmware, new stepper driver,
new board revision), only `pickup_daemon.c` changes — Python is untouched.
If the application logic changes (new recipe type, new UI feature), only Python
changes — the C daemon is untouched.

### 2.7 Option A with autonomous move_to (lateral) + hot retarget + spindle coordination

This project uses the **continuous shared-parameter** model (Option A) for the spindle, and an **autonomous trapezoidal move_to** for the lateral axis:

- **Spindle**: Python sends progressive `set_speed` commands at ~10 ms cadence for ramps. No host timing risk since the spindle position is irrelevant.
- **Lateral**: Python sends a `move_to` command. PRU1 executes the trapezoidal profile (accel → cruise → decel) and stops exactly at the target without any further host interaction. Python receives a `move_complete` event on arrival.

**Hot retarget** — consecutive same-direction moves form a seamless stream:

If a new `move_to` arrives while PRU1 is in ACCEL or CRUISE phase and the direction matches, PRU1 just updates `g_lat_target` without resetting `params->lat_interval` to `start_iv`. The motor never decelerates between waypoints. If in DECEL phase with same direction, the interval is snapped back to `lat_cruise_iv` (CRUISE resumed). For direction changes, a full re-arm occurs (motor decelerates to stop, then re-accelerates).

**Spindle-lateral speed coordination** (PRU0 `coord_tick()`):

- The daemon computes `move_sp_lat_coord = (sp_iv × 64) / lat_cruise_iv` (Q6 ratio) for each `move_to`.
- PRU0 applies `sp_adj = (sp_lat_coord × params->lat_interval) >> 6` every `CMD_CHECK_STRIDE` iterations (≈5 µs). No division in PRU.
- Coordination stays **active after `move_complete`** so the spindle stays at the proportional start speed during the reversal gap (waiting for the reverse `move_to`).
- Coordination is **disabled by `set_speed`** (Python re-takes direct control) and by safety events (endstop, limit).
- `SP_IV_MIN = 625` (160 kHz) / `SP_IV_MAX = 187500` (~534 Hz) bound the coordinated interval.

Why this choice for lateral:
- **No overshoot possible**: PRU1 owns position at step resolution. Motor STOPS at target regardless of Python scheduler jitter.
- **No streaming required for single moves**: One command per move, not 50+ `set_speed` messages.
- **Hot retarget for traversals**: Python can stream rapid waypoints — the motor maintains cruise speed.
- **Consistent winding density**: Spindle tracks lateral speed during ramps, keeping turns/mm constant.
- **Safety by design**: Even if Python crashes mid-move, the motor decelerates to its target and stops.

---

## 3. Hard Rules (Must-follow)

### 3.1 PRU Safety

- **NEVER** use dynamic memory, floats, or OS calls in PRU firmware.
- **NEVER** use division in the inner PRU step loop (intervals pre-computed host-side).
- The PRU step loop polls `IEP_NOW()` continuously — no `__delay_cycles` anywhere.
- Emergency stop (`HOST_CMD_ESTOP`) clears all STEP outputs and stops all axes immediately.
- PRU0 owns and initializes the IEP timer; PRU1 reads only — never resets it.
- All motor-related real-time code must target **PRU0**.
- PRU1 is reserved for orchestration, supervision, and host communication.
- PRU0 is a DUMB motor driver — it has no knowledge of commands, homing, or host protocol.
- All PRU shared memory accesses must use `volatile` pointers.
- Shared memory structs must be `__attribute__((packed, aligned(4)))`.

### 3.1.1 Canonical pin layout (MUST NOT change implicitly)

Motor A — Spindle (Group 1):
- `P8_41` → `EN_A` (`PRU0 R30[7]`, active-low)
- `P8_43` → `DIR_A` (`PRU0 R30[5]`)
- `P8_45` → `STEP_A` (`PRU0 R30[1]`)

Motor B — Lateral (Group 2):
- `P8_42` → `EN_B` (`PRU0 R30[3]`, active-low)
- `P8_44` → `DIR_B` (`PRU0 R30[0]`)
- `P8_46` → `STEP_B` (`PRU0 R30[2]`)

Endstops (PRU0 inputs — read by PRU0 orchestrateur via R31):
- `P9_28` → `ENDSTOP_1` (`PRU0 R31[6]`, pull-up, active-HIGH)
- `P9_30` → `ENDSTOP_2` (`PRU0 R31[2]`, pull-up, active-HIGH)

Additional board IO:
- Encoder1: `P8_11` (A), `P8_12` (B)
- Encoder2: `P8_33` (A), `P8_35` (B)
- HX711: `P9_12` (SCK), `P9_14` (DOUT)
- Footswitch: `P9_23`

Rules:
- Do not remap these pins unless the user explicitly requests it.
- Any PRU pin change must update firmware + DTS + documentation in one commit.

### 3.2 Linux Daemon Safety

- **NEVER** use heap allocation in the hot control loop or telemetry path.
  All runtime buffers are stack-local with bounded sizes or static.
- Control loop tick: <= 10 ms. Log timing warnings if exceeded.
- Daemon communicates ONLY with PRU1 (reads pru_status_t, writes host_cmd_t).
  It never accesses motor_params_t or motor_telem_t directly.

### 3.3 IPC Protocol

- Host writes commands to `host_cmd_t.cmd` and waits for PRU1 to acknowledge
  by echoing the opcode in `host_cmd_t.cmd_ack`, then setting `cmd = HOST_CMD_NOP`.
- Host must not write a new command before the previous one is acknowledged.
- Daemon reads `pru_status_t` at control loop cadence (<= 10 ms).
- `pru_ipc.h` is C-compatible: included from both PRU C and Linux C.

### 3.4 Socket Protocol

- Socket path: `/run/pickup-winder.sock` (Unix domain, SOCK_STREAM).
- All messages: newline-delimited compact JSON.
- Commands: `set_speed`, `enable`, `e_stop`, `home_start`, `ack_event`, `reset_pos`.
- Responses: `{"ok":true}` or `{"ok":false,"error":"..."}`.
- Events: `endstop_hit`, `home_complete`, `fault`, `telem`.
- Speed is specified in Hz (daemon converts Hz → IEP intervals).

### 3.5 Recipe & Persistence

- Recipe format version: `PICKUP_RECIPE_FORMAT_VERSION` (future, in Python).
- Float fields clamped after parsing.
- Storage: JSON files on BBB eMMC.

### 3.6 Sensor Safety

- Home sensor: NO=LOW + NC=HIGH → home. NO=HIGH + NC=HIGH → FAULT.
- Endstops on PRU0 R31 (P9_28/P9_30): unconditional lateral safety stop when any endstop asserts.
- PRU1 homing FSM reads endstop state from motor_telem_t.endstop_mask.

### 3.7 Build

- PRU: `pru-unknown-elf-gcc` (crosstool-NG). Makefile: `src/pru/Makefile`.
- Daemon: `gcc` (ARM native or cross). Build via root `Makefile`.
- DTS overlays: `dtc` compiler. Build via root `Makefile`.
- Root Makefile: `make all` builds dtbo + pru + daemon. Outputs in `build/`.
- Unit tests: (planned) native x86-64 host, gtest. Tests in `test/`.
- CI: `.github/workflows/beaglebone-build.yml`.

### 3.8 Logging

- Daemon uses `fprintf(stderr, ...)` for diagnostics.
- High-frequency logs gated by verbosity flags.
- Never log credentials, recipe blobs, or raw shared memory dumps in production.

---

## 4. Domain Knowledge

### 4.1 Winding State Machine (future — Python layer)

```
IDLE --(start)--> PAUSED (positioning)
                    |
                    +--(positioned+pot/resume)--> WINDING
                    |                               |
                    |                         (pot=0)--> PAUSED
                    |                  (turns>=target)--> TARGET_REACHED
                    |                               |
                    +----------(stop)---------------> IDLE
```

### 4.2 Session Arbitration (future — Python layer)

- Sources: Pot (analog), IHM (WebSocket/UART), Footswitch.
- IDLE: only IHM can produce a Start intent.
- Pot-lock: when another source takes control while pot > 0, pot locked until it returns to 0.
- Pot → RunMode::Pot (proportional). UI/Footswitch → RunMode::Max.

### 4.3 Lateral Traverse Synchronization

- Lateral speed: `effWidthMm × windingHz / (tpp × STEPS_PER_REV)`.
- Scaled by pattern `speedScale` (0.55-1.60).
- Compensation applied **host-side**: Python adjusts speeds via `set_speed` each tick.
- Speed changes take effect immediately (no move ring flush needed).

### 4.4 Winding Patterns (future — Python layer)

- **STRAIGHT**: constant traverse.
- **SCATTER**: per-layer random TPP + speed jitter.
- **HUMAN**: smooth Perlin-like noise on traverse and speed.
- All patterns deterministic per `seed`.

### 4.5 Hardware Constants

| Parameter         | Value           | Notes |
|-------------------|-----------------|-------|
| Spindle steps/rev | 6400            | 200-step × 32 µstep |
| Speed range       | ~1067 – 160000 Hz | ~10 – 1500 RPM |
| Lateral steps/mm  | 3072            | 96-step × 32 µstep, M6 1mm pitch |
| PRU clock         | 200 MHz         | 5 ns/cycle, 1 cycle/instruction |
| IEP interval min  | 1250            | = 200 MHz / 160000 Hz |
| IEP interval max  | 187500          | = 200 MHz / 1067 Hz |

---

## 5. Code Style Conventions

- **Language**: C for daemon and PRU. Python for application layer.
- **Naming**: PascalCase classes/enums, camelCase methods/members, _prefix private, UPPER_SNAKE macros.
- **Fixed buffers**: `char buf[N]` + `snprintf(buf, sizeof(buf), ...)`. Never `sprintf`.
- **PRU integers only**: speeds in Hz (uint32_t), positions in steps (int32_t). No floats.
- **Comments**: French acceptable in log messages. English for code comments.
- **Include order**: matching .h first, then system headers, then project headers.

---

## 6. Testing & Validation

- `src/python/pickup_test.py`: 9 test functions exercising the full stack
  (enable, estop, set_speed, speed_change, direction, home, reset_pos, telem, ack_event).
- Future unit tests: `test/test_<suite>/test_main.cpp` (gtest, host-compilable).
- Add/update test when adding a command or changing validation.

---

## 7. Common Pitfalls

| Pitfall | Why it matters |
|---------|----------------|
| Float or division in PRU loop | PRU has no FPU; crashes or extreme slowdown |
| OS call in PRU firmware | PRU has no OS; link error or crash |
| Missing `volatile` on shared RAM pointer | Compiler may cache stale value |
| `sprintf()` without size | Buffer overflow |
| Missing PRU0 endstop safety stop | Wire break if lateral runs past limit |
| Writing motor_params from daemon | Violates layering: daemon → PRU1 only |
| Hz=0 in set_speed without interval check | Division by zero in daemon Hz→interval conversion |
| Daemon writing new cmd before PRU1 ack | Race condition: previous command lost |
| 2-cell `pinctrl-single,pins` on kernel 6.12 | `#pinctrl-cells=<2>` expects 3-cell format; only first pin applied |
| `fragment@ {}` syntax in `/plugin/` DTS | Unreliable on 6.12; use direct `&node {}` syntax |
| `bone-pinmux-helper` on kernel 6.12 | Not compiled; node stuck at `waiting_for_supplier` |
| Wrong pad offset for GPIO input pin | Derive from pinctrl debugfs dump, not from pad name |
| `__R30`/`__R31` declared without `register` keyword | `volatile uint32_t __R30 __asm__("r30")` without `register` creates a RAM variable named `r30` — writes never reach the physical pins. PRU appears to run (step counts increment in RAM) but no GPIO changes. **Always use `register volatile uint32_t __R30 __asm__("r30")`** — see `pru/include/pru_regs.h`. |
| MODE6 on PRU0 output pins instead of MODE5 | On AM335x MCASP0 pins used by PRU0: MODE5 = `pr1_pru0_pru_r30_N` (output), MODE6 = `pr1_pru0_pru_r31_N` (input). Using MODE6 on STEP/DIR/EN pins in the DTS silently configures them as inputs — `__R30` writes are correct but the pads never drive. Verify with `cat /sys/kernel/debug/pinctrl/.../pins`: must show `pru 0 out`, not `pru 0 in`. |
| Blocking `write()` in daemon `broadcast()` | Single-threaded daemon with blocking `write()` on client fds: one slow/stuck `socat` client freezes the entire event loop (no commands, no telem, SIGINT ignored). Fix: `fcntl(cfd, F_SETFL, O_NONBLOCK)` on accept + tolerate `EAGAIN` in `broadcast()`. |

---

## 8. Quick Reference: Adding a New Feature

1. **New daemon command**: add `HOST_CMD_*` in `pru_ipc.h`, handle in PRU1
   `process_host_cmd()`, add JSON command parsing in `pickup_daemon.c`,
   add `PruClient` method, add test in `pickup_test.py`.
2. **New event type**: add `EVENT_*` in `pru_ipc.h`, detect in PRU1 and set
   `event_pending`/`event_type`, handle in daemon event broadcast, handle
   in Python `on_event` callback.
3. **New motor parameter**: add field to `motor_params_t`, set in PRU1, read
   in PRU0. Update daemon if host needs to control it.
4. **New telemetry field**: add to `motor_telem_t`, publish in PRU0, copy to
   `pru_status_t` in PRU1, add to daemon telem JSON.

---

## 9. File Organization

```
.github/                        <- CI + copilot instructions
src/
  pru/                          <- PRU firmware (active)
    include/                    <- pru_ipc.h, pru_stepper.h, pru_regs.h
    motor_control/          <- motor firmware (runs on PRU1 at runtime)
    orchestrator/           <- orchestrator firmware (runs on PRU0 at runtime)
    Makefile                    <- PRU cross-compile (pru-unknown-elf-gcc)
  linux/
    daemon/                     <- pickup_daemon.c (Layer 3)
  python/                       <- Python application (Layer 4)
    pickup_test.py              <- Test sketch (9 functions)
    pru_client.py               <- Async socket client for daemon
  dts/                          <- Device-tree overlays
build/                          <- Build outputs
  dtbo/                         <- Compiled DT overlays
  pru/                          <- PRU firmware binaries
  daemon/                       <- Daemon binary
doc/                            <- Architecture docs
resources/                      <- Reference material (ESP32, eQEP, Klipper)
test/                           <- Unit tests (gtest, planned)
Makefile                        <- Root build orchestrator
```

Do not modify `resources/`.
Do not create new source files without clear domain justification.

---

## 10. Device Tree Overlay Authoring (BBB / kernel 6.12)

### 10.1 pinctrl-single,pins format — CRITICAL

The AM335x pinmux node on Debian 12 / kernel 6.12 declares **`#pinctrl-cells = <2>`**.
This switches `pinctrl-single,pins` to **3-cell format per pin**:

```
<pad_offset   config_flags   mux_mode>
```

The driver writes `config_flags | mux_mode` to the pad register at
`pinmux_base + pad_offset`.

> ⚠️ **Using the old 2-cell format `<offset value>` silently applies only the
> first pin.** The second pin's offset is misinterpreted as a config value and
> skipped.

### 10.2 Pad register config_flags bits (AM335x)

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

### 10.3 Known pad offsets (confirmed in this project)

| BBB Pin | Pad name | Offset | Function at MODE | Config | Final reg |
|---------|----------|--------|------------------|--------|-----------|
| P8_35 | conf_mcasp0_ahclkr | `0x0D0` | MODE4 = EQEP1A_in | `0x30 0x04` | `0x34` |
| P8_33 | conf_mcasp0_fsr    | `0x0D4` | MODE4 = EQEP1B_in | `0x30 0x04` | `0x34` |

### 10.4 Minimal working overlay template

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

### 10.5 Overlay-specific pitfalls

| Pitfall | Effect |
|---------|--------|
| 2-cell format on kernel 6.12 | Only first pin applied; rest silently skipped |
| `fragment@ {}` syntax inside `/plugin/` | Unreliable on kernel 6.12; use direct `&node {}` |
| `bone-pinmux-helper` on kernel 6.12 | Not compiled; node stuck at `waiting_for_supplier` |
| Missing `-@` flag in `dtc` command | Overlay symbols not emitted; references unresolved |
| Wrong pad offset from pin name | Always derive from pinctrl debugfs dump |

### 10.6 How to find the correct pad offset

Never guess from pad name or generic AM335x docs.

1. Read pinctrl debugfs dump on the target:
   ```bash
   cat /sys/kernel/debug/pinctrl/44e10800.pinmux-pinctrl-single/pins
   ```
2. Identify pin by GPIO label (e.g. P9_23 = GPIO1[17] → look for `gpio-32-63 #17`).
3. Compute: `offset = register_address - 0x44e10800`.

Confirmed values:

| BBB Pin | GPIO | Register | Offset DTS |
|---------|------|----------|------------|
| P9_23 | GPIO1[17] | `44e10844` | `0x044` |
| P8_33 | GPIO0[11] | `44e108d4` | `0x0D4` |
| P8_35 | GPIO0[8]  | `44e108d0` | `0x0D0` |

---

## 11. BeagleBone eQEP (Kernel 6.6+)

- eQEP1 device: `48302180.counter` (modern `ti-eqep-cnt` driver, counter framework).
- Pins: P8.33 + P8.35 in MODE4 (3-cell overlay format).
- sysfs: `/sys/bus/counter/devices/counterX/`.

Runtime configuration (critical):
```bash
echo 4294967295 | sudo tee .../count0/ceiling
echo 'quadrature x4' | sudo tee .../count0/function
echo 1 | sudo tee .../count0/enable
```

Common failures: `ceiling=0` → stays at zero; `function` wrong → no counting;
2-cell pinmux → only first pin applied.
