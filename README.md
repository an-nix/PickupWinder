# PickupWinder — BeagleBone Black PRU winding controller

Automated/assisted guitar pickup coil winding on the BeagleBone Black (AM335x).
Real-time stepper control via PRU, with Linux daemon + Python application layer.

## Architecture — Option A (Continuous Shared Parameters)

4-layer stack. **No move rings.** Host writes target speeds; PRU0 orchestrates;
PRU1 generates pulses from continuous parameters.

```
Layer 4  Python application   (pickup_test.py, pru_client.py)
         │ Unix socket /run/pickup-winder.sock
Layer 3  C hardware daemon    (pickup_daemon)
         │ /dev/mem mmap
Layer 2  PRU0 orchestration   (host_cmd → motor_params, homing FSM)
         │ PRU Shared RAM
Layer 1  PRU1 motor control   (pulse_gen_t, STEP/DIR/EN, endstops)
```

**Communication flow** (224 bytes shared RAM):
```
Host ──host_cmd_t──→ PRU0 ──motor_params_t──→ PRU1
                     PRU0 ←──motor_telem_t── PRU1
Host ←─pru_status_t─ PRU0
```

See `doc/beaglebone_architecture.md` and `.github/copilot-instructions.md`
for the full architecture rationale.

## Hardware

- **Board**: BeagleBone Black (AM335x Cortex-A8 + 2× PRU)
- **Steppers**: Two A4988/DRV8825 drivers — spindle + lateral traverse
- **Microstepping**: 32 µstep (6400 steps/rev spindle, 3072 steps/mm lateral)
- **Encoders**: eQEP1 (P8.33, P8.35)
- **Endstops**: Dual-contact on PRU0 R31 (P9_28=R31[6], P9_30=R31[2], pull-up)
- **Footswitch**: P9.23 (GPIO)

## Canonical Pin Mapping

### PRU1 motor outputs

| Header pin | PRU bit   | Function | Notes |
|------------|-----------|----------|-------|
| P8_41      | R30\[7\]  | EN_A     | Spindle enable (active-low) |
| P8_43      | R30\[5\]  | DIR_A    | Spindle direction |
| P8_45      | R30\[1\]  | STEP_A   | Spindle step |
| P8_42      | R30\[3\]  | EN_B     | Lateral enable (active-low) |
| P8_44      | R30\[0\]  | DIR_B    | Lateral direction |
| P8_46      | R30\[2\]  | STEP_B   | Lateral step |

### PRU0 endstop inputs

| Header pin | PRU bit   | Function   |
|------------|-----------|------------|
| P9_28      | R31\[6\]  | ENDSTOP_1  |
| P9_30      | R31\[2\]  | ENDSTOP_2  |

### Other IO

| Pin    | Function      |
|--------|---------------|
| P8_33  | eQEP1 A       |
| P8_35  | eQEP1 B       |
| P8_11  | Encoder1 A    |
| P8_12  | Encoder1 B    |
| P9_12  | HX711 SCK     |
| P9_14  | HX711 DOUT    |
| P9_23  | Footswitch    |

## Directory Structure

```
src/
  pru/                          PRU firmware (pru-unknown-elf-gcc)
    include/                    pru_ipc.h, pru_stepper.h, pru_regs.h
    motor_control/          motor firmware (runs on PRU1)
    orchestrator/           orchestrator firmware (runs on PRU0)
    Makefile
  linux/
    daemon/                     pickup_daemon.c (Layer 3)
  python/
    pickup_test.py              Test sketch (9 functions)
    pru_client.py               Async socket client
  dts/                          Device-tree overlays
build/                          Build outputs (dtbo, pru, daemon)
doc/                            Architecture documentation
resources/                      Reference material (ESP32, eQEP, Klipper)
test/                           Unit tests (planned)
Makefile                        Root build orchestrator
```

## Build

### Prerequisites

- `pru-unknown-elf-gcc` (crosstool-NG) for PRU firmware
- ARM cross-compiler for daemon on x86 hosts (e.g. `arm-linux-gnueabihf-gcc`)
- `dtc` for device-tree overlays

### Compiler usage

- PRU firmware: built with `pru-unknown-elf-gcc` from crosstool-NG, typically found in `~/x-tools/*/bin/pru-elf-gcc` or `~/x-tools/*/bin/pru-gcc`.
- Host daemon: on x86 hosts, `make daemon` automatically searches for an ARM cross-compiler in `PATH` or in `$(HOME)/x-tools/*/bin`.
  - supported toolchain names include `arm-linux-gnueabihf-gcc`, `arm-cortex_a8-linux-gnueabihf-gcc`, `armv7l-linux-gnueabihf-gcc`, or equivalent ARM GNU toolchains.
  - if none is found, the build fails and prompts you to install an ARM cross-toolchain.
- On an ARM host, the normal native `gcc` is used for `pickup_daemon`.

### Cross compilation

When building on an x86 host, `make daemon` will automatically search for an ARM cross-compiler in your `PATH` and in `$(HOME)/x-tools/*/bin`.
If none is found, the build prints a clear error and you must either install an ARM cross-toolchain or build the daemon directly on the BeagleBone.

If you already have a cross-toolchain in `~/x-tools`, no extra flags are required.

### Build everything

```bash
make all
```

Outputs:
- `build/dtbo/*.dtbo` — DT overlays
- `build/pru/am335x-pru0-fw` — PRU0 firmware
- `build/pru/am335x-pru1-fw` — PRU1 firmware
- `build/daemon/pickup_daemon` — C daemon

### Build individual targets

```bash
make dtbo        # DT overlays only
make pru         # PRU firmware only
make daemon      # Daemon only
make clean       # Remove all build outputs
```

### Deploy to BeagleBone

```bash
make deploy BBB_IP=192.168.x.x         # Full deploy via SSH
make -C src/pru deploy BBB_IP=...      # PRU firmware only
```

## Socket Protocol

The daemon listens on `/run/pickup-winder.sock` (Unix domain, newline-delimited JSON).

**Commands** (Python → daemon):
| Command       | Fields                              | Description |
|---------------|-------------------------------------|-------------|
| `set_speed`   | `sp_hz`, `lat_hz`, `sp_dir`, `lat_dir` | Set spindle speed (Hz). `lat_hz` activates continuous lateral (avoid during winding) |
| `enable`      | `axis`, `value`                     | Enable/disable drivers |
| `set_limits`  | `axis`, `min`, `max`                | Set software position limits (steps, signed) |
| `move_to`     | `axis`, `pos`, `start_hz`, `max_hz`, `accel_steps` | Move lateral to absolute position with trapezoidal profile |
| `e_stop`      |                                     | Emergency stop |
| `home_start`  |                                     | Start lateral homing |
| `reset_pos`   | `axis`                              | Reset step counters |
| `ack_event`   |                                     | Acknowledge event and release locks |

### Command payloads and rate

- Each command is a single JSON object on its own line (newline-delimited JSON).
- The daemon rejects a new command if the previous host command is still pending for more than ~50 ms. Returns `{"ok":false,"error":"busy"}`.
- **Spindle** speed ramps: send progressive `set_speed` at ~**10 ms cadence**.
- **Lateral axis**: use `move_to` — the motor stops at the target autonomously. `lat_hz` in `set_speed` is for continuous lateral modes only.

### move_to — autonomous trapezoidal profile

```json
{"cmd":"move_to","axis":1,"pos":3072,"start_hz":200,"max_hz":4000,"accel_steps":300}
```

| Field | Type | Description |
|-------|------|-------------|
| `pos` | int | Absolute target position in steps (signed, 0 = home) |
| `start_hz` | uint | Step frequency at ramp start/end (slow, e.g. 200) |
| `max_hz` | uint | Cruise step frequency (fast, e.g. 4000–16000) |
| `accel_steps` | uint | Steps for accel and decel (≥ 1) |

The daemon converts Hz → IEP intervals (integer only) and sends `HOST_CMD_MOVE_TO`.
PRU1 executes the profile autonomously and stops exactly at `pos`. On completion, PRU0 fires `EVENT_MOVE_COMPLETE` and the daemon broadcasts:

```json
{"event":"move_complete","pos":3072}
```

Call `ack_event` after receiving `move_complete`.

**Hot retarget** — consecutive same-direction moves without stopping:

If a new `move_to` arrives while the lateral is already moving in the **same direction** (ACCEL or CRUISE phase), PRU1 simply updates its destination without resetting the speed to `start_hz`. The motor keeps running at cruise speed and decelerates only when approaching the new target. This enables seamless winding traversals made of many small steps:

```python
# Winding traversal: stream small moves; motor never stops between them
for waypoint in traverse_waypoints:
    await client.move_to(pos=waypoint, start_hz=500, max_hz=8000, accel_steps=100)
    # No need to wait for move_complete between waypoints
```

For **direction reversals** (e.g. at the coil edge), Python must wait for `move_complete` before sending the reverse move — the motor decelerates to a full stop then re-accelerates in the new direction.

**Spindle-lateral speed coordination:**

The daemon includes a Q6 ratio `move_sp_lat_coord = (sp_iv × 64) / lat_cruise_iv` in each `move_to`. PRU0's `coord_tick()` reads `params->lat_interval` and adjusts the spindle proportionally every `CMD_CHECK_STRIDE` iterations (≈ 5 µs), so turns/mm stays constant during lateral ramps:

```
sp_adj = (sp_lat_coord × lat_iv) >> 6
```

Coordination is **activated** by `move_to` and **deactivated** by `set_speed` (Python re-takes direct spindle control). It persists across `move_complete` during the reversal gap so the spindle stays slow while the lateral is momentarily stopped.

**Safety layering for lateral:**
1. PRU1 stops exactly at target (primary — no host timing dependency).
2. Software limits (`set_limits`) halt the axis if position leaves the configured range (secondary).
3. Hardware endstops (P9\_28, P9\_30) cut the lateral immediately (hardware failsafe).

Additional commands:
- `{"cmd":"set_limits","axis":1,"min":-10000,"max":10000}` — install soft limits.  If exceeded, PRU0 latches the axis and fires `limit_hit`. Send `ack_event` to release.
- `{"cmd":"set_speed","sp_hz":8000,"sp_dir":0}` — spindle speed change (10 ms ramp cadence).
- `axis` can be `0` (spindle), `1` (lateral), or `255` (all).

**Events** (daemon → Python):
| Event           | Fields               | Description |
|-----------------|----------------------|-------------|
| `endstop_hit`   |                      | Lateral endstop triggered |
| `home_complete` |                      | Homing sequence finished |
| `fault`         | `sp_faults`, `lat_faults` | Motor fault detected |
| `limit_hit`     | `axis`, `pos`        | Software position limit exceeded |
| `move_complete` | `pos`                | Autonomous move_to finished; motor stopped at target |
| `telem`         | `pru1_state`, `sp`, `lat`, `endstop` | Periodic telemetry (100 ms) |

## Testing

```bash
# Run all tests (requires daemon running on BBB)
python3 src/python/pickup_test.py

# Run specific test
python3 src/python/pickup_test.py --test test_set_speed

# List available tests
python3 src/python/pickup_test.py --list
```

## Notes

- The active BeagleBone port is the current target; ESP32/PlatformIO content
  in `resources/esp32/` is legacy reference only.
- PRU0 is the orchestrator (homing, limits, move_to arming, spindle coordination); PRU1 drives
  the steppers (IEP owner, STEP/DIR/EN, trapezoidal profile + hot retarget execution).
- Spindle speed ramps are managed host-side via `set_speed` at ~10 ms cadence.
- Lateral moves use `move_to` — PRU1 executes the profile autonomously; Python
  only needs to send one command and wait for `move_complete`.
- Consecutive same-direction `move_to` commands trigger hot retarget — the lateral
  motor never decelerates between waypoints. Direction reversal requires waiting for
  `move_complete` first.
- Spindle speed is automatically coordinated with lateral ramps via Q6 ratio
  (`move_sp_lat_coord`) so turns/mm stays constant. Deactivated by `set_speed`.

## References

- `doc/beaglebone_architecture.md` — architecture and pinmux rationale
- `.github/copilot-instructions.md` — coding guidelines and conventions
- `src/pru/README.md` — PRU firmware overview
- `src/pru/PRU_DEPLOY.md` — deployment scripts and runtime checks
- `src/dts/README.md` — overlay build/deploy instructions
