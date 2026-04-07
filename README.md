# PickupWinder — BeagleBone Black PRU winding controller

Automated/assisted guitar pickup coil winding on the BeagleBone Black (AM335x).
Real-time stepper control via PRU, with Linux daemon + Python application layer.

## Architecture — Option A (Continuous Shared Parameters)

4-layer stack. **No move rings.** Host writes target speeds; PRU1 orchestrates;
PRU0 generates pulses from continuous parameters.

```
Layer 4  Python application   (pickup_test.py, pru_client.py)
         │ Unix socket /run/pickup-winder.sock
Layer 3  C hardware daemon    (pickup_daemon)
         │ /dev/mem mmap
Layer 2  PRU1 orchestration   (host_cmd → motor_params, homing FSM)
         │ PRU Shared RAM
Layer 1  PRU0 motor control   (pulse_gen_t, STEP/DIR/EN, endstops)
```

**Communication flow** (224 bytes shared RAM):
```
Host ──host_cmd_t──→ PRU1 ──motor_params_t──→ PRU0
                     PRU1 ←──motor_telem_t── PRU0
Host ←─pru_status_t─ PRU1
```

See `doc/beaglebone_architecture.md` and `.github/copilot-instructions.md`
for the full architecture rationale.

## Hardware

- **Board**: BeagleBone Black (AM335x Cortex-A8 + 2× PRU)
- **Steppers**: Two A4988/DRV8825 drivers — spindle + lateral traverse
- **Microstepping**: 32 µstep (6400 steps/rev spindle, 3072 steps/mm lateral)
- **Encoders**: eQEP1 (P8.33, P8.35)
- **Endstops**: Dual-contact on PRU0 R31 (P8.15, P8.16)
- **Footswitch**: P9.23 (GPIO)

## Canonical Pin Mapping

### PRU0 motor outputs

| Header pin | PRU bit   | Function | Notes |
|------------|-----------|----------|-------|
| P9_25      | R30\[7\]  | EN_A     | Spindle enable (active-low) |
| P9_29      | R30\[1\]  | DIR_A    | Spindle direction |
| P9_31      | R30\[0\]  | STEP_A   | Spindle step |
| P9_41      | R30\[6\]  | EN_B     | Lateral enable (active-low) |
| P9_28      | R30\[3\]  | DIR_B    | Lateral direction |
| P9_30      | R30\[2\]  | STEP_B   | Lateral step |

### PRU0 endstop inputs

| Header pin | PRU bit   | Function   |
|------------|-----------|------------|
| P8_15      | R31\[15\] | ENDSTOP_1  |
| P8_16      | R31\[14\] | ENDSTOP_2  |

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
    pru0_motor_control/         PRU0: dumb pulse generator
    pru1_orchestration/         PRU1: orchestrator / brain
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
- `gcc` (ARM native or cross) for daemon
- `dtc` for device-tree overlays

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
| `set_speed`   | `sp_hz`, `lat_hz`, `sp_dir`, `lat_dir` | Set motor speeds (Hz) |
| `enable`      | `axis`, `value`                     | Enable/disable drivers |
| `e_stop`      |                                     | Emergency stop |
| `home_start`  |                                     | Start lateral homing |
| `reset_pos`   | `axis`                              | Reset step counters |
| `ack_event`   |                                     | Acknowledge event |

**Events** (daemon → Python):
| Event           | Description |
|-----------------|-------------|
| `endstop_hit`   | Lateral endstop triggered |
| `home_complete` | Homing sequence finished |
| `fault`         | Motor fault detected |
| `telem`         | Periodic telemetry broadcast |

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
- PRU firmware is split: PRU0 for real-time motor control, PRU1 for
  orchestration and host communication.
- Speed ramps (accel/decel) are managed host-side by Python sending
  progressive `set_speed` commands at ~10 ms cadence.

## References

- `doc/beaglebone_architecture.md` — architecture and pinmux rationale
- `.github/copilot-instructions.md` — coding guidelines and conventions
- `src/pru/README.md` — PRU firmware overview
- `src/pru/PRU_DEPLOY.md` — deployment scripts and runtime checks
- `src/dts/README.md` — overlay build/deploy instructions
