# PickupWinder — BeagleBone Black PRU winding controller

This repository contains the BeagleBone Black port of the PickupWinder coil-winding
controller. The current implementation targets the AM335x PRUs for real-time
stepper control and uses Linux for orchestration, telemetry, and recipe storage.

## Project scope

- **Hardware**: BeagleBone Black (AM335x), two stepper drivers (A4988/DRV8825),
  two eQEP encoders, dual home/diagnostic switches, UART1 for TMC2209 configuration.
- **Runtime architecture**:
  - `PRU0` owns the IEP timer and executes motor stepper timing for both axes.
  - `PRU1` handles orchestration/supervision, endstop sampling, and host-link duties.
  - The Linux host computes motion plans and pushes move triples to the PRUs.
- **Persistent data**: winding recipes and telemetry are managed by the Linux host.

## Active code paths

- `src/pru/` — PRU firmware sources, build scripts, deploy helpers, runtime tests.
- `src/dts/` — BeagleBone device tree overlay sources for pinmux configuration.
- `src/linux/` — Linux-side host application and IPC code for PRU coordination.
- `doc/` — architecture and deployment documentation.

### Legacy reference

- `resources/esp32/` contains the original ESP32/PlatformIO sources for reference
  only. Do not treat it as the active BeagleBone implementation.

## Canonical pin mapping

### PRU0 motor outputs

- `P9_25` → `EN_A`  (`PRU0 R30[7]`, active-low)
- `P9_29` → `DIR_A` (`PRU0 R30[1]`)
- `P9_31` → `STEP_A` (`PRU0 R30[0]`)

- `P9_41` → `EN_B`  (`PRU0 R30[6]`, active-low)
- `P9_28` → `DIR_B` (`PRU0 R30[3]`)
- `P9_30` → `STEP_B` (`PRU0 R30[2]`)

### Endstops / diagnostics

- `P8_15` → `ENDSTOP_1` (`PRU1 R31[15]`)
- `P8_16` → `ENDSTOP_2` (`PRU1 R31[14]`)

### Encoder inputs

- `P9_42 / P9_27` → `eQEP0` (encoder A)
- `P8_33 / P8_35` → `eQEP1` (encoder B)

### UART for TMC2209 config

- `P9_24` → UART1_TXD
- `P9_26` → UART1_RXD

## Build overview

### Compile DT overlays

From the repository root:

```bash
make
```

This compiles all `.dts` files from `src/dts/` into `build/dtbo/` with uppercase
DTBO names.

### PRU firmware build

See `src/pru/README.md` for PRU cross-compile instructions, build targets, and
deployment details.

### Linux host application

Linux-side sources are under `src/linux/`; build instructions are maintained in
`src/linux/CMakeLists.txt` and relevant docs.

## Deployment

- `src/dts/README.md` explains how to build and install the overlay files.
- `src/pru/PRU_DEPLOY.md` describes firmware deployment, scripts, and runtime tests.
- `src/pru/scripts/load_pru.sh` installs and starts the PRUs on the BBB.

## Notes

- The active BeagleBone port is the current target; all ESP32/PlatformIO content is
  legacy/reference only.
- The PRU firmware is split: `PRU0` for real-time motor control, `PRU1` for
  supervision and host communication.
- Do not use the old `resources/esp32` build instructions for production BBB work.

## Useful references

- `doc/beaglebone_architecture.md` — architecture and pinmux rationale.
- `src/pru/README.md` — PRU firmware overview and hardware mapping.
- `src/dts/README.md` — overlay build/deploy instructions.
- `src/pru/PRU_DEPLOY.md` — deploy scripts and runtime checks.
