PRU Firmware Overview

This folder contains the PRU firmware sources and build helpers for the
AM335x PRUs used by PickupWinder. Two PRU images are built:

- PRU0 — Spindle (spindle step generator)
- PRU1 — Lateral (lateral carriage step generator + sensors)

PRU0 — Spindle (pru0_spindle)
- Responsibilities:
  - Generate STEP/DIR pulses for the spindle stepper motor.
  - Run the acceleration/deceleration ramp engine (fixed-point, deterministic).
  - Read lateral speed (from shared `pru_sync`) and apply spindle/lateral
    compensation so turns-per-mm remains constant during reversals.
  - Consume command ring buffer entries addressed to the spindle and publish
    spindle telemetry (speed, position, state) to shared PRU RAM.
- GPIOs used (configured by device-tree overlay, PRU R30):
  - `R30[0]` SPINDLE_STEP
  - `R30[1]` SPINDLE_DIR
  - `R30[2]` SPINDLE_ENABLE (active low)

PRU1 — Lateral (pru1_lateral)
- Responsibilities:
  - Generate STEP/DIR pulses for the lateral carriage.
  - Sample the dual-contact home sensor (NO+NC) with hardware debounce.
  - Detect reversals, enforce software position limits, and publish telemetry.
  - Write `lateral_hz` into shared `pru_sync` so PRU0 can apply compensation.
  - Consume command ring buffer entries addressed to the lateral axis.
- GPIOs used (R30 outputs, R31 inputs):
  - `R30[0]` LATERAL_STEP
  - `R30[1]` LATERAL_DIR
  - `R30[2]` LATERAL_ENABLE (active low)
  - `R31[3]` HOME_NO (LOW when at home)
  - `R31[4]` HOME_NC (HIGH when at home)

Shared RAM / IPC
- The PRUs and Linux userspace exchange a small IPC region in PRU shared RAM.
- Key offsets are defined in `include/pru_ipc.h` (command ring, telemetry for
  each axis, and a `pru_sync` area used for spindle/lateral synchronization).
- When compiling for the PRU target the makefile defines `PRU_SRAM_PHYS_BASE` to
  `0x00010000u` so firmware uses the local PRU RAM addresses.

Build & Toolchain
- The Makefile supports pointing to a crosstool-ng toolchain. Common variables:
  - `PRU_CC` — full path to the PRU GCC binary (e.g. `/home/user/x-tools/pru-elf/bin/pru-gcc`).
  - `TOOLCHAIN_DIR` — default search dir (`$(HOME)/x-tools`) used to locate a toolchain.
  - `PRU_SWPKG` — path to TI PRU software support package headers (optional).

- Example: build using an installed toolchain

```bash
cd src/pru
make PRU_CC=/home/nicolas/x-tools/pru-elf/bin/pru-gcc
```

- The Makefile will produce:
  - `build/am335x-pru0-fw`
  - `build/am335x-pru1-fw`

Deploying to a BeagleBone
- `make install` (run on the BeagleBone itself) copies the firmwares to
  `/lib/firmware` and restarts the PRUs via the `remoteproc` sysfs interface.
- `make deploy` (run from host) copies firmwares via `scp` and reloads PRUs via
  `ssh`. Use the variables `BBB_IP`, `BBB_USER`, and `SSH_KEY` to control
  connection details. The deploy target backs up existing remote firmwares
  to `.bak.<timestamp>` before installing.

Notes & Portability
- PRU firmware must avoid dynamic memory and floating point; it uses fixed
  point and integer math for determinism.
- All shared RAM structs are packed and aligned to 4 bytes for compatibility
  between Linux and PRU code (see `include/pru_ipc.h`).
- If you modify telemetry sizes or the shared layout, update both PRU and
  Linux headers simultaneously to avoid silent IPC corruption.

Troubleshooting
- If compilation fails with missing device-specs, try removing `-mmcu` or
  point `PRU_CC` to the crosstool-ng-built compiler found under
  PRU Firmware Overview

  This folder contains the PRU firmware sources and build helpers for the
  AM335x PRUs used by PickupWinder. Two PRU images are built and loaded as
  separate remoteproc instances:

  - `build/am335x-pru0-fw` — PRU0 (spindle)
  - `build/am335x-pru1-fw` — PRU1 (lateral)

  High-level responsibilities
  - PRU0 (spindle): generate STEP/DIR pulses for the spindle, run the ramp
    engine, apply spindle/lateral compensation, publish spindle telemetry.
  - PRU1 (lateral): generate STEP/DIR pulses for the lateral carriage, sample
    home sensors with debounce, detect reversals/limits, publish lateral
    telemetry and update `pru_sync.lateral_hz`.

  Source layout & entry points
  - `pru0_spindle/main.c` — entry `main()` for PRU0. Key functions and symbols:
    - `main()` — initialization then an infinite timed loop.
    - `process_command(const volatile pru_cmd_t*, uint32_t *nominal_spindle_hz)`
      — handle one command from the ring.
    - `apply_compensation(uint32_t nominal_spindle_hz)` — compute spindle target
      from `pru_sync->lateral_hz` and `pru_sync->nominal_lat_hz`.
    - `ramp_*` helpers from `include/pru_ramp.h` — ramp_tick / ramp_start etc.

  - `pru1_lateral/main.c` — entry `main()` for PRU1. Key functions and symbols:
    - `main()` — initialization then sensor+step loop.
    - sensor debounce and `home` detection code (DEBOUNCE_COUNT / read windows).
    - lateral position limit checks and reversal handling.

  PRU loop & timing
  - Both PRUs run a tight deterministic loop with a fixed `LOOP_CYCLES` value.
    Use `__delay_cycles()` (pru builtin) to pad cycles so the loop duration is
    constant. Timing constants live in each `main.c` (e.g. `LOOP_CYCLES`,
    `LOOP_OVERHEAD`, `TELEM_INTERVAL`).

  GPIO / PRU registers
  - PRU outputs are accessed through `r30` (compiler alias `__R30`) and inputs
    through `r31` (alias `__R31`). Mapping (tied to device-tree pinmux):
    - PRU0 (spindle): `R30[0]` STEP, `R30[1]` DIR, `R30[2]` ENABLE (active low)
    - PRU1 (lateral): `R30[0]` STEP, `R30[1]` DIR, `R30[2]` ENABLE (active low)
    - PRU1 sensors: `R31[3]` HOME_NO (LOW at home), `R31[4]` HOME_NC (HIGH at home)

  IPC / Shared RAM layout
  - Offsets and sizes are defined in `include/pru_ipc.h`. Important regions:
    - `IPC_CMD_RING_OFFSET` (ring of 32 × 16 B entries) — Linux writes commands
      here; PRUs read them.
    - `IPC_SPINDLE_TELEM_OFFSET` — `pru_axis_telem_t` for spindle (48 B)
    - `IPC_LATERAL_TELEM_OFFSET` — `pru_axis_telem_t` for lateral (48 B)
    - `IPC_SYNC_OFFSET` — `pru_sync_t` (16 B) for inter-PRU sync (lateral↔spindle)

  Data structures (important fields)
  - `pru_cmd_t` (16 bytes): `cmd` (opcode), `axis`, `flags`, `value_a`, `value_b`.
    Linux enqueues commands; PRU reads and executes.
  - `pru_axis_telem_t` (48 bytes): `seq`, `step_count`, `current_hz`, `target_hz`,
    `position_steps`, `state`, `faults`, reserved fields.
  - `pru_sync_t` (16 bytes): `spindle_hz`, `lateral_hz`, `nominal_lat_hz`,
    `control_flags` (bit 0 = compensation enabled).

  Command set (summary)
  - Commands are defined as `CMD_*` in `include/pru_ipc.h` and include:
    - Spindle commands: `CMD_SPINDLE_SET_HZ`, `CMD_SPINDLE_SET_ACCEL`,
      `CMD_SPINDLE_START`, `CMD_SPINDLE_STOP`, `CMD_SPINDLE_ENABLE`, ...
    - Lateral commands: `CMD_LATERAL_SET_HZ`, `CMD_LATERAL_SET_ACCEL`,
      `CMD_LATERAL_START`, `CMD_LATERAL_STOP`, `CMD_LATERAL_ENABLE`, ...
    - Global: `CMD_EMERGENCY_STOP`, `CMD_RESET_POSITION`, `CMD_SET_NOMINAL_LAT`

  Safety & real-time rules
  - PRU code never uses dynamic memory or floating point; all arithmetic is
    integer/fixed-point.
  - Emergency stop (`CMD_EMERGENCY_STOP`) must clear STEP outputs within a few
    PRU instructions and disable drivers.
  - Shared structs are `packed` + `aligned(4)` and accesses are via `volatile`.

  Building
  - Variables accepted by the Makefile:
    - `PRU_CC` — full path to PRU gcc (recommended). Example:
      `PRU_CC=/home/nicolas/x-tools/pru-elf/bin/pru-gcc`
    - `TOOLCHAIN_DIR` — search dir for toolchains (default `$(HOME)/x-tools`).
    - `PRU_SWPKG` — path to TI PRU support headers if needed.
    - `PRU_SRAM_PHYS_BASE` is defined by the Makefile to `0x00010000u` during
      PRU compilation so firmware uses local PRU RAM addresses.

  Example build:
  ```bash
  cd src/pru
  make PRU_CC=/home/nicolas/x-tools/pru-elf/bin/pru-gcc
  ```

  Artifacts
  - `build/am335x-pru0-fw`
  - `build/am335x-pru1-fw`

  Deploy
  - `make install` (run on the BeagleBone) copies the firmwares to `/lib/firmware`
    and restarts remoteproc instances.
  - `make deploy BBB_IP=... BBB_USER=... SSH_KEY=...` — copies firmwares via
    `scp` and reloads PRUs remotely; deploy backs up existing firmwares
    to `.bak.<timestamp>` before installing.

  Debug & diagnostics
  - On the BeagleBone: check `dmesg`, `/sys/class/remoteproc/remoteproc*/state`,
    and `/lib/firmware` filenames after deploy.
  - Use `objdump -D --target=binary -m pru build/am335x-pru0-fw` to inspect
    generated code (toolchain must support pru target).

  Notes
  - Any change to the shared memory layout requires simultaneous update to the
    Linux-side headers and PRU headers (`include/pru_ipc.h`).
  - See `include/pru_ramp.h` for the ramp engine implementation details and
    constraints (no division in inner loops, fixed-point ticks).

  Files to review for behavior details:
  - `pru0_spindle/main.c`
  - `pru1_lateral/main.c`
  - `include/pru_ipc.h`
  - `include/pru_ramp.h`

  Available PRU-capable pins (config-pin grep PRU)
  ------------------------------------------------
  The following is the list of header pins on the target system that reported
  PRU-capable modes when you ran `config-pin | grep PRU` (paste from your board):

      Available modes for P8_11 are: default gpio gpio_pu gpio_pd eqep pruout
      Available modes for P8_12 are: default gpio gpio_pu gpio_pd eqep pruout
      Available modes for P8_15 are: default gpio gpio_pu gpio_pd eqep pru_ecap_pwm pruin
      Available modes for P8_16 are: default gpio gpio_pu gpio_pd eqep pruin
      Available modes for P8_20 are: default gpio gpio_pu gpio_pd pruout pruin
      Available modes for P8_21 are: default gpio gpio_pu gpio_pd pruout pruin
      Available modes for P8_27 are: default gpio gpio_pu gpio_pd pruout pruin
      Available modes for P8_28 are: default gpio gpio_pu gpio_pd pruout pruin
      Available modes for P8_29 are: default gpio gpio_pu gpio_pd pruout pruin
      Available modes for P8_30 are: default gpio gpio_pu gpio_pd pruout pruin
      Available modes for P8_39 are: default gpio gpio_pu gpio_pd eqep pruout pruin
      Available modes for P8_40 are: default gpio gpio_pu gpio_pd eqep pruout pruin
      Available modes for P8_41 are: default gpio gpio_pu gpio_pd eqep pruout pruin
      Available modes for P8_42 are: default gpio gpio_pu gpio_pd eqep pruout pruin
      Available modes for P8_43 are: default gpio gpio_pu gpio_pd pwm pruout pruin
      Available modes for P8_44 are: default gpio gpio_pu gpio_pd pwm pruout pruin
      Available modes for P8_45 are: default gpio gpio_pu gpio_pd pwm pruout pruin
      Available modes for P8_46 are: default gpio gpio_pu gpio_pd pwm pruout pruin
      Available modes for P9_17 are: default gpio gpio_pu gpio_pd spi_cs i2c pwm pru_uart
      Available modes for P9_18 are: default gpio gpio_pu gpio_pd spi i2c pwm pru_uart
      Available modes for P9_19 are: default gpio gpio_pu gpio_pd spi_cs can i2c pru_uart timer
      Available modes for P9_20 are: default gpio gpio_pu gpio_pd spi_cs can i2c pru_uart timer
      Available modes for P9_21 are: default gpio gpio_pu gpio_pd spi uart i2c pwm pru_uart
      Available modes for P9_22 are: default gpio gpio_pu gpio_pd spi_sclk uart i2c pwm pru_uart
      Available modes for P9_24 are: default gpio gpio_pu gpio_pd uart can i2c pru_uart pruin
      Available modes for P9_25 are: default gpio gpio_pu gpio_pd eqep pruout pruin
      Available modes for P9_26 are: default gpio gpio_pu gpio_pd uart can i2c pru_uart pruin
      Available modes for P9_27 are: default gpio gpio_pu gpio_pd eqep pruout pruin
      Available modes for P9_28 are: default gpio gpio_pu gpio_pd spi_cs pwm pwm2 pruout pruin
      Available modes for P9_29 are: default gpio gpio_pu gpio_pd spi pwm pruout pruin
      Available modes for P9_30 are: default gpio gpio_pu gpio_pd spi pwm pruout pruin
      Available modes for P9_31 are: default gpio gpio_pu gpio_pd spi_sclk pwm pruout pruin

  Notes:
  - `pruout`/`pruin` indicate the pin can be configured for PRU output/input
    (pinmux mode 6). Use `config-pin` to test and a device-tree overlay to make
    the mapping permanent.
  - Avoid pins required by eMMC/HDMI/USB or other essential functions for your
    image. Verify before changing active boot pins.
