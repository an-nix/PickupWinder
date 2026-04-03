# PRU Firmware — PickupWinder (BeagleBone Black)

This folder contains PRU firmware sources and build helpers for the AM335x
PRUs used by PickupWinder. Two PRU images are built:

- `build/am335x-pru0-fw` — PRU0 (spindle step generator)
- `build/am335x-pru1-fw` — PRU1 (lateral step generator + home sensor)

---

## Architecture: Klipper-inspired move rings

Step generation follows the Klipper model: the **Linux host** pre-computes
all step moves as `(interval, count, add)` triples and pushes them into
per-axis ring buffers in PRU Shared RAM. The PRUs are pure consumers — they
never compute speeds, ramps, or do divisions. All timing is driven by the
200 MHz IEP hardware counter.

```
Linux host                        PRU Shared RAM           PRU
─────────────────────────────     ─────────────────        ────────────────
MoveQueue::pushAccelSegment()  →  spindle move ring   →  PRU0 IEP edge loop
MoveQueue::pushConstantMs()    →  spindle move ring   →    (no __delay_cycles)
MoveQueue::pushDecelToStop()   →  lateral move ring   →  PRU1 IEP edge loop
```

**Move triple format** (`pru_move_t`, 16 bytes):
```c
uint32_t interval;   /* IEP cycles between two edges (min 625 = 3.125 µs) */
uint32_t count;      /* total edges to generate (= 2 × step_count)       */
int32_t  add;        /* per-edge interval delta (linear ramp, 0 = const)  */
uint8_t  direction;  /* 0 = forward, 1 = backward                         */
```

The edge algorithm (`pru_stepper.h`) at each edge:
```c
toggle STEP pin
stepper.count--
if (count == 0):
  pop next move from ring or stop (set underrun fault if ring drained)
  on success: next_edge_time += new.interval; interval += add  (Klipper pre-apply)
else:
  stepper.next_edge_time += stepper.interval
  stepper.interval        += stepper.add     /* Klipper exact order */
```

On move boundary, `interval += add` is pre-applied (matching Klipper's
`stepper_load_next`) so the first inter-edge gap of the new move uses the
ramped interval instead of the raw base interval.

---

## PRU0 — Spindle

**IEP timer owner**: PRU0 initialises the IEP counter at boot. PRU1 reads it.

Responsibilities:
- Own and initialise the IEP timer (`IEP_INIT()`)
- Consume move triples from the spindle move ring via `stepper_try_load_next()`
- Toggle `R30[0]` (STEP) with IEP-absolute timing
- Count absolute steps; publish `spindle_telem` and `pru_sync.spindle_interval`

GPIOs:
```
R30[0] → P9_31  SPINDLE_STEP
R30[1] → P9_29  SPINDLE_DIR
R30[2] → P9_30  SPINDLE_ENABLE (active low)
```

---

## PRU1 — Lateral + Sensors

**IEP timer reader**: PRU1 reads PRU0's IEP counter. Never resets it.

Responsibilities:
- Consume move triples from the lateral move ring
- Toggle `R30[0]` (STEP) with IEP-absolute timing
- Read dual-contact home sensor with IEP-based debounce (500 µs)
- In `homing_mode` (set by `CMD_HOME_START`): flush ring + stop when HOME_HIT
- Count signed position; detect overrun; publish `lateral_telem` and `pru_sync.lateral_interval`

GPIOs:
```
R30[0] → P8_45  LATERAL_STEP
R30[1] → P8_46  LATERAL_DIR
R30[2] → P8_43  LATERAL_ENABLE (active low)
R31[3] → P8_xx  HOME_NO  (LOW when at home)
R31[4] → P8_xx  HOME_NC  (HIGH when at home)
```

Home sensor logic:
```
NO=LOW + NC=HIGH  → home detected (homing latch)
NO=HIGH + NC=HIGH → FAULT (wiring fault)
Debounce: IEP_NOW()-based 500 µs window
```

---

## IPC Shared RAM Layout

Offsets defined in `include/pru_ipc.h`:
```
0x0000   512 B  Command ring (32 × 16 B)
0x0200     4 B  cmd_whead
0x0204     4 B  cmd_rhead
0x0210    48 B  spindle_telem  (pru_axis_telem_t)
0x0240    48 B  lateral_telem  (pru_axis_telem_t)
0x0270    16 B  pru_sync       (spindle_interval, lateral_interval)
0x0280     4 B  sp_move_whead
0x0284     4 B  sp_move_rhead
0x0288  2048 B  spindle move ring (128 × pru_move_t)
0x0A88     4 B  lat_move_whead
0x0A8C     4 B  lat_move_rhead
0x0A90  2048 B  lateral move ring (128 × pru_move_t)
```

## Command set (6 opcodes)

| Opcode               | Value | Description |
|----------------------|-------|-------------|
| `CMD_NOP`            | 0     | No operation |
| `CMD_ENABLE`         | 1     | Enable/disable driver (value_a=1/0) |
| `CMD_EMERGENCY_STOP` | 2     | Immediate stop; flush ring; clear STEP |
| `CMD_RESET_POSITION` | 3     | Reset step_count and position_steps to 0 |
| `CMD_HOME_START`     | 4     | PRU1: enter homing_mode |
| `CMD_QUEUE_FLUSH`    | 5     | Flush move ring (no emergency stop) |

---

## Key Headers

| Header              | Content |
|---------------------|---------|
| `include/pru_ipc.h` | Shared RAM layout, `pru_move_t`, `pru_axis_telem_t`, 6 CMD opcodes, fault flags |
| `include/pru_stepper.h` | IEP macros (`IEP_NOW`, `IEP_INIT`), `stepper_t` (with `underrun`), `move_ring_t`, inline step engine |

---

## Build & Toolchain

```bash
cd src/pru
make PRU_CC=/home/user/x-tools/pru-elf/bin/pru-gcc
```

Variables accepted by the Makefile:
- `PRU_CC` — full path to PRU GCC binary
- `TOOLCHAIN_DIR` — search dir for toolchains (default `$(HOME)/x-tools`)
- `PRU_SWPKG` — path to TI PRU software support headers (optional)

Artifacts:
- `build/am335x-pru0-fw`
- `build/am335x-pru1-fw`

Deploy to BeagleBone:
- `make install` — copies firmwares to `/lib/firmware` and restarts remoteproc
- `make deploy BBB_IP=... BBB_USER=... SSH_KEY=...` — copies via scp + ssh

---

## Safety & Real-time Rules

- PRU code never uses dynamic memory, floating point, OS calls, or division
- `__delay_cycles` is **forbidden** in the step loop — use IEP-absolute timing
- PRU0 initialises the IEP; PRU1 reads it only — never call `IEP_INIT()` in PRU1
- All shared structs use `__attribute__((packed, aligned(4)))` and `volatile`
- `CMD_EMERGENCY_STOP` must clear STEP within ≤2 PRU instructions
- Any layout change in `pru_ipc.h` must be reflected in Linux headers simultaneously

---

## Fault Detection

Both PRUs report fault flags in `pru_axis_telem_t.faults`:

| Flag                 | Bit | Description |
|----------------------|-----|-------------|
| `FAULT_HOME_SENSOR`  | 0   | NO+NC sensor consistency error (PRU1 only) |
| `FAULT_OVERRUN`      | 1   | Position exceeded software limit (PRU1 only) |
| `FAULT_WATCHDOG`     | 2   | Reserved |
| `FAULT_MOVE_UNDERRUN`| 3   | Move ring drained while stepper was running (both PRUs) |

`FAULT_MOVE_UNDERRUN` mirrors Klipper's "missed scheduling of next step"
shutdown. It indicates the host-side did not push moves fast enough to keep
the ring fed. The host should treat this as a winding error.

---

## Available PRU-capable pins (config-pin grep PRU)
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
