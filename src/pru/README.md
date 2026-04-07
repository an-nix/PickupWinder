# PRU Firmware — PickupWinder (BeagleBone Black)

Two PRU firmware images for the AM335x PRUs:

- `am335x-pru0-fw` — PRU0: dual-motor real-time pulse generator + endstop safety
- `am335x-pru1-fw` — PRU1: orchestrator / brain + host communication

---

## Architecture: Option A — Continuous Shared Parameters

PRU0 is a **dumb motor driver**. It reads motor parameters from PRU1 and
generates STEP/DIR/EN pulses via the IEP hardware counter. It knows nothing
about commands, homing, or the host.

PRU1 is the **orchestrator**. It reads commands from the host (via shared RAM),
manages the homing state machine, writes motor parameters for PRU0, and
publishes aggregated status for the daemon.

```
Host ──host_cmd_t──→ PRU1 ──motor_params_t──→ PRU0
                     PRU1 ←──motor_telem_t── PRU0
Host ←─pru_status_t─ PRU1
```

**No move rings.** Speed changes are instantaneous. The host sets target
frequencies (Hz); PRU1 converts to IEP intervals and writes `motor_params_t`.
PRU0 reads and generates continuous pulses.

---

## PRU0 — Motor Control (Layer 1)

**IEP timer owner**: PRU0 initializes the IEP counter at boot. PRU1 reads only.

Responsibilities:
- Initialize and own the IEP timer (`IEP_INIT()`)
- Read `motor_params_t` from PRU1 continuously
- Apply STEP/DIR/EN GPIO for both axes
- IEP-based pulse generation via `pulse_gen_t` engine (`pulse_update()`)
- Read R31 endstops — unconditional lateral safety stop
- Publish `motor_telem_t` (step counts, positions, faults, endstop mask)

**PRU0 does NOT**: process commands, perform homing, communicate with the host,
make decisions. It is a pure real-time motor driver.

### PRU0 Pin Table

| Header pin | PRU bit  | Function   | Notes |
|------------|----------|------------|-------|
| P9_25      | R30\[7\] | EN_A       | Spindle enable (active-low) |
| P9_29      | R30\[1\] | DIR_A      | Spindle direction |
| P9_31      | R30\[0\] | STEP_A     | Spindle step |
| P9_41      | R30\[6\] | EN_B       | Lateral enable (active-low) |
| P9_28      | R30\[3\] | DIR_B      | Lateral direction |
| P9_30      | R30\[2\] | STEP_B     | Lateral step |
| P8_15      | R31\[15\] | ENDSTOP_1 | Lateral endstop input |
| P8_16      | R31\[14\] | ENDSTOP_2 | Lateral endstop input |

### Pulse Generation Engine

The `pulse_gen_t` struct (in `pru_stepper.h`) implements continuous IEP-based
pulse generation. Each axis has its own `pulse_gen_t`:

```c
typedef struct {
    uint32_t next_edge;    /* IEP timestamp for next STEP toggle     */
    uint32_t interval;     /* IEP cycles between edges               */
    uint32_t step_count;   /* total steps since last reset           */
    int32_t  position;     /* signed position (steps)                */
    uint8_t  phase;        /* 0=rising, 1=falling                    */
    uint8_t  direction;    /* cached direction for position tracking */
    uint8_t  running;      /* actively generating pulses             */
    uint8_t  _pad;
} pulse_gen_t;
```

`pulse_update(&gen, step_bit, IEP_NOW())`:
- If not running or interval==0: skip.
- If IEP has passed `next_edge`: toggle STEP pin, count, advance next_edge.
- Handles both rising and falling edges (50% duty cycle).

---

## PRU1 — Orchestration (Layer 2)

**IEP timer reader**: PRU1 reads PRU0's IEP state. Never resets it.

Responsibilities:
- Read `host_cmd_t` from the daemon (via shared RAM)
- Process commands: SET_SPEED, ENABLE, ESTOP, HOME_START, ACK_EVENT, RESET_POS
- Own the homing state machine (IDLE → APPROACH → HIT)
- Write `motor_params_t` for PRU0 (intervals, directions, enable/run flags)
- Read `motor_telem_t` from PRU0
- Publish `pru_status_t` for the daemon (aggregated status + event flags)
- Detect events (endstop hit, home complete, fault) and signal the host

**PRU1 does NOT**: generate pulses, toggle STEP/DIR/EN pins, own the IEP timer,
read R31 endstops directly.

### Homing State Machine

```
IDLE ──(HOST_CMD_HOME_START)──→ APPROACH
  (lat_dir=HOMING_DIR, lat_interval=HOMING_INTERVAL, lat_run=1)

APPROACH ──(endstop_mask != 0)──→ HIT
  (lat_run=0, set EVENT_HOME_COMPLETE, PRU1_STATE_AT_HOME)

HIT ──(host acknowledges)──→ IDLE
```

---

## IPC Shared RAM Layout

Offsets from PRU Shared RAM base (PRU: `0x00010000`, Host: `0x4A310000`):

```
Offset   Size    Struct              Direction       Description
0x0000   64 B    host_cmd_t          Host → PRU1     Commands from daemon
0x0040   32 B    motor_params_t      PRU1 → PRU0     Motor intervals/dirs/flags
0x0060   64 B    motor_telem_t       PRU0 → PRU1     Step counts/positions/faults
0x00A0   64 B    pru_status_t        PRU1 → Host     Aggregated status for daemon
Total: 224 bytes (out of 12 KB available)
```

### Command Opcodes (host_cmd_t.cmd)

| Opcode                | Value | Description |
|-----------------------|-------|-------------|
| `HOST_CMD_NOP`        | 0     | Idle / acknowledged |
| `HOST_CMD_SET_SPEED`  | 1     | Set target IEP intervals + directions |
| `HOST_CMD_ENABLE`     | 2     | Enable/disable stepper drivers |
| `HOST_CMD_ESTOP`      | 3     | Emergency stop: immediate all-halt |
| `HOST_CMD_HOME_START` | 4     | Start lateral homing sequence |
| `HOST_CMD_ACK_EVENT`  | 5     | Acknowledge last PRU event |
| `HOST_CMD_RESET_POS`  | 6     | Reset step counters + position |

---

## Key Headers

| Header                  | Content |
|-------------------------|---------|
| `include/pru_ipc.h`    | Shared RAM layout, 4 IPC structs, HOST_CMD_* opcodes, state/fault/event flags |
| `include/pru_stepper.h` | IEP macros (`IEP_NOW`, `IEP_INIT`), `pulse_gen_t`, `pulse_update()`, `pulse_stop()` |
| `include/pru_regs.h`   | R30/R31 register aliases for STEP/DIR/EN/ENDSTOP pins |

---

## Build & Toolchain

```bash
cd src/pru
make                    # Build PRU0 + PRU1 firmware
make toolchain-info     # Show detected toolchain
make clean              # Remove artifacts
```

Variables:
- `PRU_CC` — full path to PRU GCC binary (auto-detected from `~/x-tools`)
- `TOOLCHAIN_DIR` — search dir for crosstool-NG toolchains (default: `$(HOME)/x-tools`)
- `PRU_SWPKG` — path to TI PRU software support headers (optional)
- `OUT_DIR` — output directory (default: `build`, overridden by root Makefile)

Artifacts:
- `$(OUT_DIR)/am335x-pru0-fw`
- `$(OUT_DIR)/am335x-pru1-fw`

Deploy to BeagleBone:
- `make install` — copy firmwares to `/lib/firmware`, restart remoteproc
- `make deploy BBB_IP=<ip>` — deploy via SSH/SCP

---

## Safety & Real-time Rules

- PRU code never uses dynamic memory, floating point, OS calls, or division
- `__delay_cycles` is **forbidden** in the step loop — use IEP-absolute timing
- PRU0 initializes the IEP; PRU1 reads it only — never call `IEP_INIT()` in PRU1
- All shared structs: `__attribute__((packed, aligned(4)))` and `volatile` pointers
- `HOST_CMD_ESTOP` must clear all STEP outputs and stop all axes immediately
- PRU0 endstop safety stop is unconditional — no override possible
- Any layout change in `pru_ipc.h` must update daemon + documentation simultaneously

---

## Fault Flags

| Flag                  | Bit | Description |
|-----------------------|-----|-------------|
| `FAULT_ENDSTOP_HIT`  | 0   | Endstop triggered lateral safety stop |
| `FAULT_OVERRUN`       | 1   | Position software limit exceeded |

## Event Types

| Event                 | Code | Description |
|-----------------------|------|-------------|
| `EVENT_NONE`          | 0    | No pending event |
| `EVENT_ENDSTOP_HIT`   | 1    | Lateral endstop triggered |
| `EVENT_HOME_COMPLETE` | 2    | Homing sequence finished |
| `EVENT_FAULT`         | 3    | Motor fault detected |

---

## Future TMC2209 UART Reservation

`P9_29` and `P9_31` are intentionally selected because they can be switched to
`pru_uart` TX later (TMC2209 UART mode). Keep this reservation unchanged unless
explicitly requested.
