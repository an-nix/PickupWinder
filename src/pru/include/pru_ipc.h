/* pru_ipc.h — IPC shared memory layout between PRU firmware and Linux userspace.
 *
 * RULES:
 *   - C-compatible only (no C++ features). Included from both PRU C and Linux C++.
 *   - All structs packed + aligned(4). Never rely on implicit padding.
 *   - All pointers into this area must be declared `volatile` on both sides.
 *
 * Memory: PRU Shared RAM
 *   PRU local address : 0x00010000
 *   Host physical     : 0x4A310000  (AM335x TRM §4.4.1.2.3)
 *   Size              : 12 KB
 *
 * Layout:
 *   0x0000 - 0x01FF   Command ring buffer  (32 × 16 B = 512 B)
 *   0x0200 - 0x0203   cmd_whead  (Linux writes, 0-31)
 *   0x0204 - 0x0207   cmd_rhead  (PRU updates, 0-31)
 *   0x0210 - 0x023F   spindle telemetry  (pru_axis_telem_t, 48 B)
 *   0x0240 - 0x026F   lateral telemetry  (pru_axis_telem_t, 48 B)
 *   0x0270 - 0x027F   inter-PRU sync  (pru_sync_t, 16 B)
 */

#ifndef PRU_IPC_H
#define PRU_IPC_H

#include <stdint.h>

/* ── Physical base address (used by Linux /dev/mem or prussdrv) ─────────────*/
#define PRU_SRAM_PHYS_BASE      0x4A310000u
#define PRU_SRAM_SIZE           0x3000u       /* 12 KB */

/* ── Offsets within shared RAM ───────────────────────────────────────────────*/
#define IPC_CMD_RING_OFFSET     0x0000u
#define IPC_CMD_RING_SLOTS      32u
#define IPC_CMD_WHEAD_OFFSET    0x0200u
#define IPC_CMD_RHEAD_OFFSET    0x0204u
#define IPC_SPINDLE_TELEM_OFFSET 0x0210u
#define IPC_LATERAL_TELEM_OFFSET 0x0240u
#define IPC_SYNC_OFFSET          0x0270u

/* ── Axis identifiers ────────────────────────────────────────────────────────*/
#define AXIS_SPINDLE  0u
#define AXIS_LATERAL  1u
#define AXIS_ALL      0xFFu

/* ── Command opcodes (cmd field of pru_cmd_t) ────────────────────────────────*/
#define CMD_NOP               0u
#define CMD_SPINDLE_SET_HZ    1u  /* value_a = target Hz */
#define CMD_SPINDLE_SET_ACCEL 2u  /* value_a = accel Hz/ms (uint32) */
#define CMD_SPINDLE_START     3u  /* value_a = 1:forward  0:backward */
#define CMD_SPINDLE_STOP      4u  /* controlled decel to 0 */
#define CMD_SPINDLE_FORCE     5u  /* immediate stop, keep enabled */
#define CMD_SPINDLE_ENABLE    6u  /* value_a = 1:enable  0:disable driver */
#define CMD_LATERAL_SET_HZ    7u  /* value_a = target Hz */
#define CMD_LATERAL_SET_ACCEL 8u  /* value_a = accel Hz/ms */
#define CMD_LATERAL_START     9u  /* value_a = 1:forward  0:backward */
#define CMD_LATERAL_STOP      10u
#define CMD_LATERAL_FORCE     11u
#define CMD_LATERAL_ENABLE    12u /* value_a = 1:enable  0:disable */
#define CMD_EMERGENCY_STOP    13u /* both axes: immediate, disable drivers */
#define CMD_RESET_POSITION    14u /* value_a = axis: reset step counter */
#define CMD_SET_NOMINAL_LAT   15u /* value_a = nominal lateral Hz for compensation */

/* ── State flags (state field of pru_axis_telem_t) ──────────────────────────*/
#define STATE_IDLE      (1u << 0)
#define STATE_RUNNING   (1u << 1)
#define STATE_ACCEL     (1u << 2)
#define STATE_DECEL     (1u << 3)
#define STATE_AT_HOME   (1u << 4)
#define STATE_FAULT     (1u << 5)
#define STATE_ENABLED   (1u << 6)

/* ── Fault flags (faults field of pru_axis_telem_t) ─────────────────────────*/
#define FAULT_HOME_SENSOR  (1u << 0)  /* NO+NC consistency error */
#define FAULT_OVERRUN      (1u << 1)  /* position exceeded software limit */
#define FAULT_WATCHDOG     (1u << 2)  /* no command from Linux for >500ms */
#define FAULT_CMD_OVERFLOW (1u << 3)  /* ring buffer was full, command lost */

/* ── Command struct (Linux → PRU) ────────────────────────────────────────────*/
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;         /* CMD_* opcode */
    uint8_t  axis;        /* AXIS_SPINDLE | AXIS_LATERAL | AXIS_ALL */
    uint8_t  flags;       /* reserved */
    uint8_t  _pad0;
    uint32_t value_a;     /* primary value (Hz, steps, …) */
    uint32_t value_b;     /* secondary value (accel, end position …) */
    uint32_t _reserved;
} pru_cmd_t; /* 16 bytes — must stay 16 bytes */

/* ── Per-axis telemetry struct (PRU → Linux) ─────────────────────────────────*/
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t seq;              /* monotone counter, wraps at UINT32_MAX */
    uint32_t step_count;       /* absolute step count since last CMD_RESET_POSITION */
    uint32_t current_hz;       /* real-time speed (Hz), updated every ramp tick */
    uint32_t target_hz;        /* commanded target speed (Hz) */
    int32_t  position_steps;   /* signed position (steps from home, lateral only) */
    uint16_t state;            /* OR of STATE_* flags */
    uint16_t faults;           /* OR of FAULT_* flags */
    uint32_t _reserved[2];
} pru_axis_telem_t; /* 48 bytes */

/* ── Inter-PRU synchronisation (both PRUs read+write) ───────────────────────*/
typedef struct __attribute__((packed, aligned(4))) {
    volatile uint32_t spindle_hz;      /* PRU0 writes, PRU1 may read */
    volatile uint32_t lateral_hz;      /* PRU1 writes, PRU0 reads for compensation */
    volatile uint32_t nominal_lat_hz;  /* Linux sets via CMD_SET_NOMINAL_LAT */
    volatile uint32_t control_flags;   /* bit 0: compensation enabled */
} pru_sync_t; /* 16 bytes */

/* ── Compile-time size assertions (C11 / clpru supports _Static_assert) ─────*/
#ifdef __STDC_VERSION__
#if __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(pru_cmd_t)       == 16, "pru_cmd_t must be 16 bytes");
_Static_assert(sizeof(pru_axis_telem_t)== 48, "pru_axis_telem_t must be 48 bytes");
_Static_assert(sizeof(pru_sync_t)      == 16, "pru_sync_t must be 16 bytes");
#endif
#endif

#endif /* PRU_IPC_H */
