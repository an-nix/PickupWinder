/* pru_ipc.h — Shared memory layout between Host / PRU1 / PRU0.
 *
 * Option A architecture — continuous shared-parameter motor control:
 *
 *   Host (daemon) writes commands + target speeds into shared RAM.
 *   PRU0 (orchestrator) reads host commands, owns homing/sync logic,
 *     writes motor_params that PRU1 consumes.
 *   PRU1 (motor driver) reads motor_params, generates STEP/DIR pulses
 *     via IEP, reads R31 endstops, publishes telemetry.
 *
 * Communication flow:
 *   Host → PRU0:  host_cmd_t  (commands: set_speed, enable, estop, home, etc.)
 *   PRU0 → PRU1:  motor_params_t  (intervals, directions, enable flags)
 *   PRU1 → PRU0:  motor_telem_t   (step counts, positions, endstop, faults)
 *   PRU0 → Host:  pru_status_t    (aggregated status for daemon to broadcast)
 *
 * Memory: PRU Shared RAM — 12 KB (AM335x)
 *   PRU local base : 0x00010000
 *   Host phys base : 0x4A310000
 *
 * Layout (offsets from base):
 *   0x0000  host_cmd_t            64 B   Host → PRU0
 *   0x0040  motor_params_t        32 B   PRU0 → PRU1
 *   0x0060  motor_telem_t         64 B   PRU1 → PRU0 (+ Host reads)
 *   0x00A0  pru_status_t          64 B   PRU0 → Host
 *   Total used: 224 bytes  (well within 12 KB)
 *
 * Rules:
 *   - C-compatible only (no C++). Included from PRU C and Linux C.
 *   - All structs packed, aligned(4).
 *   - All PRU-side pointers must be volatile.
 *   - PRU_SRAM_PHYS_BASE overridden to 0x00010000u by PRU Makefile.
 */

#ifndef PRU_IPC_H
#define PRU_IPC_H

#include <stdint.h>

/* ── Physical base ───────────────────────────────────────────────────────── */
#ifndef PRU_SRAM_PHYS_BASE
#  define PRU_SRAM_PHYS_BASE   0x4A310000u
#endif
#define PRU_SRAM_SIZE          0x3000u   /* 12 KB */

/* Shared clock constant */
#define PRU_CLOCK_HZ           200000000u
#define SP_IV_MIN              625u        /* fastest step: 200 MHz / (2 × 160 kHz) */
#define SP_IV_MAX              187500u     /* slowest step: 200 MHz / (2 ×  534 Hz) */

/* ── Area offsets ────────────────────────────────────────────────────────── */
#define IPC_HOST_CMD_OFFSET       0x0000u
#define IPC_MOTOR_PARAMS_OFFSET   0x0040u
#define IPC_MOTOR_TELEM_OFFSET    0x0060u
#define IPC_PRU_STATUS_OFFSET     0x00A0u

/* ── Axis identifiers ────────────────────────────────────────────────────── */
#define AXIS_SPINDLE   0u
#define AXIS_LATERAL   1u
#define AXIS_ALL       0xFFu

/* ── Host command opcodes (host_cmd_t.cmd) ───────────────────────────────── *
 * Host writes a command; PRU1 reads and acknowledges by setting cmd to NOP.
 * Host must poll cmd_ack or wait for cmd==NOP before writing next command.  */
#define HOST_CMD_NOP           0u   /* idle / no pending command             */
#define HOST_CMD_SET_SPEED     1u   /* set target speed: sp_interval_target,
                                       lat_interval_target, sp_dir, lat_dir  */
#define HOST_CMD_ENABLE        2u   /* enable/disable drivers (value_a=1/0)  */
#define HOST_CMD_ESTOP         3u   /* emergency stop: immediate all-halt    */
#define HOST_CMD_HOME_START    4u   /* start homing sequence (lateral)       */
#define HOST_CMD_ACK_EVENT     5u   /* Python acknowledged the last event    */
#define HOST_CMD_RESET_POS     6u   /* reset step counters + position        */
/* Set software limits for an axis. host_cmd_t.limit_min/limit_max hold
 * signed step positions (int32) for the axis indicated in host_cmd_t.axis.
 */
#define HOST_CMD_SET_LIMITS    7u   /* set axis software limits (min,max)      */
/* Move axis to absolute position with trapezoidal speed profile.
 * host_cmd_t.move_target, move_start_iv, move_cruise_iv, move_delta_iv.     */
#define HOST_CMD_MOVE_TO       8u   /* move axis to target position            */

/* ── PRU1 state flags (pru_status_t.pru1_state) ─────────────────────────── */
#define PRU1_STATE_IDLE        (1u << 0)
#define PRU1_STATE_HOMING      (1u << 1)
#define PRU1_STATE_RUNNING     (1u << 2)
#define PRU1_STATE_FAULT       (1u << 3)
#define PRU1_STATE_AT_HOME     (1u << 4)

/* ── Motor state flags (motor_telem_t.sp_state / lat_state) ──────────────── */
#define MOTOR_STATE_IDLE       (1u << 0)
#define MOTOR_STATE_RUNNING    (1u << 1)
#define MOTOR_STATE_ENABLED    (1u << 2)

/* ── Fault flags (motor_telem_t.sp_faults / lat_faults) ──────────────────── */
#define FAULT_ENDSTOP_HIT      (1u << 0)  /* endstop triggered safety stop   */
#define FAULT_OVERRUN          (1u << 1)  /* position software limit         */

/* ── Endstop mask bits (motor_telem_t.endstop_mask) ──────────────────────── */
#define ENDSTOP1_MASK          (1u << 0)
#define ENDSTOP2_MASK          (1u << 1)

/* ═══════════════════════════════════════════════════════════════════════════
 * Structures
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── Host → PRU1 command (64 bytes) ──────────────────────────────────────── */
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;                    /* HOST_CMD_* opcode                    */
    uint8_t  axis;                   /* AXIS_SPINDLE / LATERAL / ALL         */
    uint8_t  sp_dir;                 /* spindle direction (0=fwd, 1=rev)     */
    uint8_t  lat_dir;                /* lateral direction (0=fwd, 1=rev)     */
    uint32_t sp_interval_target;     /* target IEP interval for spindle      */
    uint32_t lat_interval_target;    /* target IEP interval for lateral      */
    uint32_t value_a;                /* general purpose (e.g. enable=1/0)    */
    uint32_t value_b;                /* general purpose                      */
    uint8_t  cmd_ack;                /* PRU1 echoes cmd opcode here on done  */
    uint8_t  _pad[3];
    int32_t  limit_min;              /* SET_LIMITS: signed limit min (steps)  */
    int32_t  limit_max;              /* SET_LIMITS: signed limit max (steps)  */
    int32_t  move_target;            /* MOVE_TO: target position (steps)      */
    uint32_t move_start_iv;          /* MOVE_TO: start/end IEP interval       */
    uint32_t move_cruise_iv;         /* MOVE_TO: cruise IEP interval (max spd)*/
    uint32_t move_delta_iv;          /* MOVE_TO: interval delta per step      */
    uint32_t move_sp_lat_coord;      /* MOVE_TO: Q6 spindle coord ratio       */
    uint32_t _reserved[3];           /* reserved                              */
} host_cmd_t;                        /* 64 bytes */

/* ── PRU0 → PRU1 motor parameters (32 bytes) ────────────────────────────── *
 * PRU0 updates these continuously. PRU1 reads them in its tight loop.
 * Changes take effect on the next PRU1 IEP cycle.                           */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t sp_interval;            /* IEP cycles between spindle edges     */
    uint32_t lat_interval;           /* IEP cycles between lateral edges     */
    uint8_t  sp_dir;                 /* spindle direction                    */
    uint8_t  lat_dir;                /* lateral direction                    */
    uint8_t  sp_enable;              /* 1=driver enabled, 0=disabled         */
    uint8_t  lat_enable;             /* 1=driver enabled, 0=disabled         */
    uint8_t  sp_run;                 /* 1=generate pulses, 0=idle            */
    uint8_t  lat_run;                /* 1=generate pulses, 0=idle            */
    uint8_t  lat_move_active;        /* 1=arm trapezoidal profile (PRU1 self-clears) */
    uint8_t  _pad;
    int32_t  lat_target_pos;         /* MOVE_TO: target position (steps)      */
    uint32_t lat_start_iv;           /* MOVE_TO: start/end IEP interval       */
    uint32_t lat_cruise_iv;          /* MOVE_TO: cruise IEP interval (max spd)*/
    uint32_t lat_delta_iv;           /* MOVE_TO: interval delta per step      */
} motor_params_t;                    /* 32 bytes */

/* ── PRU1 → PRU0 motor telemetry (64 bytes) ──────────────────────────────── *
 * PRU1 updates at ~40 kHz (every TELEM_STRIDE iterations).
 * PRU0 and Host read this data.                                             */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t sp_step_count;          /* spindle steps since last reset       */
    uint32_t lat_step_count;         /* lateral steps since last reset       */
    int32_t  lat_position;           /* lateral signed position (steps)      */
    uint32_t sp_interval_actual;     /* spindle current IEP interval         */
    uint32_t lat_interval_actual;    /* lateral current IEP interval         */
    uint8_t  sp_state;               /* MOTOR_STATE_* flags                  */
    uint8_t  lat_state;              /* MOTOR_STATE_* flags                  */
    uint8_t  sp_faults;              /* FAULT_* flags                        */
    uint8_t  lat_faults;             /* FAULT_* flags                        */
    uint8_t  endstop_mask;           /* bit0=ES1, bit1=ES2 (live reading)    */
    uint8_t  lat_move_done;          /* PRU1 sets 1 when move_to completes   */
    uint8_t  _pad[2];
    uint32_t seq;                    /* monotone counter (wraps)             */
    uint32_t _reserved[8];
} motor_telem_t;                     /* 64 bytes */

/* ── PRU0 → Host aggregated status (64 bytes) ────────────────────────────── *
 * PRU0 updates at its main loop cadence. Daemon reads and broadcasts.       */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t seq;                    /* monotone counter (wraps)             */
    uint8_t  pru1_state;             /* PRU1_STATE_* flags                   */
    uint8_t  event_pending;          /* 1 = event waiting for host ack       */
    uint8_t  event_type;             /* EVENT_* type code                    */
    uint8_t  _pad0;
    uint32_t sp_step_count;          /* copied from motor_telem              */
    uint32_t lat_step_count;         /* copied from motor_telem              */
    int32_t  lat_position;           /* copied from motor_telem              */
    uint32_t sp_interval_actual;     /* copied from motor_telem              */
    uint32_t lat_interval_actual;    /* copied from motor_telem              */
    uint8_t  endstop_mask;           /* copied from motor_telem              */
    uint8_t  sp_faults;              /* copied from motor_telem              */
    uint8_t  lat_faults;             /* copied from motor_telem              */
    uint8_t  _pad1;
    uint32_t _reserved[8];
} pru_status_t;                      /* 64 bytes */

/* ── Event types (pru_status_t.event_type) ───────────────────────────────── */
#define EVENT_NONE            0u
#define EVENT_ENDSTOP_HIT     1u     /* lateral endstop triggered            */
#define EVENT_HOME_COMPLETE   2u     /* homing sequence finished             */
#define EVENT_FAULT           3u     /* motor fault detected                 */
/* Software limit hit (position exceeded configured min/max) */
#define EVENT_LIMIT_HIT       4u
/* move_to profile completed: target position reached */
#define EVENT_MOVE_COMPLETE   5u

/* ── Compile-time size checks ────────────────────────────────────────────── */
#ifdef __STDC_VERSION__
#  if __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(host_cmd_t)     == 64, "host_cmd_t size");
_Static_assert(sizeof(motor_params_t) == 32, "motor_params_t size");
_Static_assert(sizeof(motor_telem_t)  == 64, "motor_telem_t size");
_Static_assert(sizeof(pru_status_t)   == 64, "pru_status_t size");
#  endif
#endif

#endif /* PRU_IPC_H */
