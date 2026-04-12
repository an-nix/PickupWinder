/* pru_ipc.h — Shared memory layout between Host / PRU0 / PRU1.
 *
 * Option A architecture — continuous shared-parameter motor control:
 *
 *   Host (daemon) writes commands + target speeds into shared RAM.
 *   PRU0 (orchestrator) reads host commands, owns sync logic,
 *     manages ramp segment sequences, writes motor_params for PRU1.
 *     PRU0 also reads R31 endstop pins and forwards raw pin state to
 *     pru_status_t — it does NOT interpret endstop state or stop motors.
 *     All endstop/homing decisions belong on the host daemon.
 *   PRU1 (motor driver) reads motor_params, generates STEP/DIR pulses
 *     via IEP.  PRU1 is a pure constant-time pulse generator: it executes
 *     one ramp_arm segment at a time, writes accel_count to telem every
 *     loop iteration so PRU0 can detect segment completion, and busy-waits
 *     for a fixed LOOP_CYCLES_TARGET to guarantee deterministic timing.
 *
 * Communication flow:
 *   Host → PRU0:  host_cmd_t      (commands: set_speed, enable, estop, …)
 *   PRU0 → PRU1:  motor_params_t  (intervals, directions, enable, ramp segs)
 *   PRU1 → PRU0:  motor_telem_t   (step counts, positions, accel_count)
 *   PRU0 → Host:  pru_status_t    (aggregated status for daemon broadcast)
 *
 * Motor abstraction:
 *   motor_t     — universal per-motor struct (28 B). Contains both control
 *                 and telemetry fields.  Used in motor_telem_t, pru_status_t.
 *                 Includes accel_count so PRU0 can detect segment completion
 *                 by watching this field reach 0.
 *   motor_cmd_t — per-motor command fields in host_cmd_t (8 B).
 *   motor_ctl_t — per-motor control in motor_params_t (16 B).
 *                 Includes ramp fields (ramp_arm, ramp_add, ramp_count) so
 *                 that acceleration is a generic, per-axis capability.
 *   Index convention: MOTOR_0 = 0 (spindle), MOTOR_1 = 1 (lateral).
 *
 * Ramp architecture (PRU0 advances segments, PRU1 is constant-time):
 *   The ARM (daemon) pre-computes ramp segments as ramp_seg_t[MAX_RAMP_SEGS]
 *   in shared RAM, one array per axis.  The daemon sends HOST_CMD_SET_SPEED
 *   (with seg_count>0) or HOST_CMD_MOVE_TO.  PRU0 (orchestrator) manages
 *   segment sequencing:
 *     1. Loads segment N into motor_ctl_t.{interval, ramp_add, ramp_count}
 *        and sets ramp_arm=1.
 *     2. PRU1 detects ramp_arm=1, calls pulse_set_ramp(), clears ramp_arm=0.
 *     3. PRU1 executes "interval += add" per step (Klipper inner loop).
 *        PRU1 writes accel_count to telem every loop (constant-time).
 *     4. When accel_count reaches 0, PRU0 detects it and loads segment N+1
 *        (back to step 1).  PRU1 does NOT read segment arrays.
 *     5. After last segment: PRU0 raises EVENT_SPEED_REACHED or
 *        EVENT_MOVE_COMPLETE depending on the operation type.
 *
 *   This keeps PRU1 as a dumb, constant-time pulse generator: it knows
 *   nothing about segment arrays, homing, or host commands.
 *
 * Memory: PRU Shared RAM — 12 KB (AM335x)
 *   PRU local base : 0x00010000
 *   Host phys base : 0x4A310000
 *
 * Layout (offsets from base):
 *   0x0000  host_cmd_t          64 B   Host → PRU0
 *   0x0040  motor_params_t      32 B   PRU0 → PRU1
 *   0x0060  motor_telem_t       64 B   PRU1 → PRU0 (+ Host reads)
 *   0x00A0  pru_status_t        64 B   PRU0 → Host
 *   0x00E0  ramp_seg_t[64]     768 B   Axis 0 (spindle) ramp segments
 *   0x03E0  ramp_seg_t[64]     768 B   Axis 1 (lateral) ramp/move segments
 *   Total used: 1760 bytes  (well within 12 KB)
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
#define SP_IV_MIN              625u        /* fastest: 200 MHz / (2 × 160 kHz) */
#define SP_IV_MAX              187500u     /* slowest: 200 MHz / (2 ×  534 Hz) */

/* ── Area offsets ────────────────────────────────────────────────────────── */
#define IPC_HOST_CMD_OFFSET       0x0000u
#define IPC_MOTOR_PARAMS_OFFSET   0x0040u
#define IPC_MOTOR_TELEM_OFFSET    0x0060u
#define IPC_PRU_STATUS_OFFSET     0x00A0u
#define IPC_RAMP_SEGS_0_OFFSET    0x00E0u   /* 768 B  ramp_seg_t[64] axis 0 */
#define IPC_RAMP_SEGS_1_OFFSET    0x03E0u   /* 768 B  ramp_seg_t[64] axis 1 */

/* ── Motor index convention ──────────────────────────────────────────────── */
#define MOTOR_0   0u   /* spindle (Motor B, even P8 pins) */
#define MOTOR_1   1u   /* lateral (Motor A, odd  P8 pins) */

/* ── Axis identifiers (host command addressing) ──────────────────────────── */
#define AXIS_SPINDLE   0u
#define AXIS_LATERAL   1u
#define AXIS_ALL       0xFFu

/* ── Ramp segments per axis ──────────────────────────────────────────────── *
 * 64 segments gives ≤3.1% boundary jump at 300 RPM, ≤1.6% at 600 RPM.     *
 * With slow acceleration (30 000 steps/s²), add=0 is unavoidable above     *
 * ~135 RPM (add≠0 requires a ≥ v³/100 MHz).  64 small force-load steps     *
 * produce a smooth macroscopic ramp the motor can follow.                  */
#define MAX_RAMP_SEGS  64u

/* ── Host command opcodes (host_cmd_t.cmd) ───────────────────────────────── *
 * Host writes a command; PRU0 reads and acknowledges by echoing opcode
 * into cmd_ack, then setting cmd=NOP.
 * Host must poll cmd_ack or wait for cmd==NOP before writing next command.  */
#define HOST_CMD_NOP           0u   /* idle / no pending command             */
#define HOST_CMD_SET_SPEED     1u   /* set target speed (ramp if segs loaded)*/
#define HOST_CMD_ENABLE        2u   /* enable/disable drivers (value_a=1/0)  */
#define HOST_CMD_ESTOP         3u   /* emergency stop: immediate all-halt    */
#define HOST_CMD_HOME_START    4u   /* start homing sequence (lateral)       */
#define HOST_CMD_ACK_EVENT     5u   /* Python acknowledged the last event    */
#define HOST_CMD_RESET_POS     6u   /* reset step counters + position        */
#define HOST_CMD_SET_LIMITS    7u   /* set axis software limits (min,max)    */
#define HOST_CMD_MOVE_TO       8u   /* move axis to target position          */
/* opcode 9 reserved (was RAMP_TO — removed, folded into SET_SPEED)         */

/* ── PRU state flags (pru_status_t.pru1_state) ───────────────────────────── */
#define PRU1_STATE_IDLE        (1u << 0)
#define PRU1_STATE_HOMING      (1u << 1)
#define PRU1_STATE_RUNNING     (1u << 2)
#define PRU1_STATE_FAULT       (1u << 3)
#define PRU1_STATE_AT_HOME     (1u << 4)

/* ── Motor state flags (motor_t.state) ───────────────────────────────────── */
#define MOTOR_STATE_IDLE       (1u << 0)
#define MOTOR_STATE_RUNNING    (1u << 1)
#define MOTOR_STATE_ENABLED    (1u << 2)

/* ── Fault flags (motor_t.faults) ────────────────────────────────────────── */
#define FAULT_ENDSTOP_HIT      (1u << 0)  /* endstop triggered safety stop   */
#define FAULT_OVERRUN          (1u << 1)  /* position software limit         */

/* ── Endstop mask bits (motor_telem_t.endstop_mask) ──────────────────────── */
#define ENDSTOP1_MASK          (1u << 0)
#define ENDSTOP2_MASK          (1u << 1)

/* ── Event types (pru_status_t.event_type) ───────────────────────────────── */
#define EVENT_NONE            0u
#define EVENT_ENDSTOP_HIT     1u     /* endstop pin asserted (host decides)  */
#define EVENT_HOME_COMPLETE   2u     /* homing complete (raised by host)     */
#define EVENT_FAULT           3u     /* motor fault detected                 */
#define EVENT_LIMIT_HIT       4u     /* software limit hit                   */
#define EVENT_MOVE_COMPLETE   5u     /* move_to profile completed            */
#define EVENT_SPEED_REACHED   6u     /* set_speed ramp completed             */
#define EVENT_ENDSTOP_CLEAR   7u     /* endstop returned to inactive state   */

/* ═══════════════════════════════════════════════════════════════════════════
 * Structures — motor abstraction
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── motor_t — universal per-motor struct (telemetry + status) ───────────── *
 * Contains both control echo and telemetry fields.  Used in motor_telem_t
 * and pru_status_t where the full field set is needed.
 *
 * accel_count: mirrors pulse_gen_t.accel_count (written by PRU1 every loop).
 *   PRU0 watches this field to detect ramp segment completion without any
 *   seg_done handshake flag — when accel_count reaches 0 while g_ramping[ax]
 *   is set, the segment is done and PRU0 loads the next one.
 *
 * seg_done: retained for backwards compatibility but no longer used by PRU0
 *   for segment advance; PRU1 no longer writes it.                          *
 *
 * SIZE CHANGED: 24 B → 28 B (added accel_count uint32_t, removed _pad[2]).  */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t interval;          /* IEP cycles between STEP edges             */
    uint8_t  dir;               /* direction: 0=fwd, 1=rev                   */
    uint8_t  enable;            /* driver enable: 0=disabled, 1=enabled      */
    uint8_t  run;               /* 1=generate pulses, 0=idle                 */
    uint8_t  seg_done;          /* retained; PRU1 no longer writes this      */
    uint32_t step_count;        /* steps since last reset                    */
    int32_t  position;          /* signed position in steps                  */
    uint32_t interval_actual;   /* current IEP interval (written each loop)  */
    uint32_t accel_count;       /* mirrors pulse_gen_t.accel_count (PRU1→PRU0)
                                   PRU0 watches: ==0 means segment complete  */
    uint8_t  state;             /* MOTOR_STATE_* flags                       */
    uint8_t  faults;            /* FAULT_* flags                             */
    uint8_t  _pad[2];           /* pad to 28 bytes                           */
} motor_t;                      /* 28 bytes  — SIZE CHANGED from 24 */

/* ── motor_cmd_t — per-motor command fields in host_cmd_t ────────────────── */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t interval_target;   /* target IEP interval                       */
    uint8_t  dir;               /* direction: 0=fwd, 1=rev                   */
    uint8_t  _pad[3];
} motor_cmd_t;                  /* 8 bytes */

/* ── motor_ctl_t — per-motor control for motor_params_t ──────────────────── *
 * Carries the control fields PRU0→PRU1 plus ramp segment parameters.
 *
 * Ramp handshake protocol:
 *   1. PRU0 writes {interval=start_iv, ramp_add, ramp_count, ramp_arm=1}
 *   2. PRU1 detects ramp_arm==1: arms pulse_gen, clears ramp_arm=0
 *   3. PRU1 executes segment (interval += add per step, count--)
 *   4. When segment completes: PRU1 writes params.interval=final_iv
 *      (prevents speed jump when pulse_update reads interval with
 *      accel_count==0) and sets telem.motor[].seg_done=1
 *   5. PRU0 detects seg_done==1 → loads next segment (back to 1)
 *
 * For constant-speed mode (SET_SPEED): PRU0 writes interval with
 * ramp_arm=0, ramp_count=0.  pulse_update reads interval directly.          */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t interval;          /* IEP interval / ramp start_iv              */
    uint8_t  dir;               /* direction: 0=fwd, 1=rev                   */
    uint8_t  enable;            /* driver enable: 0=disabled, 1=enabled      */
    uint8_t  run;               /* 1=generate pulses, 0=idle                 */
    uint8_t  ramp_arm;          /* 1=new ramp segment ready (PRU0→PRU1)      */
    int32_t  ramp_add;          /* per-step interval delta (Klipper style)   */
    uint32_t ramp_count;        /* steps in ramp segment (0=constant speed)  */
} motor_ctl_t;                  /* 16 bytes */

/* ═══════════════════════════════════════════════════════════════════════════
 * Structures — IPC areas
 * ═══════════════════════════════════════════════════════════════════════════ */

/* ── Host → PRU0 command (64 bytes) ──────────────────────────────────────── */
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t     cmd;                 /* HOST_CMD_* opcode                    */
    uint8_t     axis;                /* AXIS_SPINDLE / LATERAL / ALL         */
    uint8_t     cmd_ack;             /* PRU0 echoes cmd opcode here on done  */
    uint8_t     seg_count;           /* SET_SPEED/MOVE_TO: # valid ramp_seg_t  */
    motor_cmd_t motor[2];            /* per-motor target speed + direction   */
    uint32_t    value_a;             /* general purpose (e.g. enable=1/0)    */
    uint32_t    value_b;             /* general purpose                      */
    int32_t     limit_min;           /* SET_LIMITS: signed limit min (steps) */
    int32_t     limit_max;           /* SET_LIMITS: signed limit max (steps) */
    int32_t     move_target;         /* MOVE_TO: target position (steps)     */
    uint32_t    cruise_iv;           /* SET_SPEED/MOVE_TO: cruise IEP interval  */
    uint32_t    move_sp_lat_coord;   /* MOVE_TO: Q6 spindle coord ratio      */
    uint32_t    _reserved[4];        /* pad to 64 bytes                      */
} host_cmd_t;                        /* 64 bytes */

/* ── PRU0 → PRU1 motor parameters (32 bytes) ────────────────────────────── *
 * PRU0 updates these continuously.  PRU1 reads them in its tight loop.
 * Changes take effect on the next PRU1 IEP cycle.
 *
 * Each motor_ctl_t includes ramp fields (ramp_arm, ramp_add, ramp_count)
 * making acceleration a generic per-axis capability.  The orchestrator
 * feeds ramp segments one-by-one; PRU1 executes them as dumb
 * "interval += add" per step.                                               */
typedef struct __attribute__((packed, aligned(4))) {
    motor_ctl_t motor[2];            /* 32 B — per-motor control + ramp      */
} motor_params_t;                    /* 32 bytes */

/* ── PRU1 → PRU0 motor telemetry (64 bytes) ──────────────────────────────── *
 * PRU1 updates every main-loop iteration (no TELEM_STRIDE throttle).
 * PRU0 and Host read this data.
 *
 * Layout: motor[2]=56B + endstop_mask(1)+_pad[3] + seq(4) = 64B.
 * _reserved removed to accommodate the larger motor_t (28B each).          */
typedef struct __attribute__((packed, aligned(4))) {
    motor_t  motor[2];               /* 56 B — per-motor telemetry (28B each)*/
    uint8_t  endstop_mask;           /* bit0=ES1, bit1=ES2 (live reading)    */
    uint8_t  _pad[3];
    uint32_t seq;                    /* monotone counter (wraps), every loop */
} motor_telem_t;                     /* 64 bytes */

/* ── status_motor_t — per-motor status subset for pru_status_t ────────────── *
 * Identical to motor_t except accel_count is omitted — the host daemon does  *
 * not need to observe accel_count; that field is only used by PRU0.          *
 * Keeping this struct at 24 bytes preserves the pru_status_t at 64 bytes.   */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t interval;          /* IEP cycles between STEP edges             */
    uint8_t  dir;               /* direction: 0=fwd, 1=rev                   */
    uint8_t  enable;            /* driver enable: 0=disabled, 1=enabled      */
    uint8_t  run;               /* 1=generate pulses, 0=idle                 */
    uint8_t  _pad0;
    uint32_t step_count;        /* steps since last reset                    */
    int32_t  position;          /* signed position in steps                  */
    uint32_t interval_actual;   /* current IEP interval                      */
    uint8_t  state;             /* MOTOR_STATE_* flags                       */
    uint8_t  faults;            /* FAULT_* flags                             */
    uint8_t  _pad[2];
} status_motor_t;               /* 24 bytes */

/* ── PRU0 → Host aggregated status (64 bytes) ────────────────────────────── *
 * PRU0 updates at its main loop cadence. Daemon reads and broadcasts.
 *
 * Layout: seq(4) + {pru1_state,event_pending,event_type,endstop_mask}(4)
 *       + {endstop_pin_no,endstop_pin_nc,_status_pad[2]}(4)
 *       + status_motor[2](48) + _reserved[1](4) = 64B.
 *
 * endstop_pin_no/nc: raw R31 pin values forwarded by PRU0 every loop.
 *   PRU0 does NOT interpret these — the host daemon decides action.        */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t       seq;                  /* monotone counter (wraps)         */
    uint8_t        pru1_state;           /* PRU1_STATE_* flags               */
    uint8_t        event_pending;        /* 1 = event waiting for host ack   */
    uint8_t        event_type;           /* EVENT_* type code                */
    uint8_t        endstop_mask;         /* bit0=ES1,bit1=ES2 (aggregated)   */
    uint8_t        endstop_pin_no;       /* raw R31 NO pin value (0 or 1)    */
    uint8_t        endstop_pin_nc;       /* raw R31 NC pin value (0 or 1)    */
    uint8_t        _status_pad[2];       /* alignment pad                    */
    status_motor_t motor[2];             /* 48 B — per-motor status          */
    uint32_t       _reserved[1];         /* pad to 64 bytes                  */
} pru_status_t;                          /* 64 bytes */

/* ── Ramp segment (ARM planner → PRU shared RAM → PRU0 → PRU1) ──────────── *
 *
 * Universal ramp segment used for BOTH set_speed (speed change) and move_to
 * (position change).  Replaces the former sp_ramp_seg_t and step_block_t.
 *
 * The ARM fills ramp_seg_t[MAX_RAMP_SEGS] in shared RAM per axis.
 * PRU0 reads segments one-by-one and feeds them to PRU1 via motor_ctl_t.
 * PRU1 executes each segment using pulse_set_ramp() → "interval += add"
 * per step.  Force-loading start_iv at each segment boundary prevents
 * error accumulation (Klipper stepper_load_next equivalent).
 *
 * Execution on PRU1 (via pulse_gen_t):
 *   interval = start_iv;           // force-load at segment boundary
 *   for (s = 0; s < count; s++) {
 *       one_step(interval);
 *       interval += add;           // Klipper inner loop
 *   }
 *
 * See doc §2.8 and pru_stepper.h compute_accel_ramp() for derivation.      */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t start_iv;   /* IEP interval at segment start (force-loaded)     */
    int32_t  add;        /* signed interval delta per step (0 = cruise)      */
    uint32_t count;      /* number of steps in this segment (0 = skip)       */
} ramp_seg_t;                        /* 12 bytes × 64 = 768 bytes per axis */

/* ── Compile-time size checks ────────────────────────────────────────────── */
#ifdef __STDC_VERSION__
#  if __STDC_VERSION__ >= 201112L
/* motor_t: SIZE CHANGED 24→28 (added accel_count uint32_t, removed _pad[2]) */
_Static_assert(sizeof(motor_t)        == 28, "motor_t size");
_Static_assert(sizeof(status_motor_t) == 24, "status_motor_t size");
_Static_assert(sizeof(motor_cmd_t)    == 8,  "motor_cmd_t size");
_Static_assert(sizeof(motor_ctl_t)    == 16, "motor_ctl_t size");
_Static_assert(sizeof(host_cmd_t)     == 64, "host_cmd_t size");
_Static_assert(sizeof(motor_params_t) == 32, "motor_params_t size");
/* motor_telem_t: motor[2]=56B, endstop_mask+_pad[3]+seq = 8B → 64B total  */
_Static_assert(sizeof(motor_telem_t)  == 64, "motor_telem_t size");
/* pru_status_t: 4+4+4+48+4 = 64B (uses status_motor_t[2], not motor_t[2]) */
_Static_assert(sizeof(pru_status_t)   == 64, "pru_status_t size");
_Static_assert(sizeof(ramp_seg_t)     == 12, "ramp_seg_t size");
_Static_assert(MAX_RAMP_SEGS * sizeof(ramp_seg_t) == 768u,
               "ramp_seg_t[64] size");
#  endif
#endif

#endif /* PRU_IPC_H */
