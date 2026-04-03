/* pru_ipc.h — IPC shared memory layout: PRU firmware ↔ Linux host.
 *
 * Architecture: Klipper-inspired host-planned step generation.
 *   The Linux host pre-computes step moves as (interval, count, add) triples
 *   and fills per-axis ring buffers in PRU Shared RAM.
 *   Each PRU consumes its move ring using the 200 MHz IEP hardware counter
 *   for cycle-accurate edge timing — no software ramp computation on-PRU.
 *
 * Rules:
 *   - C-compatible only (no C++). Included from PRU C and Linux C++.
 *   - All structs __attribute__((packed, aligned(4))).
 *   - All PRU-side pointers must be declared volatile.
 *   - PRU_SRAM_PHYS_BASE is overridden to 0x00010000u by the PRU Makefile so
 *     the same offset macros work for both Linux host-mmap and PRU local access.
 *
 * Memory: PRU Shared RAM — 12 KB
 *   PRU local base : 0x00010000
 *   Host phys base : 0x4A310000  (AM335x TRM §4.4.1.2.3)
 *
 * Layout (offsets from base):
 *   0x0000–0x01FF  Command ring        32 × 16 B = 512 B
 *   0x0200–0x0203  cmd_whead
 *   0x0204–0x0207  cmd_rhead
 *   0x0210–0x023F  Spindle telemetry   48 B
 *   0x0240–0x026F  Lateral telemetry   48 B
 *   0x0270–0x027F  Inter-PRU sync      16 B
 *   0x0280–0x0283  sp_move_whead
 *   0x0284–0x0287  sp_move_rhead
 *   0x0288–0x0A87  Spindle move ring   128 × 16 B = 2048 B
 *   0x0A88–0x0A8B  lat_move_whead
 *   0x0A8C–0x0A8F  lat_move_rhead
 *   0x0A90–0x128F  Lateral move ring   128 × 16 B = 2048 B
 *   Total: ~4.75 KB < 12 KB limit
 */

#ifndef PRU_IPC_H
#define PRU_IPC_H

#include <stdint.h>

/* ── Physical base (Linux /dev/mem). Overridden to 0x00010000u for PRU fw. ──*/
#ifndef PRU_SRAM_PHYS_BASE
#  define PRU_SRAM_PHYS_BASE   0x4A310000u
#endif
#define PRU_SRAM_SIZE          0x3000u   /* 12 KB */

/* Shared clock constant — used by both PRU firmware and Linux host (MoveQueue). */
#define PRU_CLOCK_HZ           200000000u

/* ── IPC area offsets ────────────────────────────────────────────────────────*/
#define IPC_CMD_RING_OFFSET        0x0000u
#define IPC_CMD_RING_SLOTS         32u
#define IPC_CMD_WHEAD_OFFSET       0x0200u
#define IPC_CMD_RHEAD_OFFSET       0x0204u

#define IPC_SPINDLE_TELEM_OFFSET   0x0210u
#define IPC_LATERAL_TELEM_OFFSET   0x0240u
#define IPC_SYNC_OFFSET            0x0270u

#define IPC_SP_MOVE_WHEAD_OFFSET   0x0280u
#define IPC_SP_MOVE_RHEAD_OFFSET   0x0284u
#define IPC_SP_MOVE_RING_OFFSET    0x0288u
#define IPC_SP_MOVE_SLOTS          128u

#define IPC_LAT_MOVE_WHEAD_OFFSET  0x0A88u
#define IPC_LAT_MOVE_RHEAD_OFFSET  0x0A8Cu
#define IPC_LAT_MOVE_RING_OFFSET   0x0A90u
#define IPC_LAT_MOVE_SLOTS         128u

/* -------------------------------------------------------------------------
 * TMC UART request/response ring (PRU ↔ Linux host)
 * Each slot is 16 bytes. Host pushes requests to the REQ ring; PRU writes
 * responses to the RESP ring. This is a lightweight mailbox suitable for
 * short TMC frames (up to 8 data bytes) used by single-wire UART drivers.
 */
#define IPC_TMCUART_REQ_WHEAD_OFFSET  0x1290u
#define IPC_TMCUART_REQ_RHEAD_OFFSET  0x1294u
#define IPC_TMCUART_REQ_RING_OFFSET   0x1298u
#define IPC_TMCUART_REQ_SLOTS         8u

#define IPC_TMCUART_RESP_WHEAD_OFFSET 0x1318u
#define IPC_TMCUART_RESP_RHEAD_OFFSET 0x131Cu
#define IPC_TMCUART_RESP_RING_OFFSET  0x1320u
#define IPC_TMCUART_RESP_SLOTS        8u

/* Flags for TMC mailbox messages (host <-> PRU). Keep simple and stable. */
#define TMC_FLAG_SINGLE_WIRE  (1u << 0)
#define TMC_FLAG_PULLUP       (1u << 1)

typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  len;      /* payload length in bytes (0..8) */
    uint8_t  type;     /* 0=read,1=write,2=response */
    uint8_t  flags;    /* TMC_FLAG_* bits */
    uint8_t  chip;     /* chip id / device address (host-defined) */
    uint8_t  reg;      /* register id (host-defined) */
    uint8_t  _pad[3];  /* align to 8 bytes before payload */
    uint8_t  data[8];  /* payload */
} pru_tmcuart_msg_t; /* 16 bytes */

#ifdef __STDC_VERSION__
#  if __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(pru_tmcuart_msg_t) == 16, "pru_tmcuart_msg_t");
#  endif
#endif

/* ── Axis identifiers ────────────────────────────────────────────────────────*/
#define AXIS_SPINDLE   0u
#define AXIS_LATERAL   1u
#define AXIS_ALL       0xFFu

/* ── Command opcodes ──────────────────────────────────────────────────────────
 * Operational/lifecycle control only. Step generation is exclusively via
 * the per-axis move rings. Hz/accel/start/stop commands are gone: the host
 * pre-computes all step timing in MoveQueue before pushing moves.           */
#define CMD_NOP              0u
#define CMD_ENABLE           1u  /* value_a: 1=enable driver, 0=disable      */
#define CMD_EMERGENCY_STOP   2u  /* immediate stop + disable, both axes      */
#define CMD_RESET_POSITION   3u  /* reset step_count + position (value_a=axis)*/
#define CMD_HOME_START       4u  /* PRU1: enter homing mode (sensor-stop)    */
#define CMD_QUEUE_FLUSH      5u  /* discard pending moves (value_a=axis)     */

/* ── State flags (pru_axis_telem_t.state) ───────────────────────────────────*/
#define STATE_IDLE      (1u << 0)
#define STATE_RUNNING   (1u << 1)
#define STATE_AT_HOME   (1u << 4)
#define STATE_FAULT     (1u << 5)
#define STATE_ENABLED   (1u << 6)
#define STATE_HOMING    (1u << 7)   /* lateral: CMD_HOME_START in progress */

/* ── Fault flags (pru_axis_telem_t.faults) ──────────────────────────────────*/
#define FAULT_HOME_SENSOR    (1u << 0)  /* NO+NC consistency error           */
#define FAULT_OVERRUN        (1u << 1)  /* position exceeded software limit  */
#define FAULT_WATCHDOG       (1u << 2)  /* reserved                          */
#define FAULT_MOVE_UNDERRUN  (1u << 3)  /* move ring drained while running   */

/* ── Command struct  Linux → PRU  (16 bytes) ─────────────────────────────────*/
typedef struct __attribute__((packed, aligned(4))) {
    uint8_t  cmd;
    uint8_t  axis;
    uint8_t  flags;
    uint8_t  _pad0;
    uint32_t value_a;
    uint32_t value_b;
    uint32_t _reserved;
} pru_cmd_t;

/* ── Move struct  Linux → PRU  (16 bytes) ────────────────────────────────────
 *
 * Klipper-style step move triple:
 *   interval  IEP ticks between consecutive STEP-pin edge toggles  (200 MHz)
 *   count     total edge toggles to emit  (= 2 × physical steps)
 *   add       signed change to interval after each toggle  (linear ramp)
 *   direction 0 = forward, 1 = backward
 *
 * PRU step engine: after each toggle —
 *   next_edge_time += interval;   // advance by CURRENT interval (Klipper order)
 *   interval += add;              // update for next edge
 *
 * Host computes: interval = PRU_CLOCK_HZ / (2 × target_step_hz)
 */
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t interval;   /* IEP ticks between consecutive edges              */
    uint32_t count;      /* total edges to emit  (= 2 × physical steps)     */
    int32_t  add;        /* per-edge change to interval, signed              */
    uint8_t  direction;  /* 0 = forward, 1 = backward                       */
    uint8_t  flags;      /* reserved                                         */
    uint16_t _pad;
} pru_move_t;            /* 16 bytes */

/* ── Per-axis telemetry  PRU → Linux  (48 bytes) ─────────────────────────────*/
typedef struct __attribute__((packed, aligned(4))) {
    uint32_t seq;               /* monotone counter, wraps at UINT32_MAX     */
    uint32_t step_count;        /* steps since last CMD_RESET_POSITION       */
    uint32_t current_interval;  /* IEP interval of last edge (speed proxy)   */
    uint32_t moves_pending;     /* move-ring slots not yet consumed          */
    int32_t  position_steps;    /* signed step position (lateral only)       */
    uint16_t state;             /* OR of STATE_*                             */
    uint16_t faults;            /* OR of FAULT_*                             */
    uint32_t _reserved[6];
} pru_axis_telem_t;             /* 48 bytes */

/* ── Inter-PRU sync  (16 bytes) ──────────────────────────────────────────────*/
typedef struct __attribute__((packed, aligned(4))) {
    volatile uint32_t spindle_interval;  /* PRU0: current IEP interval       */
    volatile uint32_t lateral_interval;  /* PRU1: current IEP interval       */
    volatile uint32_t control_flags;     /* reserved                         */
    volatile uint32_t _reserved;
} pru_sync_t;

/* ── Compile-time size checks ────────────────────────────────────────────────*/
#ifdef __STDC_VERSION__
#  if __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(pru_cmd_t)        == 16, "pru_cmd_t");
_Static_assert(sizeof(pru_move_t)       == 16, "pru_move_t");
_Static_assert(sizeof(pru_axis_telem_t) == 48, "pru_axis_telem_t");
_Static_assert(sizeof(pru_sync_t)       == 16, "pru_sync_t");
#  endif
#endif

#endif /* PRU_IPC_H */
