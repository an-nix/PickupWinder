/* motor_control/main.c — PRU1 motor firmware (dumb pulse generator).
 *
 * Architecture layer 1/4.  Loaded as am335x-pru1-fw → remoteproc2 → PRU1
 * (4a338000.pru).
 *
 * PRU1 is a DUMB pulse generator.  Its sole responsibilities:
 *   1. Read motor_ctl_t from motor_params_t (written by PRU0).
 *   2. Detect ramp_arm=1 → arm pulse_set_ramp(), clear ramp_arm=0.
 *   3. Generate STEP/DIR pulses using the IEP hardware counter.
 *   4. When a ramp segment completes (accel_count==0): write final interval
 *      back to params (prevents speed jump), set seg_done=1 in telem.
 *   5. Publish motor_telem_t for PRU0 (and host) to read.
 *
 * PRU1 knows NOTHING about:
 *   - Segment arrays (ramp_seg_t[]) — those are managed by PRU0
 *   - Host commands, homing, coordination — all PRU0's job
 *   - Which operation is in progress (set_speed ramp vs move_to)
 *
 * PRU1 owns and initialises the IEP timer.  PRU0 reads IEP but never resets.
 *
 * Motor indexing (from pru_ipc.h):
 *   MOTOR_0 = spindle (Motor B, even P8 pins)
 *   MOTOR_1 = lateral (Motor A, odd  P8 pins)
 *
 * Pin mapping on PRU1 R30 — MODE5 (pr1_pru1_pru_r30_N):
 *   Motor A — lateral (odd P8 pins):
 *     STEP_A = R30[0]  P8_45     DIR_A  = R30[2]  P8_43
 *     EN_A   = R30[4]  P8_41  (active-low)
 *   Motor B — spindle (even P8 pins):
 *     STEP_B = R30[1]  P8_46     DIR_B  = R30[3]  P8_44
 *     EN_B   = R30[5]  P8_42  (active-low)
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_stepper.h"
#include "../include/pru_regs.h"
#include "../include/pru_rsc_table.h"  /* required by remoteproc */

/* ── Pin bitmasks (PRU1 R30) ─────────────────────────────────────────────── */
#define LAT_STEP_BIT   (1u << 0)   /* P8_45 R30[0]  MOTOR_1 lateral step    */
#define SP_STEP_BIT    (1u << 1)   /* P8_46 R30[1]  MOTOR_0 spindle step    */
#define LAT_DIR_BIT    (1u << 2)   /* P8_43 R30[2]  MOTOR_1 lateral dir     */
#define SP_DIR_BIT     (1u << 3)   /* P8_44 R30[3]  MOTOR_0 spindle dir     */
#define LAT_EN_BIT     (1u << 4)   /* P8_41 R30[4]  MOTOR_1 lateral en (↓)  */
#define SP_EN_BIT      (1u << 5)   /* P8_42 R30[5]  MOTOR_0 spindle en (↓)  */

/* ── Telemetry publish cadence ───────────────────────────────────────────── */
#define TELEM_STRIDE   500000u     /* ~2.5 ms at 200 MHz (≈400 Hz)          */

/* ── Shared RAM pointers ─────────────────────────────────────────────────── */
static volatile motor_params_t *params =
    (volatile motor_params_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_PARAMS_OFFSET);

static volatile motor_telem_t *telem =
    (volatile motor_telem_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_TELEM_OFFSET);

/* ── Pulse generators ────────────────────────────────────────────────────── */
static pulse_gen_t spindle = {0};
static pulse_gen_t lateral = {0};

/* ── Ramp segment arrays (shared RAM, written by ARM daemon) ─────────────── */
static volatile ramp_seg_t *g_ramp_segs[2];

/* ── Per-axis ramp tracking ──────────────────────────────────────────────── *
 * PRU1 now self-advances through ramp segments without any PRU0 handshake.
 *
 * When ramp_arm > 0 (written by PRU0), PRU1 arms seg[0] from motor_ctl_t
 * and stores the total segment count.  When each segment completes
 * (accel_count==0), PRU1 immediately loads the next segment from g_ramp_segs
 * without waiting for PRU0.  Only after ALL segments complete does PRU1
 * write the final interval back to params and set seg_done=1 so PRU0 fires
 * the EVENT_SPEED_REACHED / EVENT_MOVE_COMPLETE event.
 *
 * This eliminates the race-prone per-segment ramp_arm/seg_done handshake
 * that previously caused alternating stalls under certain timing conditions.*/
static uint8_t g_ramping[2]   = {0, 0};
static uint8_t g_seg_idx[2]   = {0, 0};
static uint8_t g_seg_total[2] = {0, 0};

/* ── Apply direction GPIO ────────────────────────────────────────────────── */
static inline void apply_dir_sp(uint8_t dir) {
    if (dir) __R30 |= SP_DIR_BIT; else __R30 &= ~SP_DIR_BIT;
}
static inline void apply_dir_lat(uint8_t dir) {
    if (dir) __R30 |= LAT_DIR_BIT; else __R30 &= ~LAT_DIR_BIT;
}

/* ── Apply enable GPIO (active-low) ─────────────────────────────────────── */
static inline void apply_enable_sp(uint8_t en) {
    if (en) __R30 &= ~SP_EN_BIT; else __R30 |= SP_EN_BIT;
}
static inline void apply_enable_lat(uint8_t en) {
    if (en) __R30 &= ~LAT_EN_BIT; else __R30 |= LAT_EN_BIT;
}

/* ── Publish telemetry ───────────────────────────────────────────────────── */
static void publish_telem(void) {
    /* Spindle (MOTOR_0) */
    telem->motor[MOTOR_0].step_count      = spindle.step_count;
    telem->motor[MOTOR_0].position        = spindle.position;
    telem->motor[MOTOR_0].interval_actual = spindle.interval;

    uint8_t sp_st = MOTOR_STATE_IDLE;
    if (spindle.running) sp_st = MOTOR_STATE_RUNNING;
    if (!(__R30 & SP_EN_BIT)) sp_st |= MOTOR_STATE_ENABLED;
    telem->motor[MOTOR_0].state = sp_st;

    /* Lateral (MOTOR_1) */
    telem->motor[MOTOR_1].step_count      = lateral.step_count;
    telem->motor[MOTOR_1].position        = lateral.position;
    telem->motor[MOTOR_1].interval_actual = lateral.interval;

    uint8_t lat_st = MOTOR_STATE_IDLE;
    if (lateral.running) lat_st = MOTOR_STATE_RUNNING;
    if (!(__R30 & LAT_EN_BIT)) lat_st |= MOTOR_STATE_ENABLED;
    telem->motor[MOTOR_1].state = lat_st;

    /* Endstop mask: 0 — endstops are owned by PRU0 (orchestrator) */
    telem->endstop_mask = 0u;
    /* seg_done flags are written directly by the ramp completion code */
    telem->seq++;
}

/* ════════════════════════════════════════════════════════════════════════════
 * Main loop
 * ════════════════════════════════════════════════════════════════════════════ */
int main(void) {
    /* Safe state: STEP/DIR low, both drivers disabled (EN active-low → HIGH) */
    __R30 = 0u;
    __R30 |= (SP_EN_BIT | LAT_EN_BIT);

    /* PRU1 owns the IEP timer. */
    IEP_INIT();

    /* Zero shared memory area owned by PRU1. */
    *telem = (motor_telem_t){0};

    /* Segment arrays in shared RAM (written by ARM daemon via mmap). */
    g_ramp_segs[MOTOR_0] =
        (volatile ramp_seg_t *)(PRU_SRAM_PHYS_BASE + IPC_RAMP_SEGS_0_OFFSET);
    g_ramp_segs[MOTOR_1] =
        (volatile ramp_seg_t *)(PRU_SRAM_PHYS_BASE + IPC_RAMP_SEGS_1_OFFSET);

    uint32_t loop_cnt = 0u;

    while (1) {
        uint32_t now = IEP_NOW();

        /* ── 1. Check ramp arm for each axis ────────────────────────────── *
         * PRU0 loads a segment by writing {interval, ramp_add, ramp_count}
         * and setting ramp_arm=1.  We arm the pulse generator and clear
         * ramp_arm to acknowledge.  This is the ONLY way PRU1 learns about
         * new ramp segments — it never reads the segment arrays directly.   */

        if (params->motor[MOTOR_0].ramp_arm) {
            /* ramp_arm holds total segment count written by PRU0. */
            g_seg_total[MOTOR_0] = params->motor[MOTOR_0].ramp_arm;
            g_seg_idx[MOTOR_0]   = 0u;
            pulse_set_ramp(&spindle,
                           params->motor[MOTOR_0].interval,
                           params->motor[MOTOR_0].ramp_add,
                           params->motor[MOTOR_0].ramp_count);
            params->motor[MOTOR_0].ramp_arm = 0u;
            telem->motor[MOTOR_0].seg_done  = 0u;
            g_ramping[MOTOR_0] = 1u;
        }

        if (params->motor[MOTOR_1].ramp_arm) {
            g_seg_total[MOTOR_1] = params->motor[MOTOR_1].ramp_arm;
            g_seg_idx[MOTOR_1]   = 0u;
            pulse_set_ramp(&lateral,
                           params->motor[MOTOR_1].interval,
                           params->motor[MOTOR_1].ramp_add,
                           params->motor[MOTOR_1].ramp_count);
            params->motor[MOTOR_1].ramp_arm = 0u;
            telem->motor[MOTOR_1].seg_done  = 0u;
            g_ramping[MOTOR_1] = 1u;
        }

        /* ── 2. Read motor parameters from PRU0 ─────────────────────────── */
        uint32_t sp_iv   = params->motor[MOTOR_0].interval;
        uint32_t lat_iv  = params->motor[MOTOR_1].interval;
        uint8_t  sp_dir  = params->motor[MOTOR_0].dir;
        uint8_t  lat_dir = params->motor[MOTOR_1].dir;
        uint8_t  sp_en   = params->motor[MOTOR_0].enable;
        uint8_t  lat_en  = params->motor[MOTOR_1].enable;
        uint8_t  sp_run  = params->motor[MOTOR_0].run;
        uint8_t  lat_run = params->motor[MOTOR_1].run;

        /* ── 3. Apply enable + direction outputs ─────────────────────────── */
        apply_enable_sp(sp_en);
        apply_enable_lat(lat_en);
        apply_dir_sp(sp_dir);
        apply_dir_lat(lat_dir);

        /* ── 4. Spindle pulse generation ────────────────────────────────── */
        uint8_t sp_pin = pulse_update(&spindle, sp_iv, sp_dir, sp_run, now);
        if (sp_pin) __R30 |= SP_STEP_BIT; else __R30 &= ~SP_STEP_BIT;

        /* ── 4b. Spindle ramp segment completion ────────────────────────── *
         * When accel_count reaches 0, the current segment is done.
         * If more segments remain, load the next one directly from shared RAM
         * — no PRU0 involvement, no race window between segments.
         * Only when ALL segments complete: write final interval and signal
         * PRU0 via seg_done=1 to fire EVENT_SPEED_REACHED.                 */
        if (g_ramping[MOTOR_0] && spindle.accel_count == 0u) {
            g_seg_idx[MOTOR_0]++;
            if (g_seg_idx[MOTOR_0] < g_seg_total[MOTOR_0]) {
                /* Load next segment directly from shared RAM. */
                volatile ramp_seg_t *s = &g_ramp_segs[MOTOR_0][g_seg_idx[MOTOR_0]];
                pulse_set_ramp(&spindle, s->start_iv, s->add, s->count);
                /* g_ramping stays 1; do NOT write params->interval here
                 * (pulse_set_ramp already sets spindle.interval = s->start_iv,
                 *  so pulse_update with accel_count>0 won't force-load it)  */
            } else {
                /* All segments done — signal PRU0 to fire completion event. */
                params->motor[MOTOR_0].interval = spindle.interval;
                telem->motor[MOTOR_0].seg_done  = 1u;
                g_ramping[MOTOR_0] = 0u;
            }
        }

        /* ── 5. Lateral pulse generation ────────────────────────────────── */
        uint8_t lat_pin = pulse_update(&lateral, lat_iv, lat_dir, lat_run, now);
        if (lat_pin) __R30 |= LAT_STEP_BIT; else __R30 &= ~LAT_STEP_BIT;

        /* ── 5b. Lateral ramp segment completion ────────────────────────── */
        if (g_ramping[MOTOR_1] && lateral.accel_count == 0u) {
            g_seg_idx[MOTOR_1]++;
            if (g_seg_idx[MOTOR_1] < g_seg_total[MOTOR_1]) {
                volatile ramp_seg_t *s = &g_ramp_segs[MOTOR_1][g_seg_idx[MOTOR_1]];
                pulse_set_ramp(&lateral, s->start_iv, s->add, s->count);
            } else {
                params->motor[MOTOR_1].interval = lateral.interval;
                telem->motor[MOTOR_1].seg_done  = 1u;
                g_ramping[MOTOR_1] = 0u;
            }
        }

        /* ── 6. Publish telemetry (throttled) ───────────────────────────── */
        if (++loop_cnt >= TELEM_STRIDE) {
            loop_cnt = 0u;
            publish_telem();
        }
    }

    return 0;
}
