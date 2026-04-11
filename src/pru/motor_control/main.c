/* motor_control/main.c — motor firmware (runs on PRU1 at runtime).
 *
 * Architecture layer 1/4.  Loaded as am335x-pru1-fw → remoteproc2 → PRU1
 * (4a338000.pru).
 *
 * PRU1 is a dumb pulse generator. Its sole responsibilities:
 *   1. Read motor_params_t (written by PRU0/orchestrator) every iteration.
 *   2. Generate STEP/DIR pulses using the IEP hardware counter.
 *   3. Publish motor_telem_t for PRU0 (and host) to read.
 *
 * PRU1 owns and initializes the IEP timer. PRU0 reads IEP but never resets it.
 *
 * Pin mapping on PRU1 R30 — LCD data pins in MODE5 (pr1_pru1_pru_r30_N):
 *   Motor A — lateral (odd P8 pins):
 *     STEP_A = R30[0]  P8_45  pr1_pru1_pru_r30_0
 *     DIR_A  = R30[2]  P8_43  pr1_pru1_pru_r30_2
 *     EN_A   = R30[4]  P8_41  pr1_pru1_pru_r30_4  (active-low)
 *   Motor B — spindle (even P8 pins):
 *     STEP_B = R30[1]  P8_46  pr1_pru1_pru_r30_1
 *     DIR_B  = R30[3]  P8_44  pr1_pru1_pru_r30_3
 *     EN_B   = R30[5]  P8_42  pr1_pru1_pru_r30_5  (active-low)
 *
 * Endstops are read by PRU0 (orchestrator) via its R31 (P9_28/P9_30).
 * This firmware does NOT read R31 or perform any endstop logic.
 *
 * Confirmed via pinctrl debugfs: pad offsets 0x0A0-0x0B4, all MODE5.
 */

#include <stdint.h>
#include "../include/pru_ipc.h"
#include "../include/pru_stepper.h"
#include "../include/pru_regs.h"
#include "../include/pru_rsc_table.h"  /* required by remoteproc */

/* ── Pin bitmasks (PRU1 R30, confirmed via pinctrl debugfs) ─────────────── */
/* Motor A — lateral (odd P8 pins, MODE5, lcd_data[0/2/4]) */
#define LAT_STEP_BIT   (1u << 0)   /* P8_45 R30[0]  pr1_pru1_pru_r30_0     */
#define LAT_DIR_BIT    (1u << 2)   /* P8_43 R30[2]  pr1_pru1_pru_r30_2     */
#define LAT_EN_BIT     (1u << 4)   /* P8_41 R30[4]  pr1_pru1_pru_r30_4  ↓0=EN */
/* Motor B — spindle (even P8 pins, MODE5, lcd_data[1/3/5]) */
#define SP_STEP_BIT    (1u << 1)   /* P8_46 R30[1]  pr1_pru1_pru_r30_1     */
#define SP_DIR_BIT     (1u << 3)   /* P8_44 R30[3]  pr1_pru1_pru_r30_3     */
#define SP_EN_BIT      (1u << 5)   /* P8_42 R30[5]  pr1_pru1_pru_r30_5  ↓0=EN */

/* ── Telemetry publish cadence ───────────────────────────────────────────── */
#define TELEM_STRIDE   500000u     /* ~2.5 ms at 200 MHz (400 Hz) */

/* ── Shared RAM pointers ─────────────────────────────────────────────────── */
static volatile motor_params_t *params =
    (volatile motor_params_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_PARAMS_OFFSET);

static volatile motor_telem_t *telem =
    (volatile motor_telem_t *)(PRU_SRAM_PHYS_BASE + IPC_MOTOR_TELEM_OFFSET);

static volatile move_ctrl_t *move_ctrl =
    (volatile move_ctrl_t *)(PRU_SRAM_PHYS_BASE + IPC_MOVE_CTRL_OFFSET);

static volatile step_block_t *move_blocks =
    (volatile step_block_t *)(PRU_SRAM_PHYS_BASE + IPC_STEP_BLOCKS_OFFSET);

/* ── Pulse generators ────────────────────────────────────────────────────── */
static pulse_gen_t spindle = {0};
static pulse_gen_t lateral = {0};

static uint8_t  g_prev_lat_pin     = 0u; /* for rising-edge detection        */

/* ── StepBlock execution state (lateral axis) ───────────────────────────── */
static uint8_t  g_lat_exec_active = 0u;
static uint8_t  g_lat_blk_idx = 0u;
static uint16_t g_lat_blk_left = 0u;
static uint32_t g_lat_blk_iv = 0u;
static int16_t  g_lat_blk_add = 0;

static void lat_exec_start(void)
{
    if (!move_ctrl->start_flag || !move_ctrl->block_ready)
        return;
    if (move_ctrl->block_count == 0u) {
        move_ctrl->start_flag = 0u;
        move_ctrl->done_flag  = 1u;
        move_ctrl->active     = 0u;
        telem->lat_move_done  = 1u;
        return;
    }

    g_lat_blk_idx    = 0u;
    g_lat_blk_iv     = move_blocks[0].interval;
    g_lat_blk_left   = move_blocks[0].count;
    g_lat_blk_add    = move_blocks[0].add;
    g_lat_exec_active = 1u;

    params->lat_interval = g_lat_blk_iv;
    params->lat_enable   = 1u;
    params->lat_run      = 1u;

    move_ctrl->executed_steps = 0u;
    move_ctrl->active         = 1u;
    move_ctrl->done_flag      = 0u;
    move_ctrl->start_flag     = 0u;
    telem->lat_move_done      = 0u;
}

/* On each lateral step, update interval with the compressed block add.
 * This is intentionally the only arithmetic used for profile execution:
 *   interval += add
 */
static void lat_exec_on_step(void)
{
    if (!g_lat_exec_active) return;

    if (g_lat_blk_left > 0u) {
        g_lat_blk_left--;
        move_ctrl->executed_steps++;
    }

    {
        int32_t iv = (int32_t)g_lat_blk_iv + (int32_t)g_lat_blk_add;
        if (iv < (int32_t)MIN_INTERVAL_CYC) iv = (int32_t)MIN_INTERVAL_CYC;
        if (iv > (int32_t)MAX_INTERVAL_CYC) iv = (int32_t)MAX_INTERVAL_CYC;
        g_lat_blk_iv = (uint32_t)iv;
        params->lat_interval = g_lat_blk_iv;
    }

    if (g_lat_blk_left == 0u) {
        g_lat_blk_idx++;
        if (g_lat_blk_idx < move_ctrl->block_count) {
            g_lat_blk_iv   = move_blocks[g_lat_blk_idx].interval;
            g_lat_blk_left = move_blocks[g_lat_blk_idx].count;
            g_lat_blk_add  = move_blocks[g_lat_blk_idx].add;
            params->lat_interval = g_lat_blk_iv;
        } else {
            g_lat_exec_active     = 0u;
            params->lat_run       = 0u;
            move_ctrl->active     = 0u;
            move_ctrl->done_flag  = 1u;
            move_ctrl->block_ready = 0u;
            telem->lat_move_done  = 1u;
        }
    }
}

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
    telem->sp_step_count     = spindle.step_count;
    telem->lat_step_count    = lateral.step_count;
    telem->lat_position      = lateral.position;
    telem->sp_interval_actual  = spindle.interval;
    telem->lat_interval_actual = lateral.interval;

    /* Spindle state */
    uint8_t sp_st = MOTOR_STATE_IDLE;
    if (spindle.running) sp_st = MOTOR_STATE_RUNNING;
    if (!(__R30 & SP_EN_BIT)) sp_st |= MOTOR_STATE_ENABLED;
    telem->sp_state = sp_st;

    /* Lateral state */
    uint8_t lat_st = MOTOR_STATE_IDLE;
    if (lateral.running) lat_st = MOTOR_STATE_RUNNING;
    if (!(__R30 & LAT_EN_BIT)) lat_st |= MOTOR_STATE_ENABLED;
    telem->lat_state = lat_st;

    /* endstop_mask is 0: endstops are owned by PRU0 (orchestrator) */
    telem->endstop_mask = 0u;
    telem->seq++;
}

/* ════════════════════════════════════════════════════════════════════════════
 * Main loop
 * ════════════════════════════════════════════════════════════════════════════ */
int main(void) {
    /* Safe state: STEP/DIR low, both drivers disabled (EN active-low → HIGH). */
    __R30 = 0u;                                    /* all bits low first      */
    __R30 |= (SP_EN_BIT | LAT_EN_BIT);            /* disable drivers (HIGH)  */

    /* PRU0 owns the IEP timer. */
    IEP_INIT();

    /* Zero shared memory areas owned by PRU0. */
    *telem = (motor_telem_t){0};

    uint32_t loop_cnt = 0u;

    while (1) {
        uint32_t now = IEP_NOW();

        /* ── 0. Arm compressed move execution if requested ─────────────── */
        lat_exec_start();

        /* ── 1. Read motor parameters from PRU1 ─────────────────────────── */
        uint32_t sp_iv   = params->sp_interval;
        uint32_t lat_iv  = params->lat_interval;
        uint8_t  sp_dir  = params->sp_dir;
        uint8_t  lat_dir = params->lat_dir;
        uint8_t  sp_en   = params->sp_enable;
        uint8_t  lat_en  = params->lat_enable;
        uint8_t  sp_run  = params->sp_run;
        uint8_t  lat_run = params->lat_run;

        /* ── 2. Apply enable outputs ────────────────────────────────────── */
        apply_enable_sp(sp_en);
        apply_enable_lat(lat_en);

        /* ── 3. Apply direction outputs ─────────────────────────────────── */
        apply_dir_sp(sp_dir);
        apply_dir_lat(lat_dir);


        /* ── 4. Spindle pulse generation ────────────────────────────────── */
        uint8_t sp_pin = pulse_update(&spindle, sp_iv, sp_dir, sp_run, now);
        if (sp_pin) __R30 |= SP_STEP_BIT; else __R30 &= ~SP_STEP_BIT;

        /* ── 5. Lateral pulse generation ────────────────────────────────── */
        uint8_t lat_pin = pulse_update(&lateral, lat_iv, lat_dir, lat_run, now);
        if (lat_pin) __R30 |= LAT_STEP_BIT; else __R30 &= ~LAT_STEP_BIT;

        /* ── 5b. Compressed StepBlock update on each step ──────────────── */
        if (lat_pin && !g_prev_lat_pin && g_lat_exec_active) {
            lat_exec_on_step();
        }
        g_prev_lat_pin = lat_pin;

        /* ── 6. Publish telemetry (throttled) ───────────────────────────── */
        if (++loop_cnt >= TELEM_STRIDE) {
            loop_cnt = 0u;
            publish_telem();
        }
    }

    return 0;
}
