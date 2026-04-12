/* pickup_daemon.c — PickupWinder hardware daemon.
 *
 * Architecture layer 3/4.
 *
 * Bridge between the Python application (layer 4) and PRU0 (layer 2).
 * Maps PRU Shared RAM via /dev/mem, writes commands into host_cmd_t,
 * reads pru_status_t, and forwards telemetry + events to Python over
 * a Unix domain socket.
 *
 * The daemon communicates ONLY with PRU0 (via shared RAM).  It never
 * writes motor_params or motor_telem — those are PRU0↔PRU1 internal.
 *
 * Socket protocol (UNIX, SOCK_STREAM, newline-delimited JSON):
 *
 *   Python → daemon (commands):
 *     {"cmd":"set_speed","sp_hz":N,"lat_hz":N,"sp_dir":0|1,"lat_dir":0|1}
 *       Spindle always ramps from current speed → target.  No explicit ramp
 *       command needed — the daemon computes and deposits ramp segments.
 *     {"cmd":"enable","axis":0|1|255,"value":0|1}
 *     {"cmd":"e_stop"}
 *     {"cmd":"home_start"}
 *     {"cmd":"ack_event"}
 *     {"cmd":"reset_pos","axis":0|1|255}
 *     {"cmd":"set_limits","axis":0|1,"min":<int>,"max":<int>}
 *     {"cmd":"move_to","axis":1,"pos":<int>}
 *     {"cmd":"set_mode","mode":"free"|"winding"}
 *     {"cmd":"set_accel","lat_max_speed":N,"lat_accel":N,"lat_decel":N,"sp_accel":N}
 *
 *   daemon → Python (responses):
 *     {"ok":true}
 *     {"ok":false,"error":"<reason>"}
 *
 *   daemon → Python (async events):
 *     {"event":"endstop_hit"}
 *     {"event":"home_complete"}
 *     {"event":"fault","sp_faults":N,"lat_faults":N}
 *     {"event":"telem","pru1_state":N,
 *      "sp":{"steps":N,"speed_hz":N,"faults":N},
 *      "lat":{"steps":N,"pos":N,"speed_hz":N,"faults":N},
 *      "endstop":N}
 *     {"event":"speed_reached","sp_hz":N}
 *     {"event":"move_complete","pos":N}
 *     {"event":"limit_hit","axis":N,"pos":N}
 *
 * Build:
 *   gcc -O2 -Wall -Wextra -I../../pru/include pickup_daemon.c -lm -o pickup_daemon
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>

#include "../../pru/include/pru_ipc.h"

/* ── Configuration ────────────────────────────────────────────────────────── */
#define SOCKET_PATH     "/run/pickup-winder.sock"
#define POLL_MS         10
#define MAX_CLIENTS     4
#define BUF_SZ          512
#define TELEM_EVERY     10   /* broadcast telem every N ticks → 100 ms */

/* ── Shared RAM pointers ──────────────────────────────────────────────────── */
static volatile host_cmd_t    *g_host_cmd     = NULL;
static volatile pru_status_t  *g_status       = NULL;
static volatile ramp_seg_t    *g_ramp_segs[2] = {NULL, NULL};

static void   *g_mmap_base = NULL;
static size_t  g_mmap_size = 0;
static int     g_mem_fd    = -1;

/* ── Client bookkeeping ───────────────────────────────────────────────────── */
static int g_clients[MAX_CLIENTS];
static int g_nclients = 0;

/* ── Shutdown flag ────────────────────────────────────────────────────────── */
static volatile int g_running = 1;

/* ── Event tracking (debounce) ────────────────────────────────────────────── */
static uint8_t  g_event_broadcast = 0u;  /* 1 = current event already broadcast */

/* ── Last spindle IEP interval (for move_to spindle coordination) ────────── */
static uint32_t g_last_sp_iv = 0u;

/* ── Last ramp target Hz (for speed_reached event broadcast) ─────────────── */
static uint32_t g_last_ramp_target_hz = 0u;

#define MAX_MOVE_PLAN_STEPS  24000

/* ──────────────────────────────────────────────────────────────────────────
 * ARM-side move planner (exact trapezoid physics + secant root solver)
 */
typedef struct {
    double t_start, t_accel_end, t_cruise_end, t_end;
    double v_start, v_cruise, v_end, accel, decel;
    int32_t start_pos;
} Move;

static double move_get_pos(const Move *m, double t)
{
    if (t <= m->t_start) return (double)m->start_pos;
    const double t_acc = m->t_accel_end - m->t_start;
    const double t_cruise = m->t_cruise_end - m->t_accel_end;
    const double dt = t - m->t_start;
    if (t <= m->t_accel_end) {
        return (double)m->start_pos + m->v_start * dt + 0.5 * m->accel * dt * dt;
    }
    const double s_acc = m->v_start * t_acc + 0.5 * m->accel * t_acc * t_acc;
    if (t <= m->t_cruise_end) {
        return (double)m->start_pos + s_acc + m->v_cruise * (dt - t_acc);
    }
    const double s_cruise = m->v_cruise * t_cruise;
    const double td = dt - t_acc - t_cruise;
    return (double)m->start_pos + s_acc + s_cruise
         + m->v_cruise * td - 0.5 * m->decel * td * td;
}

static double solve_step_time(const Move *m, int32_t target, double t_est)
{
    double t0 = t_est - (1.0 / ((m->v_cruise > 1.0) ? m->v_cruise : 1.0));
    double t1 = t_est;
    if (t0 < m->t_start) t0 = m->t_start;
    if (t1 < m->t_start) t1 = m->t_start;
    if (t1 > m->t_end)   t1 = m->t_end;
    double f0 = move_get_pos(m, t0) - (double)target;
    double f1 = move_get_pos(m, t1) - (double)target;
    for (int i = 0; i < 8; i++) {
        if (fabs(f1) < 0.5) break;
        const double den = f1 - f0;
        if (fabs(den) < 1e-12) break;
        double t2 = t1 - f1 * (t1 - t0) / den;
        if (t2 < m->t_start) t2 = m->t_start;
        if (t2 > m->t_end)   t2 = m->t_end;
        t0 = t1; f0 = f1;
        t1 = t2; f1 = move_get_pos(m, t1) - (double)target;
    }
    return t1;
}

/* Compress exact step intervals into ramp_seg_t triplets for PRU. */
static int compress_and_write(const Move *m, int32_t n_steps,
                              volatile ramp_seg_t *shram, int max_segs)
{
    static uint32_t ticks[MAX_MOVE_PLAN_STEPS + 1];
    static uint32_t iv[MAX_MOVE_PLAN_STEPS];

    if (n_steps <= 0 || n_steps > MAX_MOVE_PLAN_STEPS) return -1;

    double t_est = m->t_start + (1.0 / ((m->v_start > 1.0) ? m->v_start : 1.0));
    ticks[0] = (uint32_t)llround(m->t_start * ((double)PRU_CLOCK_HZ * 0.5));

    for (int32_t i = 0; i < n_steps; i++) {
        const int32_t target = m->start_pos + (i + 1);
        const double  t = solve_step_time(m, target, t_est);
        ticks[i + 1] = (uint32_t)llround(t * ((double)PRU_CLOCK_HZ * 0.5));
        t_est = t + (1.0 / ((m->v_cruise > 1.0) ? m->v_cruise : 1.0));
    }

    for (int32_t i = 0; i < n_steps; i++) {
        uint32_t d = ticks[i + 1] - ticks[i];
        if (d < 625u)      d = 625u;
        if (d > SP_IV_MAX) d = SP_IV_MAX;
        iv[i] = d;
    }

    int bi = 0;
    int32_t i = 0;
    while (i < n_steps) {
        if (bi >= max_segs) return -1;

        const uint32_t start_iv = iv[i];
        int32_t add = 0;
        if (i + 1 < n_steps)
            add = (int32_t)iv[i + 1] - (int32_t)iv[i];

        uint32_t count = 1u;
        int32_t j = i + 1;
        while (j < n_steps && count < 65535u) {
            int32_t d = (int32_t)iv[j] - (int32_t)iv[j - 1];
            if (d < add - 1 || d > add + 1) break;
            count++;
            j++;
        }

        shram[bi].start_iv = start_iv;
        shram[bi].count    = count;
        shram[bi].add      = add;
        bi++;
        i = j;
    }

    return bi;
}

static int build_move_profile(Move *m, int32_t n_steps,
                              uint32_t v_start_hz, uint32_t v_max_hz,
                              uint32_t accel_s2, uint32_t decel_s2)
{
    if (!m || n_steps <= 0) return -1;
    if (v_start_hz == 0u) v_start_hz = 534u;
    if (v_max_hz   == 0u) v_max_hz   = v_start_hz;
    if (v_max_hz < v_start_hz) v_max_hz = v_start_hz;
    if (accel_s2 == 0u) accel_s2 = 1u;
    if (decel_s2 == 0u) decel_s2 = 1u;

    const double vs = (double)v_start_hz;
    const double ve = vs;               /* end speed = start speed */
    double vc       = (double)v_max_hz;
    const double a  = (double)accel_s2;
    const double d  = (double)decel_s2;

    double d_acc = (vc * vc - vs * vs) / (2.0 * a);
    double d_dec = (vc * vc - ve * ve) / (2.0 * d);

    if (d_acc + d_dec > (double)n_steps) {
        /* Triangular profile: no cruise phase — solve for peak speed.
         * v_peak² = vs² + 2·a·d·n / (a + d)  (asymmetric case) */
        double vpk2 = vs * vs + 2.0 * a * d * (double)n_steps / (a + d);
        if (vpk2 < vs * vs) vpk2 = vs * vs;
        vc    = sqrt(vpk2);
        d_acc = (vc * vc - vs * vs) / (2.0 * a);
        d_dec = (vc * vc - ve * ve) / (2.0 * d);
        if (d_acc + d_dec > (double)n_steps) {  /* rounding edge case */
            d_acc = (double)n_steps * a / (a + d);
            d_dec = (double)n_steps - d_acc;
            vc    = sqrt(vs * vs + 2.0 * a * d_acc);
        }
    }

    const double d_cruise = (double)n_steps - d_acc - d_dec;
    const double t_acc    = (vc - vs) / a;
    const double t_dec    = (vc - ve) / d;
    const double t_cruise = (d_cruise > 0.0 && vc > 0.0) ? (d_cruise / vc) : 0.0;

    m->t_start      = 0.0;
    m->t_accel_end  = t_acc;
    m->t_cruise_end = t_acc + t_cruise;
    m->t_end        = t_acc + t_cruise + t_dec;
    m->v_start      = vs;
    m->v_cruise     = vc;
    m->v_end        = ve;
    m->accel        = a;
    m->decel        = d;
    m->start_pos    = 0;
    return 0;
}

/* ── Winding mode ────────────────────────────────────────────────────────── */
#define WINDING_MODE_FREE    0u
#define WINDING_MODE_WINDING 1u
static uint8_t g_winding_mode = WINDING_MODE_FREE;

/* ── Per-axis accel/speed configuration (updated by set_accel command) ───── */
static uint32_t g_lat_max_speed = 4000u;   /* Hz   lateral cruise speed      */
static uint32_t g_lat_accel     = 10000u;  /* steps/s² lateral acceleration  */
static uint32_t g_lat_decel     = 10000u;  /* steps/s² lateral deceleration  */
static uint32_t g_sp_accel      = 30000u; /* steps/s² spindle acceleration  */

static void sig_handler(int sig) { (void)sig; g_running = 0; }

static uint32_t hz_to_interval(uint32_t hz);

/* ════════════════════════════════════════════════════════════════════════════
 * PRU Shared RAM
 * ════════════════════════════════════════════════════════════════════════════ */

static int pru_map_open(const char *devmem) {
    g_mem_fd = open(devmem, O_RDWR | O_SYNC);
    if (g_mem_fd < 0) { perror("[daemon] open /dev/mem"); return -1; }

    const uint32_t page_off     = PRU_SRAM_PHYS_BASE & 0xFFFu;
    const uint32_t phys_aligned = PRU_SRAM_PHYS_BASE & ~0xFFFu;
    g_mmap_size = PRU_SRAM_SIZE + page_off;

    g_mmap_base = mmap(NULL, g_mmap_size,
                       PROT_READ | PROT_WRITE, MAP_SHARED,
                       g_mem_fd, (off_t)phys_aligned);
    if (g_mmap_base == MAP_FAILED) {
        perror("[daemon] mmap");
        close(g_mem_fd);
        return -1;
    }

    uint8_t *base = (uint8_t *)g_mmap_base + page_off;
    g_host_cmd     = (volatile host_cmd_t *)  (base + IPC_HOST_CMD_OFFSET);
    g_status       = (volatile pru_status_t *)(base + IPC_PRU_STATUS_OFFSET);
    g_ramp_segs[MOTOR_0] = (volatile ramp_seg_t *)(base + IPC_RAMP_SEGS_0_OFFSET);
    g_ramp_segs[MOTOR_1] = (volatile ramp_seg_t *)(base + IPC_RAMP_SEGS_1_OFFSET);

    return 0;
}

static void pru_map_close(void) {
    if (g_mmap_base && g_mmap_base != MAP_FAILED)
        munmap(g_mmap_base, g_mmap_size);
    if (g_mem_fd >= 0)
        close(g_mem_fd);
    g_mmap_base = NULL;
    g_mem_fd    = -1;
}

/* ── Send a host command and wait for PRU0 ack ───────────────────────────── */
static int send_host_cmd(uint8_t cmd, uint8_t axis,
                         uint32_t sp_iv, uint32_t lat_iv,
                         uint8_t sp_dir, uint8_t lat_dir,
                         uint32_t val_a) {
    if (!g_host_cmd) return -1;

    for (int i = 0; i < 5000; i++) {
        if (g_host_cmd->cmd == HOST_CMD_NOP) break;
        usleep(10);
    }
    if (g_host_cmd->cmd != HOST_CMD_NOP) return -1;

    g_host_cmd->axis                          = axis;
    g_host_cmd->motor[MOTOR_0].interval_target = sp_iv;
    g_host_cmd->motor[MOTOR_1].interval_target = lat_iv;
    g_host_cmd->motor[MOTOR_0].dir             = sp_dir;
    g_host_cmd->motor[MOTOR_1].dir             = lat_dir;
    g_host_cmd->value_a                        = val_a;
    g_host_cmd->cmd_ack                        = HOST_CMD_NOP;
    g_host_cmd->cmd                            = cmd;

    return 0;
}

/* Send limits command (signed min/max) */
static int send_host_set_limits(uint8_t axis, int32_t limit_min, int32_t limit_max) {
    if (!g_host_cmd) return -1;
    for (int i = 0; i < 5000; i++) {
        if (g_host_cmd->cmd == HOST_CMD_NOP) break;
        usleep(10);
    }
    if (g_host_cmd->cmd != HOST_CMD_NOP) return -1;

    g_host_cmd->axis      = axis;
    g_host_cmd->limit_min = limit_min;
    g_host_cmd->limit_max = limit_max;
    g_host_cmd->cmd_ack   = HOST_CMD_NOP;
    g_host_cmd->cmd       = HOST_CMD_SET_LIMITS;
    return 0;
}

/* ── Send move_to with ARM-computed exact trapezoid profile ──────────────── */
static int send_host_move_to(uint8_t axis, int32_t target_pos) {
    if (!g_host_cmd || !g_status) return -1;

    int32_t cur_pos = g_status->motor[MOTOR_1].position;
    int32_t delta   = target_pos - cur_pos;
    int32_t n_steps = (delta < 0) ? -delta : delta;

    if (n_steps == 0) return 0;

    /* Derive start speed from current lateral interval_actual. */
    uint32_t v0_hz = 534u;  /* fallback: min IEP speed ≈ PRU_CLOCK_HZ/(2×SP_IV_MAX) */
    {
        uint32_t iv = g_status->motor[MOTOR_1].interval_actual;
        if (iv > 0u && iv < SP_IV_MAX)
            v0_hz = PRU_CLOCK_HZ / (2u * iv);
    }

    Move mv;
    if (build_move_profile(&mv, n_steps, v0_hz, g_lat_max_speed,
                           g_lat_accel, g_lat_decel) != 0)
        return -1;

    int nblk = compress_and_write(&mv, n_steps,
                                  g_ramp_segs[MOTOR_1], (int)MAX_RAMP_SEGS);
    if (nblk <= 0) return -1;

    uint32_t cruise_iv = hz_to_interval(g_lat_max_speed);
    if (cruise_iv < 625u) cruise_iv = 625u;

    __sync_synchronize();  /* ARM memory barrier before cmd write */

    for (int i = 0; i < 5000; i++) {
        if (g_host_cmd->cmd == HOST_CMD_NOP) break;
        usleep(10);
    }
    if (g_host_cmd->cmd != HOST_CMD_NOP) return -1;

    g_host_cmd->axis              = axis;
    g_host_cmd->seg_count         = (uint8_t)nblk;
    g_host_cmd->move_target       = target_pos;
    g_host_cmd->cruise_iv         = cruise_iv;
    g_host_cmd->move_sp_lat_coord =
        (g_winding_mode == WINDING_MODE_WINDING &&
         g_last_sp_iv > 0u && cruise_iv > 0u)
        ? (g_last_sp_iv * 64u) / cruise_iv : 0u;
    g_host_cmd->cmd_ack           = HOST_CMD_NOP;
    g_host_cmd->cmd               = HOST_CMD_MOVE_TO;
    return 0;
}

/* ── Hz → IEP interval conversion ───────────────────────────────────────── */
static uint32_t hz_to_interval(uint32_t hz) {
    if (hz == 0u) return 0u;
    uint32_t iv = PRU_CLOCK_HZ / (2u * hz);
    return (iv < 625u) ? 625u : iv;
}

/* ── Klipper ramp computation (ARM-side, no PRU involvement) ─────────────── *
 * Divides the ramp uniformly in VELOCITY space (Δv-uniform): each sub-segment
 * covers the same Δv = total_Δv / MAX_RAMP_SEGS Hz.
 *
 * add=0 is mathematically unavoidable for v > ∛(a × 100 MHz) ≈ 135 RPM with
 * a=30 000 steps/s².  When add=0, the PRU runs at constant speed for `count`
 * steps, then PRU0 force-loads the NEXT segment's start_iv at the boundary.
 * This produces a staircase of 64 small speed steps — each step is:
 *
 *   jump_pct = dv / v_current = (total_Δv / 64) / v
 *
 * Examples for the test's incremental speed commands:
 *   32 000→64 000 Hz (300→600 RPM): dv_seg=500 Hz → jump=1.6% at 300 RPM  ✓
 *   32 000→96 000 Hz (300→900 RPM): dv_seg=1000 Hz → jump=3.1% at 300 RPM ✓
 *
 * DO NOT clamp cnt to |gap| to force add=±1: clamping makes the actual
 * acceleration 9–12× faster than specified (confirmed live: motor stalls).  */
static void compute_ramp(ramp_seg_t *segs,
                         uint32_t start_iv, uint32_t end_iv,
                         uint32_t accel_s2)
{
    const uint32_t F = PRU_CLOCK_HZ / 2u;   /* 100 MHz */
    uint32_t i;

    uint32_t v_s = F / start_iv;   /* Hz at ramp start */
    uint32_t v_e = F / end_iv;     /* Hz at ramp end   */

    int32_t total_dv = (int32_t)v_e - (int32_t)v_s;
    if (total_dv == 0) {
        for (i = 0u; i < MAX_RAMP_SEGS; i++) {
            segs[i].start_iv = start_iv; segs[i].add = 0; segs[i].count = 1u;
        }
        return;
    }

    int32_t dv = total_dv / (int32_t)MAX_RAMP_SEGS;
    if (dv == 0) dv = (total_dv > 0) ? 1 : -1;

    for (i = 0u; i < MAX_RAMP_SEGS; i++) {
        /* Velocity bounds for this segment */
        int32_t vs_i = (int32_t)v_s + dv * (int32_t)i;
        int32_t ve_i = (i == MAX_RAMP_SEGS - 1u)
                     ? (int32_t)v_e
                     : vs_i + dv;

        if (vs_i < 534)    vs_i = 534;
        if (ve_i < 534)    ve_i = 534;
        if (vs_i > 160000) vs_i = 160000;
        if (ve_i > 160000) ve_i = 160000;

        /* IEP intervals — force-loaded at each segment boundary by PRU0 */
        uint32_t ivs = F / (uint32_t)vs_i;
        uint32_t ive = F / (uint32_t)ve_i;
        if (ivs < SP_IV_MIN) ivs = SP_IV_MIN;
        if (ivs > SP_IV_MAX) ivs = SP_IV_MAX;
        if (ive < SP_IV_MIN) ive = SP_IV_MIN;
        if (ive > SP_IV_MAX) ive = SP_IV_MAX;

        /* Kinematic step count: s = (v_hi²-v_lo²)/(2a) */
        uint32_t v_hi = ((uint32_t)vs_i > (uint32_t)ve_i) ? (uint32_t)vs_i : (uint32_t)ve_i;
        uint32_t v_lo = ((uint32_t)vs_i > (uint32_t)ve_i) ? (uint32_t)ve_i : (uint32_t)vs_i;
        uint32_t cnt  = (uint32_t)(((uint64_t)(v_hi + v_lo) * (uint64_t)(v_hi - v_lo))
                         / (2u * (uint64_t)accel_s2));
        if (cnt < 1u) cnt = 1u;

        /* Per-step delta. When cnt > |gap|, add truncates to 0 — that is
         * correct: PRU1 holds speed constant for cnt steps, then PRU0
         * force-loads the next start_iv.  With MAX_RAMP_SEGS=64 the
         * resulting boundary jump is ≤3% in the normal test range.         */
        int32_t gap = (int32_t)ive - (int32_t)ivs;
        segs[i].start_iv = ivs;
        segs[i].count    = cnt;
        segs[i].add      = gap / (int32_t)cnt;
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * Unix socket helpers
 * ════════════════════════════════════════════════════════════════════════════ */

static int create_server_socket(void) {
    unlink(SOCKET_PATH);
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { perror("[daemon] socket"); return -1; }

    struct sockaddr_un addr;
    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    strncpy(addr.sun_path, SOCKET_PATH, sizeof(addr.sun_path) - 1u);

    if (bind(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("[daemon] bind"); close(fd); return -1;
    }
    chmod(SOCKET_PATH, 0666);
    listen(fd, MAX_CLIENTS);
    return fd;
}

static void broadcast(const char *msg) {
    size_t len = strlen(msg);
    int i = 0;
    while (i < g_nclients) {
        ssize_t n = write(g_clients[i], msg, len);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            i++;
        } else if (n <= 0) {
            close(g_clients[i]);
            for (int j = i; j < g_nclients - 1; j++)
                g_clients[j] = g_clients[j + 1];
            g_nclients--;
        } else {
            i++;
        }
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * Command dispatcher
 * ════════════════════════════════════════════════════════════════════════════ */

static void compact_json(char *s) {
    char *r = s, *w = s;
    while (*r) {
        *w++ = *r;
        if (*r == ':' || *r == ',') {
            ++r;
            while (*r == ' ' || *r == '\t') ++r;
        } else {
            ++r;
        }
    }
    *w = '\0';
}

static void handle_command(int client_fd, const char *line_in) {
    char resp[BUF_SZ];
    char line_buf[BUF_SZ];
    strncpy(line_buf, line_in, sizeof(line_buf) - 1);
    line_buf[sizeof(line_buf) - 1] = '\0';
    compact_json(line_buf);
    const char *line = line_buf;

#define HAS(key)  (strstr(line, key) != NULL)

    if (HAS("\"cmd\":\"e_stop\"") || HAS("\"cmd\": \"e_stop\"")) {
        int rc = send_host_cmd(HOST_CMD_ESTOP, AXIS_ALL, 0,0,0,0, 0);
        if (rc == 0) {
            /* Motor is now stopped: reset last-commanded interval so the next
             * set_speed builds a ramp from standstill, not from the previous
             * target speed (which would skip the acceleration phase).        */
            g_last_sp_iv = 0u;
        }
        snprintf(resp, sizeof(resp),
            rc == 0 ? "{\"ok\":true}\n" : "{\"ok\":false,\"error\":\"busy\"}\n");

    } else if (HAS("\"cmd\":\"set_speed\"")) {
        uint32_t sp_hz = 0, lat_hz = 0;
        uint8_t  sp_dir = 0, lat_dir = 0;
        const char *p;
        p = strstr(line, "\"sp_hz\":");
        if (p) sp_hz = (uint32_t)strtoul(p + 8, NULL, 10);
        p = strstr(line, "\"lat_hz\":");
        if (p) lat_hz = (uint32_t)strtoul(p + 9, NULL, 10);
        if (HAS("\"sp_dir\":1"))  sp_dir = 1;
        if (HAS("\"lat_dir\":1")) lat_dir = 1;

        uint32_t sp_iv  = hz_to_interval(sp_hz);
        uint32_t lat_iv = hz_to_interval(lat_hz);

        /* Spindle always ramps: compute segments from current → target.
         * Use g_last_sp_iv (last commanded interval) as start: it tracks the
         * last target exactly, independent of telem latency (telem is
         * published every ~2.5 ms and may still show the previous speed).  */
        int use_ramp = 0;
        if (sp_iv > 0u && g_ramp_segs[MOTOR_0]) {
            /* start_iv: last commanded value, or SP_IV_MAX if stopped */
            uint32_t start_iv = (g_last_sp_iv > 0u && g_last_sp_iv < SP_IV_MAX)
                                ? g_last_sp_iv : SP_IV_MAX;
            if (start_iv != sp_iv) {
                ramp_seg_t local_segs[MAX_RAMP_SEGS];
                compute_ramp(local_segs, start_iv, sp_iv, g_sp_accel);
                for (uint32_t i = 0u; i < MAX_RAMP_SEGS; i++)
                    g_ramp_segs[MOTOR_0][i] = local_segs[i];
                __sync_synchronize();
                use_ramp = 1;

                /* Diagnostic: dump computed segments to stderr */
                fprintf(stderr,
                    "[daemon] ramp %u→%u Hz  start_iv=%u  end_iv=%u  accel=%u\n",
                    (PRU_CLOCK_HZ / 2u) / start_iv,
                    sp_hz, start_iv, sp_iv, g_sp_accel);
                for (uint32_t i = 0u; i < MAX_RAMP_SEGS; i++) {
                    fprintf(stderr,
                        "  seg[%2u] start_iv=%6u  add=%5d  count=%6u\n",
                        i, local_segs[i].start_iv,
                        local_segs[i].add, local_segs[i].count);
                }
            }
        }

        g_last_sp_iv = sp_iv;

        /* Wait for PRU0 to be ready. */
        for (int i = 0; i < 5000; i++) {
            if (g_host_cmd->cmd == HOST_CMD_NOP) break;
            usleep(10);
        }
        int rc;
        if (g_host_cmd->cmd != HOST_CMD_NOP) {
            rc = -1;
        } else if (use_ramp) {
            g_host_cmd->axis                           = AXIS_SPINDLE;
            g_host_cmd->motor[MOTOR_0].dir             = sp_dir;
            g_host_cmd->motor[MOTOR_0].interval_target = sp_iv;
            g_host_cmd->motor[MOTOR_1].dir             = lat_dir;
            g_host_cmd->motor[MOTOR_1].interval_target = lat_iv;
            g_host_cmd->seg_count                      = (uint8_t)MAX_RAMP_SEGS;
            g_host_cmd->cruise_iv                      = sp_iv;
            g_host_cmd->cmd_ack                        = HOST_CMD_NOP;
            g_host_cmd->cmd                            = HOST_CMD_SET_SPEED;
            g_last_ramp_target_hz = sp_hz;
            rc = 0;
            fprintf(stderr, "[daemon] set_speed: ramp to %u Hz (iv=%u) dir=%u\n",
                    sp_hz, sp_iv, sp_dir);
        } else {
            /* Already at target speed or stopped — direct write. */
            g_host_cmd->axis                           = AXIS_ALL;
            g_host_cmd->motor[MOTOR_0].dir             = sp_dir;
            g_host_cmd->motor[MOTOR_0].interval_target = sp_iv;
            g_host_cmd->motor[MOTOR_1].dir             = lat_dir;
            g_host_cmd->motor[MOTOR_1].interval_target = lat_iv;
            g_host_cmd->seg_count                      = 0u;
            g_host_cmd->cruise_iv                      = sp_iv;
            g_host_cmd->cmd_ack                        = HOST_CMD_NOP;
            g_host_cmd->cmd                            = HOST_CMD_SET_SPEED;
            rc = 0;
        }
        snprintf(resp, sizeof(resp),
            rc == 0 ? "{\"ok\":true}\n" : "{\"ok\":false,\"error\":\"busy\"}\n");

    } else if (HAS("\"cmd\":\"enable\"")) {
        uint8_t axis = HAS("\"axis\":1")   ? AXIS_LATERAL
                     : HAS("\"axis\":255") ? AXIS_ALL
                     : AXIS_SPINDLE;
        uint32_t val = HAS("\"value\":1") ? 1u : 0u;
        int rc = send_host_cmd(HOST_CMD_ENABLE, axis, 0,0,0,0, val);
        snprintf(resp, sizeof(resp),
            rc == 0 ? "{\"ok\":true}\n" : "{\"ok\":false,\"error\":\"busy\"}\n");

    } else if (HAS("\"cmd\":\"home_start\"")) {
        int rc = send_host_cmd(HOST_CMD_HOME_START, AXIS_LATERAL, 0,0,0,0, 0);
        snprintf(resp, sizeof(resp),
            rc == 0 ? "{\"ok\":true}\n" : "{\"ok\":false,\"error\":\"busy\"}\n");

    } else if (HAS("\"cmd\":\"set_limits\"")) {
        uint8_t axis = HAS("\"axis\":1")   ? AXIS_LATERAL
                     : HAS("\"axis\":255") ? AXIS_ALL
                     : AXIS_SPINDLE;
        int32_t minv = 0, maxv = 0;
        const char *p;
        p = strstr(line, "\"min\":");
        if (p) minv = (int32_t)strtol(p + 6, NULL, 10);
        p = strstr(line, "\"max\":");
        if (p) maxv = (int32_t)strtol(p + 6, NULL, 10);
        int rc = send_host_set_limits(axis, minv, maxv);
        snprintf(resp, sizeof(resp),
            rc == 0 ? "{\"ok\":true}\n" : "{\"ok\":false,\"error\":\"busy\"}\n");

    } else if (HAS("\"cmd\":\"move_to\"")) {
        uint8_t  axis       = HAS("\"axis\":1") ? AXIS_LATERAL : AXIS_SPINDLE;
        int32_t  target_pos = 0;
        const char *p;
        p = strstr(line, "\"pos\":");
        if (p) target_pos = (int32_t)strtol(p + 6, NULL, 10);
        int rc = send_host_move_to(axis, target_pos);
        snprintf(resp, sizeof(resp),
            rc == 0 ? "{\"ok\":true}\n" : "{\"ok\":false,\"error\":\"busy\"}\n");

    } else if (HAS("\"cmd\":\"set_mode\"")) {
        if (HAS("\"mode\":\"winding\"")) {
            g_winding_mode = WINDING_MODE_WINDING;
            fprintf(stderr, "[daemon] mode: WINDING (spindle coordinated)\n");
        } else {
            g_winding_mode = WINDING_MODE_FREE;
            fprintf(stderr, "[daemon] mode: FREE (independent axes)\n");
        }
        snprintf(resp, sizeof(resp), "{\"ok\":true}\n");

    } else if (HAS("\"cmd\":\"set_accel\"")) {
        const char *p;
        p = strstr(line, "\"lat_max_speed\":");
        if (p) g_lat_max_speed = (uint32_t)strtoul(p + 16, NULL, 10);
        p = strstr(line, "\"lat_accel\":");
        if (p) g_lat_accel = (uint32_t)strtoul(p + 12, NULL, 10);
        p = strstr(line, "\"lat_decel\":");
        if (p) g_lat_decel = (uint32_t)strtoul(p + 12, NULL, 10);
        p = strstr(line, "\"sp_accel\":");
        if (p) g_sp_accel = (uint32_t)strtoul(p + 11, NULL, 10);
        if (g_lat_max_speed < 534u) g_lat_max_speed = 534u;
        if (g_lat_accel < 1u)       g_lat_accel = 1u;
        if (g_lat_decel < 1u)       g_lat_decel = 1u;
        if (g_sp_accel  < 1u)       g_sp_accel  = 1u;
        fprintf(stderr,
            "[daemon] set_accel: lat_max=%u lat_a=%u lat_d=%u sp_a=%u\n",
            g_lat_max_speed, g_lat_accel, g_lat_decel, g_sp_accel);
        snprintf(resp, sizeof(resp), "{\"ok\":true}\n");

    } else if (HAS("\"cmd\":\"ack_event\"")) {
        int rc = send_host_cmd(HOST_CMD_ACK_EVENT, 0, 0,0,0,0, 0);
        g_event_broadcast = 0u;
        snprintf(resp, sizeof(resp),
            rc == 0 ? "{\"ok\":true}\n" : "{\"ok\":false,\"error\":\"busy\"}\n");

    } else if (HAS("\"cmd\":\"reset_pos\"")) {
        uint8_t axis = HAS("\"axis\":1")   ? AXIS_LATERAL
                     : HAS("\"axis\":255") ? AXIS_ALL
                     : AXIS_SPINDLE;
        int rc = send_host_cmd(HOST_CMD_RESET_POS, axis, 0,0,0,0, 0);
        snprintf(resp, sizeof(resp),
            rc == 0 ? "{\"ok\":true}\n" : "{\"ok\":false,\"error\":\"busy\"}\n");

    } else {
        snprintf(resp, sizeof(resp), "{\"ok\":false,\"error\":\"unknown cmd\"}\n");
    }

#undef HAS

    write(client_fd, resp, strlen(resp));
}

/* ════════════════════════════════════════════════════════════════════════════
 * Main loop
 * ════════════════════════════════════════════════════════════════════════════ */

int main(int argc, char **argv) {
    const char *devmem = (argc > 1) ? argv[1] : "/dev/mem";

    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);
    signal(SIGPIPE, SIG_IGN);

    if (pru_map_open(devmem) < 0) return 1;

    int srv = create_server_socket();
    if (srv < 0) { pru_map_close(); return 1; }

    printf("[daemon] started, socket=%s devmem=%s\n", SOCKET_PATH, devmem);

    memset(g_clients, -1, sizeof(g_clients));
    g_nclients = 0;

    int   telem_tick = 0;
    char  buf[BUF_SZ];

    while (g_running) {
        struct pollfd fds[1 + MAX_CLIENTS];
        fds[0].fd     = srv;
        fds[0].events = POLLIN;
        for (int i = 0; i < g_nclients; i++) {
            fds[1 + i].fd     = g_clients[i];
            fds[1 + i].events = POLLIN;
        }

        poll(fds, (nfds_t)(1 + g_nclients), POLL_MS);

        /* ── Accept new connections ────────────────────────────────────── */
        if (fds[0].revents & POLLIN) {
            int cfd = accept(srv, NULL, NULL);
            if (cfd >= 0) {
                fcntl(cfd, F_SETFL, O_NONBLOCK);
                if (g_nclients < MAX_CLIENTS) {
                    g_clients[g_nclients++] = cfd;
                } else {
                    close(cfd);
                }
            }
        }

        /* ── Read commands from clients ────────────────────────────────── */
        for (int i = g_nclients - 1; i >= 0; i--) {
            if (!(fds[1 + i].revents & POLLIN)) continue;
            ssize_t n = read(g_clients[i], buf, sizeof(buf) - 1);
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
            if (n <= 0) {
                close(g_clients[i]);
                for (int j = i; j < g_nclients - 1; j++)
                    g_clients[j] = g_clients[j + 1];
                g_nclients--;
                continue;
            }
            buf[n] = '\0';
            char *saveptr = NULL;
            char *cmdline = strtok_r(buf, "\n", &saveptr);
            while (cmdline) {
                if (strlen(cmdline) > 2) handle_command(g_clients[i], cmdline);
                cmdline = strtok_r(NULL, "\n", &saveptr);
            }
        }

        /* ── Event polling from PRU0 status ────────────────────────────── */
        if (g_nclients > 0 && g_status) {
            if (g_status->event_pending && !g_event_broadcast) {
                g_event_broadcast = 1u;
                switch (g_status->event_type) {
                case EVENT_ENDSTOP_HIT:
                    broadcast("{\"event\":\"endstop_hit\"}\n");
                    break;
                case EVENT_HOME_COMPLETE:
                    broadcast("{\"event\":\"home_complete\"}\n");
                    break;
                case EVENT_LIMIT_HIT:
                    snprintf(buf, sizeof(buf),
                        "{\"event\":\"limit_hit\",\"axis\":%u,\"pos\":%d}\n",
                        AXIS_LATERAL,
                        (int)g_status->motor[MOTOR_1].position);
                    broadcast(buf);
                    break;
                case EVENT_MOVE_COMPLETE:
                    snprintf(buf, sizeof(buf),
                        "{\"event\":\"move_complete\",\"pos\":%d}\n",
                        (int)g_status->motor[MOTOR_1].position);
                    broadcast(buf);
                    break;
                case EVENT_SPEED_REACHED:
                    snprintf(buf, sizeof(buf),
                        "{\"event\":\"speed_reached\",\"sp_hz\":%u}\n",
                        g_last_ramp_target_hz);
                    broadcast(buf);
                    break;
                case EVENT_FAULT:
                    snprintf(buf, sizeof(buf),
                        "{\"event\":\"fault\",\"sp_faults\":%u,\"lat_faults\":%u}\n",
                        g_status->motor[MOTOR_0].faults,
                        g_status->motor[MOTOR_1].faults);
                    broadcast(buf);
                    break;
                default:
                    break;
                }
            }
            /* Reset broadcast flag when event acknowledged (pending→0). */
            if (!g_status->event_pending)
                g_event_broadcast = 0u;
        }

        /* ── Periodic telemetry broadcast ─────────────────────────────── */
        if (g_nclients > 0 && ++telem_tick >= TELEM_EVERY) {
            telem_tick = 0;
            if (g_status) {
                uint32_t sp_iv  = g_status->motor[MOTOR_0].interval_actual;
                uint32_t lat_iv = g_status->motor[MOTOR_1].interval_actual;
                uint32_t sp_hz  = sp_iv  ? (PRU_CLOCK_HZ / (2u * sp_iv))  : 0u;
                uint32_t lat_hz = lat_iv ? (PRU_CLOCK_HZ / (2u * lat_iv)) : 0u;

                snprintf(buf, sizeof(buf),
                    "{\"event\":\"telem\",\"pru1_state\":%u,"
                    "\"sp\":{\"steps\":%u,\"speed_hz\":%u,\"faults\":%u},"
                    "\"lat\":{\"steps\":%u,\"pos\":%d,\"speed_hz\":%u,\"faults\":%u},"
                    "\"endstop\":%u}\n",
                    g_status->pru1_state,
                    g_status->motor[MOTOR_0].step_count, sp_hz,
                    g_status->motor[MOTOR_0].faults,
                    g_status->motor[MOTOR_1].step_count,
                    (int)g_status->motor[MOTOR_1].position,
                    lat_hz, g_status->motor[MOTOR_1].faults,
                    g_status->endstop_mask);
                broadcast(buf);
            }
        }
    }

    /* ── Cleanup ────────────────────────────────────────────────────────── */
    for (int i = 0; i < g_nclients; i++) close(g_clients[i]);
    close(srv);
    unlink(SOCKET_PATH);
    pru_map_close();
    printf("[daemon] stopped\n");
    return 0;
}
