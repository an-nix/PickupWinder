/* pickup_daemon.c — PickupWinder hardware daemon.
 *
 * Architecture layer 3/4.
 *
 * Bridge between the Python application (layer 4) and PRU1 (layer 2).
 * Maps PRU Shared RAM via /dev/mem, writes commands into host_cmd_t,
 * reads pru_status_t, and forwards telemetry + events to Python over
 * a Unix domain socket.
 *
 * The daemon communicates ONLY with PRU0 (via shared RAM). It never
 * writes motor_params or motor_telem — those are PRU0↔PRU1 internal.
 *
 * Socket protocol (UNIX, SOCK_STREAM, newline-delimited JSON):
 *
 *   Python → daemon (commands):
 *     {"cmd":"set_speed","sp_hz":N,"lat_hz":N,"sp_dir":0|1,"lat_dir":0|1}
 *     {"cmd":"enable","axis":0|1|255,"value":0|1}
 *     {"cmd":"e_stop"}
 *     {"cmd":"home_start"}
 *     {"cmd":"ack_event"}
 *     {"cmd":"reset_pos","axis":0|1|255}
 *     {"cmd":"set_limits","axis":0|1,"min":<int>,"max":<int>}
 *     {"cmd":"move_to","axis":1,"pos":<int>,"start_hz":<uint>,"max_hz":<uint>,"accel_steps":<uint>}
 *     {"cmd":"set_mode","mode":"free"|"winding"}
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
 *
 * Build:
 *   gcc -O2 -Wall -Wextra -I../../pru/include pickup_daemon.c -o pickup_daemon
 *
 * Run:
 *   sudo ./pickup_daemon
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "../../pru/include/pru_ipc.h"

/* ── Configuration ────────────────────────────────────────────────────────── */
#define SOCKET_PATH     "/run/pickup-winder.sock"
#define POLL_MS         10
#define MAX_CLIENTS     4
#define BUF_SZ          512
#define TELEM_EVERY     10   /* broadcast telem every N ticks → 100 ms */

/* ── Shared RAM pointers ──────────────────────────────────────────────────── */
static volatile host_cmd_t     *g_host_cmd  = NULL;
static volatile pru_status_t   *g_status    = NULL;

static void   *g_mmap_base = NULL;
static size_t  g_mmap_size = 0;
static int     g_mem_fd    = -1;

/* ── Client bookkeeping ───────────────────────────────────────────────────── */
static int g_clients[MAX_CLIENTS];
static int g_nclients = 0;

/* ── Shutdown flag ────────────────────────────────────────────────────────── */
static volatile int g_running = 1;

/* ── Event tracking (debounce) ────────────────────────────────────────────── */
static uint8_t  g_last_event_sent = EVENT_NONE;

/* ── Last spindle IEP interval (for move_to spindle coordination) ────────── */
static uint32_t g_last_sp_iv = 0u;

/* ── Winding mode ───────────────────────────────────────────────────── *
 * WINDING_MODE_FREE    (0, default): no spindle–lateral coordination.       *
 *   set_speed controls spindle directly.                                    *
 *   move_to sends move_sp_lat_coord=0 → PRU0 coord_tick() is a no-op.      *
 *                                                                            *
 * WINDING_MODE_WINDING (1): spindle tracks lateral ramps proportionally.    *
 *   set_speed records the target spindle speed (Q6 ratio base).             *
 *   move_to computes and sends move_sp_lat_coord = sp_iv*64/lat_cruise_iv.  *
 *   PRU0 coord_tick() adjusts spindle interval every ~5 µs.                */
#define WINDING_MODE_FREE    0u
#define WINDING_MODE_WINDING 1u
static uint8_t  g_winding_mode = WINDING_MODE_FREE;

static void sig_handler(int sig) { (void)sig; g_running = 0; }

/* ════════════════════════════════════════════════════════════════════════════
 * PRU Shared RAM
 * ════════════════════════════════════════════════════════════════════════════ */

static int pru_map_open(const char *devmem) {
    g_mem_fd = open(devmem, O_RDWR | O_SYNC);
    if (g_mem_fd < 0) { perror("[daemon] open /dev/mem"); return -1; }

    const uint32_t page_off = PRU_SRAM_PHYS_BASE & 0xFFFu;
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
    g_host_cmd = (volatile host_cmd_t *)  (base + IPC_HOST_CMD_OFFSET);
    g_status   = (volatile pru_status_t *)(base + IPC_PRU_STATUS_OFFSET);

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

/* ── Send a host command and wait for PRU1 ack ───────────────────────────── */
static int send_host_cmd(uint8_t cmd, uint8_t axis,
                         uint32_t sp_iv, uint32_t lat_iv,
                         uint8_t sp_dir, uint8_t lat_dir,
                         uint32_t val_a) {
    if (!g_host_cmd) return -1;

    /* Wait for previous command to complete (max ~50 ms). */
    for (int i = 0; i < 5000; i++) {
        if (g_host_cmd->cmd == HOST_CMD_NOP) break;
        usleep(10);
    }
    if (g_host_cmd->cmd != HOST_CMD_NOP) return -1;

    g_host_cmd->axis              = axis;
    g_host_cmd->sp_interval_target  = sp_iv;
    g_host_cmd->lat_interval_target = lat_iv;
    g_host_cmd->sp_dir            = sp_dir;
    g_host_cmd->lat_dir           = lat_dir;
    g_host_cmd->value_a           = val_a;
    g_host_cmd->cmd_ack           = HOST_CMD_NOP;
    /* Write cmd last to avoid race. */
    g_host_cmd->cmd               = cmd;

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

    g_host_cmd->axis = axis;
    g_host_cmd->limit_min = limit_min;
    g_host_cmd->limit_max = limit_max;
    g_host_cmd->cmd_ack = HOST_CMD_NOP;
    g_host_cmd->cmd = HOST_CMD_SET_LIMITS;
    return 0;
}

/* Send move_to command with trapezoidal profile.
 * Computes IEP intervals from Hz values (integer arithmetic only).
 *   start_hz:      step frequency at start/end of ramp (slow)
 *   max_hz:        cruise step frequency (fast)
 *   accel_steps:   steps used for the acceleration ramp
 */
static int send_host_move_to(uint8_t axis, int32_t target_pos,
                              uint32_t start_hz, uint32_t max_hz,
                              uint32_t accel_steps) {
    if (!g_host_cmd) return -1;

    /* Guard: prevent division by zero */
    if (start_hz == 0u)    start_hz = 1u;
    if (max_hz   == 0u)    max_hz   = start_hz;
    if (max_hz   < start_hz) max_hz = start_hz;
    if (accel_steps == 0u) accel_steps = 1u;

    /* Hz → IEP intervals. Min interval = 625 cycles (160 kHz). */
    uint32_t start_iv  = PRU_CLOCK_HZ / (2u * start_hz);
    uint32_t cruise_iv = PRU_CLOCK_HZ / (2u * max_hz);
    if (cruise_iv < 625u) cruise_iv = 625u;
    if (start_iv  < cruise_iv) start_iv = cruise_iv;

    /* delta_iv = interval change per step during ramp (no division in PRU). */
    uint32_t diff     = start_iv - cruise_iv;
    uint32_t delta_iv = diff / accel_steps;
    /* Ensure at least 1 cycle of change if there is a real ramp. */
    if (delta_iv == 0u && diff > 0u) delta_iv = 1u;

    for (int i = 0; i < 5000; i++) {
        if (g_host_cmd->cmd == HOST_CMD_NOP) break;
        usleep(10);
    }
    if (g_host_cmd->cmd != HOST_CMD_NOP) return -1;

    g_host_cmd->axis           = axis;
    g_host_cmd->move_target    = target_pos;
    g_host_cmd->move_start_iv  = start_iv;
    g_host_cmd->move_cruise_iv = cruise_iv;
    g_host_cmd->move_delta_iv  = delta_iv;
    /* Q6 spindle-lateral coordination ratio (0 if spindle not yet started). *
     * PRU0 computes: sp_adj = (move_sp_lat_coord * lat_iv) >> 6             *
     * This keeps turns/mm constant during lateral ramps and reversals.      */
    g_host_cmd->move_sp_lat_coord =
        (g_winding_mode == WINDING_MODE_WINDING &&
         g_last_sp_iv > 0u && cruise_iv > 0u)
        ? (g_last_sp_iv * 64u) / cruise_iv : 0u;
    g_host_cmd->cmd_ack        = HOST_CMD_NOP;
    g_host_cmd->cmd            = HOST_CMD_MOVE_TO;
    return 0;
}

/* ── Hz → IEP interval conversion ───────────────────────────────────────── */
static uint32_t hz_to_interval(uint32_t hz) {
    if (hz == 0u) return 0u;
    uint32_t iv = PRU_CLOCK_HZ / (2u * hz);
    return (iv < 625u) ? 625u : iv;
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
    listen(fd, MAX_CLIENTS);
    return fd;
}

static void broadcast(const char *msg) {
    size_t len = strlen(msg);
    int i = 0;
    while (i < g_nclients) {
        ssize_t n = write(g_clients[i], msg, len);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            /* Client not reading fast enough — drop this message, keep connected.
             * This prevents a slow/stuck socat from blocking the entire daemon. */
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

static void handle_command(int client_fd, const char *line) {
    char resp[BUF_SZ];

#define HAS(key)  (strstr(line, key) != NULL)

    if (HAS("\"cmd\":\"e_stop\"") || HAS("\"cmd\": \"e_stop\"")) {
        int rc = send_host_cmd(HOST_CMD_ESTOP, AXIS_ALL, 0,0,0,0, 0);
        snprintf(resp, sizeof(resp),
            rc == 0 ? "{\"ok\":true}\n" : "{\"ok\":false,\"error\":\"busy\"}\n");

    } else if (HAS("\"cmd\":\"set_speed\"")) {
        /* {"cmd":"set_speed","sp_hz":N,"lat_hz":N,"sp_dir":0|1,"lat_dir":0|1} */
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
        g_last_sp_iv = sp_iv;  /* track for move_to coordination ratio      */

        int rc = send_host_cmd(HOST_CMD_SET_SPEED, AXIS_ALL,
                               sp_iv, lat_iv, sp_dir, lat_dir, 0);
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
        /* {"cmd":"set_limits","axis":1,"min":-1000,"max":1000} */
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
        /* {"cmd":"move_to","axis":1,"pos":1000,"start_hz":200,"max_hz":4000,"accel_steps":300} */
        uint8_t  axis       = HAS("\"axis\":1") ? AXIS_LATERAL : AXIS_SPINDLE;
        int32_t  target_pos = 0;
        uint32_t start_hz   = 200u;
        uint32_t max_hz     = 4000u;
        uint32_t accel_steps = 300u;
        const char *p;
        p = strstr(line, "\"pos\":");
        if (p) target_pos = (int32_t)strtol(p + 6, NULL, 10);
        p = strstr(line, "\"start_hz\":");
        if (p) start_hz = (uint32_t)strtoul(p + 11, NULL, 10);
        p = strstr(line, "\"max_hz\":");
        if (p) max_hz = (uint32_t)strtoul(p + 9, NULL, 10);
        p = strstr(line, "\"accel_steps\":");
        if (p) accel_steps = (uint32_t)strtoul(p + 14, NULL, 10);
        int rc = send_host_move_to(axis, target_pos, start_hz, max_hz, accel_steps);
        snprintf(resp, sizeof(resp),
            rc == 0 ? "{\"ok\":true}\n" : "{\"ok\":false,\"error\":\"busy\"}\n");

    } else if (HAS("\"cmd\":\"set_mode\"")) {
        /* {"cmd":"set_mode","mode":"free"|"winding"} */
        if (HAS("\"mode\":\"winding\"")) {
            g_winding_mode = WINDING_MODE_WINDING;
            fprintf(stderr, "[daemon] mode: WINDING (spindle coordinated)\n");
        } else {
            g_winding_mode = WINDING_MODE_FREE;
            fprintf(stderr, "[daemon] mode: FREE (independent axes)\n");
        }
        snprintf(resp, sizeof(resp), "{\"ok\":true}\n");

    } else if (HAS("\"cmd\":\"ack_event\"")) {
        int rc = send_host_cmd(HOST_CMD_ACK_EVENT, 0, 0,0,0,0, 0);
        g_last_event_sent = EVENT_NONE;
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
                /* Non-blocking: a slow client can never block broadcast(). */
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
            if (n <= 0) {
                close(g_clients[i]);
                for (int j = i; j < g_nclients - 1; j++)
                    g_clients[j] = g_clients[j + 1];
                g_nclients--;
                continue;
            }
            buf[n] = '\0';
            char *saveptr = NULL;
            char *line = strtok_r(buf, "\n", &saveptr);
            while (line) {
                if (strlen(line) > 2) handle_command(g_clients[i], line);
                line = strtok_r(NULL, "\n", &saveptr);
            }
        }

        /* ── Event polling from PRU1 status ────────────────────────────── */
        if (g_nclients > 0 && g_status) {
            if (g_status->event_pending &&
                g_status->event_type != g_last_event_sent) {
                g_last_event_sent = g_status->event_type;
                switch (g_status->event_type) {
                case EVENT_ENDSTOP_HIT:
                    broadcast("{\"event\":\"endstop_hit\"}\n");
                    break;
                case EVENT_HOME_COMPLETE:
                    broadcast("{\"event\":\"home_complete\"}\n");
                    break;
                case EVENT_LIMIT_HIT:
                    /* Notify host that a software limit was hit. */
                    snprintf(buf, sizeof(buf),
                        "{\"event\":\"limit_hit\",\"axis\":%u,\"pos\":%d}\n",
                        AXIS_LATERAL, g_status->lat_position);
                    broadcast(buf);
                    break;
                case EVENT_MOVE_COMPLETE:
                    snprintf(buf, sizeof(buf),
                        "{\"event\":\"move_complete\",\"pos\":%d}\n",
                        g_status->lat_position);
                    broadcast(buf);
                    break;
                case EVENT_FAULT:
                    snprintf(buf, sizeof(buf),
                        "{\"event\":\"fault\",\"sp_faults\":%u,\"lat_faults\":%u}\n",
                        g_status->sp_faults, g_status->lat_faults);
                    broadcast(buf);
                    break;
                default:
                    break;
                }
            }
        }

        /* ── Periodic telemetry broadcast ─────────────────────────────── */
        if (g_nclients > 0 && ++telem_tick >= TELEM_EVERY) {
            telem_tick = 0;
            if (g_status) {
                uint32_t sp_iv  = g_status->sp_interval_actual;
                uint32_t lat_iv = g_status->lat_interval_actual;
                uint32_t sp_hz  = sp_iv  ? (PRU_CLOCK_HZ / (2u * sp_iv))  : 0u;
                uint32_t lat_hz = lat_iv ? (PRU_CLOCK_HZ / (2u * lat_iv)) : 0u;

                snprintf(buf, sizeof(buf),
                    "{\"event\":\"telem\",\"pru1_state\":%u,"
                    "\"sp\":{\"steps\":%u,\"speed_hz\":%u,\"faults\":%u},"
                    "\"lat\":{\"steps\":%u,\"pos\":%d,\"speed_hz\":%u,\"faults\":%u},"
                    "\"endstop\":%u}\n",
                    g_status->pru1_state,
                    g_status->sp_step_count, sp_hz, g_status->sp_faults,
                    g_status->lat_step_count, g_status->lat_position,
                    lat_hz, g_status->lat_faults,
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
