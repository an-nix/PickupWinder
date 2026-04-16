/**
 * @file demo_local.cpp
 * @brief Host-simulation task — mirrors the Raspberry Pi SPI/I2C sender.
 *
 * ╔══════════════════════════════════════════════════════════════════════════╗
 * ║  THIS FILE SIMULATES THE RASPBERRY PI HOST.                             ║
 * ║  In production, replace with the real SPI/I2C receive task that         ║
 * ║  deserialises pre-computed step_block_t arrays from the host and        ║
 * ║  enqueues them via StepperQueue::enqueueBlock().                        ║
 * ╚══════════════════════════════════════════════════════════════════════════╝
 *
 * ── Real host architecture ─────────────────────────────────────────────────
 *
 *   Raspberry Pi (host)                    ESP32 (executor)
 *   ──────────────────                     ────────────────
 *   Compute full trajectory                SPI/I2C RX task (Core 0, pri 10)
 *   → stream step_block_t[]   ──SPI──►     → StepperQueue::enqueueBlock()
 *     as fast as queue allows              → executor task (Core 1, pri 24)
 *                                          → StepperDriver ring buffer (512)
 *                                          → RMT simple_encoder ISR
 *                                          → STEP GPIO pulses
 *
 *   The host pre-fills the queue (STEPPER_QUEUE_DEPTH = 16 blocks = 1024 steps)
 *   before the first motor pulse, then continuously refills it.
 *   This demo replicates that behaviour in a single task.
 *
 * ── Velocity profile maths ─────────────────────────────────────────────────
 *
 *   step_hz = rpm / 60.0 × STEPS_PER_REV
 *   interval_ticks = RMT_STEP_RESOLUTION_HZ / step_hz
 *   Linear accel: hz(t) = hz_start + accel_rate × t  (constant ΔHz/s)
 *
 *   The demo keeps the ramp smooth by generating intervals directly in RMT
 *   ticks and using a fractional-tick error accumulator. This avoids the
 *   large speed stair-steps caused by integer-microsecond quantization.
 */

#include "demo_local.h"

#include <math.h>
#include <stdint.h>
#include <esp_log.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static const char* TAG_GEN = "demo_gen";
static const char* TAG_LOG = "demo_log";

// ---------------------------------------------------------------------------
// Motor / profile constants
// ---------------------------------------------------------------------------

/** Full steps per revolution (200 for a 1.8° motor) */
static constexpr uint32_t FULL_STEPS_PER_REV = 200;

/** Effective steps/rev with microstepping */
static constexpr uint32_t STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS;

/** Start speed for the ramp (must be > 0 to avoid division by zero) */
static constexpr float START_HZ   = 200.0f;   // ~1.875 RPM at 6400 steps/rev

/** Motor A target speed in RPM */
static constexpr float TARGET_RPM_A = 1000.0f;

/** Motor B target speed in RPM — disabled (set to 0) */
static constexpr float TARGET_RPM_B = 0.0f;

/** Acceleration duration in seconds */
static constexpr float ACCEL_DURATION_S  = 10.0f;

/** Cruise (constant speed) duration in seconds */
static constexpr float CRUISE_DURATION_S = 3.0f;

/** Deceleration duration in seconds */
static constexpr float DECEL_DURATION_S  = 10.0f;

// ---------------------------------------------------------------------------
// Shared state (generator → logger)
// ---------------------------------------------------------------------------

struct MotorLogState {
    volatile uint32_t instant_hz;  // current step Hz (approx), updated per block
    volatile uint32_t phase;       // 0=accel 1=cruise 2=decel 3=done
};

static MotorLogState s_log[2] = {};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Convert RPM to step Hz for a given motor (all motors same STEPS_PER_REV). */
static inline float rpm_to_hz(float rpm)
{
    return rpm / 60.0f * static_cast<float>(STEPS_PER_REV);
}

/** Convert step Hz to ideal interval in RMT ticks. */
static inline float hz_to_ticks(float hz)
{
    if (hz < 0.001f) { return 0.0f; }
    return static_cast<float>(RMT_STEP_RESOLUTION_HZ) / hz;
}

// ---------------------------------------------------------------------------
// Profile generator — fills a step_block_t with the next STEP_BLOCK_SIZE
// steps from the current position in the velocity profile.
//
// Returns false when the full profile is done (block.count == 0).
// ---------------------------------------------------------------------------

struct ProfileState {
    float    hz_start;        // start speed (Hz)
    float    hz_target;       // cruise speed (Hz)
    float    accel_rate;      // Hz per second (for accel phase)
    float    decel_rate;      // Hz per second (for decel phase)
    float    accel_duration;  // seconds
    float    cruise_duration; // seconds
    float    decel_duration;  // seconds
    float    time_cursor;     // current time in seconds
    float    total_duration;  // total profile duration in seconds
    float    tick_error;      // fractional tick error for error diffusion
    uint8_t  motor_id;
};

static void profile_init(ProfileState& ps, float target_rpm, uint8_t motor_id)
{
    ps.hz_start  = START_HZ;
    ps.hz_target = rpm_to_hz(target_rpm);

    ps.accel_duration = ACCEL_DURATION_S;
    ps.cruise_duration = CRUISE_DURATION_S;
    ps.decel_duration = DECEL_DURATION_S;
    ps.total_duration = ps.accel_duration + ps.cruise_duration + ps.decel_duration;

    // Linear acceleration rate in Hz/s (constant acceleration)
    ps.accel_rate = (ps.hz_target - ps.hz_start) / ps.accel_duration;
    ps.decel_rate = (ps.hz_target - ps.hz_start) / ps.decel_duration;

    ps.time_cursor = 0.0f;
    ps.tick_error = 0.0f;
    ps.motor_id  = motor_id;
}

/**
 * @brief Compute instantaneous frequency at time t in the profile.
 *
 * The profile is: accel (linear ramp) → cruise (constant) → decel (linear ramp).
 * This gives constant acceleration in steps/s², which is physically smooth.
 */
static float profile_hz_at_time(const ProfileState& ps, float t)
{
    if (t < ps.accel_duration) {
        // Linear ramp: hz = hz_start + accel_rate * t
        return ps.hz_start + ps.accel_rate * t;
    } else if (t < ps.accel_duration + ps.cruise_duration) {
        return ps.hz_target;
    } else {
        float t_decel = t - ps.accel_duration - ps.cruise_duration;
        return ps.hz_target - ps.decel_rate * t_decel;
    }
}

/**
 * @brief Fill @p block with up to STEP_BLOCK_SIZE steps from the profile.
 *
 * Each step's interval determines when the NEXT step fires.  The time cursor
 * advances by the interval of each step.  This produces a time-accurate
 * profile with constant acceleration (smooth).
 *
 * @return true if the profile is still in progress; false if complete.
 */
static bool profile_fill_block(ProfileState& ps, step_block_t& block)
{
    block.count = 0;

    if (ps.time_cursor >= ps.total_duration) {
        return false; // done
    }

    for (uint32_t fill = 0; fill < STEP_BLOCK_SIZE; ++fill) {
        if (ps.time_cursor >= ps.total_duration) { break; }

        float hz = profile_hz_at_time(ps, ps.time_cursor);
        if (hz < 1.0f) { hz = 1.0f; }

        // Error-diffused tick quantization: convert the ideal floating-point
        // period to integer RMT ticks while carrying forward the residual.
        // This removes the large visible speed stair-steps caused by integer
        // microsecond quantization at high step rates.
        float ideal_ticks = hz_to_ticks(hz);
        float quantized_ticks_f = ideal_ticks + ps.tick_error;
        uint32_t interval_ticks = static_cast<uint32_t>(quantized_ticks_f + 0.5);
        interval_ticks = std::max<uint32_t>(interval_ticks, RMT_STEP_MIN_TICKS);
        interval_ticks = std::min<uint32_t>(interval_ticks, RMT_STEP_MAX_TICKS);
        ps.tick_error += ideal_ticks - static_cast<float>(interval_ticks);

        block.steps[fill].interval_ticks = interval_ticks;
        block.steps[fill].direction   = true;
        block.count++;

        // Advance time by the quantized interval actually sent to the driver.
        ps.time_cursor += static_cast<float>(interval_ticks)
                  / static_cast<float>(RMT_STEP_RESOLUTION_HZ);

        // Update shared logging state
        float t = ps.time_cursor;
        if (t < ps.accel_duration) {
            s_log[ps.motor_id].phase = 0;
        } else if (t < ps.accel_duration + ps.cruise_duration) {
            s_log[ps.motor_id].phase = 1;
        } else {
            s_log[ps.motor_id].phase = 2;
        }
        s_log[ps.motor_id].instant_hz = static_cast<uint32_t>(hz);
    }

    return true;
}

// ---------------------------------------------------------------------------
// Host-simulation task
// ---------------------------------------------------------------------------
//
// Simulates the Raspberry Pi SPI/I2C sender:
//
//   Phase 1 — Pre-fill (cold start):
//     Generate and enqueue STEPPER_QUEUE_DEPTH blocks before signalling
//     "motors ready".  This gives the executor a full queue to drain from
//     the first pulse, eliminating cold-start underruns.
//
//   Phase 2 — Continuous refill:
//     After pre-fill, generate and enqueue one block at a time, blocking
//     on xQueueSend(portMAX_DELAY) when the queue is full.  This matches
//     the real host behaviour: it pushes data as fast as the queue allows
//     and relies on the queue's depth as the only flow-control mechanism.
//
// In production this task is replaced by the SPI/I2C RX ISR or task that
// receives pre-computed step_block_t frames from the Raspberry Pi.

struct HostSimCtx {
    StepperQueue* queue_a;
    StepperQueue* queue_b;
};

static void host_sim_task(void* arg)
{
    auto* ctx = static_cast<HostSimCtx*>(arg);
    StepperQueue* qa = ctx->queue_a;
    delete ctx;

    ESP_LOGI(TAG_GEN, "Host sim started — target %.0f RPM, %lu steps/rev",
             TARGET_RPM_A, (unsigned long)STEPS_PER_REV);
    ESP_LOGI(TAG_GEN, "  Accel %.1f s  |  Cruise %.1f s  |  Decel %.1f s",
             ACCEL_DURATION_S, CRUISE_DURATION_S, DECEL_DURATION_S);

    ProfileState ps_a;
    profile_init(ps_a, TARGET_RPM_A, 0);

    step_block_t block;
    bool running = (TARGET_RPM_A > 0.0f);

    // ── Phase 1: pre-fill the queue before the first motor pulse ────────────
    //
    // Enqueue up to STEPPER_QUEUE_DEPTH blocks without blocking (timeout=0).
    // Once the queue is full, fall through to the continuous refill loop.
    // This guarantees the executor has a full look-ahead buffer from step 0.
    {
        uint32_t prefill = 0;
        while (running && prefill < STEPPER_QUEUE_DEPTH) {
            running = profile_fill_block(ps_a, block);
            if (block.count == 0) break;
            // Use portMAX_DELAY: if queue unexpectedly full, wait rather than drop
            if (qa->enqueueBlock(block, portMAX_DELAY) == ESP_OK) {
                prefill++;
            }
        }
        ESP_LOGI(TAG_GEN, "Pre-fill complete: %lu blocks enqueued", (unsigned long)prefill);
    }

    // ── Phase 2: continuous refill — push blocks as fast as queue allows ────
    //
    // xQueueSend blocks when the queue is full (portMAX_DELAY), resuming as
    // soon as the executor consumes a block.  This is the exact behaviour of
    // the real SPI/I2C host: it streams data and lets the queue depth control
    // pacing automatically.
    while (running) {
        running = profile_fill_block(ps_a, block);
        if (block.count == 0) break;
        esp_err_t err = qa->enqueueBlock(block, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGE(TAG_GEN, "enqueue error: %s", esp_err_to_name(err));
        }
    }

    s_log[0].phase = 3;
    s_log[1].phase = 3;

    ESP_LOGI(TAG_GEN, "Host sim done — all blocks sent.");
    vTaskDelete(nullptr);
}

// ---------------------------------------------------------------------------
// Logging task — prints instantaneous Hz every 100 ms
// ---------------------------------------------------------------------------

static void logger_task(void* /*arg*/)
{
    static const char* phase_names[] = {"ACCEL", "CRUISE", "DECEL", "DONE"};

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));

        const uint32_t hz_a    = s_log[0].instant_hz;
        const uint32_t hz_b    = s_log[1].instant_hz;
        const uint32_t phase_a = s_log[0].phase < 4 ? s_log[0].phase : 3;
        const uint32_t phase_b = s_log[1].phase < 4 ? s_log[1].phase : 3;

        // Convert step Hz to RPM for readability
        const float rpm_a = (hz_a > 0)
            ? static_cast<float>(hz_a) / static_cast<float>(STEPS_PER_REV) * 60.0f
            : 0.0f;
        const float rpm_b = (hz_b > 0)
            ? static_cast<float>(hz_b) / static_cast<float>(STEPS_PER_REV) * 60.0f
            : 0.0f;

        ESP_LOGI(TAG_LOG,
                 "A: %5lu Hz  (%6.1f RPM)  [%s]   "
                 "B: %5lu Hz  (%6.1f RPM)  [%s]",
                 (unsigned long)hz_a, (double)rpm_a, phase_names[phase_a],
                 (unsigned long)hz_b, (double)rpm_b, phase_names[phase_b]);

        if (phase_a == 3 && phase_b == 3) {
            ESP_LOGI(TAG_LOG, "Both motors done — logger exiting.");
            vTaskDelete(nullptr);
            return;
        }
    }
}

// ---------------------------------------------------------------------------
// demo_local_start()
// ---------------------------------------------------------------------------

esp_err_t demo_local_start(StepperQueue* queue_a, StepperQueue* queue_b)
{
    if (!queue_a || !queue_b) {
        return ESP_ERR_INVALID_ARG;
    }

    auto* ctx = new HostSimCtx{queue_a, queue_b};
    ESP_RETURN_ON_FALSE(ctx != nullptr, ESP_ERR_NO_MEM, TAG_GEN,
                        "failed to allocate HostSimCtx");

    // Host-sim task: Core 0, priority 9.
    //
    // Higher than the logger (5) so pre-fill completes before the first log
    // line.  Lower than the executor (24) so it never delays step output.
    // In production this is replaced by the SPI/I2C RX task (typically pri 10).
    BaseType_t rc = xTaskCreatePinnedToCore(
        host_sim_task, "host_sim",
        4096, ctx,
        9,        // priority — matches typical SPI RX task priority
        nullptr,
        0);       // Core 0 — leaves Core 1 entirely for stepper execution

    if (rc != pdPASS) {
        delete ctx;
        ESP_LOGE(TAG_GEN, "failed to create host_sim task");
        return ESP_ERR_NO_MEM;
    }

    // Logger: Core 0, low priority
    rc = xTaskCreatePinnedToCore(
        logger_task, "demo_log",
        2048, nullptr,
        5,
        nullptr,
        0);

    if (rc != pdPASS) {
        ESP_LOGW(TAG_GEN, "failed to create logger task (non-fatal)");
    }

    return ESP_OK;
}
