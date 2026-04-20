# PERF_ANALYSIS_V3

## 1) Changelog V2 -> V3

### Correctif #1 — Fenêtre d'exclusion mutuelle multi-axes
- Fichier: `src/esp32/src/comm_interface.cpp`
- Changement: `setMultiExecActive(true)` est maintenant posé pour **tous les axes valides du segment avant** la boucle d'exécution, puis `setMultiExecActive(false)` est clear **après** la boucle.
- Effet: suppression de la fenêtre race intra-segment où l'`executorTask` mono-axe pouvait ré-entrer entre deux axes d'un même segment.

### Correctif #2 — Garde temporelle anti-watchdog dans `executorTask`
- Fichier: `src/esp32/src/stepper_queue.cpp`
- Changement:
  - capture `loop_start_us` au **début de chaque itération** du `do { ... } while (...)`.
  - conservation du yield adaptatif basé sur `ring_free`.
  - ajout d'un fallback temporel: yield si `esp_timer_get_time() - loop_start_us > 5000` quand la ring est sous pression.
- Effet: borne temporelle explicite de non-yield dans la boucle de drain.

### Correctif #3 — Fréquence CPU ESP32 à 240 MHz
- Fichier: `src/esp32/sdkconfig.esp32`
- Changement:
  - `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`
  - `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240`
  - 160 MHz désactivé.
- Effet: plus de headroom CPU pour SPI/planner/executor sous charge.

---

## 2) Analyse numérique V3

### 2.1 Élimination de la fenêtre de race multi-axes
Avant V3, le flag `multi_exec_active` était togglé par axe, ouvrant une fenêtre entre axes d'un même segment. En V3, le flag couvre toute la section segment (set global avant boucle axes, clear global après), ce qui impose une exclusion mutuelle cohérente sur l'intégralité de l'émission multi-axes du segment.

### 2.2 Borne de non-yield (`WORK_BUDGET=8`) à 1500 RPM
Hypothèses firmware/projet:
- `steps/rev = 6400`
- `RPM = 1500`
- donc `f_step = 1500 * 6400 / 60 = 160000 steps/s`
- `STEP_BLOCK_SIZE = 64`

Temps de mouvement représenté par 1 block:
- `t_block = 64 / 160000 = 0.0004 s = 400 µs`

Temps représenté par `WORK_BUDGET=8` blocks:
- `t_8 = 8 * 400 µs = 3200 µs = 3.2 ms`

Conclusion:
- borne pratique de drain sans yield forcé: ~`3.2 ms` à 1500 RPM (inférieure à la garde temporelle 5 ms).
- la garde 5 ms reste le filet de sécurité pour cas défavorables CPU/latence.

### 2.3 Gain de débit SPI estimé (160 -> 240 MHz CPU, `spi_speed_hz = 1_000_000`)
- Taille frame fixe: `512 bytes = 4096 bits`
- Limite physique bus à 1 MHz: `4096 / 1e6 = 4.096 ms/frame` soit `~244.14 frames/s`

Donc:
- gain **théorique bus pur**: ~`0%` (borne par l'horloge SPI).
- gain **système**: réduction du coût CPU par frame (parsing/dispatch/planner), donc plus de marge et moins de jitter/retards côté firmware sous charge.

---

## 3) Sources complètes (sans extraits)

### 3.1 `src/esp32/src/stepper_queue.cpp`
```cpp
/**
 * @file stepper_queue.cpp
 * @brief Per-motor FreeRTOS queue + executor task implementation.
 */

#include "stepper_queue.h"

#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>

static const char* TAG = "stepper_queue";

// Compile-time invariant: the auto-start fill threshold must be large enough
// that the second encode_steps ping-pong callback never immediately underruns.
static_assert(STEP_STREAM_START_FILL >= 2 * PART_SIZE,
              "STEP_STREAM_START_FILL must be >= 2 * PART_SIZE to prevent "
              "immediate ISR underrun on second encoder callback");

// Task parameters
static constexpr uint32_t EXECUTOR_STACK_WORDS = 4096;
static constexpr UBaseType_t EXECUTOR_PRIORITY  = 24;
static constexpr BaseType_t  EXECUTOR_CORE      = 1; // Pro CPU (real-time)

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

StepperQueue::StepperQueue(StepperDriver& driver, uint8_t motor_id)
    : driver_(driver)
    , motor_id_(motor_id)
{}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t StepperQueue::init()
{
    // Each queue item is one `motion_block_t`, which can carry either a legacy
    // `step_block_t` or a compressed `segment_block_t`.
    queue_ = xQueueCreate(STEPPER_QUEUE_DEPTH, sizeof(motion_block_t));
    ESP_RETURN_ON_FALSE(queue_ != nullptr, ESP_ERR_NO_MEM, TAG,
                        "motor%u: failed to create step queue", motor_id_);

    // ── Executor task ───────────────────────────────────────────────────────
    char task_name[16];
    snprintf(task_name, sizeof(task_name), "stepper_%u", motor_id_);

    BaseType_t rc = xTaskCreatePinnedToCore(
        &StepperQueue::executorTask,
        task_name,
        EXECUTOR_STACK_WORDS,
        this,
        EXECUTOR_PRIORITY,
        &task_,
        EXECUTOR_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "motor%u: failed to create executor task", motor_id_);

    ESP_LOGI(TAG, "motor%u: queue depth=%d  task priority=%d  core=%d",
             motor_id_, STEPPER_QUEUE_DEPTH,
             (int)EXECUTOR_PRIORITY, (int)EXECUTOR_CORE);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// enqueueMotionBlock() / enqueueStepBlock() / enqueueSegmentBlock()
// ---------------------------------------------------------------------------

esp_err_t StepperQueue::enqueueMotionBlock(const motion_block_t& block,
                                           uint32_t timeout_ms)
{
    const TickType_t ticks = (timeout_ms == portMAX_DELAY)
                             ? portMAX_DELAY
                             : pdMS_TO_TICKS(timeout_ms);

    if (xQueueSend(queue_, &block, ticks) != pdTRUE) {
        ESP_LOGW(TAG, "motor%u: queue full — motion block dropped",
                 motor_id_);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

esp_err_t StepperQueue::enqueueStepBlock(const step_block_t& block,
                                         uint32_t timeout_ms)
{
    motion_block_t motion {};
    motion.kind = MOTION_BLOCK_KIND_STEP;
    motion.payload.step = block;
    return enqueueMotionBlock(motion, timeout_ms);
}

esp_err_t StepperQueue::enqueueSegmentBlock(const segment_block_t& block,
                                            uint32_t timeout_ms)
{
    motion_block_t motion {};
    motion.kind = MOTION_BLOCK_KIND_SEGMENT;
    motion.payload.segment = block;
    return enqueueMotionBlock(motion, timeout_ms);
}

// ---------------------------------------------------------------------------
// available()
// ---------------------------------------------------------------------------

uint32_t StepperQueue::available() const
{
    return static_cast<uint32_t>(uxQueueSpacesAvailable(queue_));
}

// ---------------------------------------------------------------------------
// executeConstantRateBlock() / kickStart()
// ---------------------------------------------------------------------------
//
// executeConstantRateBlock() deliberately does NOT call maybeStartDriver().
// The start decision belongs to the caller: multiAxisExecutorTask drain loop
// in comm_interface.cpp calls kickStart() ONCE per block batch, after ALL
// available blocks have been written to the ring.  This guarantees the ring
// is pre-filled with multiple segments of look-ahead before RMT starts,
// preventing the per-segment underruns that occur at low speed when each
// segment contributes only 2–5 steps.

esp_err_t StepperQueue::executeConstantRateBlock(bool direction,
                                                  uint16_t step_count,
                                                  uint32_t duration_us)
{
    if (step_count == 0) {
        return ESP_OK;
    }

    // Compute uniform interval: RMT clock is 80 MHz → 80 ticks/µs.
    uint32_t interval_ticks = (duration_us * RMT_TICKS_PER_US) / step_count;
    if (interval_ticks < RMT_STEP_MIN_TICKS) {
        interval_ticks = RMT_STEP_MIN_TICKS;
    }
    if (interval_ticks > RMT_STEP_MAX_TICKS) {
        interval_ticks = RMT_STEP_MAX_TICKS;
    }

    uint32_t remaining = step_count;
    while (remaining > 0) {
        step_block_t expanded {};
        expanded.count = (remaining > STEP_BLOCK_SIZE) ? STEP_BLOCK_SIZE : remaining;
        for (uint32_t i = 0; i < expanded.count; ++i) {
            expanded.steps[i].interval_ticks = interval_ticks;
            expanded.steps[i].direction       = direction;
        }
        esp_err_t err = pushExpandedBlock(driver_, expanded);
        if (err != ESP_OK) {
            return err;
        }
        remaining -= expanded.count;
    }
    return ESP_OK;
}

esp_err_t StepperQueue::kickStart()
{
    return maybeStartDriver(driver_, true);
}

void StepperQueue::gracefulStop()
{
    driver_.gracefulStop();
}

// ---------------------------------------------------------------------------
// maybeStartDriver() / pushExpandedBlock() / executeSegmentBlock()
// ---------------------------------------------------------------------------

esp_err_t StepperQueue::maybeStartDriver(StepperDriver& driver, bool force_start)
{
    const uint32_t buffered_steps = STEP_RING_SIZE - driver.ringFreeSlots();
    const bool ring_has_data = buffered_steps > 0;
    if (!ring_has_data) {
        return ESP_OK;
    }

    // Guard force_start: never begin streaming with fewer than PART_SIZE steps.
    // The RMT ping-pong encoder's first callback requests PART_SIZE symbols; if
    // fewer steps are available it immediately underruns and halts the motor.
    // At slow speeds (2-9 steps/segment during acceleration) this was causing a
    // stutter on every segment.  STEP_STREAM_START_FILL and the ring-full case
    // are unaffected — those scenarios already imply >= PART_SIZE steps buffered.
    const bool is_restart = !driver.isStreaming()
                            && buffered_steps > 0
                            && driver.getUnderrunCount() > 0;
    const bool should_start = (force_start && buffered_steps >= PART_SIZE)
        || (buffered_steps >= STEP_STREAM_START_FILL)
        || (driver.ringFreeSlots() == 0)
        || (is_restart && buffered_steps >= STEP_STREAM_RESTART_FILL);
    if (!should_start) {
        return ESP_OK;
    }

    if (driver.isStreaming() && !driver.isStopped()) {
        return ESP_OK;
    }

    if (driver.isStreaming()) {
        driver.stopStream();
    }
    return driver.startStream();
}

esp_err_t StepperQueue::pushExpandedBlock(StepperDriver& driver, const step_block_t& block)
{
    if (!driver.isStreaming() && driver.ringFreeSlots() == 0) {
        esp_err_t err = maybeStartDriver(driver, true);
        if (err != ESP_OK) {
            return err;
        }
    }

    // Pass the current task handle so ISR ring-space notifications wake
    // whichever task is currently blocked on this ring (multiAxisExecutorTask
    // or per-axis executorTask).
    esp_err_t err = driver.pushBlock(block, xTaskGetCurrentTaskHandle());
    if (err != ESP_OK) {
        return err;
    }
    return maybeStartDriver(driver, false);
}

esp_err_t StepperQueue::executeSegmentBlock(StepperDriver& driver, const segment_block_t& block)
{
    for (uint32_t seg_index = 0; seg_index < block.count; ++seg_index) {
        const motion_segment_t& seg = block.segments[seg_index];
        if (seg.step_count == 0) {
            continue;
        }

        uint32_t remaining = seg.step_count;
        int32_t current_ticks = seg.start_ticks;
        while (remaining > 0) {
            step_block_t expanded {};
            expanded.count = (remaining > STEP_BLOCK_SIZE) ? STEP_BLOCK_SIZE : remaining;
            for (uint32_t i = 0; i < expanded.count; ++i) {
                uint32_t clamped_ticks = static_cast<uint32_t>(current_ticks);
                if (clamped_ticks < RMT_STEP_MIN_TICKS) {
                    clamped_ticks = RMT_STEP_MIN_TICKS;
                }
                if (clamped_ticks > RMT_STEP_MAX_TICKS) {
                    clamped_ticks = RMT_STEP_MAX_TICKS;
                }
                expanded.steps[i].interval_ticks = clamped_ticks;
                expanded.steps[i].direction = (seg.direction != 0);
                current_ticks += seg.add_ticks;
            }

            esp_err_t err = pushExpandedBlock(driver, expanded);
            if (err != ESP_OK) {
                return err;
            }
            remaining -= expanded.count;
        }
    }
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// executorTask()  — Core 1, priority 24
// ---------------------------------------------------------------------------
void StepperQueue::executorTask(void* arg)
{
    StepperQueue* self = static_cast<StepperQueue*>(arg);
    StepperDriver& driver = self->driver_;
    motion_block_t block;

    ESP_LOGI(TAG, "motor%u: executor task started", self->motor_id_);

    constexpr int WORK_BUDGET = 8;

    for (;;) {
        if (xQueueReceive(self->queue_, &block, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        int work_done = 0;

        do {
            const int64_t loop_start_us = esp_timer_get_time();

            if (self->isMultiExecActive()) {
                ulTaskNotifyTake(pdFALSE, pdMS_TO_TICKS(1));
                continue;
            }

            esp_err_t err = ESP_OK;

            if (block.kind == MOTION_BLOCK_KIND_SEGMENT) {
                err = executeSegmentBlock(driver, block.payload.segment);
            } else {
                err = pushExpandedBlock(driver, block.payload.step);
            }

            if (err != ESP_OK) {
                ESP_LOGE(TAG, "motor%u: motion execute error: %s",
                         self->motor_id_, esp_err_to_name(err));
            }

            work_done++;

            if (work_done >= WORK_BUDGET) {
                work_done = 0;
                const uint32_t ring_free = driver.ringFreeSlots();
                if (ring_free > (STEP_RING_SIZE / 2U)) {
                    taskYIELD();
                } else if ((esp_timer_get_time() - loop_start_us) > 5000) {
                    taskYIELD();
                }
            }

        } while (xQueueReceive(self->queue_, &block, 0) == pdTRUE);

        esp_err_t err = maybeStartDriver(driver, true);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "motor%u: startStream error: %s",
                     self->motor_id_, esp_err_to_name(err));
        }
    }
}```

### 3.2 `src/esp32/src/stepper_queue.h`
```cpp
/**
 * @file stepper_queue.h
 * @brief Per-motor FreeRTOS queue + executor task for motion blocks.
 *
 * The host now sends compressed motion segments rather than only explicit
 * per-step blocks. `StepperQueue` remains the boundary between transport and
 * execution:
 *
 * - Core 0 / SPI task enqueues `motion_block_t`
 * - Core 1 / executor task expands segments into `step_block_t`
 * - `StepperDriver` streams the concrete steps via RMT
 */

#pragma once

#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_err.h>
#include "step_types.h"
#include "stepper_driver.h"

class StepperQueue {
public:
    /**
     * @brief Construct a StepperQueue bound to a StepperDriver.
     *
     * @param driver    Initialised StepperDriver instance for this motor.
     * @param motor_id  Logical motor index (0-based, for logging).
     */
    StepperQueue(StepperDriver& driver, uint8_t motor_id);

    /**
     * @brief Create the FreeRTOS queue and launch the executor task.
     *
     * Must be called after StepperDriver::init().
     */
    esp_err_t init();

    /**
     * @brief Enqueue a step block for execution.
     *
     * Called by the communication layer (producer side).  Blocks for up to
     * @p timeout_ms milliseconds if the queue is full.
     *
     * @param block       Block of pre-timed step commands.
     * @param timeout_ms  Maximum wait time in ms (0 = non-blocking).
     * @return ESP_OK on success, ESP_ERR_TIMEOUT if the queue was full.
     */
    esp_err_t enqueueMotionBlock(const motion_block_t& block,
                                 uint32_t timeout_ms = portMAX_DELAY);

    esp_err_t enqueueStepBlock(const step_block_t& block,
                               uint32_t timeout_ms = portMAX_DELAY);

    esp_err_t enqueueSegmentBlock(const segment_block_t& block,
                                  uint32_t timeout_ms = portMAX_DELAY);

    /** @brief Legacy compatibility wrapper for old explicit-step producers. */
    esp_err_t enqueueBlock(const step_block_t& block,
                           uint32_t timeout_ms = portMAX_DELAY) {
        return enqueueStepBlock(block, timeout_ms);
    }

    /**
     * @brief Emit a constant-rate step burst for use by the multi-axis executor.
     *
     * Computes a uniform step interval from @p duration_us / @p step_count,
     * clamps it to the RMT hardware limits, then calls pushExpandedBlock()
     * to fill the RMT ring.
     *
     * This method is intended to be called from the global multi-axis executor
     * task (Core 1) when the per-axis queue is empty and not competing for the
     * driver.  It must NOT be called concurrently with the per-axis executor
     * task for the same motor.
     *
     * @param direction   true = forward, false = reverse.
     * @param step_count  Number of steps to emit.
     * @param duration_us Segment wall-clock duration in microseconds.
     * @return ESP_OK on success, error code on RMT/ring error.
     */
    esp_err_t executeConstantRateBlock(bool direction,
                                       uint16_t step_count,
                                       uint32_t duration_us);

    /**
     * @brief Force-start the RMT stream if ring has data and is not running.
     *
     * Call after distributing steps across all axes in a multi-axis segment
     * to ensure all drivers begin streaming simultaneously.
     */
    esp_err_t kickStart();

    /**
     * @brief Signal the motor to stop after the current ring contents drain.
     *
     * Does NOT flush the ring buffer (contrast with emergencyStop via driver).
     * The motor decelerates naturally to zero as pre-queued steps are consumed.
     */
    void gracefulStop();

    /**
     * @brief Number of free slots remaining in the block queue.
     *
     * Use for flow control: signal the host when this drops below
     * FLOW_CONTROL_THRESHOLD.
     */
    uint32_t available() const;

    /** @brief Return the motor id (0 or 1). */
    uint8_t motorId() const { return motor_id_; }

    /** @brief Access the bound driver (for status / enable / estop handling). */
    StepperDriver& driver() { return driver_; }

    /** @brief Const access to the bound driver. */
    const StepperDriver& driver() const { return driver_; }

    /** @brief Mark this axis as being actively driven by the multi-axis executor. */
    void setMultiExecActive(bool active) {
        multi_exec_active_.store(active, std::memory_order_release);
    }

    /** @brief True when the multi-axis executor currently owns this driver. */
    bool isMultiExecActive() const {
        return multi_exec_active_.load(std::memory_order_acquire);
    }

private:
    StepperDriver& driver_;
    uint8_t        motor_id_;
    std::atomic<bool> multi_exec_active_ {false};

    QueueHandle_t  queue_  {nullptr};
    TaskHandle_t   task_   {nullptr};

    static esp_err_t maybeStartDriver(StepperDriver& driver, bool force_start);
    static esp_err_t pushExpandedBlock(StepperDriver& driver, const step_block_t& block);
    static esp_err_t executeSegmentBlock(StepperDriver& driver, const segment_block_t& block);

    /**
     * @brief Executor task body.
     *
    * Pinned to Core 1, priority 24. Dequeues `motion_block_t`, expands any
    * compressed segments into `step_block_t`, and keeps the software ring as
    * full as possible before starting / restarting the RMT stream.
     */
    static void executorTask(void* arg);
};
```

### 3.3 `src/esp32/src/step_types.h`
```cpp
/**
 * @file step_types.h
 * @brief Shared data types for the Klipper-style stepper architecture.
 *
 * The host (Linux / Raspberry Pi) pre-computes all trajectories and sends
 * blocks of pre-timed step commands.  The ESP32 is a pure executor: it has
 * no knowledge of acceleration, kinematics, or wire geometry.
 *
 * ── RMT streaming architecture (FastAccelStepper-style) ────────────────────
 *   A simple_encoder callback streams step pulses from a lock-free ring buffer.
 *   The RMT hardware calls the encoder on-demand (ISR context), filling
 *   PART_SIZE symbols at a time in ping-pong fashion.  This eliminates the
 *   inter-block gaps that caused step loss with the old copy_encoder approach.
 *
 * ── RMT resolution analysis ────────────────────────────────────────────────
 *   Resolution : 80 MHz  (1 tick = 12.5 ns)
 *   Step shape : one RMT symbol per step
 *     HIGH = ticks / 2  (balanced pulse, FastAccelStepper-style)
 *     LOW  = ticks − HIGH
 *     Minimum: HIGH = LOW = RMT_STEP_PULSE_TICKS = 8 ticks = 100 ns each
 *
 *   Max step rate : 80 MHz / RMT_STEP_MIN_TICKS(16) = 5 000 000 steps/sec
 *   Max RPM       : 5 000 000 / (200 × 32) = 781 RPM  (at minimum ticks)
 *   160 kHz target: interval = 80 000 000 / 160 000 = 500 ticks  (6.25 µs)
 *   100 Hz  min   : interval = 800 000 ticks → clamped to 0xFFFF (65535)
 *
 *   PART_SIZE=16: one encoder callback per 16 steps.
 *     At 160 kHz: callback every 100 µs — well within FreeRTOS tick budget.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------------------------------------------------------------------------
// RMT timing constants
// ---------------------------------------------------------------------------

/** RMT TX channel resolution: 80 MHz  (1 tick = 12.5 ns) */
#define RMT_STEP_RESOLUTION_HZ  80000000UL

/** Ticks per microsecond derived from RMT_STEP_RESOLUTION_HZ (80 at 80 MHz). */
#define RMT_TICKS_PER_US        (RMT_STEP_RESOLUTION_HZ / 1000000UL)

/** Minimum half-period guard in RMT ticks (100 ns — 8 × 12.5 ns at 80 MHz;
 *  meets DRV8825/A4988 STEP pulse width minimum of 1 µs when low half is added). */
#define RMT_STEP_PULSE_TICKS    8U

/**
 * Minimum total interval in ticks.
 * 16 ticks × 12.5 ns = 200 ns → 5 MHz step rate ceiling at hardware level.
 * Ensures duration1 = interval_ticks − PULSE_TICKS ≥ 8 ticks.
 */
#define RMT_STEP_MIN_TICKS      16U

/** Maximum interval in ticks: 16-bit RMT field → 65535 ticks = ~819 µs → ~1.2 kHz floor */
#define RMT_STEP_MAX_TICKS      0xFFFFU

/** Default hold interval before any step has been consumed (= minimum interval).
 *  Prevents duration1 wraparound to ~65535 ticks on first ring-empty hold. */
#define RMT_STEP_DEFAULT_TICKS  RMT_STEP_MIN_TICKS

// ---------------------------------------------------------------------------
// RMT streaming constants (FastAccelStepper-style ping-pong)
// ---------------------------------------------------------------------------

/** Symbols per ping-pong half-buffer.  Must divide RMT_MEM_SYMBOLS evenly.
 *  Hardware requires `mem_block_symbols` to be even and at least 64, so the
 *  minimum practical PART_SIZE is 32 (2 × PART_SIZE = 64 symbols per channel).
 *  PART_SIZE=32 gives one encoder callback per 32 steps. */
#define PART_SIZE               32U

/** Total RMT hardware memory per channel (2 × PART_SIZE for ping-pong).
 *  Must be >= 64 for IDF RMT driver constraints. */
#define RMT_MEM_SYMBOLS         (2U * PART_SIZE)

/**
 * Minimum command duration in ticks (200 µs at 2 MHz = 400 ticks).
 * Used as pause filler when the ring buffer empties or on direction changes.
 */
#define MIN_CMD_TICKS           400U

// ---------------------------------------------------------------------------
// Ring buffer sizing
// ---------------------------------------------------------------------------

/** Ring buffer size — must be a power of 2.
 *  4096 entries = 58ms at 70kHz (cruise speed, ~660 RPM).
 *  Must cover the worst-case host re-fill time: 25 segments × 1ms SPI = 25ms,
 *  plus OS jitter.  4096 gives 58ms >> 25ms, preventing ring starvation when
 *  all deferred notifications fire simultaneously. */
#define STEP_RING_SIZE          4096U

/** Bit mask for ring buffer index wrap-around. */
#define STEP_RING_MASK          (STEP_RING_SIZE - 1U)

// ---------------------------------------------------------------------------
// Block / queue sizing
// ---------------------------------------------------------------------------

/** Number of step commands per block. */
#define STEP_BLOCK_SIZE         64

/** Number of compressed motion segments per transport block. */
#define SEGMENT_BLOCK_SIZE      60

/** Buffered step target before starting/restarting the RMT stream.
 *  Must satisfy: STEP_STREAM_START_FILL >= 2 * PART_SIZE. For
 *  PART_SIZE=32 we use 128 (4 * PART_SIZE) to absorb host SPI latency
 *  during startup at high speed before the first refill arrives. */
#define STEP_STREAM_START_FILL  (4U * PART_SIZE)

/**
 * Minimum steps required to RESTART the RMT stream after an underrun.
 * Lower than STEP_STREAM_START_FILL: at low speed the ring drains faster
 * than the inter-segment gap, so we must restart with fewer steps buffered.
 * PART_SIZE/2 = 16 steps guarantees at least one half-callback of data.
 */
#define STEP_STREAM_RESTART_FILL  (PART_SIZE / 2U)   // = 16

/**
 * Depth of the FreeRTOS step-block queue (per motor).
 * Increased to 24 to handle slower host SPI latency and maintain RMT fill rate.
 * Provides ~(STEPPER_QUEUE_DEPTH × STEP_BLOCK_SIZE) steps of look-ahead.
 */
#define STEPPER_QUEUE_DEPTH     24

/**
 * Flow-control threshold: the comm layer sends a NACK / buffer-full warning
 * to the host when the number of free queue slots drops below this value.
 */
#define FLOW_CONTROL_THRESHOLD  4

/** Microstep denominator — override with -DMICROSTEPS=N in build flags. */
#ifndef MICROSTEPS
#define MICROSTEPS 32
#endif

// ---------------------------------------------------------------------------
// Core data types
// ---------------------------------------------------------------------------

/**
 * @brief A single pre-timed step command.
 *
 * @note  interval_ticks == 0  →  stop marker (end of move).
 * @note  All steps within one block MUST share the same direction.
 *        The host is responsible for splitting trajectories at direction
 *        reversals; crossing a reversal within a block is undefined behaviour.
 */
typedef struct {
    uint32_t interval_ticks; /**< RMT ticks until the NEXT step (0 = stop)          */
    bool     direction;      /**< true = forward / CW, false = reverse / CCW         */
} step_cmd_t;

/**
 * @brief A pre-computed block of step commands.
 *
 * The host fills steps[0..count-1] and sets count.  The executor iterates
 * only up to count; remaining entries are ignored.
 */
typedef struct {
    step_cmd_t steps[STEP_BLOCK_SIZE]; /**< Pre-timed step commands              */
    uint32_t   count;                  /**< Number of valid entries in steps[]   */
} step_block_t;

/**
 * @brief A compressed motion segment.
 *
 * Represents `step_count` successive steps where the interval evolves as:
 *
 *   ticks[n] = start_ticks + n * add_ticks
 *
 * This is the same basic representation used by Klipper-style trapezoid
 * segments: the host sends a compact arithmetic description, the MCU expands
 * it locally into concrete step timings.
 */
typedef struct {
    uint16_t step_count;   /**< Number of steps encoded by the segment          */
    uint16_t start_ticks;  /**< Interval for the first step in RMT ticks        */
    int16_t  add_ticks;    /**< Delta applied after each emitted step           */
    uint8_t  direction;    /**< true = forward / CW, false = reverse / CCW      */
    uint8_t  reserved;     /**< Padding / future flags                          */
} motion_segment_t;

/**
 * @brief A transport block of compressed motion segments.
 */
typedef struct {
    motion_segment_t segments[SEGMENT_BLOCK_SIZE];
    uint32_t         count;   /**< Number of valid segments[] entries             */
} segment_block_t;

typedef enum {
    MOTION_BLOCK_KIND_STEP = 0,
    MOTION_BLOCK_KIND_SEGMENT = 1,
} motion_block_kind_t;

/**
 * @brief Queue item exchanged between the comm task and the executor task.
 *
 * The comm task can enqueue either legacy per-step blocks or compressed
 * segment blocks. The executor expands segment blocks into `step_block_t`
 * chunks locally before feeding the driver ring.
 */
typedef struct {
    uint8_t kind;
    uint8_t reserved[3];
    union {
        step_block_t    step;
        segment_block_t segment;
    } payload;
} motion_block_t;

/**
 * @brief Ring buffer entry consumed by the RMT encoder callback (ISR context).
 *
 * Produced by the executor task, consumed by the simple_encoder callback.
 * SPSC: one writer (task), one reader (ISR).
 */
typedef struct {
    uint16_t ticks;      /**< Total step period in RMT ticks (2 MHz).  0 = invalid. */
    uint8_t  toggle_dir; /**< 1 = toggle DIR pin before this step.                   */
    uint8_t  pad;        /**< Padding for 4-byte alignment.                          */
} ring_entry_t;

/**
 * @brief Wire packet received from the host over UART/SPI.
 *
 * The comm layer deserialises this from the byte stream and routes it to
 * the appropriate motor queue based on motor_id.
 */
typedef struct {
    uint8_t    motor_id;               /**< Target motor: 0 or 1                 */
    uint8_t    block_seq;              /**< Rolling sequence number (loss detect) */
    uint32_t   count;                  /**< Number of valid steps in payload      */
    step_cmd_t steps[STEP_BLOCK_SIZE]; /**< Step payload                          */
} comm_packet_t;

// ---------------------------------------------------------------------------
// Multi-axis synchronised segment block (MULTI_AXIS_SEGMENT_BLOCK = 0x13)
// ---------------------------------------------------------------------------

/** Maximum number of axes in a multi-axis segment block. */
#define MULTI_AXIS_MAX_AXES  4

/** Maximum number of segments per multi-axis block. */
#define MULTI_AXIS_BLOCK_SIZE  60

/**
 * @brief One synchronised multi-axis time-based segment.
 *
 * The host sends one of these records per segment.  All axes execute their
 * respective step_counts over the shared duration_us, with steps distributed
 * evenly in time by the MCU.  direction_mask bit i = 1 means axis i reverses.
 */
typedef struct {
    uint16_t motion_sequence; /**< Globally increasing motion identifier       */
    uint16_t duration_us;     /**< Wall-clock duration of this segment in µs   */
    uint16_t direction_mask;  /**< Bit i=1: axis i runs in reverse direction   */
    uint16_t step_counts[MULTI_AXIS_MAX_AXES]; /**< Steps per axis             */
} multi_axis_segment_t;

/**
 * @brief Block of synchronised multi-axis segments received from the host.
 *
 * Enqueued into the global multi-axis queue and consumed by the executor.
 */
typedef struct {
    uint8_t              axis_ids[MULTI_AXIS_MAX_AXES]; /**< Logical axis IDs  */
    uint8_t              axis_count;                    /**< Valid entries      */
    uint8_t              segment_count;                 /**< Valid segments     */
    multi_axis_segment_t segments[MULTI_AXIS_BLOCK_SIZE];
} multi_axis_block_t;

/**
 * @brief Flush request: discard all segments with motion_sequence > threshold.
 */
typedef struct {
    uint16_t flush_sequence; /**< Keep segments ≤ this; discard the rest       */
} flush_request_t;

#ifdef __cplusplus
} /* extern "C" */
#endif
```

### 3.4 `src/esp32/src/comm_interface.cpp`
```cpp
/**
 * @file comm_interface.cpp
 * @brief SPI slave communication interface implementation.
 *
 * The ESP32 is the SPI slave. Every transfer is a fixed-size wire frame:
 *
 *   Host TX frame  ──► ESP32 parses and executes request
 *   Host RX frame ◄── ESP32 returns latest status payload
 *
 * Status is therefore naturally pipelined by one SPI transaction, which keeps
 * the slave task simple and deterministic.
 */

#include "comm_interface.h"

#include <string.h>
#include <driver/spi_slave.h>
#include <driver/gpio.h>
#include <esp_attr.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_check.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "motion_planner.h"

static const char* TAG = "comm_iface";

static constexpr uint32_t  SPI_TASK_STACK  = 4096;
static constexpr UBaseType_t SPI_TASK_PRIO = 10;
static constexpr BaseType_t  SPI_TASK_CORE = 0;

static constexpr uint32_t    MULTI_EXEC_STACK  = 8192;
static constexpr UBaseType_t MULTI_EXEC_PRIO   = 20;
static constexpr BaseType_t  MULTI_EXEC_CORE   = 1;

/**
 * @brief Hard wall-clock budget for one drain-loop sub-slice before the
 *        executor unconditionally yields to the FreeRTOS scheduler.
 *
 * The yield is UNCONDITIONAL — it does NOT depend on ring-fill level.
 * RMT pulse timing is in hardware, so a 1 ms scheduler sleep never
 * introduces step jitter.
 *
 * NOTE: Superseded by EXEC_TIME_BUDGET_US in motion_planner.h for the
 * state-machine executor. Retained for the per-axis executorTask in
 * stepper_queue.cpp which still uses it indirectly.
 */
static constexpr int64_t  YIELD_INTERVAL_US      = 400;

/**
 * @brief Maximum queue entries drained in a single bounded flush loop.
 *
 * Used by handleFlush() in the SPI ingestion path only.
 * The planner and executor have their own bounded drain constants.
 */
static constexpr uint32_t MAX_FLUSH_DRAIN        = 8;

/**
 * @brief Global queue of multi-axis segment blocks fed by the SPI task and
 *        consumed by the multi-axis executor task.
 *
 * Depth is sized to hold ~600 ms of motion at 4 ms/segment.
 */
static constexpr uint32_t MULTI_AXIS_QUEUE_DEPTH = 64;
static QueueHandle_t s_multi_axis_queue  = nullptr;

/**
 * @brief Global queue for flush requests.  Depth 4 is more than enough since
 *        the host can only issue one flush at a time.
 */
static constexpr uint32_t FLUSH_QUEUE_DEPTH = 4;
static QueueHandle_t s_flush_queue = nullptr;

DMA_ATTR static uint8_t s_rx_frame[SPI_FRAME_SIZE];
DMA_ATTR static uint8_t s_tx_frame_a[SPI_FRAME_SIZE];
DMA_ATTR static uint8_t s_tx_frame_b[SPI_FRAME_SIZE];

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

CommInterface::CommInterface(StepperQueue* queues[], uint8_t n_motors)
    : n_motors_(n_motors < SPI_MAX_AXES ? n_motors : SPI_MAX_AXES)
{
    for (uint8_t i = 0; i < SPI_MAX_AXES; ++i) {
        queues_[i] = (i < n_motors_) ? queues[i] : nullptr;
    }
}

// ---------------------------------------------------------------------------
// init()
// ---------------------------------------------------------------------------

esp_err_t CommInterface::init(const SpiBusPins& pins)
{
    pins_ = pins;

    if (pins_.home_pin_no != GPIO_NUM_NC && pins_.home_pin_nc != GPIO_NUM_NC) {
        gpio_config_t home_cfg = {};
        home_cfg.pin_bit_mask = (1ULL << static_cast<uint32_t>(pins_.home_pin_no))
                               | (1ULL << static_cast<uint32_t>(pins_.home_pin_nc));
        home_cfg.mode = GPIO_MODE_INPUT;
        home_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
        home_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        home_cfg.intr_type = GPIO_INTR_DISABLE;
        ESP_RETURN_ON_ERROR(gpio_config(&home_cfg), TAG, "failed to configure home sensor pins");
    }

    // Create the global multi-axis segment queue.
    s_multi_axis_queue = xQueueCreate(MULTI_AXIS_QUEUE_DEPTH, sizeof(multi_axis_block_t));
    ESP_RETURN_ON_FALSE(s_multi_axis_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create multi-axis queue");

    // Create the global flush request queue.
    s_flush_queue = xQueueCreate(FLUSH_QUEUE_DEPTH, sizeof(flush_request_t));
    ESP_RETURN_ON_FALSE(s_flush_queue != nullptr, ESP_ERR_NO_MEM, TAG,
                        "failed to create flush queue");

    spi_bus_config_t bus_cfg = {};
    bus_cfg.mosi_io_num = pins_.mosi;
    bus_cfg.miso_io_num = pins_.miso;
    bus_cfg.sclk_io_num = pins_.sclk;
    bus_cfg.quadwp_io_num = -1;
    bus_cfg.quadhd_io_num = -1;
    bus_cfg.max_transfer_sz = SPI_FRAME_SIZE;

    spi_slave_interface_config_t slave_cfg = {};
    slave_cfg.mode = 0;
    slave_cfg.spics_io_num = pins_.cs;
    slave_cfg.queue_size = 1;
    slave_cfg.flags = 0;
    slave_cfg.post_setup_cb = nullptr;
    slave_cfg.post_trans_cb = nullptr;

    ESP_RETURN_ON_ERROR(
        spi_slave_initialize(SPI3_HOST, &bus_cfg, &slave_cfg, SPI_DMA_CH_AUTO),
        TAG, "spi_slave_initialize failed");

    BaseType_t rc = xTaskCreatePinnedToCore(
        &CommInterface::spiTask,
        "comm_spi",
        SPI_TASK_STACK,
        this,
        SPI_TASK_PRIO,
        nullptr,
        SPI_TASK_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create comm_spi task");

    // ── Planner layer: decomposes multi-axis blocks into planned segments ───
    ESP_RETURN_ON_ERROR(planner_.init(s_multi_axis_queue, s_flush_queue),
                        TAG, "failed to init motion planner");

    rc = xTaskCreatePinnedToCore(
        &CommInterface::multiAxisExecutorTask,
        "multi_exec",
        MULTI_EXEC_STACK,
        this,
        MULTI_EXEC_PRIO,
        nullptr,
        MULTI_EXEC_CORE);

    ESP_RETURN_ON_FALSE(rc == pdPASS, ESP_ERR_NO_MEM, TAG,
                        "failed to create multi_exec task");

    // Delegate endstop ISR registration to the lateral axis driver (axis 1).
    // StepperDriver owns endstop_active_ and executor_task_, so the ISR
    // can act without going through CommInterface.
    if (n_motors_ >= 2 && queues_[1] != nullptr) {
        ESP_RETURN_ON_ERROR(
            queues_[1]->driver().initEndstopIsr(pins_.home_pin_no, pins_.home_pin_nc),
            TAG, "initEndstopIsr failed");
    }

    ESP_LOGI(TAG, "SPI slave ready  MOSI=%d MISO=%d SCLK=%d CS=%d  frame=%uB",
             (int)pins_.mosi, (int)pins_.miso, (int)pins_.sclk, (int)pins_.cs,
             (unsigned)SPI_FRAME_SIZE);
    return ESP_OK;
}

void CommInterface::buildStatusFrame(uint8_t* out_frame) const
{
    spi_message_zero_frame(out_frame);

    auto* header = reinterpret_cast<SpiMessageHeader*>(out_frame);
    auto* payload = reinterpret_cast<StatusPayload*>(out_frame + sizeof(SpiMessageHeader));

    spi_message_init_header(*header,
                            SpiMessageType::STATUS,
                            last_rx_sequence_,
                            sizeof(StatusPayload),
                            0);

    payload->uptime_ms = static_cast<uint32_t>(xTaskGetTickCount() * portTICK_PERIOD_MS);
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        if (axis < n_motors_ && queues_[axis] != nullptr) {
            payload->queue_free_slots[axis] = static_cast<uint16_t>(queues_[axis]->available());
            payload->ring_free_slots[axis] = static_cast<uint16_t>(queues_[axis]->driver().ringFreeSlots());
            payload->underrun_count[axis] = queues_[axis]->driver().getUnderrunCount();
            if (queues_[axis]->driver().isStreaming()) {
                payload->running_mask |= static_cast<uint8_t>(1U << axis);
            }
            if (queues_[axis]->driver().isEnabled()) {
                payload->enabled_mask |= static_cast<uint8_t>(1U << axis);
            }
        } else {
            payload->queue_free_slots[axis] = 0;
            payload->ring_free_slots[axis]  = 0;
            payload->underrun_count[axis]   = 0;
        }
    }
    payload->last_rx_sequence = last_rx_sequence_;
    payload->last_rx_type     = last_rx_type_;
    payload->last_result      = last_result_;
    payload->protocol_version = SPI_MSG_VERSION;
    payload->lateral_endstop_state = readLateralEndstopState();

    payload->endstop_armed_mask = 0;
    for (uint8_t axis = 0; axis < SPI_MAX_AXES; ++axis) {
        if (axis < n_motors_ && queues_[axis] != nullptr) {
            if (queues_[axis]->driver().isEndstopArmed()) {
                payload->endstop_armed_mask |= static_cast<uint8_t>(1U << axis);
            }
        }
    }

    // Atomic load — lock-free cross-core read (written by Core 1 executor).
    payload->last_executed_sequence = last_executed_sequence_.load(std::memory_order_acquire);

    // Planner lookahead pressure: how many slots are free in segment_queue_.
    const uint32_t pqf = planner_.segmentQueueFree();
    payload->planner_queue_free = static_cast<uint8_t>(pqf < 255u ? pqf : 255u);

    spi_message_finalize(out_frame);
}

esp_err_t CommInterface::handleEnableAxis(const EnableAxisPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    if (payload.enable) {
        queues_[payload.axis_id]->driver().enable();
    } else {
        queues_[payload.axis_id]->driver().disable();
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleEmergencyStop(const EmergencyStopPayload& payload)
{
    if (payload.axis_id == 0xFF) {
        for (uint8_t axis = 0; axis < n_motors_; ++axis) {
            if (queues_[axis] != nullptr) {
                queues_[axis]->driver().emergencyStop();
            }
        }
        return ESP_OK;
    }

    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    queues_[payload.axis_id]->driver().emergencyStop();
    return ESP_OK;
}

esp_err_t CommInterface::handleStopAxis(const EmergencyStopPayload& payload)
{
    // gracefulStop() marks the driver as stopped but does NOT flush the ring
    // buffer, so the motor decelerates naturally through any remaining queued
    // steps rather than cutting out instantly.
    if (payload.axis_id == 0xFF) {
        for (uint8_t axis = 0; axis < n_motors_; ++axis) {
            if (queues_[axis] != nullptr) {
                queues_[axis]->gracefulStop();
            }
        }
        return ESP_OK;
    }

    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    queues_[payload.axis_id]->gracefulStop();
    return ESP_OK;
}

esp_err_t CommInterface::handleDisableAll()
{
    for (uint8_t axis = 0; axis < n_motors_; ++axis) {
        if (queues_[axis] != nullptr) {
            queues_[axis]->driver().disable();
        }
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleResetStats()
{
    for (uint8_t axis = 0; axis < n_motors_; ++axis) {
        if (queues_[axis] != nullptr) {
            queues_[axis]->driver().resetUnderrunCount();
        }
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleEnableEndstop(const EnableEndstopPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    StepperDriver& drv = queues_[payload.axis_id]->driver();
    if (payload.arm) {
        drv.armEndstop();
        ESP_LOGI(TAG, "endstop armed on axis %u", payload.axis_id);
    } else {
        drv.disarmEndstop();
        ESP_LOGI(TAG, "endstop disarmed on axis %u", payload.axis_id);
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleStepBlock(const StepBlockPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!isLateralMovementAllowed(payload.axis_id)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (payload.step_count > STEP_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    motion_block_t block {};
    block.kind = MOTION_BLOCK_KIND_STEP;
    block.payload.step.count = payload.step_count;
    for (uint32_t i = 0; i < block.payload.step.count; ++i) {
        block.payload.step.steps[i].interval_ticks = payload.entries[i].interval_ticks;
        block.payload.step.steps[i].direction = (payload.entries[i].flags & SpiStepFlags::DIR_REVERSE) != 0;
    }

    return queues_[payload.axis_id]->enqueueMotionBlock(block, 0);
}

esp_err_t CommInterface::handleSegmentBlock(const SegmentBlockPayload& payload)
{
    if (payload.axis_id >= n_motors_ || queues_[payload.axis_id] == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!isLateralMovementAllowed(payload.axis_id)) {
        return ESP_ERR_INVALID_STATE;
    }
    if (payload.segment_count > SEGMENT_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    motion_block_t block {};
    block.kind = MOTION_BLOCK_KIND_SEGMENT;
    block.payload.segment.count = payload.segment_count;
    for (uint32_t i = 0; i < block.payload.segment.count; ++i) {
        block.payload.segment.segments[i].step_count = payload.segments[i].step_count;
        block.payload.segment.segments[i].start_ticks = payload.segments[i].start_ticks;
        block.payload.segment.segments[i].add_ticks = payload.segments[i].add_ticks;
        block.payload.segment.segments[i].direction =
            (payload.segments[i].flags & SpiStepFlags::DIR_REVERSE) != 0;
        block.payload.segment.segments[i].reserved = 0;
    }

    return queues_[payload.axis_id]->enqueueMotionBlock(block, 0);
}

esp_err_t CommInterface::handleMultiAxisSegmentBlock(const uint8_t* payload,
                                                     uint16_t payload_length)
{
    /*
     * Wire layout for MULTI_AXIS_SEGMENT_BLOCK payload:
     *
     *   MultiAxisSegmentBlockHeader   (4 bytes)
     *   uint8_t  axis_ids[axis_count] (axis_count bytes)
     *   For each segment:
     *     uint16_t motion_sequence    (2 bytes)
     *     uint16_t duration_us        (2 bytes)
     *     uint16_t direction_mask     (2 bytes)
     *     uint16_t step_counts[axis_count] (2 * axis_count bytes)
     *
     * Total minimum: 4 + axis_count + segment_count * (6 + 2*axis_count)
     */
    if (payload_length < sizeof(MultiAxisSegmentBlockHeader)) {
        return ESP_ERR_INVALID_SIZE;
    }

    MultiAxisSegmentBlockHeader hdr_val;
    memcpy(&hdr_val, payload, sizeof(hdr_val));
    const uint8_t axis_count     = hdr_val.axis_count;
    const uint8_t segment_count  = hdr_val.segment_count;

    if (axis_count == 0 || axis_count > MULTI_AXIS_MAX_AXES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (segment_count == 0 || segment_count > MULTI_AXIS_BLOCK_SIZE) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Validate total payload length before reading any further.
    const size_t expected_length =
        sizeof(MultiAxisSegmentBlockHeader)
        + static_cast<size_t>(axis_count)
        + static_cast<size_t>(segment_count) * (6u + 2u * axis_count);
    if (payload_length < static_cast<uint16_t>(expected_length)) {
        return ESP_ERR_INVALID_SIZE;
    }

    // Deserialise.
    multi_axis_block_t block {};
    block.axis_count     = axis_count;
    block.segment_count  = segment_count;

    const uint8_t* cursor = payload + sizeof(MultiAxisSegmentBlockHeader);

    // axis_ids
    for (uint8_t a = 0; a < axis_count; ++a) {
        block.axis_ids[a] = cursor[a];
    }
    cursor += axis_count;

    // segments
    for (uint8_t s = 0; s < segment_count; ++s) {
        uint16_t motion_seq, duration_us, dir_mask;
        memcpy(&motion_seq,  cursor,     2);
        memcpy(&duration_us, cursor + 2, 2);
        memcpy(&dir_mask,    cursor + 4, 2);
        cursor += 6;

        block.segments[s].motion_sequence = motion_seq;
        block.segments[s].duration_us     = duration_us;
        block.segments[s].direction_mask  = dir_mask;

        for (uint8_t a = 0; a < axis_count; ++a) {
            uint16_t steps;
            memcpy(&steps, cursor, 2);
            block.segments[s].step_counts[a] = steps;
            cursor += 2;
        }
    }

    // Non-blocking enqueue: return QUEUE_FULL immediately if full.
    if (xQueueSend(s_multi_axis_queue, &block, 0) != pdTRUE) {
        return ESP_ERR_TIMEOUT; // maps to QUEUE_FULL result code
    }
    return ESP_OK;
}

esp_err_t CommInterface::handleFlush(const FlushPayload& flush_payload)
{
    /*
     * Post a flush_request_t to the flush queue.  The executor task
     * watches this queue and applies the flush before processing the next
     * segment.  Using a queue (instead of an atomic variable) ensures that
     * a flush posted just before new segments arrive is always processed in
     * the correct order.
     */
    flush_request_t req { .flush_sequence = flush_payload.flush_sequence };
    if (xQueueSend(s_flush_queue, &req, 0) != pdTRUE) {
        // Flush queue full — this should never happen in normal operation.
        ESP_LOGW(TAG, "flush queue full — flush_seq=%u dropped",
                 (unsigned)flush_payload.flush_sequence);
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

void CommInterface::notifySegmentExecuted(uint16_t motion_seq)
{
    /*
     * Called by the executor task (Core 1) after each multi-axis segment
     * completes.  Updates last_executed_sequence_ with an atomic store
     * so the SPI task (Core 0) can safely read it in buildStatusFrame().
     *
     * Only advances the sequence — never moves it backward.  This handles
     * the 16-bit wrap-around case correctly because we only call this in
     * strict execution order.
     */
    uint16_t current = last_executed_sequence_.load(std::memory_order_relaxed);
    if (static_cast<int16_t>(motion_seq - current) > 0) {
        last_executed_sequence_.store(motion_seq, std::memory_order_release);
    }
}

uint8_t CommInterface::readLateralEndstopState() const
{
    if (pins_.home_pin_no == GPIO_NUM_NC || pins_.home_pin_nc == GPIO_NUM_NC) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }

    const int no_state = gpio_get_level(pins_.home_pin_no);
    const int nc_state = gpio_get_level(pins_.home_pin_nc);

    if (no_state == nc_state) {
        return static_cast<uint8_t>(LateralEndstopState::ABSENT);
    }
    if (no_state == 0 && nc_state == 1) {
        return static_cast<uint8_t>(LateralEndstopState::PRESENT_CLOSED);
    }
    return static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}

bool CommInterface::isLateralMovementAllowed(uint8_t axis_id) const
{
    if (axis_id != 1) {
        return true;
    }
    return readLateralEndstopState() == static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);
}

esp_err_t CommInterface::handleFrame(const SpiMessageHeader& header, const uint8_t* payload)
{
    switch (static_cast<SpiMessageType>(header.msg_type)) {
    case SpiMessageType::NOP:
    case SpiMessageType::GET_STATUS:
    case SpiMessageType::PING:
        return ESP_OK;

    case SpiMessageType::ENABLE_AXIS: {
        if (header.payload_length != sizeof(EnableAxisPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EnableAxisPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEnableAxis(p);
    }

    case SpiMessageType::ESTOP: {
        if (header.payload_length != sizeof(EmergencyStopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EmergencyStopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEmergencyStop(p);
    }

    case SpiMessageType::STOP_AXIS: {
        if (header.payload_length != sizeof(EmergencyStopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EmergencyStopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleStopAxis(p);
    }

    case SpiMessageType::DISABLE_ALL:
        if (header.payload_length != 0) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleDisableAll();

    case SpiMessageType::RESET_STATS:
        if (header.payload_length != 0) {
            return ESP_ERR_INVALID_SIZE;
        }
        return handleResetStats();

    case SpiMessageType::STEP_BLOCK: {
        if (header.payload_length != sizeof(StepBlockPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        StepBlockPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleStepBlock(p);
    }

    case SpiMessageType::SEGMENT_BLOCK: {
        if (header.payload_length != sizeof(SegmentBlockPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        SegmentBlockPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleSegmentBlock(p);
    }

    case SpiMessageType::MULTI_AXIS_SEGMENT_BLOCK:
        // Variable-length payload — pass raw buffer + length.
        return handleMultiAxisSegmentBlock(payload, header.payload_length);

    case SpiMessageType::FLUSH: {
        if (header.payload_length != sizeof(FlushPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        FlushPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleFlush(p);
    }

    case SpiMessageType::ENABLE_ENDSTOP: {
        if (header.payload_length != sizeof(EnableEndstopPayload)) {
            return ESP_ERR_INVALID_SIZE;
        }
        EnableEndstopPayload p;
        memcpy(&p, payload, sizeof(p));
        return handleEnableEndstop(p);
    }

    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
}

void CommInterface::spiTask(void* arg)
{
    auto* self = static_cast<CommInterface*>(arg);
    ESP_LOGI(TAG, "SPI task started on core %d", xPortGetCoreID());

    // Double-buffer ping-pong: while DMA transmits tx_ping, we build the next
    // status frame into tx_pong.  This reduces pipeline lag by one full SPI
    // round-trip — the status sent in transaction N reflects state AFTER
    // transaction N-1 was handled, not state from before the previous transmit.
    uint8_t* tx_ping = s_tx_frame_a;
    uint8_t* tx_pong = s_tx_frame_b;

    // Pre-build the very first frame before entering the loop so the initial
    // transaction has valid (zero-but-structured) content.
    self->buildStatusFrame(tx_ping);

    for (;;) {
        // ── Transmit the previously-built status frame ────────────────────────
        spi_slave_transaction_t txn = {};
        txn.length = SPI_FRAME_SIZE * 8;
        txn.tx_buffer = tx_ping;
        txn.rx_buffer = s_rx_frame;

        esp_err_t err = spi_slave_transmit(SPI3_HOST, &txn, portMAX_DELAY);
        // DMA is done with tx_ping — safe to reuse as the next write buffer.

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_slave_transmit failed: %s", esp_err_to_name(err));
            // Rebuild into the same ping buffer and retry.
            self->buildStatusFrame(tx_ping);
            continue;
        }

        // ── Parse and handle incoming frame ───────────────────────────────────
        SpiMessageHeader header {};
        memcpy(&header, s_rx_frame, sizeof(SpiMessageHeader));

        if (header.magic != SPI_MSG_MAGIC) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_MAGIC);
        } else if (header.version != SPI_MSG_VERSION) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_VERSION);
        } else if (header.payload_length > SPI_MAX_PAYLOAD_SIZE) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_LENGTH);
        } else if (!spi_message_validate(s_rx_frame, header)) {
            self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_CRC);
        } else {
            self->last_rx_sequence_ = header.sequence;
            self->last_rx_type_ = header.msg_type;

            const uint8_t* payload = s_rx_frame + sizeof(SpiMessageHeader);
            err = self->handleFrame(header, payload);
            if (err == ESP_OK) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::OK);
            } else if (err == ESP_ERR_TIMEOUT) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::QUEUE_FULL);
            } else if (err == ESP_ERR_INVALID_ARG) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_AXIS);
            } else if (err == ESP_ERR_INVALID_SIZE) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::BAD_LENGTH);
            } else if (err == ESP_ERR_NOT_SUPPORTED) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::UNKNOWN_TYPE);
            } else if (err == ESP_ERR_INVALID_STATE) {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::ENDSTOP_BLOCKED);
            } else {
                self->last_result_ = static_cast<uint8_t>(SpiMessageResult::INTERNAL_ERROR);
                ESP_LOGW(TAG, "message 0x%02X failed: %s",
                         header.msg_type, esp_err_to_name(err));
            }
        }

        // ── Build next status frame into the now-idle buffer ──────────────────
        // We write into tx_pong (the buffer NOT currently wired to DMA).
        // Reflects state AFTER handling the frame we just received.
        self->buildStatusFrame(tx_pong);

        // Swap: tx_pong becomes the next transmit buffer.
        uint8_t* tmp = tx_ping;
        tx_ping = tx_pong;
        tx_pong = tmp;
    }
}

// ---------------------------------------------------------------------------
// multiAxisExecutorTask()  — Core 1, priority 20 — STATE MACHINE
// ---------------------------------------------------------------------------
//
// Refactored from a monolithic nested-loop drain pattern into a bounded
// state machine.  Each state transition does bounded work (≤ EXEC_TIME_BUDGET_US
// or ≤ EXEC_BATCH_LIMIT segments) then yields to the scheduler.
//
//   IDLE ──► FETCH ──► DRAIN ──► RUN ──► IDLE
//              │                          ▲
//              ├── (flush sentinel) ──► FLUSH ──┘
//              └── (error/underrun) ──► RECOVERY ──┘
//
// Watchdog safety: NO state executes for more than ~300 µs without exiting
// to the for(;;) top-level loop which naturally yields via xQueueReceive
// or explicit vTaskDelay(1).

void CommInterface::multiAxisExecutorTask(void* arg)
{
    auto* self = static_cast<CommInterface*>(arg);

    ESP_LOGI(TAG, "multi-axis executor (state machine) started on core %d",
             xPortGetCoreID());

    // Register this task for ISR ring-space wakeups on all drivers.
    {
        TaskHandle_t my_handle = xTaskGetCurrentTaskHandle();
        for (uint8_t a = 0; a < self->n_motors_; ++a) {
            if (self->queues_[a] != nullptr) {
                self->queues_[a]->driver().setExecutorTask(my_handle);
            }
        }
    }

    ESP_LOGI(TAG, "multi_exec stack high watermark at start: %u bytes free",
             (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));

    // ── Segment queue handle from the planner ─────────────────────────────
    QueueHandle_t seg_queue = self->planner_.segmentQueue();

    // ── Deferred notification ring ────────────────────────────────────────
    static constexpr int DEFER_DEPTH = 256;
    static int64_t  defer_fire_us[DEFER_DEPTH];
    static uint32_t defer_seqs[DEFER_DEPTH];
    int      defer_head = 0;
    int      defer_tail = 0;

    // ── Active axis tracking (persists across iterations for recovery) ────
    uint8_t active_axis_ids[MULTI_AXIS_MAX_AXES] = {};
    uint8_t active_axis_count = 0;

    // ── Batch buffer for FETCH state ──────────────────────────────────────
    planned_segment_t batch[EXEC_BATCH_LIMIT];
    uint32_t batch_count = 0;
    uint32_t batch_index = 0;

    // ── State machine ─────────────────────────────────────────────────────
    ExecState state = ExecState::IDLE;

    // Lambda: fire all due deferred notifications.
    auto fireDeferred = [&]() {
        const int64_t now = esp_timer_get_time();
        while (defer_head != defer_tail) {
            const int idx = defer_head & (DEFER_DEPTH - 1);
            if (now >= defer_fire_us[idx]) {
                self->notifySegmentExecuted(
                    static_cast<uint16_t>(defer_seqs[idx]));
                ++defer_head;
            } else {
                break;
            }
        }
    };

    // Lambda: kickStart all active axes.
    auto kickStartActiveAxes = [&]() {
        for (uint8_t a = 0; a < active_axis_count; ++a) {
            const uint8_t axis_id = active_axis_ids[a];
            if (axis_id < self->n_motors_ && self->queues_[axis_id] != nullptr) {
                self->queues_[axis_id]->kickStart();
            }
        }
    };

    // ── Main loop ─────────────────────────────────────────────────────────
    for (;;) {
        static uint32_t wm_iter = 0;
        if (++wm_iter % 2000 == 0) {
            ESP_LOGD(TAG, "multi_exec stack watermark: %u bytes free",
                     (unsigned)(uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t)));
        }

        // Always fire due deferred notifications at top of loop.
        fireDeferred();

        switch (state) {

        // ══════════════════════════════════════════════════════════════════
        // IDLE: wait for segments from the planner (blocking with timeout)
        // ══════════════════════════════════════════════════════════════════
        case ExecState::IDLE: {
            // Compute wait timeout: wake early if a deferred notification
            // is about to fire.  Hard-cap at 1 ms so ISR ring-space
            // notifications (which wake ulTaskNotifyTake, not xQueueReceive)
            // don't cause >1 ms stalls.
            TickType_t wait_ticks;
            if (defer_head != defer_tail) {
                const int idx = defer_head & (DEFER_DEPTH - 1);
                const int64_t remaining_us =
                    defer_fire_us[idx] - esp_timer_get_time();
                if (remaining_us <= 500) {
                    wait_ticks = 0;
                } else {
                    wait_ticks = 1;
                }
            } else {
                wait_ticks = pdMS_TO_TICKS(1);
            }

            planned_segment_t seg;
            if (xQueueReceive(seg_queue, &seg, wait_ticks) == pdTRUE) {
                batch[0]    = seg;
                batch_count = 1;
                batch_index = 0;
                state = ExecState::FETCH;
            } else {
                // Timeout — kick-start any stalled axes (RMT underrun
                // while we were blocked on xQueueReceive).
                kickStartActiveAxes();
            }
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // FETCH: non-blocking batch fill up to EXEC_BATCH_LIMIT
        // ══════════════════════════════════════════════════════════════════
        case ExecState::FETCH: {
            // Fill remaining batch slots non-blocking.
            while (batch_count < EXEC_BATCH_LIMIT) {
                planned_segment_t seg;
                if (xQueueReceive(seg_queue, &seg, 0) != pdTRUE) break;
                batch[batch_count++] = seg;
            }
            batch_index = 0;
            state = ExecState::DRAIN;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // DRAIN: process batch segments — write steps to RMT ring
        // ══════════════════════════════════════════════════════════════════
        case ExecState::DRAIN: {
            const int64_t drain_start = esp_timer_get_time();

            while (batch_index < batch_count) {
                // ── Pre-check: yield if time budget will be exceeded ───────────
                // This prevents accumulating too much CPU time before yielding.
                if ((esp_timer_get_time() - drain_start) >= EXEC_TIME_BUDGET_US) {
                    kickStartActiveAxes();
                    taskYIELD();
                    // Restart from FETCH to get fresh batch and reset timer.
                    state = ExecState::FETCH;
                    goto exit_drain;
                }

                planned_segment_t& seg = batch[batch_index];

                // ── Flush sentinel ────────────────────────────────────────
                if (seg.is_flush) {
                    state = ExecState::FLUSH;
                    goto exit_drain;  // break out of DRAIN, handle in FLUSH
                }

                // ── Update active axis list ───────────────────────────────
                active_axis_count = seg.axis_count < MULTI_AXIS_MAX_AXES
                    ? seg.axis_count : MULTI_AXIS_MAX_AXES;
                for (uint8_t a = 0; a < active_axis_count; ++a) {
                    active_axis_ids[a] = seg.axis_ids[a];
                }

                // ── Endstop check (per-segment, real-time) ───────────────
                bool endstop_hit = false;
                for (uint8_t a = 0; a < seg.axis_count && !endstop_hit; ++a) {
                    const uint8_t eid = seg.axis_ids[a];
                    if (eid >= self->n_motors_ ||
                        self->queues_[eid] == nullptr) continue;
                    if (self->queues_[eid]->driver().isEndstopActive()) {
                        // Drain remaining batch, e-stop, notify host.
                        self->queues_[eid]->driver().emergencyStop();
                        self->notifySegmentExecuted(seg.motion_sequence);
                        ESP_LOGW(TAG, "endstop on axis %u at seq=%u",
                                 eid, seg.motion_sequence);
                        endstop_hit = true;
                    }
                }
                if (endstop_hit) {
                    state = ExecState::RECOVERY;
                    goto exit_drain;
                }

                // ── Lateral endstop gate (read once per segment) ──────────
                const uint8_t lateral_state = self->readLateralEndstopState();
                const bool lateral_blocked =
                    lateral_state !=
                    static_cast<uint8_t>(LateralEndstopState::PRESENT_OPEN);

                uint8_t guarded_axis_ids[MULTI_AXIS_MAX_AXES] = {};
                uint8_t guarded_axis_count = 0;
                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    if (axis_id < self->n_motors_ && self->queues_[axis_id] != nullptr) {
                        self->queues_[axis_id]->setMultiExecActive(true);
                        if (guarded_axis_count < MULTI_AXIS_MAX_AXES) {
                            guarded_axis_ids[guarded_axis_count++] = axis_id;
                        }
                    }
                }

                auto clearMultiExecFlags = [&]() {
                    for (uint8_t i = 0; i < guarded_axis_count; ++i) {
                        const uint8_t axis_id = guarded_axis_ids[i];
                        if (axis_id < self->n_motors_ && self->queues_[axis_id] != nullptr) {
                            self->queues_[axis_id]->setMultiExecActive(false);
                        }
                    }
                };

                // ── Write steps to ring buffer (no RMT start) ─────────────
                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    if (axis_id >= self->n_motors_ ||
                        self->queues_[axis_id] == nullptr) continue;
                    if (seg.axes[a].step_count == 0) continue;
                    if (axis_id == 1 && lateral_blocked) {
                        ESP_LOGD(TAG, "axis1 blocked, skip %u steps",
                                 seg.axes[a].step_count);
                        continue;
                    }

                    StepperQueue* axis_queue = self->queues_[axis_id];
                    esp_err_t err = axis_queue->executeConstantRateBlock(
                        seg.axes[a].direction,
                        seg.axes[a].step_count,
                        seg.duration_us);

                    if (err == ESP_ERR_INVALID_STATE) {
                        clearMultiExecFlags();
                        ESP_LOGW(TAG, "axis %u endstop mid-seg seq=%u",
                                 axis_id, seg.motion_sequence);
                        axis_queue->driver().emergencyStop();
                        self->notifySegmentExecuted(seg.motion_sequence);
                        state = ExecState::RECOVERY;
                        goto exit_drain;
                    } else if (err != ESP_OK) {
                        ESP_LOGW(TAG, "axis %u seg %u: %s",
                                 axis_id, seg.motion_sequence,
                                 esp_err_to_name(err));
                    }
                }
                clearMultiExecFlags();

                // ── Schedule deferred notification ────────────────────────
                if ((defer_tail - defer_head) < DEFER_DEPTH) {
                    const int idx = defer_tail & (DEFER_DEPTH - 1);
                    defer_fire_us[idx] = seg.scheduled_time_us
                                         + static_cast<int64_t>(seg.duration_us);
                    defer_seqs[idx]    = seg.motion_sequence;
                    ++defer_tail;
                } else {
                    // Ring full: evict the oldest (earliest scheduled) entry,
                    // notify it now (it is already overdue), then enqueue the
                    // current segment normally. This preserves ordering and
                    // avoids signalling completion before steps reach the ring.
                    const int evict_idx = defer_head & (DEFER_DEPTH - 1);
                    const uint32_t evicted_seq = defer_seqs[evict_idx];
                    self->notifySegmentExecuted(
                        static_cast<uint16_t>(evicted_seq));
                    ++defer_head;
                    // Enqueue current segment.
                    const int idx = defer_tail & (DEFER_DEPTH - 1);
                    defer_fire_us[idx] = seg.scheduled_time_us
                                         + static_cast<int64_t>(seg.duration_us);
                    defer_seqs[idx]    = seg.motion_sequence;
                    ++defer_tail;
                    ESP_LOGW(TAG, "defer ring full: evicted seq=%u to make room for seq=%u",
                             (unsigned)evicted_seq,
                             (unsigned)seg.motion_sequence);
                }

                // Restart RMT immediately if it stopped mid-batch due to ring drain.
                // Do not wait for ExecState::RUN — the ring may fill with unconsumed
                // steps causing pushBlock() to deadlock on ulTaskNotifyTake.
                for (uint8_t a = 0; a < seg.axis_count; ++a) {
                    const uint8_t axis_id = seg.axis_ids[a];
                    if (axis_id >= self->n_motors_ ||
                        self->queues_[axis_id] == nullptr) continue;
                    if (!self->queues_[axis_id]->driver().isStreaming()) {
                        self->queues_[axis_id]->kickStart();
                    }
                }

                ++batch_index;

                // ── Time budget check (watchdog safety) ───────────────────
                if ((esp_timer_get_time() - drain_start) >= EXEC_TIME_BUDGET_US) {
                    // Budget exhausted — transition to RUN to kickStart,
                    // then yield before processing remaining segments.
                    kickStartActiveAxes();
                    taskYIELD();
                    // Continue draining after yield (reset budget).
                    break;  // will re-enter DRAIN on next iteration
                }
            }

            // All segments in batch processed — transition to RUN.
            if (batch_index >= batch_count) {
                state = ExecState::RUN;
            }
            // else: budget break, stay in DRAIN for remaining segments.
            break;

        exit_drain:
            break;  // state already set by the goto target
        }

        // ══════════════════════════════════════════════════════════════════
        // RUN: kickStart RMT on all active axes, return to IDLE
        // ══════════════════════════════════════════════════════════════════
        case ExecState::RUN: {
            kickStartActiveAxes();
            fireDeferred();
            state = ExecState::IDLE;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // FLUSH: reset pipeline state, notify host
        // ══════════════════════════════════════════════════════════════════
        case ExecState::FLUSH: {
            // The flush sentinel is at batch[batch_index].
            const planned_segment_t& flush_seg = batch[batch_index];

            // Reset deferred notification ring.
            defer_head = defer_tail = 0;

            // Notify host with flush sequence.
            self->notifySegmentExecuted(flush_seg.flush_sequence);

            ESP_LOGI(TAG, "executor flush at seq=%u",
                     (unsigned)flush_seg.flush_sequence);

            // Clear batch and return to idle.
            batch_count = 0;
            batch_index = 0;
            state = ExecState::IDLE;
            break;
        }

        // ══════════════════════════════════════════════════════════════════
        // RECOVERY: handle endstop / error, drain remaining, return to IDLE
        // ══════════════════════════════════════════════════════════════════
        case ExecState::RECOVERY: {
            // Drain any remaining segments in the planner's output queue
            // (bounded drain to avoid spending too long here).
            planned_segment_t discard;
            uint32_t drained = 0;
            while (drained < SEGMENT_QUEUE_DEPTH &&
                   xQueueReceive(seg_queue, &discard, 0) == pdTRUE) {
                ++drained;
            }

            // Reset deferred notifications.
            defer_head = defer_tail = 0;

            ESP_LOGW(TAG, "recovery: drained %lu remaining segments",
                     (unsigned long)drained);

            batch_count = 0;
            batch_index = 0;
            state = ExecState::IDLE;
            break;
        }

        } // switch(state)

        // ── Watchdog safety: yield if idle, sleep if very idle ───────────────
        // If we fetched zero segments in FETCH, sleep to let IDLE1 run.
        // Otherwise, yield to respect other tasks without 10ms stalls.
        if (state == ExecState::IDLE && batch_count == 0) {
            vTaskDelay(1);  // Very idle — sleep and let watchdog reset
        } else {
            taskYIELD();    // Still have work — yield but stay ready
        }
    } // for(;;)
}
```

### 3.5 `src/rpi/transport/streamer.py`
```python
from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass
import json
import time
from typing import Any, Iterator

from transport.messages import (
    MULTI_AXIS_SEGMENT_BLOCK_SIZE,
    MultiAxisSegment,
    MultiAxisSegmentBlockPayload,
    SpiMessageResult,
    sequence_is_greater,
    sequence_is_less_equal,
)
from motion import AxisMotionConfig, MultiAxisSegmentGenerator, RampConfig
from transport.spi_transport import Esp32SpiTransport

logger = logging.getLogger(__name__)


@dataclass(slots=True)
class StreamAxisConfig:
    axis_id: int
    ramp: RampConfig
    minimum_free_blocks: int = 1
    prefill_blocks: int | None = None
    low_watermark_blocks: int | None = None
    max_queued_blocks: int | None = None
    ring_send_threshold: int = 1


class MultiAxisRampStreamer:
    """Minimal deterministic SPI motion streamer.

    The streamer is the host-side source of truth for motion segments.
    It sends pre-computed multi-axis segment blocks over SPI, tracks in-flight
    motion, and uses MCU status feedback to keep the ESP32 queue and ring filled
    without overflowing them.

    Multiple segments are packed per SPI frame (up to MULTI_AXIS_SEGMENT_BLOCK_SIZE)
    to ensure the firmware drain loop has deep look-ahead before starting the RMT.
    """

    TARGET_BUFFER_TIME_S = 0.10
    MIN_BUFFER_TIME_S = 0.06
    MAX_BUFFER_TIME_S = 0.12
    MIN_SEGMENT_TIME_S = 0.002
    MAX_SEGMENT_TIME_S = 0.005
    POLL_SLEEP_S = 0.0005
    MAX_INFLIGHT_SEGMENTS = 24

    # Planner→executor segment queue depth on the ESP32 (matches SEGMENT_QUEUE_DEPTH in firmware).
    SEGMENT_QUEUE_DEPTH = 128
    # Legacy constant kept for reference (= EXEC_BATCH_LIMIT * 2).
    # The active gate is now required_lookahead() which is speed-dependent.
    PLANNER_QUEUE_SEND_THRESHOLD = 32

    # ESP32 step ring capacity in firmware: one step consumes one ring entry.
    STEP_RING_CAPACITY = 4096
    RING_BUFFER_HEADROOM = 0.8

    @staticmethod
    def required_lookahead(steps_per_segment: int) -> int:
        """Speed-dependent minimum segment lookahead depth in the ESP32 planner queue.

        At low speed each segment contains very few steps, so the ring drains
        faster relative to the inter-segment host→ESP32 pipeline latency (~3–5 ms).
        A deeper buffer prevents ring underruns and motor stutter.

        Thresholds match firmware EXEC_BATCH_LIMIT tiers:
          < 10  steps → 48 segments (low speed,  ~50 RPM)
          < 50  steps → 32 segments (mid speed)
          >= 50 steps → 16 segments (high speed, > ~200 RPM)
        """
        if steps_per_segment < 10:
            return 48
        elif steps_per_segment < 50:
            return 32
        else:
            return 16

    def __init__(
        self,
        transport: Esp32SpiTransport,
        axis_streams: list[StreamAxisConfig],
        *,
        segment_duration_s: float = 0.004,
        target_buffer_time_s: float = TARGET_BUFFER_TIME_S,
        poll_interval_s: float = 0.001,
        print_every: int = 1,
        log_each_send: bool = False,
        send_log_path: str | None = None,
    ):
        self._initialize_streamer_state(
            transport=transport,
            axis_configs=[AxisMotionConfig(axis_id=s.axis_id, ramp=s.ramp) for s in axis_streams],
            axis_ids=[s.axis_id for s in axis_streams],
            segment_duration_s=segment_duration_s,
            target_buffer_time_s=target_buffer_time_s,
            poll_interval_s=poll_interval_s,
            print_every=print_every,
            log_each_send=log_each_send,
            send_log_path=send_log_path,
            explicit_target_hz=None,
        )

    @classmethod
    def from_axis_ids(
        cls,
        transport: Esp32SpiTransport,
        axis_ids: list[int],
        *,
        target_hz: float,
        segment_duration_s: float = 0.004,
        poll_interval_s: float = 0.001,
        print_every: int = 1,
        target_buffer_time_s: float = 0.150,
    ) -> "MultiAxisRampStreamer":
        """Build a streamer from explicit axis IDs and a known target frequency.

        Use this constructor when the move generator is external (e.g. `WoundMove`)
        and no reliable `RampConfig` objects are available.

        Differences vs `__init__`:
          - `__init__`: derives `target_hz` from `RampConfig.target_hz`.
          - `from_axis_ids`: receives `target_hz` explicitly and avoids synthetic ramps.
        """
        if not axis_ids:
            raise ValueError("axis_ids must not be empty")
        if target_hz <= 0.0:
            raise ValueError("target_hz must be positive")

        streamer = cls.__new__(cls)
        streamer._initialize_streamer_state(
            transport=transport,
            axis_configs=[],
            axis_ids=axis_ids,
            segment_duration_s=segment_duration_s,
            target_buffer_time_s=target_buffer_time_s,
            poll_interval_s=poll_interval_s,
            print_every=print_every,
            log_each_send=False,
            send_log_path=None,
            explicit_target_hz=target_hz,
        )
        return streamer

    def _initialize_streamer_state(
        self,
        *,
        transport: Esp32SpiTransport,
        axis_configs: list[AxisMotionConfig],
        axis_ids: list[int],
        segment_duration_s: float,
        target_buffer_time_s: float,
        poll_interval_s: float,
        print_every: int,
        log_each_send: bool,
        send_log_path: str | None,
        explicit_target_hz: float | None,
    ) -> None:
        self._transport = transport
        self._poll_interval_s = poll_interval_s
        self._print_every = max(print_every, 1)
        self._log_each_send = log_each_send
        self._send_log_path = send_log_path
        self._send_events: list[dict] = []
        self._stop_requested = False
        self._flush_sequence_requested: int | None = None
        self._endstop_triggered = False
        self._endstop_armed_axes: set[int] = set()

        self._axis_configs = axis_configs
        self._axis_ids = list(axis_ids)
        self._segment_duration_s = max(self.MIN_SEGMENT_TIME_S, min(self.MAX_SEGMENT_TIME_S, segment_duration_s))
        if explicit_target_hz is None:
            max_hz = 0.0
            if self._axis_configs:
                max_hz = max(config.ramp.target_hz for config in self._axis_configs)
            self._target_buffer_time_s = self._safe_buffer_time_s(target_buffer_time_s, max_hz)
        else:
            self._target_buffer_time_s = self._safe_buffer_time_s(target_buffer_time_s, explicit_target_hz)
        self._min_buffer_time_s = min(self.MIN_BUFFER_TIME_S, self._target_buffer_time_s * 0.5)

        self._inflight: deque[tuple[MultiAxisSegment, int]] = deque()
        self._buffered_time_s = 0.0
        self._last_sent_motion_seq = -1
        self._last_sent_transport_seq = -1
        self._planner_under_pressure = False  # True while planner_queue_free < threshold
        self._buffered_segments = 0           # SEGMENT_QUEUE_DEPTH - planner_queue_free
        self._current_steps_per_segment = 0  # updated per-segment; drives required_lookahead()
        self._initial_steps_per_segment = 0  # fixed startup estimate used by prefill
        if explicit_target_hz is not None:
            self._initial_steps_per_segment = max(1, int(round(explicit_target_hz * self._segment_duration_s)))
            self._current_steps_per_segment = self._initial_steps_per_segment
        elif self._axis_configs:
            max_hz = max(
                (config.ramp.target_hz for config in self._axis_configs),
                default=0.0,
            )
            if max_hz > 0.0:
                self._initial_steps_per_segment = max(
                    1,
                    int(round(max_hz * self._segment_duration_s)),
                )
                self._current_steps_per_segment = self._initial_steps_per_segment
        self._prefilling = False              # suppresses pressure gate during initial prefill
        # Premature-completion detection
        self._last_confirmed_sequence: int = -1
        self._premature_notify_count: int = 0
        self._premature_notify_window_start: float = 0.0
        self._last_sequence_advance_time: float = time.time()
        self._last_sequence_advance_value: int = -1
        self._stall_timeout_s: float = 5.0  # stall if no progress for 5s

        self._sync_with_firmware_status()
        start_sequence = (
            (self._last_confirmed_sequence + 1) & 0xFFFF
            if self._last_confirmed_sequence >= 0
            else 0
        )
        if self._axis_configs:
            self._generator = iter(
                MultiAxisSegmentGenerator(
                    self._axis_configs,
                    segment_duration_s=self._segment_duration_s,
                    start_sequence=start_sequence,
                )
            )
        else:
            self._generator = iter(())
        self._generator_finished = False

    # -- Helpers ---------------------------------------------------------------

    @property
    def buffered_segments(self) -> int:
        """Number of segments currently buffered in the planner→executor queue.

        Computed from the last received planner_queue_free field:
            buffered = SEGMENT_QUEUE_DEPTH - planner_queue_free

        This mirrors Klipper's "move queue available" check: when buffered_segments
        approaches SEGMENT_QUEUE_DEPTH the host should stop requesting more motion.
        Value is 0 when no status has been received yet.
        """
        return self._buffered_segments

    def _planner_queue_free(self, status) -> int:
        """Return planner_queue_free from status, defaulting to full if absent."""
        return int(getattr(status, "planner_queue_free", self.SEGMENT_QUEUE_DEPTH))

    def _check_planner_pressure(self, status) -> bool:
        """Return True (blocked) when the ESP32 planner buffer already has enough lookahead.

        Uses Klipper's move-queue model: send if buffered < needed, not if free > threshold.
        During initial prefill (_prefilling=True) the gate is bypassed entirely so
        the host can fill up to the speed-appropriate prefill target without interference.

        Logs edge transitions:
          - 'planner pressure' when planner_queue_free drops below 16
          - 'planner recovered' when planner_queue_free recovers above 64
        """
        pqf = self._planner_queue_free(status)
        self._buffered_segments = self.SEGMENT_QUEUE_DEPTH - pqf

        if pqf < 16 and not self._planner_under_pressure:
            self._planner_under_pressure = True
            logger.debug("planner pressure: planner_queue_free=%s (< 16)", pqf)
        elif pqf > 64 and self._planner_under_pressure:
            self._planner_under_pressure = False
            logger.debug("planner recovered: planner_queue_free=%s (> 64)", pqf)

        # During prefill we bypass the pressure gate so the host can seed a deep buffer.
        if self._prefilling:
            return False

        # Klipper model: block if the buffer already holds the required lookahead depth.
        # This inverts the old "send if free slots >= threshold" gate: we now gate on
        # buffered depth rather than remaining free space, which is speed-aware.
        needed = self.required_lookahead(self._current_steps_per_segment)
        return self._buffered_segments >= needed

    def _max_segments_per_cycle(self) -> int:
        """Speed-dependent send cap per polling cycle.

        At low speed (few steps/segment) the defer ring on the ESP32 cannot
        overflow (each segment contributes <10 ring entries) so a higher cap
        is safe and necessary to keep the ring fed between underruns.
        At high speed a lower cap prevents burst-after-throttle overflow.

          < 10  steps/segment → 16 segments/cycle (low speed, 50 RPM)
          < 50  steps/segment →  8 segments/cycle (mid speed)
          >= 50 steps/segment →  4 segments/cycle (high speed, > 200 RPM)
        """
        if self._current_steps_per_segment < 10:
            return 16
        elif self._current_steps_per_segment < 50:
            return 8
        else:
            return 4

    def _max_inflight_segments(self) -> int:
        """Speed-dependent in-flight segment cap.

        At low speed each segment executes slowly so more can be in-flight
        simultaneously without risking the host advancing too far ahead
        of the motor's actual position.
        """
        if self._current_steps_per_segment < 10:
            return 96
        elif self._current_steps_per_segment < 50:
            return 48
        else:
            return 24

    def _timestamp(self) -> str:
        now = time.time()
        seconds = int(now)
        milliseconds = int((now - seconds) * 1000)
        return time.strftime(f"%H:%M:%S.{milliseconds:03d}", time.localtime(now))

    def _safe_buffer_time_s(self, requested_time_s: float, max_hz: float) -> float:
        if max_hz <= 0.0:
            return max(self.MIN_BUFFER_TIME_S, min(self.MAX_BUFFER_TIME_S, requested_time_s))
        safe_time_s = (self.STEP_RING_CAPACITY * self.RING_BUFFER_HEADROOM) / max_hz
        # Use the minimum of what was requested and what the ring can hold.
        # Never go below MIN_BUFFER_TIME_S (needed for SPI pipeline latency).
        return max(self.MIN_BUFFER_TIME_S, min(requested_time_s, safe_time_s))

    def set_generator(self, generator: Iterator[MultiAxisSegment]) -> None:
        """Override the segment generator for this streamer.

        Call before stream_all() when the segments are produced externally
        (e.g. by a WoundMove or RampMove).
        """
        self._generator = generator
        self._generator_finished = False

    def _sync_with_firmware_status(self) -> None:
        """Synchronize stream state with the ESP32's last executed sequence."""
        try:
            status = self._transport.get_status()
        except Exception:
            return

        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return

        self._last_confirmed_sequence = received_sequence
        self._last_sequence_advance_value = received_sequence
        self._last_sequence_advance_time = time.time()

    def _enable_axes(self) -> None:
        for axis_id in self._axis_ids:
            sequence, status = self._transport.set_axis_enabled_request(axis_id, True)
            status = self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(f"enable axis {axis_id} failed with result=0x{status.last_result:02X}")

    def _disable_axes(self) -> None:
        for axis_id in self._axis_ids:
            sequence, status = self._transport.set_axis_enabled_request(axis_id, False)
            status = self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
            if status.last_result != int(SpiMessageResult.OK):
                raise RuntimeError(f"disable axis {axis_id} failed with result=0x{status.last_result:02X}")

    def _queue_full(self, status) -> bool:
        for axis_id in self._axis_ids:
            if axis_id < len(status.queue_free_slots) and status.queue_free_slots[axis_id] == 0:
                return True
            if hasattr(status, "ring_free_slots") and axis_id < len(status.ring_free_slots) and status.ring_free_slots[axis_id] == 0:
                return True
        return False

    def _remove_confirmed_segments(self, status) -> None:
        last_executed = int(getattr(status, "last_executed_sequence", -1))
        while self._inflight:
            segment, _transport_seq = self._inflight[0]
            if sequence_is_less_equal(segment.sequence, last_executed):
                self._buffered_time_s -= segment.duration_us / 1_000_000.0
                self._buffered_time_s = max(0.0, self._buffered_time_s)
                self._inflight.popleft()
            else:
                break

    def _check_premature_completion(self, status) -> None:
        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return

        # True premature: ESP32 reports completion of a seq we never sent.
        if self._last_sent_motion_seq >= 0 and sequence_is_greater(received_sequence, self._last_sent_motion_seq):
            now = time.time()
            if now - self._premature_notify_window_start > 1.0:
                self._premature_notify_count = 0
                self._premature_notify_window_start = now
            self._premature_notify_count += 1
            logger.warning(
                "premature completion: got seq=%s but last sent=%s — ESP32 reported completion before host sent this segment (count=%s)",
                received_sequence,
                self._last_sent_motion_seq,
                self._premature_notify_count,
            )

        # Advance confirmed pointer only when sequence strictly increases.
        if self._last_confirmed_sequence < 0 or sequence_is_greater(
            received_sequence, self._last_confirmed_sequence
        ):
            self._last_confirmed_sequence = received_sequence

    def _check_stall(self, status) -> bool:
        """Return True and request stop if last_executed_sequence has not
        advanced for _stall_timeout_s while segments are in flight.

        A stall means the RMT is dead and pushBlock() is likely deadlocked.
        Requesting a stop+flush allows the host to recover gracefully.
        """
        received_sequence = int(getattr(status, "last_executed_sequence", -1))
        if received_sequence == 0xFFFF or received_sequence < 0:
            return False

        # Do not arm stall detection until the first executed segment
        # has been confirmed by firmware.
        if self._last_confirmed_sequence < 0:
            self._last_sequence_advance_time = time.time()
            return False

        if not self._inflight:
            # No in-flight segments — not a stall, just idle.
            self._last_sequence_advance_time = time.time()
            return False

        if self._last_sequence_advance_value < 0 or sequence_is_greater(
            received_sequence, self._last_sequence_advance_value
        ):
            self._last_sequence_advance_value = received_sequence
            self._last_sequence_advance_time = time.time()
            return False

        elapsed = time.time() - self._last_sequence_advance_time
        if elapsed > self._stall_timeout_s:
            logger.warning(
                "motor stall detected: last_executed_sequence=%s unchanged for %.1fs with %s segments in flight — requesting stop and flush",
                received_sequence,
                elapsed,
                len(self._inflight),
            )
            self.request_stop()
            self.request_flush(self._last_sent_motion_seq)
            return True
        return False

    def _check_endstop(self, status) -> bool:
        """Return True if an endstop was triggered on any armed axis.

        Reads endstop_armed_mask from the status frame. Sets
        _endstop_triggered and requests a stop + flush when triggered.
        """
        armed_mask = int(getattr(status, "endstop_armed_mask", 0))
        lateral_state = int(getattr(status, "lateral_endstop_state", 0xFF))
        # lateral_endstop_state values (from firmware LateralEndstopState):
        #   0x00 = PRESENT_OPEN, 0x01 = PRESENT_CLOSED, 0xFF = ABSENT
        PRESENT_CLOSED = 0x01
        if lateral_state == PRESENT_CLOSED and armed_mask != 0:
            if not self._endstop_triggered:
                self._endstop_triggered = True
                flush_seq = self._last_sent_motion_seq
                self.request_stop()
                self.request_flush(flush_seq)
            return True
        return False

    def _record_send_event(self, segment: MultiAxisSegment, transport_seq: int, status) -> None:
        event = {
            "timestamp": time.time(),
            "timestamp_str": self._timestamp(),
            "transport_sequence": transport_seq,
            "segment_sequence": segment.sequence,
            "duration_us": segment.duration_us,
            "axis_count": len(segment.steps),
            "total_steps": sum(segment.steps),
            "queue_free": list(status.queue_free_slots),
            "ring_free": list(status.ring_free_slots),
            "last_result": int(status.last_result),
            "enabled_mask": int(status.enabled_mask),
            "running_mask": int(status.running_mask),
        }
        self._send_events.append(event)
        if self._log_each_send:
            logger.debug(
                "[%s] send tx_seq=%s motion_seq=%s duration_us=%s total_steps=%s result=0x%02X",
                event["timestamp_str"],
                transport_seq,
                segment.sequence,
                segment.duration_us,
                event["total_steps"],
                event["last_result"],
            )

    def _write_send_log(self) -> None:
        if self._send_log_path is None:
            return
        with open(self._send_log_path, "w", encoding="utf-8") as handle:
            json.dump(self._send_events, handle, indent=2)

    # -- Endstop control -------------------------------------------------------

    def arm_endstop(self, axis_id: int) -> None:
        """Send ENABLE_ENDSTOP arm command to firmware and track locally.

        Call before starting a move that should stop on endstop contact.
        """
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=True)
        self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
        self._endstop_armed_axes.add(axis_id)

    def disarm_endstop(self, axis_id: int) -> None:
        """Send ENABLE_ENDSTOP disarm command to firmware.

        Call before a clearance move that must pass through the endstop.
        """
        sequence, _ = self._transport.enable_endstop_request(axis_id, arm=False)
        self._transport.wait_for_request_result(sequence, poll_interval_s=self._poll_interval_s)
        self._endstop_armed_axes.discard(axis_id)

    @property
    def endstop_triggered(self) -> bool:
        return self._endstop_triggered

    # -- Stop / flush ----------------------------------------------------------

    def request_stop(self) -> None:
        self._stop_requested = True

    def has_stop_been_requested(self) -> bool:
        return self._stop_requested

    def request_flush(self, sequence: int) -> None:
        self._flush_sequence_requested = sequence

    def flush_until(self, sequence: int):
        status = self._transport.flush_until(sequence)
        self._inflight.clear()
        self._buffered_time_s = 0.0
        return status

    # -- Core streaming primitives ---------------------------------------------

    def _collect_and_send_batch(self, status) -> tuple[int, Any] | None:
        """Collect up to MULTI_AXIS_SEGMENT_BLOCK_SIZE segments and send one frame.

        Returns (segments_sent, last_status) on success, (0, status) on
        QUEUE_FULL, or None when nothing can be batched (buffer target
        reached, inflight limit reached, or generator already exhausted).

        Packing multiple segments per frame is critical for ring pre-fill:
        the firmware drain loop processes all queued frames before starting
        the RMT, so more segments per frame = deeper ring buffer at startup.
        """
        if self._generator_finished:
            return None

        batch: list[MultiAxisSegment] = []
        while (
            len(batch) < MULTI_AXIS_SEGMENT_BLOCK_SIZE
            and self._buffered_time_s < self._target_buffer_time_s
            and len(self._inflight) < self._max_inflight_segments()
            and not self._queue_full(status)
            and not self._check_planner_pressure(status)
        ):
            try:
                segment = next(self._generator)
            except StopIteration:
                self._generator_finished = True
                break

            if self._last_sent_motion_seq >= 0 and not sequence_is_greater(
                segment.sequence, self._last_sent_motion_seq
            ):
                raise RuntimeError(
                    f"motion sequence not strictly increasing: "
                    f"got {segment.sequence}, last was {self._last_sent_motion_seq}"
                )
            batch.append(segment)
            # Keep speed estimate current so required_lookahead() uses fresh data.
            if segment.steps:
                self._current_steps_per_segment = sum(segment.steps)

        if not batch:
            return None

        payload = MultiAxisSegmentBlockPayload(
            axis_ids=self._axis_ids,
            block_seq=batch[0].sequence,
            segments=batch,
        )
        transport_seq, send_status = self._transport.send_multi_axis_segment_block_request(payload)

        if send_status.last_result == int(SpiMessageResult.OK):
            for seg in batch:
                self._inflight.append((seg, transport_seq))
                self._buffered_time_s += seg.duration_us / 1_000_000.0
                self._last_sent_motion_seq = seg.sequence
                self._record_send_event(seg, transport_seq, send_status)
            self._last_sent_transport_seq = transport_seq
            return len(batch), send_status
        elif send_status.last_result == int(SpiMessageResult.QUEUE_FULL):
            return 0, send_status
        else:
            raise RuntimeError(
                f"segment batch starting motion_seq={batch[0].sequence} "
                f"failed with result=0x{send_status.last_result:02X}"
            )

    def _prefill(self, status) -> tuple[int, Any]:
        """Pre-send segments to seed the ESP32 planner queue before RMT starts.

        The prefill target is computed from the startup speed estimate
        (_initial_steps_per_segment), not the live segment state, to avoid
        low-speed misclassification at startup.

        Speed tiers (steps_per_segment):
          - < 10: 64 segments (low speed, conservative fill)
          - >= 10: required_lookahead(initial_steps) (speed-appropriate fill)

        The _prefilling flag is set for the duration of this call so that
        _check_planner_pressure() does not prematurely gate sends before the
        target depth has been reached.

        Returns (total_segments_sent, last_status).
        """
        # Use initial startup speed estimate, not the live segment state.
        initial_steps = self._initial_steps_per_segment
        if initial_steps < 10:
            prefill_target = 64   # half of SEGMENT_QUEUE_DEPTH
        else:
            prefill_target = self.required_lookahead(initial_steps)

        total = 0
        last_status = status
        self._prefilling = True
        try:
            while total < prefill_target:
                result = self._collect_and_send_batch(last_status)
                if result is None:
                    break
                n, last_status = result
                total += n
                if n == 0:  # QUEUE_FULL — firmware can't accept more right now
                    break
        finally:
            self._prefilling = False
        return total, last_status

    def _should_sleep(self) -> float:
        """Return sleep duration in seconds based on buffer fullness.

        Returns 0.0 if the buffer needs immediate refill.
        """
        if self._buffered_time_s >= self._target_buffer_time_s:
            return self._segment_duration_s
        if self._buffered_time_s >= self._min_buffer_time_s:
            return self._segment_duration_s / 2.0
        return 0.0

    # -- Main streaming loop ---------------------------------------------------

    def stream_all(self) -> int:
        self._generator_finished = False
        status = self._transport.get_status()
        axes_enabled = False

        try:
            self._enable_axes()
            axes_enabled = True
            status = self._transport.get_status()

            total_segments, status = self._prefill(status)

            while True:
                if self._stop_requested:
                    if self._flush_sequence_requested is not None:
                        self.flush_until(self._flush_sequence_requested)
                    break

                status = self._transport.get_status()
                self._remove_confirmed_segments(status)
                self._check_premature_completion(status)
                if self._check_stall(status):
                    break

                if self._check_endstop(status):
                    break

                # Rate-limit sends to MAX_SEGMENTS_PER_CYCLE per polling iteration to
                # prevent burst-after-throttle that overflows the ESP32 defer ring.
                cycle_segments_sent = 0
                while not self._generator_finished:
                    if cycle_segments_sent >= self._max_segments_per_cycle():
                        break
                    result = self._collect_and_send_batch(status)
                    if result is None:
                        break
                    n, status = result
                    if n == 0:  # QUEUE_FULL
                        break
                    cycle_segments_sent += n
                    total_segments += n
                    if total_segments % self._print_every == 0:
                        logger.debug(
                            "segments=%s buffered=%.1fms inflight=%s queue_free=%s ring_free=%s underrun=%s",
                            total_segments,
                            self._buffered_time_s * 1000.0,
                            len(self._inflight),
                            status.queue_free_slots,
                            status.ring_free_slots,
                            status.underrun_count,
                        )

                if self._flush_sequence_requested is not None:
                    self.flush_until(self._flush_sequence_requested)
                    self._flush_sequence_requested = None

                if self._generator_finished and not self._inflight:
                    break

                sleep_s = self._should_sleep()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        finally:
            if axes_enabled:
                try:
                    self._disable_axes()
                except Exception as exc:
                    logger.error("failed to disable axes: %s", exc)

        self._write_send_log()
        return total_segments
```

### 3.6 `src/esp32/sdkconfig.esp32` (CPU 240 MHz)
```ini
#
# Automatically generated file. DO NOT EDIT.
# Espressif IoT Development Framework (ESP-IDF) 5.5.3 Project Configuration
#
# default:
CONFIG_SOC_CAPS_ECO_VER_MAX=301
# default:
CONFIG_SOC_ADC_SUPPORTED=y
# default:
CONFIG_SOC_DAC_SUPPORTED=y
# default:
CONFIG_SOC_UART_SUPPORTED=y
# default:
CONFIG_SOC_MCPWM_SUPPORTED=y
# default:
CONFIG_SOC_GPTIMER_SUPPORTED=y
# default:
CONFIG_SOC_SDMMC_HOST_SUPPORTED=y
# default:
CONFIG_SOC_BT_SUPPORTED=y
# default:
CONFIG_SOC_PCNT_SUPPORTED=y
# default:
CONFIG_SOC_PHY_SUPPORTED=y
# default:
CONFIG_SOC_WIFI_SUPPORTED=y
# default:
CONFIG_SOC_SDIO_SLAVE_SUPPORTED=y
# default:
CONFIG_SOC_TWAI_SUPPORTED=y
# default:
CONFIG_SOC_EFUSE_SUPPORTED=y
# default:
CONFIG_SOC_EMAC_SUPPORTED=y
# default:
CONFIG_SOC_ULP_SUPPORTED=y
# default:
CONFIG_SOC_CCOMP_TIMER_SUPPORTED=y
# default:
CONFIG_SOC_RTC_FAST_MEM_SUPPORTED=y
# default:
CONFIG_SOC_RTC_SLOW_MEM_SUPPORTED=y
# default:
CONFIG_SOC_RTC_MEM_SUPPORTED=y
# default:
CONFIG_SOC_I2S_SUPPORTED=y
# default:
CONFIG_SOC_RMT_SUPPORTED=y
# default:
CONFIG_SOC_SDM_SUPPORTED=y
# default:
CONFIG_SOC_GPSPI_SUPPORTED=y
# default:
CONFIG_SOC_LEDC_SUPPORTED=y
# default:
CONFIG_SOC_I2C_SUPPORTED=y
# default:
CONFIG_SOC_SUPPORT_COEXISTENCE=y
# default:
CONFIG_SOC_AES_SUPPORTED=y
# default:
CONFIG_SOC_MPI_SUPPORTED=y
# default:
CONFIG_SOC_SHA_SUPPORTED=y
# default:
CONFIG_SOC_FLASH_ENC_SUPPORTED=y
# default:
CONFIG_SOC_SECURE_BOOT_SUPPORTED=y
# default:
CONFIG_SOC_TOUCH_SENSOR_SUPPORTED=y
# default:
CONFIG_SOC_BOD_SUPPORTED=y
# default:
CONFIG_SOC_ULP_FSM_SUPPORTED=y
# default:
CONFIG_SOC_CLK_TREE_SUPPORTED=y
# default:
CONFIG_SOC_MPU_SUPPORTED=y
# default:
CONFIG_SOC_WDT_SUPPORTED=y
# default:
CONFIG_SOC_SPI_FLASH_SUPPORTED=y
# default:
CONFIG_SOC_RNG_SUPPORTED=y
# default:
CONFIG_SOC_LIGHT_SLEEP_SUPPORTED=y
# default:
CONFIG_SOC_DEEP_SLEEP_SUPPORTED=y
# default:
CONFIG_SOC_LP_PERIPH_SHARE_INTERRUPT=y
# default:
CONFIG_SOC_PM_SUPPORTED=y
# default:
CONFIG_SOC_DPORT_WORKAROUND_DIS_INTERRUPT_LVL=5
# default:
CONFIG_SOC_XTAL_SUPPORT_26M=y
# default:
CONFIG_SOC_XTAL_SUPPORT_40M=y
# default:
CONFIG_SOC_XTAL_SUPPORT_AUTO_DETECT=y
# default:
CONFIG_SOC_ADC_RTC_CTRL_SUPPORTED=y
# default:
CONFIG_SOC_ADC_DIG_CTRL_SUPPORTED=y
# default:
CONFIG_SOC_ADC_DMA_SUPPORTED=y
# default:
CONFIG_SOC_ADC_PERIPH_NUM=2
# default:
CONFIG_SOC_ADC_MAX_CHANNEL_NUM=10
# default:
CONFIG_SOC_ADC_ATTEN_NUM=4
# default:
CONFIG_SOC_ADC_DIGI_CONTROLLER_NUM=2
# default:
CONFIG_SOC_ADC_PATT_LEN_MAX=16
# default:
CONFIG_SOC_ADC_DIGI_MIN_BITWIDTH=9
# default:
CONFIG_SOC_ADC_DIGI_MAX_BITWIDTH=12
# default:
CONFIG_SOC_ADC_DIGI_RESULT_BYTES=2
# default:
CONFIG_SOC_ADC_DIGI_DATA_BYTES_PER_CONV=4
# default:
CONFIG_SOC_ADC_DIGI_MONITOR_NUM=0
# default:
CONFIG_SOC_ADC_SAMPLE_FREQ_THRES_HIGH=2
# default:
CONFIG_SOC_ADC_SAMPLE_FREQ_THRES_LOW=20
# default:
CONFIG_SOC_ADC_RTC_MIN_BITWIDTH=9
# default:
CONFIG_SOC_ADC_RTC_MAX_BITWIDTH=12
# default:
CONFIG_SOC_ADC_SHARED_POWER=y
# default:
CONFIG_SOC_BROWNOUT_RESET_SUPPORTED=y
# default:
CONFIG_SOC_SHARED_IDCACHE_SUPPORTED=y
# default:
CONFIG_SOC_IDCACHE_PER_CORE=y
# default:
CONFIG_SOC_CPU_CORES_NUM=2
# default:
CONFIG_SOC_CPU_INTR_NUM=32
# default:
CONFIG_SOC_CPU_HAS_FPU=y
# default:
CONFIG_SOC_HP_CPU_HAS_MULTIPLE_CORES=y
# default:
CONFIG_SOC_CPU_BREAKPOINTS_NUM=2
# default:
CONFIG_SOC_CPU_WATCHPOINTS_NUM=2
# default:
CONFIG_SOC_CPU_WATCHPOINT_MAX_REGION_SIZE=0x40
# default:
CONFIG_SOC_DAC_CHAN_NUM=2
# default:
CONFIG_SOC_DAC_RESOLUTION=8
# default:
CONFIG_SOC_DAC_DMA_16BIT_ALIGN=y
# default:
CONFIG_SOC_GPIO_PORT=1
# default:
CONFIG_SOC_GPIO_PIN_COUNT=40
# default:
CONFIG_SOC_GPIO_VALID_GPIO_MASK=0xFFFFFFFFFF
# default:
CONFIG_SOC_GPIO_IN_RANGE_MAX=39
# default:
CONFIG_SOC_GPIO_OUT_RANGE_MAX=33
# default:
CONFIG_SOC_GPIO_VALID_DIGITAL_IO_PAD_MASK=0xEF0FEA
# default:
CONFIG_SOC_GPIO_CLOCKOUT_BY_IO_MUX=y
# default:
CONFIG_SOC_GPIO_CLOCKOUT_CHANNEL_NUM=3
# default:
CONFIG_SOC_I2C_NUM=2
# default:
CONFIG_SOC_HP_I2C_NUM=2
# default:
CONFIG_SOC_I2C_FIFO_LEN=32
# default:
CONFIG_SOC_I2C_CMD_REG_NUM=16
# default:
CONFIG_SOC_I2C_SUPPORT_SLAVE=y
# default:
CONFIG_SOC_I2C_SUPPORT_APB=y
# default:
CONFIG_SOC_I2C_SUPPORT_10BIT_ADDR=y
# default:
CONFIG_SOC_I2C_STOP_INDEPENDENT=y
# default:
CONFIG_SOC_I2S_NUM=2
# default:
CONFIG_SOC_I2S_HW_VERSION_1=y
# default:
CONFIG_SOC_I2S_SUPPORTS_APLL=y
# default:
CONFIG_SOC_I2S_SUPPORTS_PLL_F160M=y
# default:
CONFIG_SOC_I2S_SUPPORTS_PDM=y
# default:
CONFIG_SOC_I2S_SUPPORTS_PDM_TX=y
# default:
CONFIG_SOC_I2S_SUPPORTS_PCM2PDM=y
# default:
CONFIG_SOC_I2S_SUPPORTS_PDM_RX=y
# default:
CONFIG_SOC_I2S_SUPPORTS_PDM2PCM=y
# default:
CONFIG_SOC_I2S_PDM_MAX_TX_LINES=1
# default:
CONFIG_SOC_I2S_PDM_MAX_RX_LINES=1
# default:
CONFIG_SOC_I2S_SUPPORTS_ADC_DAC=y
# default:
CONFIG_SOC_I2S_SUPPORTS_ADC=y
# default:
CONFIG_SOC_I2S_SUPPORTS_DAC=y
# default:
CONFIG_SOC_I2S_SUPPORTS_LCD_CAMERA=y
# default:
CONFIG_SOC_I2S_MAX_DATA_WIDTH=24
# default:
CONFIG_SOC_I2S_TRANS_SIZE_ALIGN_WORD=y
# default:
CONFIG_SOC_I2S_LCD_I80_VARIANT=y
# default:
CONFIG_SOC_LCD_I80_SUPPORTED=y
# default:
CONFIG_SOC_LCD_I80_BUSES=2
# default:
CONFIG_SOC_LCD_I80_BUS_WIDTH=24
# default:
CONFIG_SOC_LEDC_HAS_TIMER_SPECIFIC_MUX=y
# default:
CONFIG_SOC_LEDC_SUPPORT_APB_CLOCK=y
# default:
CONFIG_SOC_LEDC_SUPPORT_REF_TICK=y
# default:
CONFIG_SOC_LEDC_SUPPORT_HS_MODE=y
# default:
CONFIG_SOC_LEDC_TIMER_NUM=4
# default:
CONFIG_SOC_LEDC_CHANNEL_NUM=8
# default:
CONFIG_SOC_LEDC_TIMER_BIT_WIDTH=20
# default:
CONFIG_SOC_MCPWM_GROUPS=2
# default:
CONFIG_SOC_MCPWM_TIMERS_PER_GROUP=3
# default:
CONFIG_SOC_MCPWM_OPERATORS_PER_GROUP=3
# default:
CONFIG_SOC_MCPWM_COMPARATORS_PER_OPERATOR=2
# default:
CONFIG_SOC_MCPWM_GENERATORS_PER_OPERATOR=2
# default:
CONFIG_SOC_MCPWM_TRIGGERS_PER_OPERATOR=2
# default:
CONFIG_SOC_MCPWM_GPIO_FAULTS_PER_GROUP=3
# default:
CONFIG_SOC_MCPWM_CAPTURE_TIMERS_PER_GROUP=y
# default:
CONFIG_SOC_MCPWM_CAPTURE_CHANNELS_PER_TIMER=3
# default:
CONFIG_SOC_MCPWM_GPIO_SYNCHROS_PER_GROUP=3
# default:
CONFIG_SOC_MMU_PERIPH_NUM=2
# default:
CONFIG_SOC_MMU_LINEAR_ADDRESS_REGION_NUM=3
# default:
CONFIG_SOC_MPU_MIN_REGION_SIZE=0x20000000
# default:
CONFIG_SOC_MPU_REGIONS_MAX_NUM=8
# default:
CONFIG_SOC_PCNT_GROUPS=1
# default:
CONFIG_SOC_PCNT_UNITS_PER_GROUP=8
# default:
CONFIG_SOC_PCNT_CHANNELS_PER_UNIT=2
# default:
CONFIG_SOC_PCNT_THRES_POINT_PER_UNIT=2
# default:
CONFIG_SOC_RMT_GROUPS=1
# default:
CONFIG_SOC_RMT_TX_CANDIDATES_PER_GROUP=8
# default:
CONFIG_SOC_RMT_RX_CANDIDATES_PER_GROUP=8
# default:
CONFIG_SOC_RMT_CHANNELS_PER_GROUP=8
# default:
CONFIG_SOC_RMT_MEM_WORDS_PER_CHANNEL=64
# default:
CONFIG_SOC_RMT_SUPPORT_REF_TICK=y
# default:
CONFIG_SOC_RMT_SUPPORT_APB=y
# default:
CONFIG_SOC_RMT_CHANNEL_CLK_INDEPENDENT=y
# default:
CONFIG_SOC_RTCIO_PIN_COUNT=18
# default:
CONFIG_SOC_RTCIO_INPUT_OUTPUT_SUPPORTED=y
# default:
CONFIG_SOC_RTCIO_HOLD_SUPPORTED=y
# default:
CONFIG_SOC_RTCIO_WAKE_SUPPORTED=y
# default:
CONFIG_SOC_SDM_GROUPS=1
# default:
CONFIG_SOC_SDM_CHANNELS_PER_GROUP=8
# default:
CONFIG_SOC_SDM_CLK_SUPPORT_APB=y
# default:
CONFIG_SOC_SPI_HD_BOTH_INOUT_SUPPORTED=y
# default:
CONFIG_SOC_SPI_AS_CS_SUPPORTED=y
# default:
CONFIG_SOC_SPI_PERIPH_NUM=3
# default:
CONFIG_SOC_SPI_DMA_CHAN_NUM=2
# default:
CONFIG_SOC_SPI_MAX_CS_NUM=3
# default:
CONFIG_SOC_SPI_SUPPORT_CLK_APB=y
# default:
CONFIG_SOC_SPI_MAXIMUM_BUFFER_SIZE=64
# default:
CONFIG_SOC_SPI_MAX_PRE_DIVIDER=8192
# default:
CONFIG_SOC_MEMSPI_SRC_FREQ_80M_SUPPORTED=y
# default:
CONFIG_SOC_MEMSPI_SRC_FREQ_40M_SUPPORTED=y
# default:
CONFIG_SOC_MEMSPI_SRC_FREQ_26M_SUPPORTED=y
# default:
CONFIG_SOC_MEMSPI_SRC_FREQ_20M_SUPPORTED=y
# default:
CONFIG_SOC_TIMER_GROUPS=2
# default:
CONFIG_SOC_TIMER_GROUP_TIMERS_PER_GROUP=2
# default:
CONFIG_SOC_TIMER_GROUP_COUNTER_BIT_WIDTH=64
# default:
CONFIG_SOC_TIMER_GROUP_TOTAL_TIMERS=4
# default:
CONFIG_SOC_TIMER_GROUP_SUPPORT_APB=y
# default:
CONFIG_SOC_LP_TIMER_BIT_WIDTH_LO=32
# default:
CONFIG_SOC_LP_TIMER_BIT_WIDTH_HI=16
# default:
CONFIG_SOC_TOUCH_SENSOR_VERSION=1
# default:
CONFIG_SOC_TOUCH_SENSOR_NUM=10
# default:
CONFIG_SOC_TOUCH_MIN_CHAN_ID=0
# default:
CONFIG_SOC_TOUCH_MAX_CHAN_ID=9
# default:
CONFIG_SOC_TOUCH_SUPPORT_SLEEP_WAKEUP=y
# default:
CONFIG_SOC_TOUCH_SAMPLE_CFG_NUM=1
# default:
CONFIG_SOC_TWAI_CONTROLLER_NUM=1
# default:
CONFIG_SOC_TWAI_MASK_FILTER_NUM=1
# default:
CONFIG_SOC_TWAI_BRP_MIN=2
# default:
CONFIG_SOC_TWAI_CLK_SUPPORT_APB=y
# default:
CONFIG_SOC_TWAI_SUPPORT_MULTI_ADDRESS_LAYOUT=y
# default:
CONFIG_SOC_UART_NUM=3
# default:
CONFIG_SOC_UART_HP_NUM=3
# default:
CONFIG_SOC_UART_SUPPORT_APB_CLK=y
# default:
CONFIG_SOC_UART_SUPPORT_REF_TICK=y
# default:
CONFIG_SOC_UART_FIFO_LEN=128
# default:
CONFIG_SOC_UART_BITRATE_MAX=5000000
# default:
CONFIG_SOC_UART_WAKEUP_SUPPORT_ACTIVE_THRESH_MODE=y
# default:
CONFIG_SOC_SPIRAM_SUPPORTED=y
# default:
CONFIG_SOC_SPI_MEM_SUPPORT_CONFIG_GPIO_BY_EFUSE=y
# default:
CONFIG_SOC_SHA_SUPPORT_PARALLEL_ENG=y
# default:
CONFIG_SOC_SHA_ENDIANNESS_BE=y
# default:
CONFIG_SOC_SHA_SUPPORT_SHA1=y
# default:
CONFIG_SOC_SHA_SUPPORT_SHA256=y
# default:
CONFIG_SOC_SHA_SUPPORT_SHA384=y
# default:
CONFIG_SOC_SHA_SUPPORT_SHA512=y
# default:
CONFIG_SOC_MPI_MEM_BLOCKS_NUM=4
# default:
CONFIG_SOC_MPI_OPERATIONS_NUM=1
# default:
CONFIG_SOC_RSA_MAX_BIT_LEN=4096
# default:
CONFIG_SOC_AES_SUPPORT_AES_128=y
# default:
CONFIG_SOC_AES_SUPPORT_AES_192=y
# default:
CONFIG_SOC_AES_SUPPORT_AES_256=y
# default:
CONFIG_SOC_SECURE_BOOT_V1=y
# default:
CONFIG_SOC_EFUSE_SECURE_BOOT_KEY_DIGESTS=1
# default:
CONFIG_SOC_FLASH_ENCRYPTED_XTS_AES_BLOCK_MAX=32
# default:
CONFIG_SOC_PHY_DIG_REGS_MEM_SIZE=21
# default:
CONFIG_SOC_PM_SUPPORT_EXT0_WAKEUP=y
# default:
CONFIG_SOC_PM_SUPPORT_EXT1_WAKEUP=y
# default:
CONFIG_SOC_PM_SUPPORT_EXT_WAKEUP=y
# default:
CONFIG_SOC_PM_SUPPORT_TOUCH_SENSOR_WAKEUP=y
# default:
CONFIG_SOC_PM_SUPPORT_RTC_PERIPH_PD=y
# default:
CONFIG_SOC_PM_SUPPORT_RTC_FAST_MEM_PD=y
# default:
CONFIG_SOC_PM_SUPPORT_RTC_SLOW_MEM_PD=y
# default:
CONFIG_SOC_PM_SUPPORT_RC_FAST_PD=y
# default:
CONFIG_SOC_PM_SUPPORT_VDDSDIO_PD=y
# default:
CONFIG_SOC_PM_SUPPORT_MODEM_PD=y
# default:
CONFIG_SOC_CONFIGURABLE_VDDSDIO_SUPPORTED=y
# default:
CONFIG_SOC_PM_MODEM_PD_BY_SW=y
# default:
CONFIG_SOC_CLK_APLL_SUPPORTED=y
# default:
CONFIG_SOC_CLK_RC_FAST_D256_SUPPORTED=y
# default:
CONFIG_SOC_RTC_SLOW_CLK_SUPPORT_RC_FAST_D256=y
# default:
CONFIG_SOC_CLK_RC_FAST_SUPPORT_CALIBRATION=y
# default:
CONFIG_SOC_CLK_XTAL32K_SUPPORTED=y
# default:
CONFIG_SOC_CLK_LP_FAST_SUPPORT_XTAL_D4=y
# default:
CONFIG_SOC_SDMMC_USE_IOMUX=y
# default:
CONFIG_SOC_SDMMC_NUM_SLOTS=2
# default:
CONFIG_SOC_WIFI_WAPI_SUPPORT=y
# default:
CONFIG_SOC_WIFI_CSI_SUPPORT=y
# default:
CONFIG_SOC_WIFI_MESH_SUPPORT=y
# default:
CONFIG_SOC_WIFI_SUPPORT_VARIABLE_BEACON_WINDOW=y
# default:
CONFIG_SOC_WIFI_NAN_SUPPORT=y
# default:
CONFIG_SOC_BLE_SUPPORTED=y
# default:
CONFIG_SOC_BLE_MESH_SUPPORTED=y
# default:
CONFIG_SOC_BT_CLASSIC_SUPPORTED=y
# default:
CONFIG_SOC_BLUFI_SUPPORTED=y
# default:
CONFIG_SOC_BT_H2C_ENC_KEY_CTRL_ENH_VSC_SUPPORTED=y
# default:
CONFIG_SOC_BLE_MULTI_CONN_OPTIMIZATION=y
# default:
CONFIG_SOC_ULP_HAS_ADC=y
# default:
CONFIG_SOC_PHY_COMBO_MODULE=y
# default:
CONFIG_SOC_EMAC_RMII_CLK_OUT_INTERNAL_LOOPBACK=y
# default:
CONFIG_IDF_CMAKE=y
# default:
CONFIG_IDF_TOOLCHAIN="gcc"
# default:
CONFIG_IDF_TOOLCHAIN_GCC=y
# default:
CONFIG_IDF_TARGET_ARCH_XTENSA=y
# default:
CONFIG_IDF_TARGET_ARCH="xtensa"
# default:
CONFIG_IDF_TARGET="esp32"
# default:
CONFIG_IDF_INIT_VERSION="5.5.3"
# default:
CONFIG_IDF_TARGET_ESP32=y
# default:
CONFIG_IDF_FIRMWARE_CHIP_ID=0x0000

#
# Build type
#
# default:
CONFIG_APP_BUILD_TYPE_APP_2NDBOOT=y
# default:
# CONFIG_APP_BUILD_TYPE_RAM is not set
# default:
CONFIG_APP_BUILD_GENERATE_BINARIES=y
# default:
CONFIG_APP_BUILD_BOOTLOADER=y
# default:
CONFIG_APP_BUILD_USE_FLASH_SECTIONS=y
# default:
# CONFIG_APP_REPRODUCIBLE_BUILD is not set
# default:
# CONFIG_APP_NO_BLOBS is not set
# default:
# CONFIG_APP_COMPATIBLE_PRE_V2_1_BOOTLOADERS is not set
# default:
# CONFIG_APP_COMPATIBLE_PRE_V3_1_BOOTLOADERS is not set
# end of Build type

#
# Bootloader config
#

#
# Bootloader manager
#
# default:
CONFIG_BOOTLOADER_COMPILE_TIME_DATE=y
# default:
CONFIG_BOOTLOADER_PROJECT_VER=1
# end of Bootloader manager

#
# Application Rollback
#
# default:
# CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE is not set
# end of Application Rollback

#
# Recovery Bootloader and Rollback
#
# end of Recovery Bootloader and Rollback

# default:
CONFIG_BOOTLOADER_OFFSET_IN_FLASH=0x1000
# default:
CONFIG_BOOTLOADER_COMPILER_OPTIMIZATION_SIZE=y
# default:
# CONFIG_BOOTLOADER_COMPILER_OPTIMIZATION_DEBUG is not set
# default:
# CONFIG_BOOTLOADER_COMPILER_OPTIMIZATION_PERF is not set
# default:
# CONFIG_BOOTLOADER_COMPILER_OPTIMIZATION_NONE is not set

#
# Log
#
# default:
CONFIG_BOOTLOADER_LOG_VERSION_1=y
# default:
CONFIG_BOOTLOADER_LOG_VERSION=1
# default:
# CONFIG_BOOTLOADER_LOG_LEVEL_NONE is not set
# default:
# CONFIG_BOOTLOADER_LOG_LEVEL_ERROR is not set
# default:
# CONFIG_BOOTLOADER_LOG_LEVEL_WARN is not set
# default:
CONFIG_BOOTLOADER_LOG_LEVEL_INFO=y
# default:
# CONFIG_BOOTLOADER_LOG_LEVEL_DEBUG is not set
# default:
# CONFIG_BOOTLOADER_LOG_LEVEL_VERBOSE is not set
# default:
CONFIG_BOOTLOADER_LOG_LEVEL=3

#
# Format
#
# default:
# CONFIG_BOOTLOADER_LOG_COLORS is not set
# default:
CONFIG_BOOTLOADER_LOG_TIMESTAMP_SOURCE_CPU_TICKS=y
# end of Format

#
# Settings
#
# default:
CONFIG_BOOTLOADER_LOG_MODE_TEXT_EN=y
# default:
CONFIG_BOOTLOADER_LOG_MODE_TEXT=y
# end of Settings
# end of Log

#
# Serial Flash Configurations
#
# default:
# CONFIG_BOOTLOADER_FLASH_DC_AWARE is not set
# default:
CONFIG_BOOTLOADER_FLASH_XMC_SUPPORT=y
# end of Serial Flash Configurations

# default:
# CONFIG_BOOTLOADER_VDDSDIO_BOOST_1_8V is not set
# default:
CONFIG_BOOTLOADER_VDDSDIO_BOOST_1_9V=y
# default:
# CONFIG_BOOTLOADER_FACTORY_RESET is not set
# default:
# CONFIG_BOOTLOADER_APP_TEST is not set
# default:
CONFIG_BOOTLOADER_REGION_PROTECTION_ENABLE=y
# default:
CONFIG_BOOTLOADER_WDT_ENABLE=y
# default:
# CONFIG_BOOTLOADER_WDT_DISABLE_IN_USER_CODE is not set
# default:
CONFIG_BOOTLOADER_WDT_TIME_MS=9000
# default:
# CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP is not set
# default:
# CONFIG_BOOTLOADER_SKIP_VALIDATE_ON_POWER_ON is not set
# default:
# CONFIG_BOOTLOADER_SKIP_VALIDATE_ALWAYS is not set
# default:
CONFIG_BOOTLOADER_RESERVE_RTC_SIZE=0
# default:
# CONFIG_BOOTLOADER_CUSTOM_RESERVE_RTC is not set
# end of Bootloader config

#
# Security features
#
# default:
CONFIG_SECURE_BOOT_V1_SUPPORTED=y
# default:
# CONFIG_SECURE_SIGNED_APPS_NO_SECURE_BOOT is not set
# default:
# CONFIG_SECURE_BOOT is not set
# default:
# CONFIG_SECURE_FLASH_ENC_ENABLED is not set
# end of Security features

#
# Application manager
#
# default:
CONFIG_APP_COMPILE_TIME_DATE=y
# default:
# CONFIG_APP_EXCLUDE_PROJECT_VER_VAR is not set
# default:
# CONFIG_APP_EXCLUDE_PROJECT_NAME_VAR is not set
# default:
# CONFIG_APP_PROJECT_VER_FROM_CONFIG is not set
# default:
CONFIG_APP_RETRIEVE_LEN_ELF_SHA=9
# end of Application manager

# default:
CONFIG_ESP_ROM_HAS_CRC_LE=y
# default:
CONFIG_ESP_ROM_HAS_CRC_BE=y
# default:
CONFIG_ESP_ROM_HAS_MZ_CRC32=y
# default:
CONFIG_ESP_ROM_HAS_JPEG_DECODE=y
# default:
CONFIG_ESP_ROM_HAS_UART_BUF_SWITCH=y
# default:
CONFIG_ESP_ROM_NEEDS_SWSETUP_WORKAROUND=y
# default:
CONFIG_ESP_ROM_HAS_NEWLIB=y
# default:
CONFIG_ESP_ROM_HAS_NEWLIB_NANO_FORMAT=y
# default:
CONFIG_ESP_ROM_HAS_NEWLIB_32BIT_TIME=y
# default:
CONFIG_ESP_ROM_HAS_SW_FLOAT=y
# default:
CONFIG_ESP_ROM_USB_OTG_NUM=-1
# default:
CONFIG_ESP_ROM_USB_SERIAL_DEVICE_NUM=-1
# default:
CONFIG_ESP_ROM_SUPPORT_DEEP_SLEEP_WAKEUP_STUB=y
# default:
CONFIG_ESP_ROM_HAS_OUTPUT_PUTC_FUNC=y

#
# Serial flasher config
#
# default:
# CONFIG_ESPTOOLPY_NO_STUB is not set
# default:
# CONFIG_ESPTOOLPY_FLASHMODE_QIO is not set
# default:
# CONFIG_ESPTOOLPY_FLASHMODE_QOUT is not set
# default:
CONFIG_ESPTOOLPY_FLASHMODE_DIO=y
# default:
# CONFIG_ESPTOOLPY_FLASHMODE_DOUT is not set
# default:
CONFIG_ESPTOOLPY_FLASH_SAMPLE_MODE_STR=y
# default:
CONFIG_ESPTOOLPY_FLASHMODE="dio"
# default:
# CONFIG_ESPTOOLPY_FLASHFREQ_80M is not set
# default:
CONFIG_ESPTOOLPY_FLASHFREQ_40M=y
# default:
# CONFIG_ESPTOOLPY_FLASHFREQ_26M is not set
# default:
# CONFIG_ESPTOOLPY_FLASHFREQ_20M is not set
# default:
CONFIG_ESPTOOLPY_FLASHFREQ="40m"
# default:
# CONFIG_ESPTOOLPY_FLASHSIZE_1MB is not set
# default:
CONFIG_ESPTOOLPY_FLASHSIZE_2MB=y
# default:
# CONFIG_ESPTOOLPY_FLASHSIZE_4MB is not set
# default:
# CONFIG_ESPTOOLPY_FLASHSIZE_8MB is not set
# default:
# CONFIG_ESPTOOLPY_FLASHSIZE_16MB is not set
# default:
# CONFIG_ESPTOOLPY_FLASHSIZE_32MB is not set
# default:
# CONFIG_ESPTOOLPY_FLASHSIZE_64MB is not set
# default:
# CONFIG_ESPTOOLPY_FLASHSIZE_128MB is not set
# default:
CONFIG_ESPTOOLPY_FLASHSIZE="2MB"
# default:
# CONFIG_ESPTOOLPY_HEADER_FLASHSIZE_UPDATE is not set
# default:
CONFIG_ESPTOOLPY_BEFORE_RESET=y
# default:
# CONFIG_ESPTOOLPY_BEFORE_NORESET is not set
# default:
CONFIG_ESPTOOLPY_BEFORE="default_reset"
# default:
CONFIG_ESPTOOLPY_AFTER_RESET=y
# default:
# CONFIG_ESPTOOLPY_AFTER_NORESET is not set
# default:
CONFIG_ESPTOOLPY_AFTER="hard_reset"
# default:
CONFIG_ESPTOOLPY_MONITOR_BAUD=115200
# end of Serial flasher config

#
# Partition Table
#
# default:
CONFIG_PARTITION_TABLE_SINGLE_APP=y
# default:
# CONFIG_PARTITION_TABLE_SINGLE_APP_LARGE is not set
# default:
# CONFIG_PARTITION_TABLE_TWO_OTA is not set
# default:
# CONFIG_PARTITION_TABLE_TWO_OTA_LARGE is not set
# default:
# CONFIG_PARTITION_TABLE_CUSTOM is not set
# default:
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
# default:
CONFIG_PARTITION_TABLE_FILENAME="partitions_singleapp.csv"
# default:
CONFIG_PARTITION_TABLE_OFFSET=0x8000
# default:
CONFIG_PARTITION_TABLE_MD5=y
# end of Partition Table

#
# Compiler options
#
# default:
CONFIG_COMPILER_OPTIMIZATION_DEBUG=y
# default:
# CONFIG_COMPILER_OPTIMIZATION_SIZE is not set
# default:
# CONFIG_COMPILER_OPTIMIZATION_PERF is not set
# default:
# CONFIG_COMPILER_OPTIMIZATION_NONE is not set
# default:
CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_ENABLE=y
# default:
# CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_SILENT is not set
# default:
# CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_DISABLE is not set
# default:
CONFIG_COMPILER_ASSERT_NDEBUG_EVALUATE=y
# default:
CONFIG_COMPILER_FLOAT_LIB_FROM_GCCLIB=y
# default:
CONFIG_COMPILER_OPTIMIZATION_ASSERTION_LEVEL=2
# default:
# CONFIG_COMPILER_OPTIMIZATION_CHECKS_SILENT is not set
# default:
CONFIG_COMPILER_HIDE_PATHS_MACROS=y
# default:
# CONFIG_COMPILER_CXX_EXCEPTIONS is not set
# default:
# CONFIG_COMPILER_CXX_RTTI is not set
# default:
CONFIG_COMPILER_STACK_CHECK_MODE_NONE=y
# default:
# CONFIG_COMPILER_STACK_CHECK_MODE_NORM is not set
# default:
# CONFIG_COMPILER_STACK_CHECK_MODE_STRONG is not set
# default:
# CONFIG_COMPILER_STACK_CHECK_MODE_ALL is not set
# default:
# CONFIG_COMPILER_NO_MERGE_CONSTANTS is not set
# default:
# CONFIG_COMPILER_WARN_WRITE_STRINGS is not set
# default:
CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS=y
# default:
# CONFIG_COMPILER_DISABLE_GCC12_WARNINGS is not set
# default:
# CONFIG_COMPILER_DISABLE_GCC13_WARNINGS is not set
# default:
# CONFIG_COMPILER_DISABLE_GCC14_WARNINGS is not set
# default:
# CONFIG_COMPILER_DUMP_RTL_FILES is not set
# default:
CONFIG_COMPILER_RT_LIB_GCCLIB=y
# default:
CONFIG_COMPILER_RT_LIB_NAME="gcc"
# default:
CONFIG_COMPILER_ORPHAN_SECTIONS_WARNING=y
# default:
# CONFIG_COMPILER_ORPHAN_SECTIONS_PLACE is not set
# default:
# CONFIG_COMPILER_STATIC_ANALYZER is not set
# end of Compiler options

#
# Component config
#

#
# Application Level Tracing
#
# default:
# CONFIG_APPTRACE_DEST_JTAG is not set
# default:
CONFIG_APPTRACE_DEST_NONE=y
# default:
# CONFIG_APPTRACE_DEST_UART1 is not set
# default:
# CONFIG_APPTRACE_DEST_UART2 is not set
# default:
CONFIG_APPTRACE_DEST_UART_NONE=y
# default:
CONFIG_APPTRACE_UART_TASK_PRIO=1
# default:
CONFIG_APPTRACE_LOCK_ENABLE=y
# end of Application Level Tracing

#
# Bluetooth
#
# default:
# CONFIG_BT_ENABLED is not set

#
# Common Options
#

#
# BLE Log
#
# default:
# CONFIG_BLE_LOG_ENABLED is not set
# end of BLE Log

# default:
# CONFIG_BT_BLE_LOG_SPI_OUT_ENABLED is not set
# default:
# CONFIG_BT_BLE_LOG_UHCI_OUT_ENABLED is not set
# default:
# CONFIG_BT_LE_USED_MEM_STATISTICS_ENABLED is not set
# end of Common Options
# end of Bluetooth

#
# Console Library
#
# default:
# CONFIG_CONSOLE_SORTED_HELP is not set
# end of Console Library

#
# Driver Configurations
#

#
# Legacy TWAI Driver Configurations
#
# default:
# CONFIG_TWAI_SKIP_LEGACY_CONFLICT_CHECK is not set
# default:
CONFIG_TWAI_ERRATA_FIX_BUS_OFF_REC=y
# default:
CONFIG_TWAI_ERRATA_FIX_TX_INTR_LOST=y
# default:
CONFIG_TWAI_ERRATA_FIX_RX_FRAME_INVALID=y
# default:
CONFIG_TWAI_ERRATA_FIX_RX_FIFO_CORRUPT=y
# default:
CONFIG_TWAI_ERRATA_FIX_LISTEN_ONLY_DOM=y
# end of Legacy TWAI Driver Configurations

#
# Legacy ADC Driver Configuration
#
# default:
CONFIG_ADC_DISABLE_DAC=y
# default:
# CONFIG_ADC_SUPPRESS_DEPRECATE_WARN is not set
# default:
# CONFIG_ADC_SKIP_LEGACY_CONFLICT_CHECK is not set

#
# Legacy ADC Calibration Configuration
#
# default:
CONFIG_ADC_CAL_EFUSE_TP_ENABLE=y
# default:
CONFIG_ADC_CAL_EFUSE_VREF_ENABLE=y
# default:
CONFIG_ADC_CAL_LUT_ENABLE=y
# default:
# CONFIG_ADC_CALI_SUPPRESS_DEPRECATE_WARN is not set
# end of Legacy ADC Calibration Configuration
# end of Legacy ADC Driver Configuration

#
# Legacy DAC Driver Configurations
#
# default:
# CONFIG_DAC_SUPPRESS_DEPRECATE_WARN is not set
# default:
# CONFIG_DAC_SKIP_LEGACY_CONFLICT_CHECK is not set
# end of Legacy DAC Driver Configurations

#
# Legacy MCPWM Driver Configurations
#
# default:
# CONFIG_MCPWM_SUPPRESS_DEPRECATE_WARN is not set
# default:
# CONFIG_MCPWM_SKIP_LEGACY_CONFLICT_CHECK is not set
# end of Legacy MCPWM Driver Configurations

#
# Legacy Timer Group Driver Configurations
#
# default:
# CONFIG_GPTIMER_SUPPRESS_DEPRECATE_WARN is not set
# default:
# CONFIG_GPTIMER_SKIP_LEGACY_CONFLICT_CHECK is not set
# end of Legacy Timer Group Driver Configurations

#
# Legacy RMT Driver Configurations
#
# default:
# CONFIG_RMT_SUPPRESS_DEPRECATE_WARN is not set
# default:
# CONFIG_RMT_SKIP_LEGACY_CONFLICT_CHECK is not set
# end of Legacy RMT Driver Configurations

#
# Legacy I2S Driver Configurations
#
# default:
# CONFIG_I2S_SUPPRESS_DEPRECATE_WARN is not set
# default:
# CONFIG_I2S_SKIP_LEGACY_CONFLICT_CHECK is not set
# end of Legacy I2S Driver Configurations

#
# Legacy I2C Driver Configurations
#
# default:
# CONFIG_I2C_SKIP_LEGACY_CONFLICT_CHECK is not set
# end of Legacy I2C Driver Configurations

#
# Legacy PCNT Driver Configurations
#
# default:
# CONFIG_PCNT_SUPPRESS_DEPRECATE_WARN is not set
# default:
# CONFIG_PCNT_SKIP_LEGACY_CONFLICT_CHECK is not set
# end of Legacy PCNT Driver Configurations

#
# Legacy SDM Driver Configurations
#
# default:
# CONFIG_SDM_SUPPRESS_DEPRECATE_WARN is not set
# default:
# CONFIG_SDM_SKIP_LEGACY_CONFLICT_CHECK is not set
# end of Legacy SDM Driver Configurations

#
# Legacy Touch Sensor Driver Configurations
#
# default:
# CONFIG_TOUCH_SUPPRESS_DEPRECATE_WARN is not set
# default:
# CONFIG_TOUCH_SKIP_LEGACY_CONFLICT_CHECK is not set
# end of Legacy Touch Sensor Driver Configurations
# end of Driver Configurations

#
# eFuse Bit Manager
#
# default:
# CONFIG_EFUSE_CUSTOM_TABLE is not set
# default:
# CONFIG_EFUSE_VIRTUAL is not set
# default:
# CONFIG_EFUSE_CODE_SCHEME_COMPAT_NONE is not set
# default:
CONFIG_EFUSE_CODE_SCHEME_COMPAT_3_4=y
# default:
# CONFIG_EFUSE_CODE_SCHEME_COMPAT_REPEAT is not set
# default:
CONFIG_EFUSE_MAX_BLK_LEN=192
# end of eFuse Bit Manager

#
# ESP-TLS
#
# default:
CONFIG_ESP_TLS_USING_MBEDTLS=y
# default:
# CONFIG_ESP_TLS_USE_SECURE_ELEMENT is not set
# default:
# CONFIG_ESP_TLS_CLIENT_SESSION_TICKETS is not set
# default:
# CONFIG_ESP_TLS_SERVER_SESSION_TICKETS is not set
# default:
# CONFIG_ESP_TLS_SERVER_CERT_SELECT_HOOK is not set
# default:
# CONFIG_ESP_TLS_SERVER_MIN_AUTH_MODE_OPTIONAL is not set
# default:
# CONFIG_ESP_TLS_PSK_VERIFICATION is not set
# default:
# CONFIG_ESP_TLS_INSECURE is not set
# default:
CONFIG_ESP_TLS_DYN_BUF_STRATEGY_SUPPORTED=y
# end of ESP-TLS

#
# ADC and ADC Calibration
#
# default:
# CONFIG_ADC_ONESHOT_CTRL_FUNC_IN_IRAM is not set
# default:
# CONFIG_ADC_CONTINUOUS_ISR_IRAM_SAFE is not set

#
# ADC Calibration Configurations
#
# default:
CONFIG_ADC_CALI_EFUSE_TP_ENABLE=y
# default:
CONFIG_ADC_CALI_EFUSE_VREF_ENABLE=y
# default:
CONFIG_ADC_CALI_LUT_ENABLE=y
# end of ADC Calibration Configurations

# default:
CONFIG_ADC_DISABLE_DAC_OUTPUT=y
# default:
# CONFIG_ADC_ENABLE_DEBUG_LOG is not set
# end of ADC and ADC Calibration

#
# Wireless Coexistence
#
# default:
CONFIG_ESP_COEX_ENABLED=y
# default:
# CONFIG_ESP_COEX_GPIO_DEBUG is not set
# end of Wireless Coexistence

#
# Common ESP-related
#
# default:
CONFIG_ESP_ERR_TO_NAME_LOOKUP=y
# end of Common ESP-related

#
# ESP-Driver:DAC Configurations
#
# default:
# CONFIG_DAC_CTRL_FUNC_IN_IRAM is not set
# default:
# CONFIG_DAC_ISR_IRAM_SAFE is not set
# default:
# CONFIG_DAC_ENABLE_DEBUG_LOG is not set
# default:
CONFIG_DAC_DMA_AUTO_16BIT_ALIGN=y
# end of ESP-Driver:DAC Configurations

#
# ESP-Driver:GPIO Configurations
#
# default:
# CONFIG_GPIO_ESP32_SUPPORT_SWITCH_SLP_PULL is not set
# default:
# CONFIG_GPIO_CTRL_FUNC_IN_IRAM is not set
# end of ESP-Driver:GPIO Configurations

#
# ESP-Driver:GPTimer Configurations
#
# default:
CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM=y
# default:
# CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM is not set
# default:
# CONFIG_GPTIMER_ISR_CACHE_SAFE is not set
# default:
CONFIG_GPTIMER_OBJ_CACHE_SAFE=y
# default:
# CONFIG_GPTIMER_ENABLE_DEBUG_LOG is not set
# end of ESP-Driver:GPTimer Configurations

#
# ESP-Driver:I2C Configurations
#
# default:
# CONFIG_I2C_ISR_IRAM_SAFE is not set
# default:
# CONFIG_I2C_ENABLE_DEBUG_LOG is not set
# default:
# CONFIG_I2C_ENABLE_SLAVE_DRIVER_VERSION_2 is not set
# default:
CONFIG_I2C_MASTER_ISR_HANDLER_IN_IRAM=y
# end of ESP-Driver:I2C Configurations

#
# ESP-Driver:I2S Configurations
#
# default:
# CONFIG_I2S_ISR_IRAM_SAFE is not set
# default:
# CONFIG_I2S_ENABLE_DEBUG_LOG is not set
# end of ESP-Driver:I2S Configurations

#
# ESP-Driver:LEDC Configurations
#
# default:
# CONFIG_LEDC_CTRL_FUNC_IN_IRAM is not set
# end of ESP-Driver:LEDC Configurations

#
# ESP-Driver:MCPWM Configurations
#
# default:
CONFIG_MCPWM_ISR_HANDLER_IN_IRAM=y
# default:
# CONFIG_MCPWM_ISR_CACHE_SAFE is not set
# default:
# CONFIG_MCPWM_CTRL_FUNC_IN_IRAM is not set
# default:
CONFIG_MCPWM_OBJ_CACHE_SAFE=y
# default:
# CONFIG_MCPWM_ENABLE_DEBUG_LOG is not set
# end of ESP-Driver:MCPWM Configurations

#
# ESP-Driver:PCNT Configurations
#
# default:
# CONFIG_PCNT_CTRL_FUNC_IN_IRAM is not set
# default:
# CONFIG_PCNT_ISR_IRAM_SAFE is not set
# default:
# CONFIG_PCNT_ENABLE_DEBUG_LOG is not set
# end of ESP-Driver:PCNT Configurations

#
# ESP-Driver:RMT Configurations
#
# default:
CONFIG_RMT_ENCODER_FUNC_IN_IRAM=y
# default:
CONFIG_RMT_TX_ISR_HANDLER_IN_IRAM=y
# default:
CONFIG_RMT_RX_ISR_HANDLER_IN_IRAM=y
# default:
# CONFIG_RMT_RECV_FUNC_IN_IRAM is not set
# default:
# CONFIG_RMT_TX_ISR_CACHE_SAFE is not set
# default:
# CONFIG_RMT_RX_ISR_CACHE_SAFE is not set
# default:
CONFIG_RMT_OBJ_CACHE_SAFE=y
# default:
# CONFIG_RMT_ENABLE_DEBUG_LOG is not set
# default:
# CONFIG_RMT_ISR_IRAM_SAFE is not set
# end of ESP-Driver:RMT Configurations

#
# ESP-Driver:Sigma Delta Modulator Configurations
#
# default:
# CONFIG_SDM_CTRL_FUNC_IN_IRAM is not set
# default:
# CONFIG_SDM_ENABLE_DEBUG_LOG is not set
# end of ESP-Driver:Sigma Delta Modulator Configurations

#
# ESP-Driver:SPI Configurations
#
# default:
# CONFIG_SPI_MASTER_IN_IRAM is not set
# default:
CONFIG_SPI_MASTER_ISR_IN_IRAM=y
# default:
# CONFIG_SPI_SLAVE_IN_IRAM is not set
# default:
CONFIG_SPI_SLAVE_ISR_IN_IRAM=y
# end of ESP-Driver:SPI Configurations

#
# ESP-Driver:Touch Sensor Configurations
#
# default:
# CONFIG_TOUCH_CTRL_FUNC_IN_IRAM is not set
# default:
# CONFIG_TOUCH_ISR_IRAM_SAFE is not set
# default:
# CONFIG_TOUCH_ENABLE_DEBUG_LOG is not set
# default:
# CONFIG_TOUCH_SKIP_FSM_CHECK is not set
# end of ESP-Driver:Touch Sensor Configurations

#
# ESP-Driver:TWAI Configurations
#
# default:
# CONFIG_TWAI_ISR_IN_IRAM is not set
# default:
# CONFIG_TWAI_IO_FUNC_IN_IRAM is not set
# default:
# CONFIG_TWAI_ISR_CACHE_SAFE is not set
# default:
# CONFIG_TWAI_ENABLE_DEBUG_LOG is not set
# end of ESP-Driver:TWAI Configurations

#
# ESP-Driver:UART Configurations
#
# default:
# CONFIG_UART_ISR_IN_IRAM is not set
# end of ESP-Driver:UART Configurations

#
# ESP-Driver:UHCI Configurations
#
# default:
# CONFIG_UHCI_ISR_HANDLER_IN_IRAM is not set
# default:
# CONFIG_UHCI_ISR_CACHE_SAFE is not set
# default:
# CONFIG_UHCI_ENABLE_DEBUG_LOG is not set
# end of ESP-Driver:UHCI Configurations

#
# Ethernet
#
# default:
CONFIG_ETH_ENABLED=y
# default:
CONFIG_ETH_USE_ESP32_EMAC=y
# default:
CONFIG_ETH_PHY_INTERFACE_RMII=y
# default:
CONFIG_ETH_RMII_CLK_INPUT=y
# default:
# CONFIG_ETH_RMII_CLK_OUTPUT is not set
# default:
CONFIG_ETH_RMII_CLK_IN_GPIO=0
# default:
CONFIG_ETH_DMA_BUFFER_SIZE=512
# default:
CONFIG_ETH_DMA_RX_BUFFER_NUM=10
# default:
CONFIG_ETH_DMA_TX_BUFFER_NUM=10
# default:
# CONFIG_ETH_IRAM_OPTIMIZATION is not set
# default:
CONFIG_ETH_USE_SPI_ETHERNET=y
# default:
# CONFIG_ETH_SPI_ETHERNET_DM9051 is not set
# default:
# CONFIG_ETH_SPI_ETHERNET_W5500 is not set
# default:
# CONFIG_ETH_SPI_ETHERNET_KSZ8851SNL is not set
# default:
# CONFIG_ETH_USE_OPENETH is not set
# default:
# CONFIG_ETH_TRANSMIT_MUTEX is not set
# end of Ethernet

#
# Event Loop Library
#
# default:
# CONFIG_ESP_EVENT_LOOP_PROFILING is not set
# default:
CONFIG_ESP_EVENT_POST_FROM_ISR=y
# default:
CONFIG_ESP_EVENT_POST_FROM_IRAM_ISR=y
# end of Event Loop Library

#
# GDB Stub
#
# default:
CONFIG_ESP_GDBSTUB_ENABLED=y
# default:
# CONFIG_ESP_SYSTEM_GDBSTUB_RUNTIME is not set
# default:
CONFIG_ESP_GDBSTUB_SUPPORT_TASKS=y
# default:
CONFIG_ESP_GDBSTUB_MAX_TASKS=32
# end of GDB Stub

#
# ESP HID
#
# default:
CONFIG_ESPHID_TASK_SIZE_BT=2048
# default:
CONFIG_ESPHID_TASK_SIZE_BLE=4096
# end of ESP HID

#
# ESP HTTP client
#
# default:
CONFIG_ESP_HTTP_CLIENT_ENABLE_HTTPS=y
# default:
# CONFIG_ESP_HTTP_CLIENT_ENABLE_BASIC_AUTH is not set
# default:
# CONFIG_ESP_HTTP_CLIENT_ENABLE_DIGEST_AUTH is not set
# default:
# CONFIG_ESP_HTTP_CLIENT_ENABLE_CUSTOM_TRANSPORT is not set
# default:
CONFIG_ESP_HTTP_CLIENT_EVENT_POST_TIMEOUT=2000
# end of ESP HTTP client

#
# HTTP Server
#
# default:
CONFIG_HTTPD_MAX_REQ_HDR_LEN=1024
# default:
CONFIG_HTTPD_MAX_URI_LEN=512
# default:
CONFIG_HTTPD_ERR_RESP_NO_DELAY=y
# default:
CONFIG_HTTPD_PURGE_BUF_LEN=32
# default:
# CONFIG_HTTPD_LOG_PURGE_DATA is not set
# default:
# CONFIG_HTTPD_WS_SUPPORT is not set
# default:
# CONFIG_HTTPD_QUEUE_WORK_BLOCKING is not set
# default:
CONFIG_HTTPD_SERVER_EVENT_POST_TIMEOUT=2000
# end of HTTP Server

#
# ESP HTTPS OTA
#
# default:
# CONFIG_ESP_HTTPS_OTA_DECRYPT_CB is not set
# default:
# CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP is not set
# default:
CONFIG_ESP_HTTPS_OTA_EVENT_POST_TIMEOUT=2000
# end of ESP HTTPS OTA

#
# ESP HTTPS server
#
# default:
# CONFIG_ESP_HTTPS_SERVER_ENABLE is not set
# default:
CONFIG_ESP_HTTPS_SERVER_EVENT_POST_TIMEOUT=2000
# default:
# CONFIG_ESP_HTTPS_SERVER_CERT_SELECT_HOOK is not set
# end of ESP HTTPS server

#
# Hardware Settings
#
# default:
CONFIG_ESP_HW_SUPPORT_FUNC_IN_IRAM=y

#
# Chip revision
#
# default:
CONFIG_ESP32_REV_MIN_0=y
# default:
# CONFIG_ESP32_REV_MIN_1 is not set
# default:
# CONFIG_ESP32_REV_MIN_1_1 is not set
# default:
# CONFIG_ESP32_REV_MIN_2 is not set
# default:
# CONFIG_ESP32_REV_MIN_3 is not set
# default:
# CONFIG_ESP32_REV_MIN_3_1 is not set
# default:
CONFIG_ESP32_REV_MIN=0
# default:
CONFIG_ESP32_REV_MIN_FULL=0
# default:
CONFIG_ESP_REV_MIN_FULL=0

#
# Maximum Supported ESP32 Revision (Rev v3.99)
#
# default:
CONFIG_ESP32_REV_MAX_FULL=399
# default:
CONFIG_ESP_REV_MAX_FULL=399
# default:
CONFIG_ESP_EFUSE_BLOCK_REV_MIN_FULL=0
# default:
CONFIG_ESP_EFUSE_BLOCK_REV_MAX_FULL=99

#
# Maximum Supported ESP32 eFuse Block Revision (eFuse Block Rev v0.99)
#
# end of Chip revision

#
# MAC Config
#
# default:
CONFIG_ESP_MAC_ADDR_UNIVERSE_WIFI_STA=y
# default:
CONFIG_ESP_MAC_ADDR_UNIVERSE_WIFI_AP=y
# default:
CONFIG_ESP_MAC_ADDR_UNIVERSE_BT=y
# default:
CONFIG_ESP_MAC_ADDR_UNIVERSE_ETH=y
# default:
CONFIG_ESP_MAC_UNIVERSAL_MAC_ADDRESSES_FOUR=y
# default:
CONFIG_ESP_MAC_UNIVERSAL_MAC_ADDRESSES=4
# default:
# CONFIG_ESP32_UNIVERSAL_MAC_ADDRESSES_TWO is not set
# default:
CONFIG_ESP32_UNIVERSAL_MAC_ADDRESSES_FOUR=y
# default:
CONFIG_ESP32_UNIVERSAL_MAC_ADDRESSES=4
# default:
# CONFIG_ESP_MAC_IGNORE_MAC_CRC_ERROR is not set
# default:
# CONFIG_ESP_MAC_USE_CUSTOM_MAC_AS_BASE_MAC is not set
# end of MAC Config

#
# Sleep Config
#
# default:
# CONFIG_ESP_SLEEP_POWER_DOWN_FLASH is not set
# default:
CONFIG_ESP_SLEEP_FLASH_LEAKAGE_WORKAROUND=y
# default:
# CONFIG_ESP_SLEEP_MSPI_NEED_ALL_IO_PU is not set
# default:
CONFIG_ESP_SLEEP_RTC_BUS_ISO_WORKAROUND=y
# default:
# CONFIG_ESP_SLEEP_GPIO_RESET_WORKAROUND is not set
# default:
CONFIG_ESP_SLEEP_WAIT_FLASH_READY_EXTRA_DELAY=2000
# default:
# CONFIG_ESP_SLEEP_CACHE_SAFE_ASSERTION is not set
# default:
# CONFIG_ESP_SLEEP_DEBUG is not set
# default:
CONFIG_ESP_SLEEP_GPIO_ENABLE_INTERNAL_RESISTORS=y
# end of Sleep Config

#
# RTC Clock Config
#
# default:
CONFIG_RTC_CLK_SRC_INT_RC=y
# default:
# CONFIG_RTC_CLK_SRC_EXT_CRYS is not set
# default:
# CONFIG_RTC_CLK_SRC_EXT_OSC is not set
# default:
# CONFIG_RTC_CLK_SRC_INT_8MD256 is not set
# default:
CONFIG_RTC_CLK_CAL_CYCLES=1024
# default:
CONFIG_RTC_CLK_FUNC_IN_IRAM=y
# default:
CONFIG_RTC_TIME_FUNC_IN_IRAM=y
# end of RTC Clock Config

#
# Peripheral Control
#
# default:
CONFIG_ESP_PERIPH_CTRL_FUNC_IN_IRAM=y
# default:
CONFIG_ESP_REGI2C_CTRL_FUNC_IN_IRAM=y
# end of Peripheral Control

#
# Main XTAL Config
#
# default:
# CONFIG_XTAL_FREQ_26 is not set
# default:
# CONFIG_XTAL_FREQ_32 is not set
# default:
CONFIG_XTAL_FREQ_40=y
# default:
# CONFIG_XTAL_FREQ_AUTO is not set
# default:
CONFIG_XTAL_FREQ=40
# end of Main XTAL Config

#
# Power Supplier
#

#
# Brownout Detector
#
# default:
CONFIG_ESP_BROWNOUT_DET=y
# default:
CONFIG_ESP_BROWNOUT_DET_LVL_SEL_0=y
# default:
# CONFIG_ESP_BROWNOUT_DET_LVL_SEL_1 is not set
# default:
# CONFIG_ESP_BROWNOUT_DET_LVL_SEL_2 is not set
# default:
# CONFIG_ESP_BROWNOUT_DET_LVL_SEL_3 is not set
# default:
# CONFIG_ESP_BROWNOUT_DET_LVL_SEL_4 is not set
# default:
# CONFIG_ESP_BROWNOUT_DET_LVL_SEL_5 is not set
# default:
# CONFIG_ESP_BROWNOUT_DET_LVL_SEL_6 is not set
# default:
# CONFIG_ESP_BROWNOUT_DET_LVL_SEL_7 is not set
# default:
CONFIG_ESP_BROWNOUT_DET_LVL=0
# default:
CONFIG_ESP_BROWNOUT_USE_INTR=y
# end of Brownout Detector
# end of Power Supplier

# default:
CONFIG_ESP_SPI_BUS_LOCK_ISR_FUNCS_IN_IRAM=y
# default:
CONFIG_ESP_INTR_IN_IRAM=y
# end of Hardware Settings

#
# ESP-Driver:LCD Controller Configurations
#
# default:
# CONFIG_LCD_ENABLE_DEBUG_LOG is not set
# end of ESP-Driver:LCD Controller Configurations

#
# ESP-MM: Memory Management Configurations
#
# end of ESP-MM: Memory Management Configurations

#
# ESP NETIF Adapter
#
# default:
CONFIG_ESP_NETIF_IP_LOST_TIMER_INTERVAL=120
# default:
# CONFIG_ESP_NETIF_PROVIDE_CUSTOM_IMPLEMENTATION is not set
# default:
CONFIG_ESP_NETIF_TCPIP_LWIP=y
# default:
# CONFIG_ESP_NETIF_LOOPBACK is not set
# default:
CONFIG_ESP_NETIF_USES_TCPIP_WITH_BSD_API=y
# default:
CONFIG_ESP_NETIF_REPORT_DATA_TRAFFIC=y
# default:
# CONFIG_ESP_NETIF_RECEIVE_REPORT_ERRORS is not set
# default:
# CONFIG_ESP_NETIF_L2_TAP is not set
# default:
# CONFIG_ESP_NETIF_BRIDGE_EN is not set
# default:
# CONFIG_ESP_NETIF_SET_DNS_PER_DEFAULT_NETIF is not set
# end of ESP NETIF Adapter

#
# Partition API Configuration
#
# end of Partition API Configuration

#
# PHY
#
# default:
CONFIG_ESP_PHY_ENABLED=y
# default:
CONFIG_ESP_PHY_CALIBRATION_AND_DATA_STORAGE=y
# default:
# CONFIG_ESP_PHY_INIT_DATA_IN_PARTITION is not set
# default:
CONFIG_ESP_PHY_MAX_WIFI_TX_POWER=20
# default:
CONFIG_ESP_PHY_MAX_TX_POWER=20
# default:
# CONFIG_ESP_PHY_REDUCE_TX_POWER is not set
# default:
# CONFIG_ESP_PHY_ENABLE_CERT_TEST is not set
# default:
CONFIG_ESP_PHY_RF_CAL_PARTIAL=y
# default:
# CONFIG_ESP_PHY_RF_CAL_NONE is not set
# default:
# CONFIG_ESP_PHY_RF_CAL_FULL is not set
# default:
CONFIG_ESP_PHY_CALIBRATION_MODE=0
# default:
CONFIG_ESP_PHY_PLL_TRACK_PERIOD_MS=1000
# default:
# CONFIG_ESP_PHY_PLL_TRACK_DEBUG is not set
# default:
# CONFIG_ESP_PHY_RECORD_USED_TIME is not set
# default:
CONFIG_ESP_PHY_IRAM_OPT=y
# default:
# CONFIG_ESP_PHY_DEBUG is not set
# end of PHY

#
# Power Management
#
# default:
# CONFIG_PM_SLEEP_FUNC_IN_IRAM is not set
# default:
# CONFIG_PM_ENABLE is not set
# default:
# CONFIG_PM_SLP_IRAM_OPT is not set
# end of Power Management

#
# ESP PSRAM
#
# default:
# CONFIG_SPIRAM is not set
# end of ESP PSRAM

#
# ESP Ringbuf
#
# default:
# CONFIG_RINGBUF_PLACE_FUNCTIONS_INTO_FLASH is not set
# end of ESP Ringbuf

#
# ESP-ROM
#
# default:
CONFIG_ESP_ROM_PRINT_IN_IRAM=y
# end of ESP-ROM

#
# ESP Security Specific
#
# end of ESP Security Specific

#
# ESP System Settings
#
# default:
# CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_80 is not set
# default:
# CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_160 is not set
# default:
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y
# default:
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=240

#
# Memory
#
# default:
# CONFIG_ESP32_USE_FIXED_STATIC_RAM_SIZE is not set

#
# Non-backward compatible options
#
# default:
# CONFIG_ESP_SYSTEM_ESP32_SRAM1_REGION_AS_IRAM is not set
# end of Non-backward compatible options
# end of Memory

#
# Trace memory
#
# default:
# CONFIG_ESP32_TRAX is not set
# default:
CONFIG_ESP32_TRACEMEM_RESERVE_DRAM=0x0
# end of Trace memory

# default:
CONFIG_ESP_SYSTEM_IN_IRAM=y
# default:
# CONFIG_ESP_SYSTEM_PANIC_PRINT_HALT is not set
# default:
CONFIG_ESP_SYSTEM_PANIC_PRINT_REBOOT=y
# default:
# CONFIG_ESP_SYSTEM_PANIC_SILENT_REBOOT is not set
# default:
# CONFIG_ESP_SYSTEM_PANIC_GDBSTUB is not set
# default:
CONFIG_ESP_SYSTEM_PANIC_REBOOT_DELAY_SECONDS=0

#
# Memory protection
#
# end of Memory protection

# default:
CONFIG_ESP_SYSTEM_EVENT_QUEUE_SIZE=32
# default:
CONFIG_ESP_SYSTEM_EVENT_TASK_STACK_SIZE=2304
# default:
CONFIG_ESP_MAIN_TASK_STACK_SIZE=3584
# default:
CONFIG_ESP_MAIN_TASK_AFFINITY_CPU0=y
# default:
# CONFIG_ESP_MAIN_TASK_AFFINITY_CPU1 is not set
# default:
# CONFIG_ESP_MAIN_TASK_AFFINITY_NO_AFFINITY is not set
# default:
CONFIG_ESP_MAIN_TASK_AFFINITY=0x0
# default:
CONFIG_ESP_MINIMAL_SHARED_STACK_SIZE=2048
# default:
CONFIG_ESP_CONSOLE_UART_DEFAULT=y
# default:
# CONFIG_ESP_CONSOLE_UART_CUSTOM is not set
# default:
# CONFIG_ESP_CONSOLE_NONE is not set
# default:
CONFIG_ESP_CONSOLE_UART=y
# default:
CONFIG_ESP_CONSOLE_UART_NUM=0
# default:
CONFIG_ESP_CONSOLE_ROM_SERIAL_PORT_NUM=0
# default:
CONFIG_ESP_CONSOLE_UART_BAUDRATE=115200
# default:
CONFIG_ESP_INT_WDT=y
# default:
CONFIG_ESP_INT_WDT_TIMEOUT_MS=300
# default:
CONFIG_ESP_INT_WDT_CHECK_CPU1=y
# default:
# CONFIG_ESP_TASK_WDT_EN is not set
# default:
# CONFIG_ESP_PANIC_HANDLER_IRAM is not set
# default:
# CONFIG_ESP_DEBUG_STUBS_ENABLE is not set
# default:
CONFIG_ESP_DEBUG_OCDAWARE=y
# default:
# CONFIG_ESP_SYSTEM_CHECK_INT_LEVEL_5 is not set
# default:
CONFIG_ESP_SYSTEM_CHECK_INT_LEVEL_4=y
# default:
# CONFIG_ESP32_DISABLE_BASIC_ROM_CONSOLE is not set
# end of ESP System Settings

#
# IPC (Inter-Processor Call)
#
# default:
CONFIG_ESP_IPC_ENABLE=y
# default:
CONFIG_ESP_IPC_TASK_STACK_SIZE=1024
# default:
CONFIG_ESP_IPC_USES_CALLERS_PRIORITY=y
# default:
CONFIG_ESP_IPC_ISR_ENABLE=y
# end of IPC (Inter-Processor Call)

#
# ESP Timer (High Resolution Timer)
#
# default:
CONFIG_ESP_TIMER_IN_IRAM=y
# default:
# CONFIG_ESP_TIMER_PROFILING is not set
# default:
CONFIG_ESP_TIME_FUNCS_USE_RTC_TIMER=y
# default:
CONFIG_ESP_TIME_FUNCS_USE_ESP_TIMER=y
# default:
CONFIG_ESP_TIMER_TASK_STACK_SIZE=3584
# default:
CONFIG_ESP_TIMER_INTERRUPT_LEVEL=1
# default:
# CONFIG_ESP_TIMER_SHOW_EXPERIMENTAL is not set
# default:
CONFIG_ESP_TIMER_TASK_AFFINITY=0x0
# default:
CONFIG_ESP_TIMER_TASK_AFFINITY_CPU0=y
# default:
CONFIG_ESP_TIMER_ISR_AFFINITY_CPU0=y
# default:
# CONFIG_ESP_TIMER_SUPPORTS_ISR_DISPATCH_METHOD is not set
# default:
CONFIG_ESP_TIMER_IMPL_TG0_LAC=y
# end of ESP Timer (High Resolution Timer)

#
# Wi-Fi
#
# default:
CONFIG_ESP_WIFI_ENABLED=y
# default:
CONFIG_ESP_WIFI_STATIC_RX_BUFFER_NUM=10
# default:
CONFIG_ESP_WIFI_DYNAMIC_RX_BUFFER_NUM=32
# default:
# CONFIG_ESP_WIFI_STATIC_TX_BUFFER is not set
# default:
CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER=y
# default:
CONFIG_ESP_WIFI_TX_BUFFER_TYPE=1
# default:
CONFIG_ESP_WIFI_DYNAMIC_TX_BUFFER_NUM=32
# default:
CONFIG_ESP_WIFI_STATIC_RX_MGMT_BUFFER=y
# default:
# CONFIG_ESP_WIFI_DYNAMIC_RX_MGMT_BUFFER is not set
# default:
CONFIG_ESP_WIFI_DYNAMIC_RX_MGMT_BUF=0
# default:
CONFIG_ESP_WIFI_RX_MGMT_BUF_NUM_DEF=5
# default:
# CONFIG_ESP_WIFI_CSI_ENABLED is not set
# default:
CONFIG_ESP_WIFI_AMPDU_TX_ENABLED=y
# default:
CONFIG_ESP_WIFI_TX_BA_WIN=6
# default:
CONFIG_ESP_WIFI_AMPDU_RX_ENABLED=y
# default:
CONFIG_ESP_WIFI_RX_BA_WIN=6
# default:
CONFIG_ESP_WIFI_NVS_ENABLED=y
# default:
CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0=y
# default:
# CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_1 is not set
# default:
CONFIG_ESP_WIFI_SOFTAP_BEACON_MAX_LEN=752
# default:
CONFIG_ESP_WIFI_MGMT_SBUF_NUM=32
# default:
CONFIG_ESP_WIFI_IRAM_OPT=y
# default:
# CONFIG_ESP_WIFI_EXTRA_IRAM_OPT is not set
# default:
CONFIG_ESP_WIFI_RX_IRAM_OPT=y
# default:
CONFIG_ESP_WIFI_ENABLE_WPA3_SAE=y
# default:
CONFIG_ESP_WIFI_ENABLE_SAE_PK=y
# default:
CONFIG_ESP_WIFI_ENABLE_SAE_H2E=y
# default:
CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT=y
# default:
CONFIG_ESP_WIFI_ENABLE_WPA3_OWE_STA=y
# default:
# CONFIG_ESP_WIFI_SLP_IRAM_OPT is not set
# default:
CONFIG_ESP_WIFI_SLP_DEFAULT_MIN_ACTIVE_TIME=50
# default:
# CONFIG_ESP_WIFI_BSS_MAX_IDLE_SUPPORT is not set
# default:
CONFIG_ESP_WIFI_SLP_DEFAULT_MAX_ACTIVE_TIME=10
# default:
CONFIG_ESP_WIFI_SLP_DEFAULT_WAIT_BROADCAST_DATA_TIME=15
# default:
CONFIG_ESP_WIFI_STA_DISCONNECTED_PM_ENABLE=y
# default:
CONFIG_ESP_WIFI_GMAC_SUPPORT=y
# default:
CONFIG_ESP_WIFI_SOFTAP_SUPPORT=y
# default:
# CONFIG_ESP_WIFI_SLP_BEACON_LOST_OPT is not set
# default:
CONFIG_ESP_WIFI_ESPNOW_MAX_ENCRYPT_NUM=7
# default:
# CONFIG_ESP_WIFI_NAN_ENABLE is not set
# default:
CONFIG_ESP_WIFI_MBEDTLS_CRYPTO=y
# default:
CONFIG_ESP_WIFI_MBEDTLS_TLS_CLIENT=y
# default:
# CONFIG_ESP_WIFI_WAPI_PSK is not set
# default:
# CONFIG_ESP_WIFI_11KV_SUPPORT is not set
# default:
# CONFIG_ESP_WIFI_MBO_SUPPORT is not set
# default:
# CONFIG_ESP_WIFI_DPP_SUPPORT is not set
# default:
# CONFIG_ESP_WIFI_11R_SUPPORT is not set
# default:
# CONFIG_ESP_WIFI_WPS_SOFTAP_REGISTRAR is not set

#
# WPS Configuration Options
#
# default:
# CONFIG_ESP_WIFI_WPS_STRICT is not set
# default:
# CONFIG_ESP_WIFI_WPS_PASSPHRASE is not set
# default:
# CONFIG_ESP_WIFI_WPS_RECONNECT_ON_FAIL is not set
# end of WPS Configuration Options

# default:
# CONFIG_ESP_WIFI_DEBUG_PRINT is not set
# default:
CONFIG_ESP_WIFI_ENTERPRISE_SUPPORT=y
# default:
# CONFIG_ESP_WIFI_ENT_FREE_DYNAMIC_BUFFER is not set
# end of Wi-Fi

#
# Core dump
#
# default:
# CONFIG_ESP_COREDUMP_ENABLE_TO_FLASH is not set
# default:
# CONFIG_ESP_COREDUMP_ENABLE_TO_UART is not set
# default:
CONFIG_ESP_COREDUMP_ENABLE_TO_NONE=y
# end of Core dump

#
# FAT Filesystem support
#
# default:
CONFIG_FATFS_VOLUME_COUNT=2
# default:
CONFIG_FATFS_LFN_NONE=y
# default:
# CONFIG_FATFS_LFN_HEAP is not set
# default:
# CONFIG_FATFS_LFN_STACK is not set
# default:
# CONFIG_FATFS_SECTOR_512 is not set
# default:
CONFIG_FATFS_SECTOR_4096=y
# default:
# CONFIG_FATFS_CODEPAGE_DYNAMIC is not set
# default:
CONFIG_FATFS_CODEPAGE_437=y
# default:
# CONFIG_FATFS_CODEPAGE_720 is not set
# default:
# CONFIG_FATFS_CODEPAGE_737 is not set
# default:
# CONFIG_FATFS_CODEPAGE_771 is not set
# default:
# CONFIG_FATFS_CODEPAGE_775 is not set
# default:
# CONFIG_FATFS_CODEPAGE_850 is not set
# default:
# CONFIG_FATFS_CODEPAGE_852 is not set
# default:
# CONFIG_FATFS_CODEPAGE_855 is not set
# default:
# CONFIG_FATFS_CODEPAGE_857 is not set
# default:
# CONFIG_FATFS_CODEPAGE_860 is not set
# default:
# CONFIG_FATFS_CODEPAGE_861 is not set
# default:
# CONFIG_FATFS_CODEPAGE_862 is not set
# default:
# CONFIG_FATFS_CODEPAGE_863 is not set
# default:
# CONFIG_FATFS_CODEPAGE_864 is not set
# default:
# CONFIG_FATFS_CODEPAGE_865 is not set
# default:
# CONFIG_FATFS_CODEPAGE_866 is not set
# default:
# CONFIG_FATFS_CODEPAGE_869 is not set
# default:
# CONFIG_FATFS_CODEPAGE_932 is not set
# default:
# CONFIG_FATFS_CODEPAGE_936 is not set
# default:
# CONFIG_FATFS_CODEPAGE_949 is not set
# default:
# CONFIG_FATFS_CODEPAGE_950 is not set
# default:
CONFIG_FATFS_CODEPAGE=437
# default:
CONFIG_FATFS_FS_LOCK=0
# default:
CONFIG_FATFS_TIMEOUT_MS=10000
# default:
CONFIG_FATFS_PER_FILE_CACHE=y
# default:
# CONFIG_FATFS_USE_FASTSEEK is not set
# default:
CONFIG_FATFS_USE_STRFUNC_NONE=y
# default:
# CONFIG_FATFS_USE_STRFUNC_WITHOUT_CRLF_CONV is not set
# default:
# CONFIG_FATFS_USE_STRFUNC_WITH_CRLF_CONV is not set
# default:
CONFIG_FATFS_VFS_FSTAT_BLKSIZE=0
# default:
# CONFIG_FATFS_IMMEDIATE_FSYNC is not set
# default:
# CONFIG_FATFS_USE_LABEL is not set
# default:
CONFIG_FATFS_LINK_LOCK=y
# default:
# CONFIG_FATFS_USE_DYN_BUFFERS is not set

#
# File system free space calculation behavior
#
# default:
CONFIG_FATFS_DONT_TRUST_FREE_CLUSTER_CNT=0
# default:
CONFIG_FATFS_DONT_TRUST_LAST_ALLOC=0
# end of File system free space calculation behavior
# end of FAT Filesystem support

#
# FreeRTOS
#

#
# Kernel
#
# default:
# CONFIG_FREERTOS_SMP is not set
# default:
# CONFIG_FREERTOS_UNICORE is not set
# default:
CONFIG_FREERTOS_HZ=100
# default:
# CONFIG_FREERTOS_CHECK_STACKOVERFLOW_NONE is not set
# default:
# CONFIG_FREERTOS_CHECK_STACKOVERFLOW_PTRVAL is not set
# default:
CONFIG_FREERTOS_CHECK_STACKOVERFLOW_CANARY=y
# default:
CONFIG_FREERTOS_THREAD_LOCAL_STORAGE_POINTERS=1
# default:
CONFIG_FREERTOS_IDLE_TASK_STACKSIZE=1536
# default:
# CONFIG_FREERTOS_USE_IDLE_HOOK is not set
# default:
# CONFIG_FREERTOS_USE_TICK_HOOK is not set
# default:
CONFIG_FREERTOS_MAX_TASK_NAME_LEN=16
# default:
# CONFIG_FREERTOS_ENABLE_BACKWARD_COMPATIBILITY is not set
# default:
CONFIG_FREERTOS_USE_TIMERS=y
# default:
CONFIG_FREERTOS_TIMER_SERVICE_TASK_NAME="Tmr Svc"
# default:
# CONFIG_FREERTOS_TIMER_TASK_AFFINITY_CPU0 is not set
# default:
# CONFIG_FREERTOS_TIMER_TASK_AFFINITY_CPU1 is not set
# default:
CONFIG_FREERTOS_TIMER_TASK_NO_AFFINITY=y
# default:
CONFIG_FREERTOS_TIMER_SERVICE_TASK_CORE_AFFINITY=0x7FFFFFFF
# default:
CONFIG_FREERTOS_TIMER_TASK_PRIORITY=1
# default:
CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH=2048
# default:
CONFIG_FREERTOS_TIMER_QUEUE_LENGTH=10
# default:
CONFIG_FREERTOS_QUEUE_REGISTRY_SIZE=0
# default:
CONFIG_FREERTOS_TASK_NOTIFICATION_ARRAY_ENTRIES=1
# default:
# CONFIG_FREERTOS_USE_TRACE_FACILITY is not set
# default:
# CONFIG_FREERTOS_USE_LIST_DATA_INTEGRITY_CHECK_BYTES is not set
# default:
# CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS is not set
# default:
# CONFIG_FREERTOS_USE_APPLICATION_TASK_TAG is not set
# end of Kernel

#
# Port
#
# default:
CONFIG_FREERTOS_TASK_FUNCTION_WRAPPER=y
# default:
# CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK is not set
# default:
CONFIG_FREERTOS_TLSP_DELETION_CALLBACKS=y
# default:
# CONFIG_FREERTOS_TASK_PRE_DELETION_HOOK is not set
# default:
# CONFIG_FREERTOS_ENABLE_STATIC_TASK_CLEAN_UP is not set
# default:
CONFIG_FREERTOS_CHECK_MUTEX_GIVEN_BY_OWNER=y
# default:
CONFIG_FREERTOS_ISR_STACKSIZE=1536
# default:
CONFIG_FREERTOS_INTERRUPT_BACKTRACE=y
# default:
# CONFIG_FREERTOS_FPU_IN_ISR is not set
# default:
CONFIG_FREERTOS_TICK_SUPPORT_CORETIMER=y
# default:
CONFIG_FREERTOS_CORETIMER_0=y
# default:
# CONFIG_FREERTOS_CORETIMER_1 is not set
# default:
CONFIG_FREERTOS_SYSTICK_USES_CCOUNT=y
# default:
# CONFIG_FREERTOS_PLACE_FUNCTIONS_INTO_FLASH is not set
# default:
# CONFIG_FREERTOS_CHECK_PORT_CRITICAL_COMPLIANCE is not set
# end of Port

#
# Extra
#
# end of Extra

# default:
CONFIG_FREERTOS_PORT=y
# default:
CONFIG_FREERTOS_NO_AFFINITY=0x7FFFFFFF
# default:
CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION=y
# default:
CONFIG_FREERTOS_DEBUG_OCDAWARE=y
# default:
CONFIG_FREERTOS_ENABLE_TASK_SNAPSHOT=y
# default:
CONFIG_FREERTOS_PLACE_SNAPSHOT_FUNS_INTO_FLASH=y
# default:
CONFIG_FREERTOS_NUMBER_OF_CORES=2
# default:
CONFIG_FREERTOS_IN_IRAM=y
# end of FreeRTOS

#
# Hardware Abstraction Layer (HAL) and Low Level (LL)
#
# default:
CONFIG_HAL_ASSERTION_EQUALS_SYSTEM=y
# default:
# CONFIG_HAL_ASSERTION_DISABLE is not set
# default:
# CONFIG_HAL_ASSERTION_SILENT is not set
# default:
# CONFIG_HAL_ASSERTION_ENABLE is not set
# default:
CONFIG_HAL_DEFAULT_ASSERTION_LEVEL=2
# end of Hardware Abstraction Layer (HAL) and Low Level (LL)

#
# Heap memory debugging
#
# default:
CONFIG_HEAP_POISONING_DISABLED=y
# default:
# CONFIG_HEAP_POISONING_LIGHT is not set
# default:
# CONFIG_HEAP_POISONING_COMPREHENSIVE is not set
# default:
CONFIG_HEAP_TRACING_OFF=y
# default:
# CONFIG_HEAP_TRACING_STANDALONE is not set
# default:
# CONFIG_HEAP_TRACING_TOHOST is not set
# default:
# CONFIG_HEAP_USE_HOOKS is not set
# default:
# CONFIG_HEAP_TASK_TRACKING is not set
# default:
# CONFIG_HEAP_ABORT_WHEN_ALLOCATION_FAILS is not set
# default:
# CONFIG_HEAP_PLACE_FUNCTION_INTO_FLASH is not set
# end of Heap memory debugging

#
# Log
#
# default:
CONFIG_LOG_VERSION_1=y
# default:
# CONFIG_LOG_VERSION_2 is not set
# default:
CONFIG_LOG_VERSION=1

#
# Log Level
#
# default:
# CONFIG_LOG_DEFAULT_LEVEL_NONE is not set
# default:
# CONFIG_LOG_DEFAULT_LEVEL_ERROR is not set
# default:
# CONFIG_LOG_DEFAULT_LEVEL_WARN is not set
# default:
CONFIG_LOG_DEFAULT_LEVEL_INFO=y
# default:
# CONFIG_LOG_DEFAULT_LEVEL_DEBUG is not set
# default:
# CONFIG_LOG_DEFAULT_LEVEL_VERBOSE is not set
# default:
CONFIG_LOG_DEFAULT_LEVEL=3
# default:
CONFIG_LOG_MAXIMUM_EQUALS_DEFAULT=y
# default:
# CONFIG_LOG_MAXIMUM_LEVEL_DEBUG is not set
# default:
# CONFIG_LOG_MAXIMUM_LEVEL_VERBOSE is not set
# default:
CONFIG_LOG_MAXIMUM_LEVEL=3

#
# Level Settings
#
# default:
# CONFIG_LOG_MASTER_LEVEL is not set
# default:
CONFIG_LOG_DYNAMIC_LEVEL_CONTROL=y
# default:
# CONFIG_LOG_TAG_LEVEL_IMPL_NONE is not set
# default:
# CONFIG_LOG_TAG_LEVEL_IMPL_LINKED_LIST is not set
# default:
CONFIG_LOG_TAG_LEVEL_IMPL_CACHE_AND_LINKED_LIST=y
# default:
# CONFIG_LOG_TAG_LEVEL_CACHE_ARRAY is not set
# default:
CONFIG_LOG_TAG_LEVEL_CACHE_BINARY_MIN_HEAP=y
# default:
CONFIG_LOG_TAG_LEVEL_IMPL_CACHE_SIZE=31
# end of Level Settings
# end of Log Level

#
# Format
#
# default:
# CONFIG_LOG_COLORS is not set
# default:
CONFIG_LOG_TIMESTAMP_SOURCE_RTOS=y
# default:
# CONFIG_LOG_TIMESTAMP_SOURCE_SYSTEM is not set
# end of Format

#
# Settings
#
# default:
CONFIG_LOG_MODE_TEXT_EN=y
# default:
CONFIG_LOG_MODE_TEXT=y
# end of Settings

# default:
CONFIG_LOG_IN_IRAM=y
# end of Log

#
# LWIP
#
# default:
CONFIG_LWIP_ENABLE=y
# default:
CONFIG_LWIP_LOCAL_HOSTNAME="espressif"
# default:
CONFIG_LWIP_TCPIP_TASK_PRIO=18
# default:
# CONFIG_LWIP_TCPIP_CORE_LOCKING is not set
# default:
# CONFIG_LWIP_CHECK_THREAD_SAFETY is not set
# default:
CONFIG_LWIP_DNS_SUPPORT_MDNS_QUERIES=y
# default:
# CONFIG_LWIP_L2_TO_L3_COPY is not set
# default:
# CONFIG_LWIP_IRAM_OPTIMIZATION is not set
# default:
# CONFIG_LWIP_EXTRA_IRAM_OPTIMIZATION is not set
# default:
CONFIG_LWIP_TIMERS_ONDEMAND=y
# default:
CONFIG_LWIP_ND6=y
# default:
# CONFIG_LWIP_FORCE_ROUTER_FORWARDING is not set
# default:
CONFIG_LWIP_MAX_SOCKETS=10
# default:
# CONFIG_LWIP_USE_ONLY_LWIP_SELECT is not set
# default:
# CONFIG_LWIP_SO_LINGER is not set
# default:
CONFIG_LWIP_SO_REUSE=y
# default:
CONFIG_LWIP_SO_REUSE_RXTOALL=y
# default:
# CONFIG_LWIP_SO_RCVBUF is not set
# default:
# CONFIG_LWIP_NETBUF_RECVINFO is not set
# default:
CONFIG_LWIP_IP_DEFAULT_TTL=64
# default:
CONFIG_LWIP_IP4_FRAG=y
# default:
CONFIG_LWIP_IP6_FRAG=y
# default:
# CONFIG_LWIP_IP4_REASSEMBLY is not set
# default:
# CONFIG_LWIP_IP6_REASSEMBLY is not set
# default:
CONFIG_LWIP_IP_REASS_MAX_PBUFS=10
# default:
# CONFIG_LWIP_IP_FORWARD is not set
# default:
# CONFIG_LWIP_STATS is not set
# default:
CONFIG_LWIP_ESP_GRATUITOUS_ARP=y
# default:
CONFIG_LWIP_GARP_TMR_INTERVAL=60
# default:
CONFIG_LWIP_ESP_MLDV6_REPORT=y
# default:
CONFIG_LWIP_MLDV6_TMR_INTERVAL=40
# default:
CONFIG_LWIP_TCPIP_RECVMBOX_SIZE=32
# default:
CONFIG_LWIP_DHCP_DOES_ARP_CHECK=y
# default:
# CONFIG_LWIP_DHCP_DOES_ACD_CHECK is not set
# default:
# CONFIG_LWIP_DHCP_DOES_NOT_CHECK_OFFERED_IP is not set
# default:
# CONFIG_LWIP_DHCP_DISABLE_CLIENT_ID is not set
# default:
CONFIG_LWIP_DHCP_DISABLE_VENDOR_CLASS_ID=y
# default:
# CONFIG_LWIP_DHCP_RESTORE_LAST_IP is not set
# default:
CONFIG_LWIP_DHCP_OPTIONS_LEN=69
# default:
CONFIG_LWIP_NUM_NETIF_CLIENT_DATA=0
# default:
CONFIG_LWIP_DHCP_COARSE_TIMER_SECS=1

#
# DHCP server
#
# default:
CONFIG_LWIP_DHCPS=y
# default:
CONFIG_LWIP_DHCPS_LEASE_UNIT=60
# default:
CONFIG_LWIP_DHCPS_MAX_STATION_NUM=8
# default:
CONFIG_LWIP_DHCPS_STATIC_ENTRIES=y
# default:
CONFIG_LWIP_DHCPS_ADD_DNS=y
# end of DHCP server

# default:
# CONFIG_LWIP_AUTOIP is not set
# default:
CONFIG_LWIP_IPV4=y
# default:
CONFIG_LWIP_IPV6=y
# default:
# CONFIG_LWIP_IPV6_AUTOCONFIG is not set
# default:
CONFIG_LWIP_IPV6_NUM_ADDRESSES=3
# default:
# CONFIG_LWIP_IPV6_FORWARD is not set
# default:
# CONFIG_LWIP_NETIF_STATUS_CALLBACK is not set
# default:
CONFIG_LWIP_NETIF_LOOPBACK=y
# default:
CONFIG_LWIP_LOOPBACK_MAX_PBUFS=8

#
# TCP
#
# default:
CONFIG_LWIP_MAX_ACTIVE_TCP=16
# default:
CONFIG_LWIP_MAX_LISTENING_TCP=16
# default:
CONFIG_LWIP_TCP_HIGH_SPEED_RETRANSMISSION=y
# default:
CONFIG_LWIP_TCP_MAXRTX=12
# default:
CONFIG_LWIP_TCP_SYNMAXRTX=12
# default:
CONFIG_LWIP_TCP_MSS=1440
# default:
CONFIG_LWIP_TCP_TMR_INTERVAL=250
# default:
CONFIG_LWIP_TCP_MSL=60000
# default:
CONFIG_LWIP_TCP_FIN_WAIT_TIMEOUT=20000
# default:
CONFIG_LWIP_TCP_SND_BUF_DEFAULT=5760
# default:
CONFIG_LWIP_TCP_WND_DEFAULT=5760
# default:
CONFIG_LWIP_TCP_RECVMBOX_SIZE=6
# default:
CONFIG_LWIP_TCP_ACCEPTMBOX_SIZE=6
# default:
CONFIG_LWIP_TCP_QUEUE_OOSEQ=y
# default:
CONFIG_LWIP_TCP_OOSEQ_TIMEOUT=6
# default:
CONFIG_LWIP_TCP_OOSEQ_MAX_PBUFS=4
# default:
# CONFIG_LWIP_TCP_SACK_OUT is not set
# default:
CONFIG_LWIP_TCP_OVERSIZE_MSS=y
# default:
# CONFIG_LWIP_TCP_OVERSIZE_QUARTER_MSS is not set
# default:
# CONFIG_LWIP_TCP_OVERSIZE_DISABLE is not set
# default:
CONFIG_LWIP_TCP_RTO_TIME=1500
# end of TCP

#
# UDP
#
# default:
CONFIG_LWIP_MAX_UDP_PCBS=16
# default:
CONFIG_LWIP_UDP_RECVMBOX_SIZE=6
# end of UDP

#
# Checksums
#
# default:
# CONFIG_LWIP_CHECKSUM_CHECK_IP is not set
# default:
# CONFIG_LWIP_CHECKSUM_CHECK_UDP is not set
# default:
CONFIG_LWIP_CHECKSUM_CHECK_ICMP=y
# end of Checksums

# default:
CONFIG_LWIP_TCPIP_TASK_STACK_SIZE=3072
# default:
CONFIG_LWIP_TCPIP_TASK_AFFINITY_NO_AFFINITY=y
# default:
# CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0 is not set
# default:
# CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU1 is not set
# default:
CONFIG_LWIP_TCPIP_TASK_AFFINITY=0x7FFFFFFF
# default:
CONFIG_LWIP_IPV6_MEMP_NUM_ND6_QUEUE=3
# default:
CONFIG_LWIP_IPV6_ND6_NUM_NEIGHBORS=5
# default:
CONFIG_LWIP_IPV6_ND6_NUM_PREFIXES=5
# default:
CONFIG_LWIP_IPV6_ND6_NUM_ROUTERS=3
# default:
CONFIG_LWIP_IPV6_ND6_NUM_DESTINATIONS=10
# default:
# CONFIG_LWIP_IPV6_ND6_ROUTE_INFO_OPTION_SUPPORT is not set
# default:
# CONFIG_LWIP_PPP_SUPPORT is not set
# default:
# CONFIG_LWIP_SLIP_SUPPORT is not set

#
# ICMP
#
# default:
CONFIG_LWIP_ICMP=y
# default:
# CONFIG_LWIP_MULTICAST_PING is not set
# default:
# CONFIG_LWIP_BROADCAST_PING is not set
# end of ICMP

#
# LWIP RAW API
#
# default:
CONFIG_LWIP_MAX_RAW_PCBS=16
# end of LWIP RAW API

#
# SNTP
#
# default:
CONFIG_LWIP_SNTP_MAX_SERVERS=1
# default:
# CONFIG_LWIP_DHCP_GET_NTP_SRV is not set
# default:
CONFIG_LWIP_SNTP_UPDATE_DELAY=3600000
# default:
CONFIG_LWIP_SNTP_STARTUP_DELAY=y
# default:
CONFIG_LWIP_SNTP_MAXIMUM_STARTUP_DELAY=5000
# end of SNTP

#
# DNS
#
# default:
CONFIG_LWIP_DNS_MAX_HOST_IP=1
# default:
CONFIG_LWIP_DNS_MAX_SERVERS=3
# default:
# CONFIG_LWIP_FALLBACK_DNS_SERVER_SUPPORT is not set
# default:
# CONFIG_LWIP_DNS_SETSERVER_WITH_NETIF is not set
# default:
# CONFIG_LWIP_USE_ESP_GETADDRINFO is not set
# end of DNS

# default:
CONFIG_LWIP_BRIDGEIF_MAX_PORTS=7
# default:
CONFIG_LWIP_ESP_LWIP_ASSERT=y

#
# Hooks
#
# default:
# CONFIG_LWIP_HOOK_TCP_ISN_NONE is not set
# default:
CONFIG_LWIP_HOOK_TCP_ISN_DEFAULT=y
# default:
# CONFIG_LWIP_HOOK_TCP_ISN_CUSTOM is not set
# default:
CONFIG_LWIP_HOOK_IP6_ROUTE_NONE=y
# default:
# CONFIG_LWIP_HOOK_IP6_ROUTE_DEFAULT is not set
# default:
# CONFIG_LWIP_HOOK_IP6_ROUTE_CUSTOM is not set
# default:
CONFIG_LWIP_HOOK_ND6_GET_GW_NONE=y
# default:
# CONFIG_LWIP_HOOK_ND6_GET_GW_DEFAULT is not set
# default:
# CONFIG_LWIP_HOOK_ND6_GET_GW_CUSTOM is not set
# default:
CONFIG_LWIP_HOOK_IP6_SELECT_SRC_ADDR_NONE=y
# default:
# CONFIG_LWIP_HOOK_IP6_SELECT_SRC_ADDR_DEFAULT is not set
# default:
# CONFIG_LWIP_HOOK_IP6_SELECT_SRC_ADDR_CUSTOM is not set
# default:
CONFIG_LWIP_HOOK_DHCP_EXTRA_OPTION_NONE=y
# default:
# CONFIG_LWIP_HOOK_DHCP_EXTRA_OPTION_DEFAULT is not set
# default:
# CONFIG_LWIP_HOOK_DHCP_EXTRA_OPTION_CUSTOM is not set
# default:
CONFIG_LWIP_HOOK_NETCONN_EXT_RESOLVE_NONE=y
# default:
# CONFIG_LWIP_HOOK_NETCONN_EXT_RESOLVE_DEFAULT is not set
# default:
# CONFIG_LWIP_HOOK_NETCONN_EXT_RESOLVE_CUSTOM is not set
# default:
CONFIG_LWIP_HOOK_DNS_EXT_RESOLVE_NONE=y
# default:
# CONFIG_LWIP_HOOK_DNS_EXT_RESOLVE_CUSTOM is not set
# default:
# CONFIG_LWIP_HOOK_IP6_INPUT_NONE is not set
# default:
CONFIG_LWIP_HOOK_IP6_INPUT_DEFAULT=y
# default:
# CONFIG_LWIP_HOOK_IP6_INPUT_CUSTOM is not set
# end of Hooks

# default:
# CONFIG_LWIP_DEBUG is not set
# end of LWIP

#
# mbedTLS
#
# default:
CONFIG_MBEDTLS_INTERNAL_MEM_ALLOC=y
# default:
# CONFIG_MBEDTLS_DEFAULT_MEM_ALLOC is not set
# default:
# CONFIG_MBEDTLS_CUSTOM_MEM_ALLOC is not set
# default:
CONFIG_MBEDTLS_ASYMMETRIC_CONTENT_LEN=y
# default:
CONFIG_MBEDTLS_SSL_IN_CONTENT_LEN=16384
# default:
CONFIG_MBEDTLS_SSL_OUT_CONTENT_LEN=4096
# default:
# CONFIG_MBEDTLS_DYNAMIC_BUFFER is not set
# default:
# CONFIG_MBEDTLS_DEBUG is not set

#
# mbedTLS v3.x related
#
# default:
# CONFIG_MBEDTLS_SSL_PROTO_TLS1_3 is not set
# default:
# CONFIG_MBEDTLS_SSL_VARIABLE_BUFFER_LENGTH is not set
# default:
# CONFIG_MBEDTLS_X509_TRUSTED_CERT_CALLBACK is not set
# default:
# CONFIG_MBEDTLS_SSL_CONTEXT_SERIALIZATION is not set
# default:
CONFIG_MBEDTLS_SSL_KEEP_PEER_CERTIFICATE=y
# default:
# CONFIG_MBEDTLS_SSL_KEYING_MATERIAL_EXPORT is not set
# default:
CONFIG_MBEDTLS_PKCS7_C=y
# end of mbedTLS v3.x related

#
# Certificate Bundle
#
# default:
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE=y
# default:
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_DEFAULT_FULL=y
# default:
# CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_DEFAULT_CMN is not set
# default:
# CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_DEFAULT_NONE is not set
# default:
# CONFIG_MBEDTLS_CUSTOM_CERTIFICATE_BUNDLE is not set
# default:
# CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_DEPRECATED_LIST is not set
# default:
CONFIG_MBEDTLS_CERTIFICATE_BUNDLE_MAX_CERTS=200
# end of Certificate Bundle

# default:
# CONFIG_MBEDTLS_ECP_RESTARTABLE is not set
# default:
CONFIG_MBEDTLS_CMAC_C=y
# default:
CONFIG_MBEDTLS_HARDWARE_AES=y
# default:
CONFIG_MBEDTLS_GCM_SUPPORT_NON_AES_CIPHER=y
# default:
CONFIG_MBEDTLS_HARDWARE_MPI=y
# default:
# CONFIG_MBEDTLS_LARGE_KEY_SOFTWARE_MPI is not set
# default:
CONFIG_MBEDTLS_HARDWARE_SHA=y
# default:
CONFIG_MBEDTLS_ROM_MD5=y
# default:
# CONFIG_MBEDTLS_ATCA_HW_ECDSA_SIGN is not set
# default:
# CONFIG_MBEDTLS_ATCA_HW_ECDSA_VERIFY is not set
# default:
CONFIG_MBEDTLS_HAVE_TIME=y
# default:
# CONFIG_MBEDTLS_PLATFORM_TIME_ALT is not set
# default:
# CONFIG_MBEDTLS_HAVE_TIME_DATE is not set
# default:
CONFIG_MBEDTLS_ECDSA_DETERMINISTIC=y
# default:
CONFIG_MBEDTLS_SHA1_C=y
# default:
CONFIG_MBEDTLS_SHA512_C=y
# default:
# CONFIG_MBEDTLS_SHA3_C is not set
# default:
CONFIG_MBEDTLS_TLS_SERVER_AND_CLIENT=y
# default:
# CONFIG_MBEDTLS_TLS_SERVER_ONLY is not set
# default:
# CONFIG_MBEDTLS_TLS_CLIENT_ONLY is not set
# default:
# CONFIG_MBEDTLS_TLS_DISABLED is not set
# default:
CONFIG_MBEDTLS_TLS_SERVER=y
# default:
CONFIG_MBEDTLS_TLS_CLIENT=y
# default:
CONFIG_MBEDTLS_TLS_ENABLED=y

#
# TLS Key Exchange Methods
#
# default:
# CONFIG_MBEDTLS_PSK_MODES is not set
# default:
CONFIG_MBEDTLS_KEY_EXCHANGE_RSA=y
# default:
CONFIG_MBEDTLS_KEY_EXCHANGE_ELLIPTIC_CURVE=y
# default:
CONFIG_MBEDTLS_KEY_EXCHANGE_ECDHE_RSA=y
# default:
CONFIG_MBEDTLS_KEY_EXCHANGE_ECDHE_ECDSA=y
# default:
CONFIG_MBEDTLS_KEY_EXCHANGE_ECDH_ECDSA=y
# default:
CONFIG_MBEDTLS_KEY_EXCHANGE_ECDH_RSA=y
# end of TLS Key Exchange Methods

# default:
CONFIG_MBEDTLS_SSL_RENEGOTIATION=y
# default:
CONFIG_MBEDTLS_SSL_PROTO_TLS1_2=y
# default:
# CONFIG_MBEDTLS_SSL_PROTO_GMTSSL1_1 is not set
# default:
# CONFIG_MBEDTLS_SSL_PROTO_DTLS is not set
# default:
CONFIG_MBEDTLS_SSL_ALPN=y
# default:
CONFIG_MBEDTLS_CLIENT_SSL_SESSION_TICKETS=y
# default:
CONFIG_MBEDTLS_SERVER_SSL_SESSION_TICKETS=y

#
# Symmetric Ciphers
#
# default:
CONFIG_MBEDTLS_AES_C=y
# default:
# CONFIG_MBEDTLS_CAMELLIA_C is not set
# default:
# CONFIG_MBEDTLS_DES_C is not set
# default:
# CONFIG_MBEDTLS_BLOWFISH_C is not set
# default:
# CONFIG_MBEDTLS_XTEA_C is not set
# default:
CONFIG_MBEDTLS_CCM_C=y
# default:
CONFIG_MBEDTLS_GCM_C=y
# default:
# CONFIG_MBEDTLS_NIST_KW_C is not set
# end of Symmetric Ciphers

# default:
# CONFIG_MBEDTLS_RIPEMD160_C is not set

#
# Certificates
#
# default:
CONFIG_MBEDTLS_PEM_PARSE_C=y
# default:
CONFIG_MBEDTLS_PEM_WRITE_C=y
# default:
CONFIG_MBEDTLS_X509_CRL_PARSE_C=y
# default:
CONFIG_MBEDTLS_X509_CSR_PARSE_C=y
# end of Certificates

# default:
CONFIG_MBEDTLS_ECP_C=y
# default:
CONFIG_MBEDTLS_PK_PARSE_EC_EXTENDED=y
# default:
CONFIG_MBEDTLS_PK_PARSE_EC_COMPRESSED=y
# default:
# CONFIG_MBEDTLS_DHM_C is not set
# default:
CONFIG_MBEDTLS_ECDH_C=y
# default:
CONFIG_MBEDTLS_ECDSA_C=y
# default:
# CONFIG_MBEDTLS_ECJPAKE_C is not set
# default:
CONFIG_MBEDTLS_ECP_DP_SECP192R1_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_DP_SECP224R1_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_DP_SECP256R1_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_DP_SECP384R1_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_DP_SECP521R1_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_DP_SECP192K1_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_DP_SECP224K1_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_DP_SECP256K1_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_DP_BP256R1_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_DP_BP384R1_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_DP_BP512R1_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_DP_CURVE25519_ENABLED=y
# default:
CONFIG_MBEDTLS_ECP_NIST_OPTIM=y
# default:
# CONFIG_MBEDTLS_ECP_FIXED_POINT_OPTIM is not set
# default:
# CONFIG_MBEDTLS_POLY1305_C is not set
# default:
# CONFIG_MBEDTLS_CHACHA20_C is not set
# default:
# CONFIG_MBEDTLS_HKDF_C is not set
# default:
# CONFIG_MBEDTLS_THREADING_C is not set
# default:
CONFIG_MBEDTLS_ERROR_STRINGS=y
# default:
CONFIG_MBEDTLS_FS_IO=y
# default:
# CONFIG_MBEDTLS_ALLOW_WEAK_CERTIFICATE_VERIFICATION is not set
# end of mbedTLS

#
# ESP-MQTT Configurations
#
# default:
CONFIG_MQTT_PROTOCOL_311=y
# default:
# CONFIG_MQTT_PROTOCOL_5 is not set
# default:
CONFIG_MQTT_TRANSPORT_SSL=y
# default:
CONFIG_MQTT_TRANSPORT_WEBSOCKET=y
# default:
CONFIG_MQTT_TRANSPORT_WEBSOCKET_SECURE=y
# default:
# CONFIG_MQTT_MSG_ID_INCREMENTAL is not set
# default:
# CONFIG_MQTT_SKIP_PUBLISH_IF_DISCONNECTED is not set
# default:
# CONFIG_MQTT_REPORT_DELETED_MESSAGES is not set
# default:
# CONFIG_MQTT_USE_CUSTOM_CONFIG is not set
# default:
# CONFIG_MQTT_TASK_CORE_SELECTION_ENABLED is not set
# default:
# CONFIG_MQTT_CUSTOM_OUTBOX is not set
# end of ESP-MQTT Configurations

#
# LibC
#
# default:
CONFIG_LIBC_NEWLIB=y
# default:
CONFIG_LIBC_MISC_IN_IRAM=y
# default:
CONFIG_LIBC_LOCKS_PLACE_IN_IRAM=y
# default:
CONFIG_LIBC_STDOUT_LINE_ENDING_CRLF=y
# default:
# CONFIG_LIBC_STDOUT_LINE_ENDING_LF is not set
# default:
# CONFIG_LIBC_STDOUT_LINE_ENDING_CR is not set
# default:
# CONFIG_LIBC_STDIN_LINE_ENDING_CRLF is not set
# default:
# CONFIG_LIBC_STDIN_LINE_ENDING_LF is not set
# default:
CONFIG_LIBC_STDIN_LINE_ENDING_CR=y
# default:
# CONFIG_LIBC_NEWLIB_NANO_FORMAT is not set
# default:
CONFIG_LIBC_TIME_SYSCALL_USE_RTC_HRT=y
# default:
# CONFIG_LIBC_TIME_SYSCALL_USE_RTC is not set
# default:
# CONFIG_LIBC_TIME_SYSCALL_USE_HRT is not set
# default:
# CONFIG_LIBC_TIME_SYSCALL_USE_NONE is not set
# end of LibC

#
# NVS
#
# default:
# CONFIG_NVS_ASSERT_ERROR_CHECK is not set
# default:
# CONFIG_NVS_LEGACY_DUP_KEYS_COMPATIBILITY is not set
# end of NVS

#
# OpenThread
#
# default:
# CONFIG_OPENTHREAD_ENABLED is not set

#
# OpenThread Spinel
#
# default:
# CONFIG_OPENTHREAD_SPINEL_ONLY is not set
# end of OpenThread Spinel

# default:
# CONFIG_OPENTHREAD_DEBUG is not set
# end of OpenThread

#
# Protocomm
#
# default:
CONFIG_ESP_PROTOCOMM_SUPPORT_SECURITY_VERSION_0=y
# default:
CONFIG_ESP_PROTOCOMM_SUPPORT_SECURITY_VERSION_1=y
# default:
CONFIG_ESP_PROTOCOMM_SUPPORT_SECURITY_VERSION_2=y
# default:
CONFIG_ESP_PROTOCOMM_SUPPORT_SECURITY_PATCH_VERSION=y
# end of Protocomm

#
# PThreads
#
# default:
CONFIG_PTHREAD_TASK_PRIO_DEFAULT=5
# default:
CONFIG_PTHREAD_TASK_STACK_SIZE_DEFAULT=3072
# default:
CONFIG_PTHREAD_STACK_MIN=768
# default:
CONFIG_PTHREAD_DEFAULT_CORE_NO_AFFINITY=y
# default:
# CONFIG_PTHREAD_DEFAULT_CORE_0 is not set
# default:
# CONFIG_PTHREAD_DEFAULT_CORE_1 is not set
# default:
CONFIG_PTHREAD_TASK_CORE_DEFAULT=-1
# default:
CONFIG_PTHREAD_TASK_NAME_DEFAULT="pthread"
# end of PThreads

#
# MMU Config
#
# default:
CONFIG_MMU_PAGE_SIZE_64KB=y
# default:
CONFIG_MMU_PAGE_MODE="64KB"
# default:
CONFIG_MMU_PAGE_SIZE=0x10000
# end of MMU Config

#
# Main Flash configuration
#

#
# SPI Flash behavior when brownout
#
# default:
CONFIG_SPI_FLASH_BROWNOUT_RESET_XMC=y
# default:
CONFIG_SPI_FLASH_BROWNOUT_RESET=y
# end of SPI Flash behavior when brownout

#
# Optional and Experimental Features (READ DOCS FIRST)
#

#
# Features here require specific hardware (READ DOCS FIRST!)
#
# default:
CONFIG_SPI_FLASH_SUSPEND_TSUS_VAL_US=50
# default:
# CONFIG_SPI_FLASH_FORCE_ENABLE_XMC_C_SUSPEND is not set
# default:
# CONFIG_SPI_FLASH_FORCE_ENABLE_C6_H2_SUSPEND is not set
# default:
CONFIG_SPI_FLASH_PLACE_FUNCTIONS_IN_IRAM=y
# end of Optional and Experimental Features (READ DOCS FIRST)
# end of Main Flash configuration

#
# SPI Flash driver
#
# default:
# CONFIG_SPI_FLASH_VERIFY_WRITE is not set
# default:
# CONFIG_SPI_FLASH_ENABLE_COUNTERS is not set
# default:
CONFIG_SPI_FLASH_ROM_DRIVER_PATCH=y
# default:
CONFIG_SPI_FLASH_DANGEROUS_WRITE_ABORTS=y
# default:
# CONFIG_SPI_FLASH_DANGEROUS_WRITE_FAILS is not set
# default:
# CONFIG_SPI_FLASH_DANGEROUS_WRITE_ALLOWED is not set
# default:
# CONFIG_SPI_FLASH_SHARE_SPI1_BUS is not set
# default:
# CONFIG_SPI_FLASH_BYPASS_BLOCK_ERASE is not set
# default:
CONFIG_SPI_FLASH_YIELD_DURING_ERASE=y
# default:
CONFIG_SPI_FLASH_ERASE_YIELD_DURATION_MS=20
# default:
CONFIG_SPI_FLASH_ERASE_YIELD_TICKS=1
# default:
CONFIG_SPI_FLASH_WRITE_CHUNK_SIZE=8192
# default:
# CONFIG_SPI_FLASH_SIZE_OVERRIDE is not set
# default:
# CONFIG_SPI_FLASH_CHECK_ERASE_TIMEOUT_DISABLED is not set
# default:
# CONFIG_SPI_FLASH_OVERRIDE_CHIP_DRIVER_LIST is not set

#
# Auto-detect flash chips
#
# default:
CONFIG_SPI_FLASH_VENDOR_XMC_SUPPORT_ENABLED=y
# default:
CONFIG_SPI_FLASH_VENDOR_GD_SUPPORT_ENABLED=y
# default:
CONFIG_SPI_FLASH_VENDOR_ISSI_SUPPORT_ENABLED=y
# default:
CONFIG_SPI_FLASH_VENDOR_MXIC_SUPPORT_ENABLED=y
# default:
CONFIG_SPI_FLASH_VENDOR_WINBOND_SUPPORT_ENABLED=y
# default:
CONFIG_SPI_FLASH_SUPPORT_ISSI_CHIP=y
# default:
CONFIG_SPI_FLASH_SUPPORT_MXIC_CHIP=y
# default:
CONFIG_SPI_FLASH_SUPPORT_GD_CHIP=y
# default:
CONFIG_SPI_FLASH_SUPPORT_WINBOND_CHIP=y
# default:
# CONFIG_SPI_FLASH_SUPPORT_BOYA_CHIP is not set
# default:
# CONFIG_SPI_FLASH_SUPPORT_TH_CHIP is not set
# end of Auto-detect flash chips

# default:
CONFIG_SPI_FLASH_ENABLE_ENCRYPTED_READ_WRITE=y
# end of SPI Flash driver

#
# SPIFFS Configuration
#
# default:
CONFIG_SPIFFS_MAX_PARTITIONS=3

#
# SPIFFS Cache Configuration
#
# default:
CONFIG_SPIFFS_CACHE=y
# default:
CONFIG_SPIFFS_CACHE_WR=y
# default:
# CONFIG_SPIFFS_CACHE_STATS is not set
# end of SPIFFS Cache Configuration

# default:
CONFIG_SPIFFS_PAGE_CHECK=y
# default:
CONFIG_SPIFFS_GC_MAX_RUNS=10
# default:
# CONFIG_SPIFFS_GC_STATS is not set
# default:
CONFIG_SPIFFS_PAGE_SIZE=256
# default:
CONFIG_SPIFFS_OBJ_NAME_LEN=32
# default:
# CONFIG_SPIFFS_FOLLOW_SYMLINKS is not set
# default:
CONFIG_SPIFFS_USE_MAGIC=y
# default:
CONFIG_SPIFFS_USE_MAGIC_LENGTH=y
# default:
CONFIG_SPIFFS_META_LENGTH=4
# default:
CONFIG_SPIFFS_USE_MTIME=y

#
# Debug Configuration
#
# default:
# CONFIG_SPIFFS_DBG is not set
# default:
# CONFIG_SPIFFS_API_DBG is not set
# default:
# CONFIG_SPIFFS_GC_DBG is not set
# default:
# CONFIG_SPIFFS_CACHE_DBG is not set
# default:
# CONFIG_SPIFFS_CHECK_DBG is not set
# default:
# CONFIG_SPIFFS_TEST_VISUALISATION is not set
# end of Debug Configuration
# end of SPIFFS Configuration

#
# TCP Transport
#

#
# Websocket
#
# default:
CONFIG_WS_TRANSPORT=y
# default:
CONFIG_WS_BUFFER_SIZE=1024
# default:
# CONFIG_WS_DYNAMIC_BUFFER is not set
# end of Websocket
# end of TCP Transport

#
# Ultra Low Power (ULP) Co-processor
#
# default:
# CONFIG_ULP_COPROC_ENABLED is not set

#
# ULP Debugging Options
#
# end of ULP Debugging Options
# end of Ultra Low Power (ULP) Co-processor

#
# Unity unit testing library
#
# default:
CONFIG_UNITY_ENABLE_FLOAT=y
# default:
CONFIG_UNITY_ENABLE_DOUBLE=y
# default:
# CONFIG_UNITY_ENABLE_64BIT is not set
# default:
# CONFIG_UNITY_ENABLE_COLOR is not set
# default:
CONFIG_UNITY_ENABLE_IDF_TEST_RUNNER=y
# default:
# CONFIG_UNITY_ENABLE_FIXTURE is not set
# default:
# CONFIG_UNITY_ENABLE_BACKTRACE_ON_FAIL is not set
# default:
# CONFIG_UNITY_TEST_ORDER_BY_FILE_PATH_AND_LINE is not set
# end of Unity unit testing library

#
# Virtual file system
#
# default:
CONFIG_VFS_SUPPORT_IO=y
# default:
CONFIG_VFS_SUPPORT_DIR=y
# default:
CONFIG_VFS_SUPPORT_SELECT=y
# default:
CONFIG_VFS_SUPPRESS_SELECT_DEBUG_OUTPUT=y
# default:
# CONFIG_VFS_SELECT_IN_RAM is not set
# default:
CONFIG_VFS_SUPPORT_TERMIOS=y
# default:
CONFIG_VFS_MAX_COUNT=8

#
# Host File System I/O (Semihosting)
#
# default:
CONFIG_VFS_SEMIHOSTFS_MAX_MOUNT_POINTS=1
# end of Host File System I/O (Semihosting)

# default:
CONFIG_VFS_INITIALIZE_DEV_NULL=y
# end of Virtual file system

#
# Wear Levelling
#
# default:
# CONFIG_WL_SECTOR_SIZE_512 is not set
# default:
CONFIG_WL_SECTOR_SIZE_4096=y
# default:
CONFIG_WL_SECTOR_SIZE=4096
# end of Wear Levelling

#
# Wi-Fi Provisioning Manager
#
# default:
CONFIG_WIFI_PROV_SCAN_MAX_ENTRIES=16
# default:
CONFIG_WIFI_PROV_AUTOSTOP_TIMEOUT=30
# default:
CONFIG_WIFI_PROV_STA_ALL_CHANNEL_SCAN=y
# default:
# CONFIG_WIFI_PROV_STA_FAST_SCAN is not set
# end of Wi-Fi Provisioning Manager
# end of Component config

# default:
# CONFIG_IDF_EXPERIMENTAL_FEATURES is not set

# Deprecated options for backward compatibility
# CONFIG_APP_BUILD_TYPE_ELF_RAM is not set
# CONFIG_NO_BLOBS is not set
# CONFIG_ESP32_NO_BLOBS is not set
# CONFIG_ESP32_COMPATIBLE_PRE_V2_1_BOOTLOADERS is not set
# CONFIG_ESP32_COMPATIBLE_PRE_V3_1_BOOTLOADERS is not set
# CONFIG_APP_ROLLBACK_ENABLE is not set
# CONFIG_LOG_BOOTLOADER_LEVEL_NONE is not set
# CONFIG_LOG_BOOTLOADER_LEVEL_ERROR is not set
# CONFIG_LOG_BOOTLOADER_LEVEL_WARN is not set
CONFIG_LOG_BOOTLOADER_LEVEL_INFO=y
# CONFIG_LOG_BOOTLOADER_LEVEL_DEBUG is not set
# CONFIG_LOG_BOOTLOADER_LEVEL_VERBOSE is not set
CONFIG_LOG_BOOTLOADER_LEVEL=3
# CONFIG_FLASH_ENCRYPTION_ENABLED is not set
# CONFIG_FLASHMODE_QIO is not set
# CONFIG_FLASHMODE_QOUT is not set
CONFIG_FLASHMODE_DIO=y
# CONFIG_FLASHMODE_DOUT is not set
CONFIG_MONITOR_BAUD=115200
CONFIG_OPTIMIZATION_LEVEL_DEBUG=y
CONFIG_COMPILER_OPTIMIZATION_LEVEL_DEBUG=y
CONFIG_COMPILER_OPTIMIZATION_DEFAULT=y
# CONFIG_OPTIMIZATION_LEVEL_RELEASE is not set
# CONFIG_COMPILER_OPTIMIZATION_LEVEL_RELEASE is not set
CONFIG_OPTIMIZATION_ASSERTIONS_ENABLED=y
# CONFIG_OPTIMIZATION_ASSERTIONS_SILENT is not set
# CONFIG_OPTIMIZATION_ASSERTIONS_DISABLED is not set
CONFIG_OPTIMIZATION_ASSERTION_LEVEL=2
# CONFIG_CXX_EXCEPTIONS is not set
CONFIG_STACK_CHECK_NONE=y
# CONFIG_STACK_CHECK_NORM is not set
# CONFIG_STACK_CHECK_STRONG is not set
# CONFIG_STACK_CHECK_ALL is not set
# CONFIG_WARN_WRITE_STRINGS is not set
# CONFIG_ESP32_APPTRACE_DEST_TRAX is not set
CONFIG_ESP32_APPTRACE_DEST_NONE=y
CONFIG_ESP32_APPTRACE_LOCK_ENABLE=y
CONFIG_ADC2_DISABLE_DAC=y
# CONFIG_GPTIMER_ISR_IRAM_SAFE is not set
# CONFIG_MCPWM_ISR_IRAM_SAFE is not set
# CONFIG_EVENT_LOOP_PROFILING is not set
CONFIG_POST_EVENTS_FROM_ISR=y
CONFIG_POST_EVENTS_FROM_IRAM_ISR=y
CONFIG_GDBSTUB_SUPPORT_TASKS=y
CONFIG_GDBSTUB_MAX_TASKS=32
# CONFIG_OTA_ALLOW_HTTP is not set
# CONFIG_TWO_UNIVERSAL_MAC_ADDRESS is not set
CONFIG_FOUR_UNIVERSAL_MAC_ADDRESS=y
CONFIG_NUMBER_OF_UNIVERSAL_MAC_ADDRESS=4
# CONFIG_ESP_SYSTEM_PD_FLASH is not set
CONFIG_ESP32_DEEP_SLEEP_WAKEUP_DELAY=2000
CONFIG_ESP_SLEEP_DEEP_SLEEP_WAKEUP_DELAY=2000
CONFIG_ESP32_RTC_CLK_SRC_INT_RC=y
CONFIG_ESP32_RTC_CLOCK_SOURCE_INTERNAL_RC=y
# CONFIG_ESP32_RTC_CLK_SRC_EXT_CRYS is not set
# CONFIG_ESP32_RTC_CLOCK_SOURCE_EXTERNAL_CRYSTAL is not set
# CONFIG_ESP32_RTC_CLK_SRC_EXT_OSC is not set
# CONFIG_ESP32_RTC_CLOCK_SOURCE_EXTERNAL_OSC is not set
# CONFIG_ESP32_RTC_CLK_SRC_INT_8MD256 is not set
# CONFIG_ESP32_RTC_CLOCK_SOURCE_INTERNAL_8MD256 is not set
CONFIG_ESP32_RTC_CLK_CAL_CYCLES=1024
CONFIG_PERIPH_CTRL_FUNC_IN_IRAM=y
# CONFIG_ESP32_XTAL_FREQ_26 is not set
CONFIG_ESP32_XTAL_FREQ_40=y
# CONFIG_ESP32_XTAL_FREQ_AUTO is not set
CONFIG_ESP32_XTAL_FREQ=40
CONFIG_BROWNOUT_DET=y
CONFIG_ESP32_BROWNOUT_DET=y
CONFIG_BROWNOUT_DET_LVL_SEL_0=y
CONFIG_ESP32_BROWNOUT_DET_LVL_SEL_0=y
# CONFIG_BROWNOUT_DET_LVL_SEL_1 is not set
# CONFIG_ESP32_BROWNOUT_DET_LVL_SEL_1 is not set
# CONFIG_BROWNOUT_DET_LVL_SEL_2 is not set
# CONFIG_ESP32_BROWNOUT_DET_LVL_SEL_2 is not set
# CONFIG_BROWNOUT_DET_LVL_SEL_3 is not set
# CONFIG_ESP32_BROWNOUT_DET_LVL_SEL_3 is not set
# CONFIG_BROWNOUT_DET_LVL_SEL_4 is not set
# CONFIG_ESP32_BROWNOUT_DET_LVL_SEL_4 is not set
# CONFIG_BROWNOUT_DET_LVL_SEL_5 is not set
# CONFIG_ESP32_BROWNOUT_DET_LVL_SEL_5 is not set
# CONFIG_BROWNOUT_DET_LVL_SEL_6 is not set
# CONFIG_ESP32_BROWNOUT_DET_LVL_SEL_6 is not set
# CONFIG_BROWNOUT_DET_LVL_SEL_7 is not set
# CONFIG_ESP32_BROWNOUT_DET_LVL_SEL_7 is not set
CONFIG_BROWNOUT_DET_LVL=0
CONFIG_ESP32_BROWNOUT_DET_LVL=0
CONFIG_ESP_SYSTEM_BROWNOUT_INTR=y
CONFIG_ESP32_PHY_CALIBRATION_AND_DATA_STORAGE=y
# CONFIG_ESP32_PHY_INIT_DATA_IN_PARTITION is not set
CONFIG_ESP32_PHY_MAX_WIFI_TX_POWER=20
CONFIG_ESP32_PHY_MAX_TX_POWER=20
# CONFIG_REDUCE_PHY_TX_POWER is not set
# CONFIG_ESP32_REDUCE_PHY_TX_POWER is not set
# CONFIG_SPIRAM_SUPPORT is not set
# CONFIG_ESP32_SPIRAM_SUPPORT is not set
# CONFIG_ESP32_DEFAULT_CPU_FREQ_80 is not set
# CONFIG_ESP32_DEFAULT_CPU_FREQ_160 is not set
CONFIG_ESP32_DEFAULT_CPU_FREQ_240=y
CONFIG_ESP32_DEFAULT_CPU_FREQ_MHZ=240
CONFIG_TRACEMEM_RESERVE_DRAM=0x0
# CONFIG_ESP32_PANIC_PRINT_HALT is not set
CONFIG_ESP32_PANIC_PRINT_REBOOT=y
# CONFIG_ESP32_PANIC_SILENT_REBOOT is not set
# CONFIG_ESP32_PANIC_GDBSTUB is not set
CONFIG_SYSTEM_EVENT_QUEUE_SIZE=32
CONFIG_SYSTEM_EVENT_TASK_STACK_SIZE=2304
CONFIG_MAIN_TASK_STACK_SIZE=3584
CONFIG_CONSOLE_UART_DEFAULT=y
# CONFIG_CONSOLE_UART_CUSTOM is not set
# CONFIG_CONSOLE_UART_NONE is not set
# CONFIG_ESP_CONSOLE_UART_NONE is not set
CONFIG_CONSOLE_UART=y
CONFIG_CONSOLE_UART_NUM=0
CONFIG_CONSOLE_UART_BAUDRATE=115200
CONFIG_INT_WDT=y
CONFIG_INT_WDT_TIMEOUT_MS=300
CONFIG_INT_WDT_CHECK_CPU1=y
# CONFIG_ESP32_DEBUG_STUBS_ENABLE is not set
CONFIG_ESP32_DEBUG_OCDAWARE=y
# CONFIG_DISABLE_BASIC_ROM_CONSOLE is not set
CONFIG_IPC_TASK_STACK_SIZE=1024
CONFIG_TIMER_TASK_STACK_SIZE=3584
CONFIG_ESP32_WIFI_ENABLED=y
CONFIG_ESP32_WIFI_STATIC_RX_BUFFER_NUM=10
CONFIG_ESP32_WIFI_DYNAMIC_RX_BUFFER_NUM=32
# CONFIG_ESP32_WIFI_STATIC_TX_BUFFER is not set
CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER=y
CONFIG_ESP32_WIFI_TX_BUFFER_TYPE=1
CONFIG_ESP32_WIFI_DYNAMIC_TX_BUFFER_NUM=32
# CONFIG_ESP32_WIFI_CSI_ENABLED is not set
CONFIG_ESP32_WIFI_AMPDU_TX_ENABLED=y
CONFIG_ESP32_WIFI_TX_BA_WIN=6
CONFIG_ESP32_WIFI_AMPDU_RX_ENABLED=y
CONFIG_ESP32_WIFI_RX_BA_WIN=6
CONFIG_ESP32_WIFI_NVS_ENABLED=y
CONFIG_ESP32_WIFI_TASK_PINNED_TO_CORE_0=y
# CONFIG_ESP32_WIFI_TASK_PINNED_TO_CORE_1 is not set
CONFIG_ESP32_WIFI_SOFTAP_BEACON_MAX_LEN=752
CONFIG_ESP32_WIFI_MGMT_SBUF_NUM=32
CONFIG_ESP32_WIFI_IRAM_OPT=y
CONFIG_ESP32_WIFI_RX_IRAM_OPT=y
CONFIG_ESP32_WIFI_ENABLE_WPA3_SAE=y
CONFIG_ESP32_WIFI_ENABLE_WPA3_OWE_STA=y
CONFIG_WPA_MBEDTLS_CRYPTO=y
CONFIG_WPA_MBEDTLS_TLS_CLIENT=y
# CONFIG_WPA_WAPI_PSK is not set
# CONFIG_WPA_11KV_SUPPORT is not set
# CONFIG_WPA_MBO_SUPPORT is not set
# CONFIG_WPA_DPP_SUPPORT is not set
# CONFIG_WPA_11R_SUPPORT is not set
# CONFIG_WPA_WPS_SOFTAP_REGISTRAR is not set
# CONFIG_WPA_WPS_STRICT is not set
# CONFIG_WPA_DEBUG_PRINT is not set
# CONFIG_ESP32_ENABLE_COREDUMP_TO_FLASH is not set
# CONFIG_ESP32_ENABLE_COREDUMP_TO_UART is not set
CONFIG_ESP32_ENABLE_COREDUMP_TO_NONE=y
CONFIG_TIMER_TASK_PRIORITY=1
CONFIG_TIMER_TASK_STACK_DEPTH=2048
CONFIG_TIMER_QUEUE_LENGTH=10
# CONFIG_ENABLE_STATIC_TASK_CLEAN_UP_HOOK is not set
# CONFIG_HAL_ASSERTION_SILIENT is not set
# CONFIG_L2_TO_L3_COPY is not set
CONFIG_ESP_GRATUITOUS_ARP=y
CONFIG_GARP_TMR_INTERVAL=60
CONFIG_TCPIP_RECVMBOX_SIZE=32
CONFIG_TCP_MAXRTX=12
CONFIG_TCP_SYNMAXRTX=12
CONFIG_TCP_MSS=1440
CONFIG_TCP_MSL=60000
CONFIG_TCP_SND_BUF_DEFAULT=5760
CONFIG_TCP_WND_DEFAULT=5760
CONFIG_TCP_RECVMBOX_SIZE=6
CONFIG_TCP_QUEUE_OOSEQ=y
CONFIG_TCP_OVERSIZE_MSS=y
# CONFIG_TCP_OVERSIZE_QUARTER_MSS is not set
# CONFIG_TCP_OVERSIZE_DISABLE is not set
CONFIG_UDP_RECVMBOX_SIZE=6
CONFIG_TCPIP_TASK_STACK_SIZE=3072
CONFIG_TCPIP_TASK_AFFINITY_NO_AFFINITY=y
# CONFIG_TCPIP_TASK_AFFINITY_CPU0 is not set
# CONFIG_TCPIP_TASK_AFFINITY_CPU1 is not set
CONFIG_TCPIP_TASK_AFFINITY=0x7FFFFFFF
# CONFIG_PPP_SUPPORT is not set
CONFIG_NEWLIB_STDOUT_LINE_ENDING_CRLF=y
# CONFIG_NEWLIB_STDOUT_LINE_ENDING_LF is not set
# CONFIG_NEWLIB_STDOUT_LINE_ENDING_CR is not set
# CONFIG_NEWLIB_STDIN_LINE_ENDING_CRLF is not set
# CONFIG_NEWLIB_STDIN_LINE_ENDING_LF is not set
CONFIG_NEWLIB_STDIN_LINE_ENDING_CR=y
# CONFIG_NEWLIB_NANO_FORMAT is not set
CONFIG_NEWLIB_TIME_SYSCALL_USE_RTC_HRT=y
CONFIG_ESP32_TIME_SYSCALL_USE_RTC_HRT=y
CONFIG_ESP32_TIME_SYSCALL_USE_RTC_FRC1=y
# CONFIG_NEWLIB_TIME_SYSCALL_USE_RTC is not set
# CONFIG_ESP32_TIME_SYSCALL_USE_RTC is not set
# CONFIG_NEWLIB_TIME_SYSCALL_USE_HRT is not set
# CONFIG_ESP32_TIME_SYSCALL_USE_HRT is not set
# CONFIG_ESP32_TIME_SYSCALL_USE_FRC1 is not set
# CONFIG_NEWLIB_TIME_SYSCALL_USE_NONE is not set
# CONFIG_ESP32_TIME_SYSCALL_USE_NONE is not set
CONFIG_ESP32_PTHREAD_TASK_PRIO_DEFAULT=5
CONFIG_ESP32_PTHREAD_TASK_STACK_SIZE_DEFAULT=3072
CONFIG_ESP32_PTHREAD_STACK_MIN=768
CONFIG_ESP32_DEFAULT_PTHREAD_CORE_NO_AFFINITY=y
# CONFIG_ESP32_DEFAULT_PTHREAD_CORE_0 is not set
# CONFIG_ESP32_DEFAULT_PTHREAD_CORE_1 is not set
CONFIG_ESP32_PTHREAD_TASK_CORE_DEFAULT=-1
CONFIG_ESP32_PTHREAD_TASK_NAME_DEFAULT="pthread"
CONFIG_SPI_FLASH_WRITING_DANGEROUS_REGIONS_ABORTS=y
# CONFIG_SPI_FLASH_WRITING_DANGEROUS_REGIONS_FAILS is not set
# CONFIG_SPI_FLASH_WRITING_DANGEROUS_REGIONS_ALLOWED is not set
# CONFIG_ESP32_ULP_COPROC_ENABLED is not set
CONFIG_SUPPRESS_SELECT_DEBUG_OUTPUT=y
CONFIG_SUPPORT_TERMIOS=y
CONFIG_SEMIHOSTFS_MAX_MOUNT_POINTS=1
# End of deprecated options
```

---

## 4) Sources complètes pour analyse future (sans extraits)

### 4.1 `src/esp32/src/comm_interface.h`
```cpp
/**
 * @file comm_interface.h
 * @brief SPI slave transport for host → ESP32 motion messages.
 *
 * The communication layer is intentionally separated from the stepper runtime:
 *
 * - `messages.h` defines all wire-level structures and result codes
 * - `CommInterface` owns the SPI slave task and DMA buffers
 * - handler methods translate validated messages into queue/driver operations
 *
 * This structure keeps the code maintainable when adding new commands:
 * define a new payload in `messages.h`, then add one handler here.
 */

#pragma once

#include <atomic>
#include <driver/gpio.h>
#include <esp_err.h>

#include "messages.h"
#include "stepper_queue.h"
#include "motion_planner.h"

struct SpiBusPins {
    gpio_num_t mosi;
    gpio_num_t miso;
    gpio_num_t sclk;
    gpio_num_t cs;
    gpio_num_t home_pin_no;
    gpio_num_t home_pin_nc;
};

class CommInterface {
public:
    /**
     * @brief Construct the SPI communication interface.
     *
     * @param queues   Stepper queues indexed by axis id.
     * @param n_motors Number of valid queues.
     */
    CommInterface(StepperQueue* queues[], uint8_t n_motors);

    /**
     * @brief Initialize the ESP32 SPI slave and start the communication task.
     */
    esp_err_t init(const SpiBusPins& pins);

private:
    StepperQueue*   queues_[SPI_MAX_AXES];
    uint8_t         n_motors_;
    SpiBusPins      pins_ {};
    MotionPlanner   planner_;              ///< Planning layer (SPI → executor)

    uint16_t        last_rx_sequence_ {0};
    uint8_t         last_rx_type_ {static_cast<uint8_t>(SpiMessageType::NOP)};
    uint8_t         last_result_ {static_cast<uint8_t>(SpiMessageResult::OK)};

    /**
     * @brief Motion sequence of the most recently fully-executed multi-axis
     *        segment.  Updated by the executor task (Core 1) and read by the
     *        SPI task (Core 0).  std::atomic provides lock-free cross-core
     *        visibility without a portMUX spinlock.
     *        Initialised to 0xFFFF so the host's first segment always
     *        compares as "not yet executed".
     */
    std::atomic<uint16_t>   last_executed_sequence_ {0xFFFFu};

    /** Build the status payload for the next SPI response frame. */
    void buildStatusFrame(uint8_t* out_frame) const;

    /** Read the current lateral endstop state from the configured pins. */
    uint8_t readLateralEndstopState() const;

    /** Return true if the given axis may move given the lateral endstop state. */
    bool isLateralMovementAllowed(uint8_t axis_id) const;

    /** Parse and execute one validated request frame. */
    esp_err_t handleFrame(const SpiMessageHeader& header, const uint8_t* payload);

    esp_err_t handleEnableAxis(const EnableAxisPayload& payload);
    esp_err_t handleEmergencyStop(const EmergencyStopPayload& payload);
    esp_err_t handleStopAxis(const EmergencyStopPayload& payload);
    esp_err_t handleDisableAll();
    esp_err_t handleResetStats();
    esp_err_t handleStepBlock(const StepBlockPayload& payload);
    esp_err_t handleSegmentBlock(const SegmentBlockPayload& payload);

    /**
     * @brief Handle a MULTI_AXIS_SEGMENT_BLOCK (0x13) frame.
     *
     * Decodes the variable-length multi-axis segment payload and dispatches
     * one multi_axis_segment_block_t to the global multi-axis queue.
     */
    esp_err_t handleMultiAxisSegmentBlock(const uint8_t* payload, uint16_t payload_length);

    /**
     * @brief Handle a FLUSH (0x12) frame.
     *
     * Instructs the executor to discard all queued segments whose
     * motion_sequence > flush_sequence, allowing the host to inject a
     * new trajectory without draining the current buffer first.
     */
    esp_err_t handleFlush(const FlushPayload& payload);

    /**
     * @brief Update last_executed_sequence_ under the spinlock.
     *
     * Must be called by the executor task whenever it completes a
     * multi-axis segment.
     *
     * @param motion_seq  The motion_sequence of the just-completed segment.
     */
    void notifySegmentExecuted(uint16_t motion_seq);

    /**
     * @brief Handle an ENABLE_ENDSTOP (0x14) frame.
     */
    esp_err_t handleEnableEndstop(const EnableEndstopPayload& payload);

    /** Core 0 SPI slave task. */
    static void spiTask(void* arg);

    /**
     * @brief Core 1 multi-axis segment executor task.
     *
     * Consumes multi_axis_block_t objects from the global multi-axis queue,
     * distributes constant-rate step bursts to each per-axis StepperQueue,
     * and calls notifySegmentExecuted() after each segment completes.  Also
     * drains the global flush queue between blocks to support host trajectory
     * cancellation without draining the entire axis queue first.
     *
     * Pinned to Core 1 at priority 24 (same as per-axis executor tasks).
     * Only one multi-axis executor task is ever launched.
     */
    static void multiAxisExecutorTask(void* arg);
};
```

### 4.2 `src/esp32/src/motion_planner.h`
```cpp
/**
 * @file motion_planner.h
 * @brief GRBL/Klipper-inspired motion planning layer.
 *
 * ── Architecture role ──────────────────────────────────────────────────────
 *
 *   SPI ingestion (Core 0)              Planner (Core 0)           Executor (Core 1)
 *   ────────────────────                ───────────────            ─────────────────
 *   spiTask → handleFrame()   ──►   s_multi_axis_queue   ──►   plannerTask()
 *                                        (existing)              │
 *                                                                ▼
 *                                                          segment_queue_
 *                                                           (NEW bounded)
 *                                                                │
 *                                                                ▼
 *                                                        executorTask (Core 1)
 *                                                        state machine
 *                                                                │
 *                                                                ▼
 *                                                          RMT ring buffer
 *
 * The planner consumes multi_axis_block_t (bulk blocks from SPI) and
 * decomposes them into individual planned_segment_t entries with Klipper-style
 * monotonic timestamps.  The executor consumes these one at a time through a
 * bounded state machine.
 *
 * ── Backpressure ───────────────────────────────────────────────────────────
 *
 *   segment_queue_ is bounded to SEGMENT_QUEUE_DEPTH.  If the executor is
 *   slow, the planner blocks (with timeout) providing natural backpressure
 *   all the way back to the SPI ingestion queue.
 *
 * ── Flush path ─────────────────────────────────────────────────────────────
 *
 *   On flush, the planner drains both cmd_queue_ and segment_queue_, then
 *   pushes a flush sentinel (is_flush=true) so the executor can reset its
 *   state atomically.
 */

#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <esp_err.h>
#include "step_types.h"

// ---------------------------------------------------------------------------
// Planned segment — immutable output of planner, input to executor
// ---------------------------------------------------------------------------

/**
 * @brief Per-axis motion within a planned segment.
 */
typedef struct {
    uint16_t step_count;
    bool     direction;
} planned_axis_motion_t;

/**
 * @brief One fully-planned segment ready for execution.
 *
 * Produced by the planner task, consumed by the executor task.
 * Immutable after enqueue — no synchronisation needed beyond the queue.
 */
typedef struct {
    uint16_t              motion_sequence;     ///< Host-assigned sequence ID
    uint16_t              duration_us;         ///< Wall-clock duration
    int64_t               scheduled_time_us;   ///< Monotonic execution timestamp (Klipper-style)
    uint8_t               axis_count;
    uint8_t               axis_ids[MULTI_AXIS_MAX_AXES];
    planned_axis_motion_t axes[MULTI_AXIS_MAX_AXES];
    bool                  is_flush;            ///< True = flush sentinel, not a real segment
    uint16_t              flush_sequence;      ///< Valid only when is_flush == true
} planned_segment_t;

// ---------------------------------------------------------------------------
// Executor state machine
// ---------------------------------------------------------------------------

/**
 * @brief Executor FSM states.
 *
 * The state machine ensures bounded CPU usage per iteration and eliminates
 * the nested drain loops that caused watchdog resets.
 *
 *   IDLE ──► FETCH ──► DRAIN ──► RUN ──► (back to IDLE)
 *              │                           ▲
 *              ▼                           │
 *            FLUSH ────────────────────────┘
 *              │
 *              ▼
 *           RECOVERY ──────────────────────┘
 */
enum class ExecState : uint8_t {
    IDLE,       ///< Waiting for segments (blocking queue receive)
    FETCH,      ///< Pulling segment(s) from planner queue (non-blocking batch)
    DRAIN,      ///< Writing steps to RMT ring buffer
    RUN,        ///< KickStart RMT, fire deferred notifications
    FLUSH,      ///< Processing flush sentinel — reset pipeline
    RECOVERY,   ///< Recovering from RMT underrun / error
};

// ---------------------------------------------------------------------------
// Tuning constants
// ---------------------------------------------------------------------------

/** Segment queue: planner → executor.  ~512 ms lookahead at 4 ms/segment. */
static constexpr uint32_t SEGMENT_QUEUE_DEPTH = 128;

/** Minimum segments buffered before executor begins first RMT kickStart. */
static constexpr uint32_t SEGMENT_PREFILL_THRESHOLD = 16;

/** Maximum segments the executor fetches per FETCH iteration.
 *  Set to 1 to force frequent yields and allow planner to refill. */
static constexpr uint32_t EXEC_BATCH_LIMIT = 16;

/** Time budget per executor iteration in microseconds (watchdog safe). */
static constexpr int64_t  EXEC_TIME_BUDGET_US = 3000;

/** Planner tuning: time budget and per-iteration limit (watchdog-safe).
 *  200 µs budget allows processing 32+ segments per loop iteration at 5-10 µs/segment.
 *  Non-blocking xQueueSend ensures no watchdog blocking despite higher throughput.
 *  Higher batch size prevents executor starvation when planner runs infrequently.
 */
static constexpr int64_t  PLANNER_TIME_BUDGET_US = 2000; // µs per planner loop
static constexpr uint32_t PLANNER_MAX_SEGMENTS_PER_ITER = 60; // segments per loop to balance yield

// ---------------------------------------------------------------------------
// MotionPlanner class
// ---------------------------------------------------------------------------

class MotionPlanner {
public:
    MotionPlanner();

    /**
     * @brief Initialise the segment output queue and launch the planner task.
     *
     * @param cmd_queue   Existing s_multi_axis_queue (SPI → planner input).
     * @param flush_queue Existing s_flush_queue (SPI → planner input).
     * @return ESP_OK on success.
     */
    esp_err_t init(QueueHandle_t cmd_queue, QueueHandle_t flush_queue);

    /** @brief Output queue handle for the executor to consume. */
    QueueHandle_t segmentQueue() const { return segment_queue_; }

    /** @brief Number of free slots in the segment output queue. */
    uint32_t segmentQueueFree() const;

    // ── Statistics ──────────────────────────────────────────────────────────
    uint32_t segmentsPlanned() const { return segments_planned_; }
    uint32_t segmentsDropped() const { return segments_dropped_; }

private:
    QueueHandle_t cmd_queue_     {nullptr};  ///< Input: s_multi_axis_queue
    QueueHandle_t flush_queue_   {nullptr};  ///< Input: s_flush_queue
    QueueHandle_t segment_queue_ {nullptr};  ///< Output: planned_segment_t

    int64_t  timeline_us_       {0};         ///< Monotonic scheduling timeline
    uint32_t segments_planned_  {0};
    uint32_t segments_dropped_  {0};

    // ── Incremental planner state (to avoid burst processing) ───────────
    multi_axis_block_t pending_block_ {};   ///< Currently-being-expanded block
    uint16_t            pending_segment_idx_ {0};
    bool                has_pending_block_   {false};

    // Non-blocking flush sentinel retry state
    bool     flush_pending_ {false};
    uint16_t pending_flush_sequence_ {0};

    /**
     * @brief Expand one multi_axis_block_t into planned_segment_t entries.
     *
     * Each segment in the block becomes one planned_segment_t with a
     * Klipper-style monotonic timestamp derived from cumulative duration.
     * Enqueues to segment_queue_ with bounded backpressure wait.
     */
    void planBlock(const multi_axis_block_t& block);

    /**
     * @brief Handle a flush request: drain queues, push flush sentinel.
     */
    void handleFlush(const flush_request_t& req);

    /**
     * @brief Planner task body.
     *
     * Pinned to Core 0, priority 8 (below SPI task at 10, above idle).
     * Runs in SPI task's idle time between spi_slave_transmit() calls.
     */
    static void plannerTask(void* arg);
};
```
