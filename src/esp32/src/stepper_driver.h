/**
 * @file stepper_driver.h
 * @brief Physical-layer RMT stepper driver — one instance per motor axis.
 *
 * Converts pre-timed step_block_t arrays into a gapless RMT symbol stream
 * using a simple_encoder callback (FastAccelStepper-style ping-pong).
 *
 * ── Coast-mode streaming architecture ──────────────────────────────────────
 *   Producer (pushBlock, task context):
 *     1. Convert step_block_t → ring_entry_t[] in the lock-free ring buffer.
 *     2. If the ring is full, block on a task notification from the ISR.
 *
 *   Consumer (encode_steps callback, ISR context):
 *     3. RMT hardware calls encode_steps() when it needs more symbols.
 *     4. Callback reads up to PART_SIZE entries from the ring buffer,
 *        converts each to one rmt_symbol_word_t (balanced pulse).
 *     5. On empty ring: emit LOW-level PAUSE symbols (coast) — the RMT
 *        transaction stays alive. No stop/restart overhead.
 *     6. After COAST_IDLE_LIMIT consecutive empty callbacks (~1.25 s),
 *        auto-stop the transaction to free RMT resources.
 *
 *   on_trans_done ISR:
 *     7. Marks rmt_running_ = false so the next kickStart() restarts.
 *
 *   Coast mode eliminates the ~200-500 µs gaps caused by stop/restart
 *   cycles that were the primary source of underruns at low speed.
 *
 * ── Direction constraint ───────────────────────────────────────────────────
 *   Direction changes are handled in ISR context via gpio_ll (register-level).
 *   When a ring entry has toggle_dir=1 and the previous chunk contained step
 *   pulses, a pause chunk is emitted first to meet the driver IC's direction
 *   setup time, and the toggle is deferred to the next callback invocation.
 */

#pragma once

#include <atomic>
#include <driver/rmt_tx.h>
#include <driver/gpio.h>
#include <hal/gpio_ll.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include "step_types.h"

class StepperDriver {
public:
    // ─── RMT clock constants (typed aliases — authoritative values in step_types.h) ──
    // These use different names to avoid colliding with the same-named macros.
    //
    // Key relationships:
    //   RMT_CLK_HZ = 80 MHz  →  1 tick = 12.5 ns
    //   RMT_TICKS_PER_US_C = RMT_CLK_HZ / 1e6 = 80 ticks/µs
    //   RMT_MIN_TICKS_C = 16  →  max_step_rate = 80e6 / 16 = 5 MHz
    //   max_rpm = max_step_rate / (steps_per_rev × microsteps)
    //           = 5 000 000 / (200 × 32) = 781 RPM
    //   At cruise 160 kHz: interval_ticks = 80e6 / 160000 = 500 ticks
    //static constexpr uint32_t RMT_CLK_HZ          = 80000000UL;          // 80 MHz
    //static constexpr uint32_t RMT_TICKS_PER_US_C  = RMT_CLK_HZ / 1000000UL; // 80
    //static constexpr uint32_t RMT_PULSE_TICKS_C   = 8U;   // 8 × 12.5 ns = 100 ns HIGH
    //static constexpr uint32_t RMT_MIN_TICKS_C     = 16U;  // 16 × 12.5 ns = 200 ns → 5 MHz ceiling
    //static constexpr uint32_t RMT_MAX_TICKS_C     = 0xFFFFU;
    /**
     * @brief Construct a StepperDriver.
     *
     * @param step_pin  GPIO for the STEP signal (RMT output).
     * @param dir_pin   GPIO for the DIR signal.
     * @param en_pin    GPIO for the EN signal (active-LOW on DRV8825/A4988).
     * @param motor_id  Logical motor index (0-based, for logging).
     */
    StepperDriver(gpio_num_t step_pin, gpio_num_t dir_pin,
                  gpio_num_t en_pin,   uint8_t    motor_id);

    /** @brief Initialise GPIO, create RMT channel and simple encoder, register ISR. */
    esp_err_t init();

    /** @brief Assert EN pin (driver IC powered, coils energised). */
    void enable();

    /** @brief De-assert EN pin (driver IC off, coils de-energised). */
    void disable();

    /** @brief True if the driver output is currently enabled. */
    bool isEnabled() const { return enabled_; }

    /**
     * @brief Immediate stop: flush the RMT TX queue and reset ring buffer.
     *
     * Called from task context only.
     */
    void emergencyStop();

    /**
     * @brief Gracefully end the RMT stream.  Blocks until the current
     *        transmission finishes.  Safe to call from task context only.
     */
    void stopStream();

    /**
     * @brief Signal the RMT stream to stop after the current ring contents
     *        have been consumed.  Does NOT reset the ring buffer.
     *
     * Contrast with emergencyStop() which flushes the ring immediately.
     * Safe to call from task context only.
     */
    void gracefulStop();

    /**
     * @brief Push a block of steps into the ring buffer for streaming.
     *
     * Converts step_block_t → ring_entry_t[] and writes them to the SPSC
     * ring buffer.  If the ring is full, blocks on a task notification from
     * the ISR until space becomes available.
     *
     * @param block        Block of pre-timed step commands.
     * @param caller_task  Handle of the calling task; stored as producer_task_
     *                     so the ISR ring-space notification wakes the right
     *                     task.  Pass xTaskGetCurrentTaskHandle() from the
     *                     caller (StepperQueue::pushExpandedBlock).
     * @return ESP_OK, or an RMT error code on stream start failure.
     */
    esp_err_t pushBlock(const step_block_t& block, TaskHandle_t caller_task);

    /** @brief Return the motor id supplied at construction (0 or 1). */
    uint8_t motorId() const { return motor_id_; }

    /**
     * @brief Start (or restart) RMT streaming.
     *
     * Must be called by the executor task after draining all available step
     * blocks into the ring buffer so that the ring is maximally full before
     * the RMT hardware starts consuming entries.  Safe to call from task
     * context only.
     */
    esp_err_t startStream();

    /** @brief Approximate number of free slots remaining in the software ring. */
    uint32_t ringFreeSlots() const { return ringFree(); }

    /** @brief True while an RMT transaction is currently active. */
    bool isStreaming() const { return rmt_running_.load(std::memory_order_acquire); }
    bool isStopped()  const { return rmt_stopped_.load(std::memory_order_acquire); }

    // ─── Ring buffer (SPSC: task writes, ISR reads) ────────────────────────
    // Public because the C encoder callback needs direct access in ISR context.

    ring_entry_t          ring_[STEP_RING_SIZE];     /**< Step ring buffer      */
    /**
     * Ring write index. Written ONLY by the producer task, read by ISR.
     * std::atomic with release/acquire ordering guarantees the ISR sees
     * fully-written ring_entry_t data before the index advances.
     */
    std::atomic<uint32_t> ring_write_ {0};
    /**
     * Ring read index. Written ONLY by the ISR (encode_steps), read by
     * the producer task for free-slot calculation.
     */
    std::atomic<uint32_t> ring_read_  {0};

    gpio_num_t            dir_pin_;                  /**< For ISR gpio_ll       */
    /**
     * Set by encode_steps() ISR to signal the RMT transaction should end.
     * Read by pushBlock()/startStream() in task context.
     */
    std::atomic<bool>     rmt_stopped_ {true};
    bool                  last_chunk_had_steps_ {false}; /**< Dir-change safety (ISR only) */
    uint16_t              last_ticks_ {RMT_STEP_DEFAULT_TICKS}; /**< Last step interval (ISR only) */
    uint32_t              coast_idle_count_ {0}; /**< Consecutive empty-ring callbacks (ISR only, coast mode) */

    /**
     * @brief Task handle of the current ring producer.
     *
     * Set to the calling task every time pushBlock() is entered so the ring
     * back-pressure (ulTaskNotifyTake) always wakes the correct task.
     * Written from task context, read from ISR — std::atomic for safety.
     */
    std::atomic<TaskHandle_t> producer_task_ {nullptr};

    /**
     * @brief Task handle of the multi-axis executor (Core 1).
     *
     * Set once at startup by CommInterface via setExecutorTask().
     * encode_steps notifies BOTH this handle and producer_task_ so the
     * executor can pre-emptively refill the ring before it stalls.
     */
    std::atomic<TaskHandle_t> executor_task_ {nullptr};

    /** Register the multi-axis executor task handle for ring-low wakeups. */
    void setExecutorTask(TaskHandle_t t) { executor_task_.store(t, std::memory_order_release); }

    /**
     * @brief Set by the GPIO endstop ISR when contact is detected.
     * Read by encode_steps() in ISR context to stop the RMT immediately.
     * Cleared by the host via SPI ENABLE_ENDSTOP command or when the
     * endstop sensor returns to open state.
     * std::atomic for ISR ↔ task safety.
     */
    std::atomic<bool> endstop_active_ {false};

    /** @brief Arm the endstop — ISR will stop motion on trigger. */
    void armEndstop() {
        endstop_active_.store(false, std::memory_order_release);
        endstop_armed_.store(true, std::memory_order_release);
    }

    /** @brief Disarm the endstop — ISR will not stop motion on trigger.
     *  Use during intentional clearance moves commanded by the host. */
    void disarmEndstop() { endstop_armed_.store(false, std::memory_order_release); }

    /** @brief True if the endstop is currently armed. */
    bool isEndstopArmed() const { return endstop_armed_.load(std::memory_order_acquire); }

    /** @brief True if the endstop is currently triggered. */
    bool isEndstopActive() const { return endstop_active_.load(std::memory_order_acquire); }

    /**
     * @brief Install GPIO edge-triggered ISR on the NO/NC endstop pins.
     *
     * Called once from CommInterface::init() after the driver is ready.
     * Stores the pin numbers so the static ISR can access them via the
     * driver pointer without touching CommInterface state.
     *
     * @param no_pin  GPIO of the Normally-Open contact  (GPIO_NUM_NC = skip).
     * @param nc_pin  GPIO of the Normally-Closed contact (GPIO_NUM_NC = skip).
     */
    esp_err_t initEndstopIsr(gpio_num_t no_pin, gpio_num_t nc_pin);

    /**
     * Incremented in ISR each time encode_steps() finds the ring empty
     * and emits a pause chunk before stopping the transaction.
     * std::atomic for ISR ↔ task safety.
     */
    std::atomic<uint32_t> ring_underrun_count_ {0};

    uint32_t getUnderrunCount()  const { return ring_underrun_count_.load(std::memory_order_relaxed); }
    void     resetUnderrunCount()      { ring_underrun_count_.store(0, std::memory_order_relaxed); }
private:
    gpio_num_t            step_pin_;
    gpio_num_t            en_pin_;
    uint8_t               motor_id_;

    rmt_channel_handle_t  channel_   {nullptr};
    rmt_encoder_handle_t  encoder_   {nullptr};
    rmt_transmit_config_t tx_config_ {};

    std::atomic<bool>     rmt_running_ {false};
    bool                  last_dir_    {true};
    bool                  enabled_     {false};
    std::atomic<bool>     endstop_armed_  {false};

    /** Endstop pin numbers — set by initEndstopIsr(), read by endstopIsrHandler(). */
    gpio_num_t            endstop_no_pin_ {GPIO_NUM_NC};
    gpio_num_t            endstop_nc_pin_ {GPIO_NUM_NC};

    /** @brief Number of free slots in the ring buffer. */
    uint32_t ringFree() const {
        return STEP_RING_SIZE
            - (ring_write_.load(std::memory_order_relaxed)
               - ring_read_.load(std::memory_order_relaxed));
    }

    // ── Static ISR callbacks ────────────────────────────────────────────────
    static bool on_trans_done_isr(rmt_channel_handle_t tx_chan,
                                  const rmt_tx_done_event_data_t* edata,
                                  void* user_ctx);

    /**
     * @brief GPIO ISR — fires on any edge of either endstop pin (NO or NC).
     * arg = StepperDriver* that owns the endstop.
     * Validates NO/NC logic and sets endstop_active_ for sub-100 µs RMT stop.
     */
    static void IRAM_ATTR endstopIsrHandler(void* arg);
};

/**
 * @brief Simple encoder callback — called from ISR context by the RMT driver.
 *
 * Reads up to PART_SIZE entries from the ring buffer and converts them to
 * RMT symbols.  Uses coast-mode: on empty ring, emits pause symbols instead
 * of stopping the RMT transaction, eliminating restart overhead.
 */
extern "C" size_t encode_steps(const void* data, size_t data_size,
                                          size_t symbols_written,
                                          size_t symbols_free,
                                          rmt_symbol_word_t* symbols,
                                          bool* done, void* arg);
