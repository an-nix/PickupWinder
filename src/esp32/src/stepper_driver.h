/**
 * @file stepper_driver.h
 * @brief Physical-layer RMT stepper driver — one instance per motor axis.
 *
 * Converts pre-timed step_block_t arrays into a gapless RMT symbol stream
 * using a simple_encoder callback (FastAccelStepper-style ping-pong).
 *
 * ── Streaming architecture ─────────────────────────────────────────────────
 *   Producer (pushBlock, task context):
 *     1. Convert step_block_t → ring_entry_t[] in the lock-free ring buffer.
 *     2. If the ring is full, block on a task notification from the ISR.
 *     3. If RMT is not streaming, start a new rmt_transmit().
 *
 *   Consumer (encode_steps callback, ISR context):
 *     4. RMT hardware calls encode_steps() when it needs more symbols.
 *     5. Callback reads PART_SIZE entries from the ring buffer, converts
 *        each to one rmt_symbol_word_t (PULSE_TICKS HIGH, remainder LOW).
 *     6. On starvation, callback emits one pause chunk, arms stop, and ends
 *        the transaction on the next callback (FastAccelStepper-style).
 *
 *   on_trans_done ISR:
 *     7. Marks rmt_running_ = false so the next pushBlock() restarts.
 *
 *   This eliminates inter-block gaps in the normal case without task-side
 *   busy-wait loops.
 *
 * ── Direction constraint ───────────────────────────────────────────────────
 *   Direction changes are handled in ISR context via gpio_ll (register-level).
 *   When a ring entry has toggle_dir=1 and the previous chunk contained step
 *   pulses, a pause chunk is emitted first to meet the driver IC's direction
 *   setup time, and the toggle is deferred to the next callback invocation.
 */

#pragma once

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
    static constexpr uint32_t RMT_CLK_HZ          = 80000000UL;          // 80 MHz
    static constexpr uint32_t RMT_TICKS_PER_US_C  = RMT_CLK_HZ / 1000000UL; // 80
    static constexpr uint32_t RMT_PULSE_TICKS_C   = 8U;   // 8 × 12.5 ns = 100 ns HIGH
    static constexpr uint32_t RMT_MIN_TICKS_C     = 16U;  // 16 × 12.5 ns = 200 ns → 5 MHz ceiling
    static constexpr uint32_t RMT_MAX_TICKS_C     = 0xFFFFU;
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
    bool isStreaming() const { return rmt_running_; }

    // ─── Ring buffer (SPSC: task writes, ISR reads) ────────────────────────
    // Public because the C encoder callback needs direct access in ISR context.

    ring_entry_t          ring_[STEP_RING_SIZE];     /**< Step ring buffer      */
    volatile uint32_t     ring_write_ {0};           /**< Write idx (task only) */
    volatile uint32_t     ring_read_  {0};           /**< Read idx  (ISR only)  */

    gpio_num_t            dir_pin_;                  /**< For ISR gpio_ll       */
    volatile bool         rmt_stopped_ {true};       /**< Set by encoder ISR    */
    bool                  last_chunk_had_steps_ {false}; /**< Dir-change safety */
    uint16_t              last_ticks_ {RMT_STEP_DEFAULT_TICKS}; /**< Last step interval for hold symbols */

    /**
     * @brief Task handle of the current ring producer.
     *
     * Set to the calling task every time pushBlock() is entered so the ring
     * back-pressure (ulTaskNotifyTake) always wakes the correct task.
     * Written from task context, read from ISR — must be treated as volatile.
     */
    TaskHandle_t          producer_task_ {nullptr};

    /**
     * @brief Task handle of the multi-axis executor (Core 1).
     *
     * Set once at startup by CommInterface via setExecutorTask().
     * encode_steps notifies BOTH this handle and producer_task_ so the
     * executor can pre-emptively refill the ring before it stalls.
     */
    TaskHandle_t          executor_task_ {nullptr};

    /** Register the multi-axis executor task handle for ring-low wakeups. */
    void setExecutorTask(TaskHandle_t t) { executor_task_ = t; }

    // Incremented in ISR each time encode_steps() finds the ring empty
    // and emits a pause chunk before stopping the transaction. Use to detect
    // pipeline starvation at runtime.
    volatile uint32_t     ring_underrun_count_ {0};

    uint32_t getUnderrunCount()  const { return ring_underrun_count_; }
    void     resetUnderrunCount()      { ring_underrun_count_ = 0; }
private:
    gpio_num_t            step_pin_;
    gpio_num_t            en_pin_;
    uint8_t               motor_id_;

    rmt_channel_handle_t  channel_   {nullptr};
    rmt_encoder_handle_t  encoder_   {nullptr};
    rmt_transmit_config_t tx_config_ {};

    volatile bool         rmt_running_ {false};
    bool                  last_dir_    {true};
    bool                  enabled_     {false};

    /** @brief Number of free slots in the ring buffer. */
    uint32_t ringFree() const {
        return STEP_RING_SIZE - (ring_write_ - ring_read_);
    }

    // ── Static ISR callbacks ────────────────────────────────────────────────
    static bool on_trans_done_isr(rmt_channel_handle_t tx_chan,
                                  const rmt_tx_done_event_data_t* edata,
                                  void* user_ctx);
};

/**
 * @brief Simple encoder callback — called from ISR context by the RMT driver.
 *
 * Reads up to PART_SIZE entries from the ring buffer and converts them to
 * RMT symbols.  Handles direction changes with safety pauses.
 */
extern "C" size_t encode_steps(const void* data, size_t data_size,
                                          size_t symbols_written,
                                          size_t symbols_free,
                                          rmt_symbol_word_t* symbols,
                                          bool* done, void* arg);
