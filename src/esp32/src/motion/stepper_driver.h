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
 *   setup time, and the absolute DIR level from target_dir is applied on the
 *   next callback invocation. The driver keeps the applied GPIO state separate
 *   from the last commanded direction so queued reversals cannot desynchronize
 *   software state from the physical DIR pin.
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
    StepperDriver(gpio_num_t step_pin, gpio_num_t dir_pin,
                  gpio_num_t en_pin,   uint8_t    motor_id);

    esp_err_t init();
    void enable();
    void disable();
    bool isEnabled() const { return enabled_; }
    void emergencyStop();
    void stopStream();
    void gracefulStop();
    esp_err_t pushBlock(const step_block_t& block, TaskHandle_t caller_task);
    uint8_t motorId() const { return motor_id_; }
    esp_err_t startStream();
    uint32_t ringFreeSlots() const { return ringFree(); }
    bool isStreaming() const { return rmt_running_.load(std::memory_order_acquire); }
    bool isStopped()  const { return rmt_stopped_.load(std::memory_order_acquire); }
    void setAppliedDirection(bool direction) {
        applied_dir_.store(direction, std::memory_order_release);
    }

    ring_entry_t          ring_[STEP_RING_SIZE];
    std::atomic<uint32_t> ring_write_ {0};
    std::atomic<uint32_t> ring_read_  {0};

    gpio_num_t            dir_pin_;
    /// Physical inversion flag: when true, the logical DIR level is XOR'd
    /// before driving the GPIO pin. All endstop/direction logic operates on
    /// the logical (pre-XOR) value; only the GPIO output and ring target_dir
    /// see the physical level. Must be set before init() is called.
    bool                  invert_direction_ {false};

    void setInvertDirection(bool invert) { invert_direction_ = invert; }
    std::atomic<bool>     rmt_stopped_ {true};
    bool                  last_chunk_had_steps_ {false};
    uint16_t              last_ticks_ {RMT_STEP_DEFAULT_TICKS};
    uint32_t              coast_idle_count_ {0};

    std::atomic<TaskHandle_t> producer_task_ {nullptr};
    std::atomic<TaskHandle_t> executor_task_ {nullptr};

    void setExecutorTask(TaskHandle_t t) { executor_task_.store(t, std::memory_order_release); }

    enum class EndstopSignalState : uint8_t {
        OPEN = 0,
        CLOSED = 1,
        INVALID = 2,
    };

    std::atomic<bool> endstop_active_ {false};
    std::atomic<bool> endstop_clearance_pending_ {false};
    std::atomic<bool> endstop_clearance_direction_ {false};
    std::atomic<uint32_t> endstop_hit_count_ {0};
    std::atomic<uint8_t> endstop_signal_state_ {
        static_cast<uint8_t>(EndstopSignalState::OPEN)
    };
    std::atomic<uint8_t> endstop_last_stable_state_ {
        static_cast<uint8_t>(EndstopSignalState::OPEN)
    };
    std::atomic<TickType_t> endstop_invalid_since_tick_ {0};

    /**
     * @note After an endstop hit, endstop_active_ remains set and blocks
     * the RMT encoder (encode_steps returns done=true immediately).
     * The host MUST send ENABLE_ENDSTOP(arm=1) before issuing any backoff
     * motion — including the slow seek — so that armEndstop() clears
     * the latch and allows the RMT to resume. This is the expected
     * Klipper-style sequence:
     *   1. fast seek  → hit detected → endstop_active_ latched
     *   2. host sends ENABLE_ENDSTOP(arm=1)  → latch cleared, driver re-armed
     *   3. host sends backoff segments
     *   4. host sends ENABLE_ENDSTOP(arm=1) again before slow seek
     *   5. slow seek  → hit detected → latch set again
     *   6. host sends ENABLE_ENDSTOP(arm=0) to disarm after homing completes
     */
    void armEndstop() {
        endstop_active_.store(false, std::memory_order_release);
        endstop_hit_count_.store(0, std::memory_order_relaxed);
        endstop_closed_confirmations_ = 0;
        endstop_armed_.store(true, std::memory_order_release);
    }

    void disarmEndstop() {
        endstop_armed_.store(false, std::memory_order_release);
        endstop_active_.store(false, std::memory_order_release);
        endstop_clearance_pending_.store(false, std::memory_order_release);
        endstop_hit_count_.store(0, std::memory_order_relaxed);
        endstop_closed_confirmations_ = 0;
    }

    bool isEndstopArmed() const { return endstop_armed_.load(std::memory_order_acquire); }
    bool isEndstopActive() const { return endstop_active_.load(std::memory_order_acquire); }
    uint8_t reportedEndstopState() const;
    bool isEndstopMoveAllowed(bool direction) const;
    bool prepareEndstopMove(bool direction);

    void clearEndstopHit() {
        endstop_hit_count_.store(0, std::memory_order_relaxed);
    }

    uint32_t getEndstopHitCount() const {
        return endstop_hit_count_.load(std::memory_order_relaxed);
    }

    esp_err_t initEndstopIsr(gpio_num_t no_pin, gpio_num_t nc_pin);

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
    std::atomic<bool>     applied_dir_ {true};
    std::atomic<bool>     last_dir_commanded_ {true};
    bool                  enabled_     {false};
    std::atomic<bool>     endstop_armed_  {false};
    // Number of consecutive CLOSED readings required before latching
    // endstop_active_. The dual-contact NO/NC sensor fires two GPIO
    // edges almost simultaneously on actuation (~µs apart at hardware
    // level), so 2 confirmations filter single-edge spikes without
    // adding measurable stop latency. Do not raise above 3 — the ISR
    // fires on ANYEDGE of either pin, so a genuine hit produces at
    // most 2 rapid CLOSED edges before the ISR may see an INVALID
    // crossover transient that resets the counter.
    static constexpr uint8_t ENDSTOP_CLOSED_CONFIRM_COUNT = 3;
    static constexpr TickType_t ENDSTOP_INVALID_DEBOUNCE_TICKS = pdMS_TO_TICKS(5);
    volatile uint8_t      endstop_closed_confirmations_ {0};

    gpio_num_t            endstop_no_pin_ {GPIO_NUM_NC};
    gpio_num_t            endstop_nc_pin_ {GPIO_NUM_NC};

    uint32_t ringFree() const {
        return STEP_RING_SIZE
            - (ring_write_.load(std::memory_order_relaxed)
               - ring_read_.load(std::memory_order_relaxed));
    }

    static bool on_trans_done_isr(rmt_channel_handle_t tx_chan,
                                  const rmt_tx_done_event_data_t* edata,
                                  void* user_ctx);

    static void IRAM_ATTR endstopIsrHandler(void* arg);
};

extern "C" size_t encode_steps(const void* data, size_t data_size,
                                          size_t symbols_written,
                                          size_t symbols_free,
                                          rmt_symbol_word_t* symbols,
                                          bool* done, void* arg);
