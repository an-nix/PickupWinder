/* command_queue.h — Lock-free SPSC ring buffer for Core 0 → Core 1.
 *
 * Single-Producer (Core 0 / SPI ISR) → Single-Consumer (Core 1 / stepper).
 * Wait-free: push and pop never block.
 * Cache-line padding not needed on ESP32 (no L2, Xtensa cores share SRAM).
 *
 * Uses acquire/release atomics for correct visibility across cores.
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>
#include "protocol.h"

template <size_t Capacity>
class CommandQueue {
    static_assert((Capacity & (Capacity - 1)) == 0,
                  "Capacity must be a power of 2");
public:
    CommandQueue() : head_(0), tail_(0) {}

    /// Push a command (producer side — Core 0).
    /// Returns false if queue is full.
    bool push(const CmdFrame& frame) {
        const uint32_t h = head_.load(std::memory_order_relaxed);
        const uint32_t next = (h + 1) & MASK;
        if (next == tail_.load(std::memory_order_acquire)) {
            return false;  // full
        }
        buf_[h] = frame;
        head_.store(next, std::memory_order_release);
        return true;
    }

    /// Pop a command (consumer side — Core 1).
    /// Returns false if queue is empty.
    bool pop(CmdFrame& out) {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        if (t == head_.load(std::memory_order_acquire)) {
            return false;  // empty
        }
        out = buf_[t];
        tail_.store((t + 1) & MASK, std::memory_order_release);
        return true;
    }

    /// Number of pending items.
    uint32_t size() const {
        uint32_t h = head_.load(std::memory_order_acquire);
        uint32_t t = tail_.load(std::memory_order_acquire);
        return (h - t) & MASK;
    }

    bool empty() const { return size() == 0; }
    bool full()  const { return size() == Capacity - 1; }

private:
    static constexpr uint32_t MASK = Capacity - 1;

    CmdFrame             buf_[Capacity];
    std::atomic<uint32_t> head_;
    std::atomic<uint32_t> tail_;
};

// Default queue: 16 slots should handle SPI bursts without overflow.
using CmdQueue = CommandQueue<16>;
