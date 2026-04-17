#pragma once

#include <cstdint>
#include <cstddef>
#include <atomic>

/**
 * Lock-free single-producer / single-consumer ring buffer.
 * Designed for one task pushing commands and one task consuming them.
 */
template <typename T, size_t Capacity>
class CommandQueue {
    static_assert((Capacity & (Capacity - 1)) == 0,
                  "Capacity must be a power of 2");
public:
    CommandQueue() : head_(0), tail_(0) {}

    bool push(const T& item) {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        const uint32_t next = (head + 1) & MASK;
        if (next == tail_.load(std::memory_order_acquire)) {
            return false; // full
        }
        buf_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    bool pop(T& out) {
        const uint32_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) {
            return false; // empty
        }
        out = buf_[tail];
        tail_.store((tail + 1) & MASK, std::memory_order_release);
        return true;
    }

    uint32_t size() const {
        const uint32_t head = head_.load(std::memory_order_acquire);
        const uint32_t tail = tail_.load(std::memory_order_acquire);
        return (head - tail) & MASK;
    }

    bool empty() const { return size() == 0; }
    bool full() const { return size() == Capacity - 1; }

private:
    static constexpr uint32_t MASK = Capacity - 1;
    T                    buf_[Capacity];
    std::atomic<uint32_t> head_;
    std::atomic<uint32_t> tail_;
};
