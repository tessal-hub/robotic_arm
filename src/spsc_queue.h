#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <esp_attr.h>

/**
 * @brief Lock-free Single-Producer Single-Consumer (SPSC) Ring Buffer
 * 
 * Invariants:
 * 1. m_head is ONLY written by the Producer (MotionTask on Core 1).
 * 2. m_flushEpoch is ONLY written by the Producer (MotionTask on Core 1).
 * 3. m_tail is ONLY written by the Consumer (StepISR on Core 1).
 * 4. Capacity MUST be a power of 2.
 */
template <typename T, size_t Capacity>
class SPSCQueue {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be a power of 2");

    // Producer Domain (Core 1 Motion Task) - Cache-line aligned
    alignas(64) std::atomic<size_t>   m_head{0};
    alignas(64) std::atomic<uint32_t> m_flushEpoch{0};

    // Consumer Domain (Core 1 Step ISR) - Cache-line aligned
    alignas(64) std::atomic<size_t>   m_tail{0};
    uint32_t                          m_localEpoch{0};

    // Storage buffer
    T m_buffer[Capacity];

public:
    SPSCQueue() = default;

    /**
     * @brief Invalidate all pending items via epoch increment (Producer only)
     * Does NOT touch m_tail to preserve Single-Writer invariant.
     */
    void IRAM_ATTR invalidate() {
        m_flushEpoch.fetch_add(1, std::memory_order_release);
    }

    /**
     * @brief Push an item into the queue (Producer only)
     * @return true if pushed, false if full
     */
    bool IRAM_ATTR push(const T& item) {
        const size_t head = m_head.load(std::memory_order_relaxed);
        const size_t tail = m_tail.load(std::memory_order_acquire);

        if ((head - tail) >= Capacity) {
            return false; // Queue is full
        }

        m_buffer[head & (Capacity - 1)] = item;
        m_head.store(head + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Pop an item from the queue (Consumer / ISR only)
     * @return true if popped, false if empty or invalidated
     */
    bool IRAM_ATTR pop(T& item) {
        // 1. Check for epoch invalidation from Producer
        const uint32_t currentEpoch = m_flushEpoch.load(std::memory_order_acquire);
        if (m_localEpoch != currentEpoch) {
            // Consumer safely updates its own m_tail to match m_head
            m_tail.store(m_head.load(std::memory_order_relaxed), std::memory_order_release);
            m_localEpoch = currentEpoch;
            return false; // Buffer flushed
        }

        const size_t tail = m_tail.load(std::memory_order_relaxed);
        if (tail == m_head.load(std::memory_order_acquire)) {
            return false; // Queue is empty
        }

        item = m_buffer[tail & (Capacity - 1)];
        m_tail.store(tail + 1, std::memory_order_release);
        return true;
    }

    /**
     * @brief Check if queue is empty (Approximate, non-blocking)
     */
    bool IRAM_ATTR isEmpty() const {
        return m_head.load(std::memory_order_relaxed) == m_tail.load(std::memory_order_relaxed);
    }

    /**
     * @brief Get current number of items in queue
     */
    size_t IRAM_ATTR size() const {
        const size_t head = m_head.load(std::memory_order_relaxed);
        const size_t tail = m_tail.load(std::memory_order_relaxed);
        return (head >= tail) ? (head - tail) : 0;
    }

    /**
     * @brief Get queue capacity
     */
    static constexpr size_t capacity() {
        return Capacity;
    }
};
