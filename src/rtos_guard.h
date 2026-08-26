#ifndef RTOS_GUARD_H
#define RTOS_GUARD_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

/**
 * @brief RAII Lock Guard for FreeRTOS Mutexes / Binary Semaphores.
 * Automatically acquires the semaphore on construction and releases it on destruction.
 */
class RtosLockGuard {
private:
    SemaphoreHandle_t mutex_;
    bool locked_;

public:
    explicit RtosLockGuard(SemaphoreHandle_t mutex, TickType_t timeoutTicks = portMAX_DELAY)
        : mutex_(mutex), locked_(false) {
        if (mutex_ != nullptr) {
            locked_ = (xSemaphoreTake(mutex_, timeoutTicks) == pdTRUE);
        }
    }

    ~RtosLockGuard() {
        unlock();
    }

    // Non-copyable, movable
    RtosLockGuard(const RtosLockGuard&) = delete;
    RtosLockGuard& operator=(const RtosLockGuard&) = delete;

    RtosLockGuard(RtosLockGuard&& other) noexcept
        : mutex_(other.mutex_), locked_(other.locked_) {
        other.mutex_ = nullptr;
        other.locked_ = false;
    }

    RtosLockGuard& operator=(RtosLockGuard&& other) noexcept {
        if (this != &other) {
            unlock();
            mutex_ = other.mutex_;
            locked_ = other.locked_;
            other.mutex_ = nullptr;
            other.locked_ = false;
        }
        return *this;
    }

    [[nodiscard]] bool isLocked() const noexcept {
        return locked_;
    }

    explicit operator bool() const noexcept {
        return locked_;
    }

    void unlock() noexcept {
        if (locked_ && mutex_ != nullptr) {
            xSemaphoreGive(mutex_);
            locked_ = false;
        }
    }
};

/**
 * @brief Convenience helper for taking mutex with milliseconds timeout.
 */
inline RtosLockGuard makeTimedLock(SemaphoreHandle_t mutex, uint32_t timeoutMs) {
    return RtosLockGuard(mutex, pdMS_TO_TICKS(timeoutMs));
}

#endif // RTOS_GUARD_H
