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
        if (locked_) xSemaphoreGive(mutex_);
    }

    // Non-copyable
    RtosLockGuard(const RtosLockGuard&) = delete;
    RtosLockGuard& operator=(const RtosLockGuard&) = delete;

    explicit operator bool() const noexcept {
        return locked_;
    }

};

/**
 * @brief Convenience helper for taking mutex with milliseconds timeout.
 */
inline RtosLockGuard makeTimedLock(SemaphoreHandle_t mutex, uint32_t timeoutMs) {
    TickType_t ticks = pdMS_TO_TICKS(timeoutMs);
    if (timeoutMs > 0 && ticks == 0) ticks = 1;
    return RtosLockGuard(mutex, ticks);
}

#endif // RTOS_GUARD_H
