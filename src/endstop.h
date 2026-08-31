#ifndef ENDSTOP_H
#define ENDSTOP_H

#include <Arduino.h>
#include <atomic>
#include "config.h"

#ifndef IRAM_ATTR
#define IRAM_ATTR
#endif

class Motor;
class SafetyManager;

enum class EndstopWhich : uint8_t { MIN = 0, MAX = 1 };

/**
 * Quản lý 6 công tắc hành trình (J1..J3, mỗi khớp MIN+MAX).
 * - Input pull-up, kích hoạt mức LOW (ENDSTOP_ACTIVE_STATE).
 * - ISR tối minimal: chỉ ghi pending + timestamp (<3µs), không delay, không gpio_get_level.
 * - Debounce 50ms + xác nhận mức LOW được thực hiện ở SafetyManager::pollEndstops() @100Hz.
 * - Latch do SafetyManager quản lý; Endstops giữ mirror để hỗ trợ homing/arm interim.
 * - SafetyManager là single owner: ISR forward qua isrNotify, poll quyết định latch/E_STOP.
 */
class Endstops {
public:
    Endstops();

    Endstops(const Endstops&) = delete;
    Endstops& operator=(const Endstops&) = delete;

    // Khởi tạo GPIO + attach interrupt. motors: mảng NUM_MOTORS con trỏ để clear latches (không còn stopFromISR trong ISR).
    void begin(Motor** motors);

    // Inject SafetyManager — ISR sẽ forward pending/time qua safety->isrNotify.
    void setSafetyManager(SafetyManager* sm) noexcept;

    [[nodiscard]] bool hasPin(uint8_t axis, EndstopWhich w) const noexcept;
    [[nodiscard]] bool isPressed(uint8_t axis, EndstopWhich w) const noexcept;
    [[nodiscard]] bool isLatched(uint8_t axis, EndstopWhich w) const noexcept;

    // Đọc và xoá latch (FSM dùng). Trả về true nếu latch đang set trước khi xoá.
    bool consumeLatch(uint8_t axis, EndstopWhich w) noexcept;
    void clearLatch(uint8_t axis, EndstopWhich w) noexcept;
    void clearAllLatches() noexcept;

    [[nodiscard]] bool anyLatched() const noexcept;

    // Host/debug: ISR pending state
    [[nodiscard]] bool isrPending(uint8_t axis, EndstopWhich w) const noexcept;
    [[nodiscard]] int64_t isrTimeUs(uint8_t axis, EndstopWhich w) const noexcept;

    String toJson() const;

private:
    struct Channel {
        int8_t pin{-1};
        std::atomic<bool> latched{false};
    };
    struct IsrCtx {
        std::atomic<bool> pending{false};
        std::atomic<int64_t> isrTime{0};
        uint8_t axis{0};
        EndstopWhich which{EndstopWhich::MIN};
        SafetyManager* safety{nullptr};
        Endstops* self{nullptr};
    };

    static void IRAM_ATTR isrHandler(void* arg);

    void installPin(uint8_t axis, EndstopWhich w);
    [[nodiscard]] Channel& ch(uint8_t axis, EndstopWhich w) noexcept {
        return (w == EndstopWhich::MIN) ? minCh[axis] : maxCh[axis];
    }
    [[nodiscard]] const Channel& ch(uint8_t axis, EndstopWhich w) const noexcept {
        return (w == EndstopWhich::MIN) ? minCh[axis] : maxCh[axis];
    }

    Channel minCh[NUM_MOTORS];
    Channel maxCh[NUM_MOTORS];
    IsrCtx ctx[NUM_MOTORS][2];
    Motor* owner[NUM_MOTORS]{};
    SafetyManager* safety_{nullptr};
    std::atomic<bool> initialized{false};
};

#endif // ENDSTOP_H
