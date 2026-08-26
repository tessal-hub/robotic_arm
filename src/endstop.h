#ifndef ENDSTOP_H
#define ENDSTOP_H

#include <Arduino.h>
#include <atomic>
#include "config.h"

class Motor;

enum class EndstopWhich : uint8_t { MIN = 0, MAX = 1 };

/**
 * Quản lý 6 công tắc hành trình (J1..J3, mỗi khớp MIN+MAX).
 * - Input pull-up, kích hoạt mức LOW (ENDSTOP_ACTIVE_STATE).
 * - ISR trên cạnh xuống: dừng ngay motor của trục đó (stopFromISR) + ghi latch.
 * - Debounce mềm: bỏ qua cạnh trong khoảng ENDSTOP_DEBOUNCE_US.
 * - Latch do FSM homing chủ động đọc/xoá; trạng thái tức thời phục vụ JSON/Web/E-stop.
 */
class Endstops {
public:
    Endstops();

    Endstops(const Endstops&) = delete;
    Endstops& operator=(const Endstops&) = delete;

    // Khởi tạo GPIO + attach interrupt. motors: mảng NUM_MOTORS con trỏ để ISR abort.
    void begin(Motor** motors);

    [[nodiscard]] bool hasPin(uint8_t axis, EndstopWhich w) const noexcept;
    [[nodiscard]] bool isPressed(uint8_t axis, EndstopWhich w) const noexcept;
    [[nodiscard]] bool isLatched(uint8_t axis, EndstopWhich w) const noexcept;

    // Đọc và xoá latch (FSM dùng). Trả về true nếu latch đang set trước khi xoá.
    bool consumeLatch(uint8_t axis, EndstopWhich w) noexcept;
    void clearLatch(uint8_t axis, EndstopWhich w) noexcept;

    [[nodiscard]] bool anyLatched() const noexcept;

    String toJson() const;

private:
    struct Channel {
        int8_t pin{-1};
        volatile bool latched{false};
        volatile int64_t lastEdgeUs{0};
    };
    struct IsrCtx {
        Endstops* self;
        uint8_t axis;
        EndstopWhich which;
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
    std::atomic<bool> initialized{false};
};

#endif // ENDSTOP_H
