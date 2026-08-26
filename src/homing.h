#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include "config.h"

class Motor;
class Endstops;
class JointModel;

enum class HomePhase : uint8_t {
    IDLE = 0,     // không chạy
    APPROACH,     // chạy chậm về phía min endstop / stall
    BACKOFF,      // lùi ra khỏi công tắc HOME_BACKOFF_DEG
    CENTERING,    // J1/J2: di chuyển nửa hành trình để về vị trí home trung tâm
    DONE          // khớp hiện tại xong (chờ advance)
};

/**
 * FSM homing tuần tự J1 -> J2 -> J3 -> J4.
 *  - J1/J2: min-stop + backoff + di chuyển stroke/2 => home ở GIỮA hành trình.
 *  - J3:    min-stop + backoff       => home ngay tại điểm duỗi thẳng (+backoff).
 *  - J4:    stallguard bump (không có endstop vật lý).
 *  - J5/J6: A4988 — chỉ Set-Home thủ công qua web, FSM bỏ qua.
 * Phát hiện chạm: latch ISR endstop HOẶC StallGuard (SG_RESULT < STALL_SG_LEVEL
 * trong STALL_CONSECUTIVE_POLLS lần poll liên tiếp). Timeout mỗi khớp.
 * tick() phải được gọi định kỳ từ motion task (MOTION_TASK_PERIOD_MS).
 */
class HomingController {
public:
    HomingController();

    HomingController(const HomingController&) = delete;
    HomingController& operator=(const HomingController&) = delete;

    void begin(Motor** motors, Endstops* endstops, JointModel* joints);

    [[nodiscard]] bool startAll();               // chuỗi J1..J4
    [[nodiscard]] bool startAxis(uint8_t axis);  // một khớp (chỉ 0..3)
    void cancel();

    void tick();

    [[nodiscard]] bool isActive() const noexcept { return active_; }
    [[nodiscard]] bool lastRunOK() const noexcept { return lastOk_; }
    [[nodiscard]] uint8_t currentAxis() const noexcept { return curAxis_; }
    [[nodiscard]] HomePhase phase() const noexcept { return phase_; }

    String toJson() const;

private:
    void beginJoint(uint8_t axis);
    void enterApproach();
    void contactMade();
    void enterBackoff();
    void enterCentering();
    void finishJoint(bool ok);
    void restoreDriverDefaults(uint8_t axis);

    Motor* motors[NUM_MOTORS]{};
    Endstops* es{nullptr};
    JointModel* jm{nullptr};

    bool active_{false};
    bool lastOk_{true};
    uint8_t seq_[4]{};
    uint8_t seqLen_{0};
    uint8_t seqIdx_{0};
    uint8_t curAxis_{0};
    HomePhase phase_{HomePhase::IDLE};
    uint32_t phaseStartMs_{0};
    uint32_t lastPollMs_{0};
    uint8_t stallCount_{0};
};

#endif // HOMING_H
