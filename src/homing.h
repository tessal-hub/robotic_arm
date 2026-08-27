#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include "config.h"
#include "endstop.h"

class Motor;
class Endstops;
class JointModel;

enum class HomePhase : uint8_t {
    IDLE = 0,
    SAFE_MODE,     // Stage 1: hạ dòng TMC2209
    WARMUP,        // Stage 2: xác định enc_dir_mult + hướng an toàn
    SCAN_MIN,      // Stage 3: quét tới endstop Min, lưu enc_min, reset step=0
    SCAN_MAX,      // Stage 4: quét tới endstop Max, lưu enc_max, step_max
    APPROACH,      // (legacy J3) chạy tới min endstop
    BACKOFF,       // (legacy J3) lùi khỏi endstop
    REAPPROACH,    // (legacy J3) dò lại chậm
    CENTERING,     // Stage 6: định tâm (thô + tinh) / (legacy J3 step-counted
    DONE
};

/**
 * FSM homing tuần tự J1 -> J2 -> J3 -> J4.
 *  - J1, J2: kiến trúc 7 giai đoạn — quét Min+Max (có cả 2 endstop),
 *            home tại TÂM cơ khí, đo enc_dir_mult + real_step_to_enc (áp dụng điều khiển).
 *  - J3:     home tại điểm riêng (endstop Min + offset) — giữ nguyên legacy.
 *  - J4:     stallguard bump (không endstop vật lý) — giữ nguyên legacy.
 *  - J5,J6:  A4988 — chỉ Set-Home thủ công qua web, FSM bỏ qua.
 * Phát hiện chạm: latch ISR endstop (nguồn sự thật duy nhất). Timeout mỗi khớp.
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
    // Khớp có ĐỦ 2 endstop Min+Max -> chạy kiến trúc quét 7 giai đoạn.
    [[nodiscard]] bool isScanAxis(uint8_t axis) const noexcept;
    // J1/J2: home tại TÂM cơ khí (step_max/2). J3: home tại điểm riêng gần MIN (Min + offset).
    [[nodiscard]] bool homeAtMinOffset(uint8_t axis) const noexcept;

    // ---- Scan path (J1, J2): kiến trúc 7 giai đoạn ----
    void beginScan();
    void enterWarmup();
    void enterScanMin();
    void enterScanMax();
    void enterCenteringScan();
    void tickScan(uint32_t now, Motor& m);

    // ---- Legacy path (J3, J4) ----
    void beginJoint(uint8_t axis);
    void enterApproach();
    void contactMade();
    void enterBackoff();
    void enterReapproach();
    bool gotoNearHome();
    void enterCentering();
    void tickLegacy(uint32_t now, Motor& m);

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

    // Scan state (J1,J2)
    float encMinRaw_{0.0f};
    float encMaxRaw_{0.0f};
    float encCenterRaw_{0.0f};
    float encBefore_{0.0f};
    int64_t stepMax_{0};
    float encDirMult_{1.0f};
    bool warmupCW_{false};
    bool centeringCoarse_{false};
    EndstopWhich minSide_{EndstopWhich::MIN}; // cực chạm đầu tiên (có thể là MAX nếu ngược chiều)

    // Legacy state (J3,J4)
    uint32_t stallStartMs_{0};
    float angleEncAtContact_{0.0f};
};

#endif // HOMING_H
