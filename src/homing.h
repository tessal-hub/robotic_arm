#ifndef HOMING_H
#define HOMING_H

#include <Arduino.h>
#include "config.h"
#include "endstop.h"

class Motor;
class Endstops;
class JointModel;
class SafetyManager;

enum class HomePhase : uint8_t {
    IDLE = 0,
    WARMUP,        // Stage 1+2: hạ dòng + StealthChop, chạy +3° đo enc_dir_mult + hướng an toàn
    SCAN_MIN,      // Stage 3: quét FAST tới cữ đầu tiên (MIN hoặc MAX tùy lắp đặt)
    SCAN_BACKOFF,  // Stage 4: lùi khỏi cữ vừa chạm trước khi tiếp cận lại chậm
    SCAN_SLOW,     // Stage 5: tiếp cận lại cữ ở tốc độ chậm -> mốc chính xác
    SCAN_MAX,      // Stage 6: quét FAST tới cữ đối diện
    CENTERING,     // Stage 7: chạy về tâm cơ khí / MIN+offset (J3) bằng bước tương đối
    VERIFY,        // Stage 8: chờ EMA AS5600 ổn định, đối chiếu encoder vs vị trí mong đợi, trim nếu lệch
    DONE
};

/**
 * FSM homing tuần tự J1 -> J2 -> J3 -> J4 — kiến trúc quét 2 cữ, 2 tốc độ:
 *  - J1, J2, J3: endstop vật lý MIN+MAX — điểm chạm chậm xác định qua latch ISR (±1 bước).
 *  - J4:         không endstop — chạm xác định qua StallGuard (fast) + step-lag encoder
 *                (fast & slow); vị trí chạm slow được BÙ trừ độ trễ cửa sổ phát hiện.
 *  - J1/J2/J4:   home tại TÂM cơ khí (chạm chậm 2 cữ). J3: home tại MIN endstop + offset.
 *  - J5, J6:     A4988 — chỉ Set-Home thủ công qua web, FSM bỏ qua.
 * Chuỗi mỗi khớp: WARMUP -> SCAN_MIN(fast) -> BACKOFF -> SLOW -> SCAN_MAX(fast) ->
 * BACKOFF -> SLOW -> crosscheck -> CENTERING -> VERIFY -> SETREF.
 * Pha FAST chỉ tìm thô — glitch giữa đường tự hồi phục vì pha SLOW dò lại đúng cữ đó.
 * VERIFY đối chiếu encoder độc lập (trim tối đa 2 lần); FAIL -> retry khớp
 * (HOMING_MAX_ATTEMPTS) rồi mới hủy chuỗi. tick() phải được gọi định kỳ từ motion task
 * (MOTION_TASK_PERIOD_MS). Phát hiện chạm: latch ISR endstop là nguồn sự thật; stall
 * detection chỉ là fallback/fusion. Timeout mỗi khớp + mỗi pha scan.
 */
class HomingController {
public:
    HomingController();

    HomingController(const HomingController&) = delete;
    HomingController& operator=(const HomingController&) = delete;

    void begin(Motor** motors, Endstops* endstops, JointModel* joints);
    void setSafetyManager(SafetyManager* s) noexcept { safety_ = s; }

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
    // J1/J2/J4 (0,1,3): home = TÂM cơ khí giữa 2 cữ. J3 (2): home = endstop MIN + offset (2.5°).
    [[nodiscard]] bool homeAtMinOffset(uint8_t axis) const noexcept;

    void beginScan();
    void enterWarmup();
    void enterScanMin();
    void enterScanMax();
    void enterScanBackoff();
    void enterScanSlow();
    void enterCenteringScan();
    void enterVerify();
    void tickScan(uint32_t now, Motor& m);

    // Đặt lại mốc cửa sổ step-lag (gọi khi bắt đầu mỗi pha quét/tiếp cận).
    void resetStallWindow(Motor& m);
    // Cửa sổ step-lag: motor đã phát đủ windowSteps mà encoder dịch < ngưỡng => stall.
    // Trả true khi stall (KHÔNG cuộn mốc — caller dùng lastCheckSteps_ + encDelta để bù
    // vị trí chạm); encDeltaDeg = độ dịch encoder trong cửa sổ. Rotor còn chạy tự do thì
    // tự cuộn mốc và trả false.
    [[nodiscard]] bool stallWindowCheck(uint8_t axis, Motor& m, float& encDeltaDeg);

    void finishJoint(bool ok);
    void retryOrFail();
    void restoreDriverDefaults(uint8_t axis);

    Motor* motors[NUM_MOTORS]{};
    Endstops* es{nullptr};
    JointModel* jm{nullptr};
    SafetyManager* safety_{nullptr};

    bool active_{false};
    bool lastOk_{true};
    uint8_t seq_[4]{};
    uint8_t seqLen_{0};
    uint8_t seqIdx_{0};
    uint8_t attempt_{0};       // số lần đã thử khớp hiện tại (retry trước khi hủy chuỗi)
    uint8_t curAxis_{0};
    HomePhase phase_{HomePhase::IDLE};
    uint32_t phaseStartMs_{0};
    uint32_t lastPollMs_{0};
    uint8_t tmcStallCount_{0};

    // Scan state (J1..J4)
    float encFirstRaw_{0.0f};   // raw encoder tại cữ ĐẦU TIÊN (chạm chậm)
    float encSecondRaw_{0.0f};  // raw encoder tại cữ THỨ HAI (chạm chậm)
    float encCenterRaw_{0.0f};
    float encBefore_{0.0f};
    int64_t contactSpan_{0};    // khoảng cách bước (đã bù) giữa 2 điểm chạm chậm
    float encDirMult_{1.0f};
    bool warmupCW_{false};
    bool warmupProbed_{false}; // đã thử chiều ngược khi công tắc vẫn nhấn sau bước warmup
    uint32_t warmupSteps_{0};  // số bước warmup (probe chiều ngược chạy lại đúng khoảng này)
    // FIX #1: settle flag — chờ EMA encoder ổn định SAU KHI motor dừng, trước khi lấy mẫu
    bool warmupSettling_{false};      // đang trong giai đoạn settle sau khi motor warmup dừng
    uint32_t warmupSettleStartMs_{0}; // mốc thời gian bắt đầu settle
    // FIX #6: ghi nhớ trạng thái endstop lúc bắt đầu WARMUP — dùng để chọn hướng probe
    bool warmupFromMinP_{false};      // MIN endstop đang nhấn khi bắt đầu enterWarmup()
    bool warmupFromMaxP_{false};      // MAX endstop đang nhấn khi bắt đầu enterWarmup()
    bool secondSide_{false};    // false: đang dò cữ đầu tiên; true: cữ thứ hai
    bool cwApproach_{false};    // chiều quét hiện tại (fast & slow cùng chiều; backoff đảo)
    uint8_t backoffExtend_{0};  // số lần đã nới rộng backoff cho pha hiện tại (chống hysteresis công tắc)
    float backoffStartEnc_{0.0f}; // encoder tại thời điểm BẮT ĐẦU backoff (chẩn đoán stall)
    EndstopWhich firstSide_{EndstopWhich::MIN};     // cữ chạm ĐẦU TIÊN
    EndstopWhich approachSide_{EndstopWhich::MIN};  // cữ đang dò (backoff/slow)
    uint32_t settleStartMs_{0};

    // Verify state
    float targetEncRaw_{0.0f};  // raw encoder mong đợi tại home
    uint8_t trimCount_{0};      // số lần đã trim chậm theo encoder
    float trimStartErr_{0.0f};  // |err| lúc bắt đầu lần trim hiện tại (guard chống diverge)
    int64_t trimStartSteps_{0}; // vị trí bước lúc bắt đầu trim (guard giới hạn hành trình)

    // Stall detection (fast + slow dùng chung)
    float lastStallEnc_{0.0f};
    uint32_t lastStallCheckMs_{0};
    int64_t lastCheckSteps_{0};
    float lastCheckEnc_{0.0f};
    uint8_t encStallCount_{0};
};

#endif // HOMING_H
