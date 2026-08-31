#include "homing.h"
#include "endstop.h"
#include "joint_model.h"
#include "motor.h"
#include "safety_manager.h"

#include <algorithm>

namespace {
// Ân hạn khởi động trước khi bật phát hiện stall: TMC2209 trả SG_RESULT=0 lúc chưa
// sinh đủ Back-EMF (lỗi false-trip 30ms đã gặp trên J4).
constexpr uint32_t FAST_CONTACT_GRACE_MS = 600;
constexpr uint32_t SLOW_CONTACT_GRACE_MS = 400;
// Tỷ số measuredSpd/config chấp nhận được (ngoài khoảng -> giữ config)
constexpr float MEASURED_RATIO_MIN = 0.5f;
constexpr float MEASURED_RATIO_MAX = 2.0f;
// Timeout cho các pha quét toàn hành trình (dài hơn timeout khớp thường)
constexpr uint32_t SCAN_TIMEOUT_MS = 60000;
// Home của J3 = endstop MIN + offset (độ)
constexpr float HOME_OFFSET_FROM_MIN_DEG = HOME_BACKOFF_DEG + 0.5f;
// Khoảng nghỉ cho bộ lọc EMA của AS5600 (50Hz, alpha=0.2) ổn định sau khi motor dừng
constexpr uint32_t ENC_SETTLE_MS = 350;
// Deadzone encoder khi đo hướng warmup — dưới ngưỡng coi như encoder không phản hồi.
// Mỗi khớp có gear + đặc tính encoder khác nhau; dùng mảng riêng để tránh
// ngưỡng cứng gây false-fail trên khớp gear nhỏ (J1: 6:1) hoặc EMA chậm.
constexpr float ENC_DIR_DEADZONE_DEG[NUM_MOTORS] = {
    0.30f,  // J1: gear 6:1, encoder 1:1 với khớp → 3° khớp là đủ
    0.30f,  // J2: gear 20:1 — encoder thực ra rất nhạy
    0.30f,  // J3: gear 20:1
    0.20f,  // J4: encoder gắn motor 4:1 → warmup đo góc motor ~ 12° (3° khớp × 4)
    0.30f,  // J5: A4988, không homing FSM
    0.30f,  // J6: A4988, không homing FSM
};
// Nhịp log tiến trình serial
constexpr uint32_t PROGRESS_LOG_MS = 2000;
// Thời gian settle sau khi motor dừng trong WARMUP trước khi đọc encoder
constexpr uint32_t WARMUP_ENC_SETTLE_MS = 200;

} // namespace

HomingController::HomingController() {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = nullptr;
}

void HomingController::begin(Motor** motors_, Endstops* endstops_, JointModel* joints_) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = motors_[i];
    es = endstops_;
    jm = joints_;
}

bool HomingController::startAll() {
    if (active_) return false;
    seqLen_ = 4;
    for (uint8_t i = 0; i < seqLen_; ++i) seq_[i] = i; // J1..J4
    seqIdx_ = 0;
    attempt_ = 0;
    active_ = true;
    lastOk_ = true;
    if (safety_) safety_->assertHoming(true);
    beginScan();
    return true;
}

bool HomingController::startAxis(uint8_t axis) {
    if (active_) return false;
    if (axis >= 4 || motors[axis] == nullptr) return false; // J5/J6: manual only
    seqLen_ = 1;
    seq_[0] = axis;
    seqIdx_ = 0;
    attempt_ = 0;
    active_ = true;
    lastOk_ = true;
    if (safety_) safety_->assertHoming(true);
    beginScan();
    return true;
}

bool HomingController::homeAtMinOffset(uint8_t axis) const noexcept {
    // J3 (2): home = endstop MIN + offset (2.5°). J1/J2/J4: home = TÂM cơ khí giữa 2 cữ.
    return axis == 2;
}

// ===========================================================================
// SCAN PATH (J1..J4) — quét 2 cữ, 2 tốc độ
// ===========================================================================

void HomingController::beginScan() {
    curAxis_ = seq_[seqIdx_];
    if (jm != nullptr) jm->resetHomingCalibration(curAxis_);
    tmcStallCount_ = 0;
    warmupProbed_ = false;
    backoffExtend_ = 0;
    secondSide_ = false;
    firstSide_ = EndstopWhich::MIN;
    approachSide_ = EndstopWhich::MIN;
    cwApproach_ = false;
    encFirstRaw_ = 0.0f;
    encSecondRaw_ = 0.0f;
    encCenterRaw_ = 0.0f;
    contactSpan_ = 0;
    targetEncRaw_ = 0.0f;
    trimCount_ = 0;
    warmupSettling_ = false;
    warmupSettleStartMs_ = 0;
    settleStartMs_ = 0;
    pendingBackoffSteps_ = 0;
    pendingBackoffCw_ = false;
    phaseStartMs_ = millis();
    lastPollMs_ = millis();
    // Xoá latch cũ không phải press thật (cả MIN và MAX)
    for (const auto w : {EndstopWhich::MIN, EndstopWhich::MAX}) {
        if (es->hasPin(curAxis_, w) && es->isLatched(curAxis_, w) &&
            !es->isPressed(curAxis_, w)) {
            es->clearLatch(curAxis_, w);
        }
    }
    enterWarmup();
}

void HomingController::enterWarmup() {
    Motor& m = *motors[curAxis_];
    // SAFE_MODE: hạ dòng TMC2209 (A4988: Vref cứng, bỏ qua) + StealthChop bật StallGuard4
    if (m.isTmc()) {
        const uint16_t hc = DEFAULT_AXIS_HOMING_CURRENTS[curAxis_];
        if (hc > 0) m.setCurrent(hc);
        m.setChopperMode(false);
    }
    m.setSpeed(DEFAULT_AXIS_HOMING_SPEEDS[curAxis_]);
    // Chọn hướng an toàn — tránh BẤT KỲ endstop đang nhấn
    const bool minP = es->hasPin(curAxis_, EndstopWhich::MIN) &&
                      (es->isPressed(curAxis_, EndstopWhich::MIN) ||
                       es->isLatched(curAxis_, EndstopWhich::MIN));
    const bool maxP = es->hasPin(curAxis_, EndstopWhich::MAX) &&
                      (es->isPressed(curAxis_, EndstopWhich::MAX) ||
                       es->isLatched(curAxis_, EndstopWhich::MAX));
    // Số bước đủ lớn để encoder đo được (>2–3° góc khớp) — phụ thuộc gear ratio
    const float targetJointDeg = 3.0f;
    const float spd = JointModel::stepsPerDegree(curAxis_);
    const uint32_t warmupSteps = static_cast<uint32_t>(targetJointDeg * spd) + 1;

    if (minP && !maxP) {
        warmupCW_ = JointModel::cwForDelta(curAxis_, +targetJointDeg);  // lùi khỏi MIN -> chạy chiều dương (+)
    } else if (maxP && !minP) {
        warmupCW_ = JointModel::cwForDelta(curAxis_, -targetJointDeg);  // lùi khỏi MAX -> chạy chiều âm (-)
    } else {
        warmupCW_ = JointModel::cwForDelta(curAxis_, +targetJointDeg);  // mặc định -> chạy chiều dương (+)
    }
    // Ghi nhớ endstop nào đang nhấn lúc bắt đầu WARMUP — dùng trong probe logic để phân biệt:
    // (a) "đang thoát endstop ban đầu, thoát thất bại" → thử lùi xa hơn CÙNG chiều
    // (b) "va phải endstop bất ngờ sau warmup" → thử chiều ngược (kiểm tra model mapping)
    warmupFromMinP_ = minP;
    warmupFromMaxP_ = maxP;
    // FIX #1: encBefore_ lấy TRƯỚC m.run() — đây là baseline trước khi motor di chuyển.
    // Sau khi motor dừng, chờ thêm WARMUP_ENC_SETTLE_MS (200ms) cho EMA AS5600 ổn định
    // rồi mới đọc encAfter và tính delta. Trước đây delta = encAfter - encBefore_ đọc ngay
    // sau motor dừng (không settle) → EMA chưa hội tụ → delta < deadzone → false-fail.
    encBefore_ = (jm != nullptr) ? jm->rawEncoder(curAxis_) : 0.0f;
    warmupSteps_ = warmupSteps;
    warmupSettling_ = false;
    warmupSettleStartMs_ = 0;
    m.run(warmupCW_, warmupSteps);
    phase_ = HomePhase::WARMUP;
    phaseStartMs_ = millis();
    Serial.printf("[HOME] J%u: SAFE_MODE + WARMUP (cw=%d, %u steps, minP=%d, maxP=%d)\n",
                  curAxis_ + 1, static_cast<int>(warmupCW_), warmupSteps,
                  static_cast<int>(minP), static_cast<int>(maxP));
}

void HomingController::enterScanMin() {
    Motor& m = *motors[curAxis_];
    m.setSpeed(DEFAULT_AXIS_HOMING_SPEEDS[curAxis_]);
    // Xoá latch cũ trước khi bắt đầu quét liên tục
    if (es != nullptr) {
        es->clearLatch(curAxis_, EndstopWhich::MIN);
        es->clearLatch(curAxis_, EndstopWhich::MAX);
    }
    cwApproach_ = JointModel::cwForDelta(curAxis_, -360.0f);
    tmcStallCount_ = 0;
    resetStallWindow(m);
    m.runContinuous(cwApproach_);
    phase_ = HomePhase::SCAN_MIN;
    phaseStartMs_ = millis();
    lastPollMs_ = millis();
    Serial.printf("[HOME] J%u: SCAN_MIN fast (cw=%d)\n", curAxis_ + 1, static_cast<int>(cwApproach_));
}

void HomingController::enterScanMax() {
    Motor& m = *motors[curAxis_];
    m.setSpeed(DEFAULT_AXIS_HOMING_SPEEDS[curAxis_]);
    if (es != nullptr) {
        es->clearLatch(curAxis_, EndstopWhich::MIN);
        es->clearLatch(curAxis_, EndstopWhich::MAX);
    }
    // FIX #2: Reset backoffExtend_ khi chuyển sang quét cữ thứ hai.
    // Trước đây backoffExtend_ giữ nguyên từ lần backoff cữ ĐẦU TIÊN → bắt đầu
    // ở lần nới rộng giữa chừng thay vì từ mức cơ bản, gây "jammed" ảo.
    backoffExtend_ = 0;
    cwApproach_ = JointModel::cwForDelta(curAxis_, +360.0f);
    tmcStallCount_ = 0;
    resetStallWindow(m);
    m.runContinuous(cwApproach_);
    phase_ = HomePhase::SCAN_MAX;
    phaseStartMs_ = millis();
    lastPollMs_ = millis();
    Serial.printf("[HOME] J%u: SCAN_MAX fast (cw=%d)\n", curAxis_ + 1, static_cast<int>(cwApproach_));
}

void HomingController::enterScanBackoff() {
    Motor& m = *motors[curAxis_];
    m.setSpeed(DEFAULT_AXIS_HOMING_SPEEDS[curAxis_]);
    // Nới rộng mỗi lần lùi lại: 2.5° → 5° → 10° → 20° — đòn bẩy/cam công tắc có hysteresis
    // nhả lớn, lùi đúng 2.5° chưa chắc đã rời vùng actuation. Bậc 4 (20°) đủ cho hầu hết
    // đòn bẩy micro-switch; nếu vẫn nhấn → cơ khí kẹt thật.
    static constexpr float BACKOFF_EXTEND_SCALE[] = {1.0f, 2.0f, 4.0f, 8.0f};
    static_assert(sizeof(BACKOFF_EXTEND_SCALE)/sizeof(BACKOFF_EXTEND_SCALE[0]) > HOMING_BACKOFF_MAX_EXTEND,
                  "BACKOFF_EXTEND_SCALE phai co it nhat HOMING_BACKOFF_MAX_EXTEND+1 phan tu");
    const float scale = BACKOFF_EXTEND_SCALE[backoffExtend_];
    const int64_t steps = JointModel::degreesToSteps(curAxis_, HOME_OFFSET_FROM_MIN_DEG * scale);
    // FIX #3: Xoá latch của endstop approachSide_ trước khi lùi, tránh latch cũ
    // gây false-positive khi kiểm tra "cong tac van nhan" sau backoff.
    if (es != nullptr) es->clearLatch(curAxis_, approachSide_);
    // Non-blocking settle: thay delay(30) bằng BACKOFF_SETTLE_WAIT — motion task không bị block.
    pendingBackoffSteps_ = steps;
    pendingBackoffCw_ = !cwApproach_;
    phase_ = HomePhase::BACKOFF_SETTLE_WAIT;
    settleStartMs_ = millis();
    phaseStartMs_ = settleStartMs_;
    Serial.printf("[HOME] J%u: BACKOFF settle %u ms before %lld steps (cw=%d, lan %u)\n",
                  curAxis_ + 1, HOMING_BACKOFF_SETTLE_MS, static_cast<long long>(steps),
                  static_cast<int>(pendingBackoffCw_), backoffExtend_ + 1);
}

void HomingController::enterScanSlow() {
    Motor& m = *motors[curAxis_];
    m.setSpeed(HOMING_SLOW_SCAN_INTERVAL_US);
    if (es != nullptr) es->clearLatch(curAxis_, approachSide_);
    tmcStallCount_ = 0;
    resetStallWindow(m);
    m.runContinuous(cwApproach_);
    phase_ = HomePhase::SCAN_SLOW;
    phaseStartMs_ = millis();
    lastPollMs_ = millis();
    Serial.printf("[HOME] J%u: SCAN %s slow (cw=%d, %u us/step)\n",
                  curAxis_ + 1, secondSide_ ? "cữ thứ hai" : "cữ đầu tiên",
                  static_cast<int>(cwApproach_), HOMING_SLOW_SCAN_INTERVAL_US);
}

void HomingController::resetStallWindow(Motor& m) {
    lastStallCheckMs_ = millis();
    lastStallEnc_ = (jm != nullptr && jm->encOK(curAxis_)) ? jm->rawEncoder(curAxis_) : 0.0f;
    lastCheckSteps_ = m.getAbsoluteSteps();
    lastCheckEnc_ = lastStallEnc_;
    encStallCount_ = 0;
}

bool HomingController::stallWindowCheck(uint8_t axis, Motor& m, float& encDeltaDeg) {
    encDeltaDeg = 0.0f;
    if (jm == nullptr || !jm->encOK(axis)) return false;
    // Cửa sổ theo bước, scale theo gear ratio: độ dịch lý thuyết của cửa sổ phải vượt
    // ngưỡng encoder đủ xa (>= 2x) để chạy tự do không bị nhầm là stall.
    const int64_t windowSteps = std::max<int64_t>(
        HOMING_STALL_WINDOW_MIN_STEPS,
        JointModel::degreesToSteps(axis, HOMING_STALL_WINDOW_MIN_DEG));
    const int64_t cur = m.getAbsoluteSteps();
    const int64_t stepDelta = llabs(cur - lastCheckSteps_);
    if (stepDelta < windowSteps) return false;
    const float curEnc = jm->rawEncoder(axis);
    encDeltaDeg = fabsf(curEnc - lastCheckEnc_);
    if (encDeltaDeg >= HOMING_STALL_ENC_DELTA_DEG) {
        // Rotor đang di chuyển bình thường -> cuộn cửa sổ
        lastCheckSteps_ = cur;
        lastCheckEnc_ = curEnc;
        encStallCount_ = 0;
        return false;
    }
    // Encoder dịch < ngưỡng: tăng biến đếm xác nhận liên tiếp
    ++encStallCount_;
    if (encStallCount_ >= STALL_CONSECUTIVE_POLLS) {
        // Stall xác nhận: KHÔNG cuộn mốc — caller bù vị trí chạm
        return true;
    }
    // Cuộn mốc bước để kiểm tra cửa sổ tiếp theo
    lastCheckSteps_ = cur;
    lastCheckEnc_ = curEnc;
    return false;
}

void HomingController::enterCenteringScan() {
    Motor& m = *motors[curAxis_];
    // ---- Stage 5: crosscheck — tâm + tỷ số steps/độ thực tế từ góc tích lũy unwrapped ----
    encCenterRaw_ = (encFirstRaw_ + encSecondRaw_) / 2.0f;
    float spanRaw = fabsf(encSecondRaw_ - encFirstRaw_);

    const bool hasEndstop = es != nullptr && (es->hasPin(curAxis_, EndstopWhich::MIN) ||
                                              es->hasPin(curAxis_, EndstopWhich::MAX));

    // Integrity check:
    // Với trục không endstop (J4): kiểm tra contactSpan_ cơ khí phải >= 45° góc khớp (chặn kẹt cữ ảo).
    const int64_t minMechanicalSpanSteps = JointModel::degreesToSteps(curAxis_, 45.0f);
    if (!hasEndstop && contactSpan_ < minMechanicalSpanSteps) {
        Serial.printf("[HOME] J%u: CROSSCHECK contactSpan %lld < %lld steps (45 deg) — ket cu ao, HUY KHOP\n",
                      curAxis_ + 1, static_cast<long long>(contactSpan_),
                      static_cast<long long>(minMechanicalSpanSteps));
        finishJoint(false);
        return;
    }

    const float cfgSpd = (static_cast<float>(DEFAULT_FULL_STEPS) *
                          static_cast<float>(DEFAULT_MICROSTEPS) *
                          DEFAULT_AXIS_GEAR_RATIOS[curAxis_]) / 360.0f;
    float measuredSpd = cfgSpd;
    float ratio = 1.0f;
    if (spanRaw >= HOMING_MIN_ENC_SPAN_DEG[curAxis_]) {
        measuredSpd = static_cast<float>(contactSpan_) / spanRaw;
        ratio = (cfgSpd > 0.0f) ? (measuredSpd / cfgSpd) : 1.0f;
        if (ratio < MEASURED_RATIO_MIN || ratio > MEASURED_RATIO_MAX) {
            measuredSpd = cfgSpd;
            Serial.printf("[HOME] J%u: measured ratio %.2f ngoai [%g,%g] -> giu config\n",
                          curAxis_ + 1, ratio, MEASURED_RATIO_MIN, MEASURED_RATIO_MAX);
        }
    } else {
        measuredSpd = cfgSpd;
    }
    if (jm != nullptr) jm->applyHomingCalibration(curAxis_, encDirMult_, measuredSpd);
    Serial.printf("[HOME] J%u: CROSSCHECK enc_c=%.1f span=%lld spd=%.2f (x%.2f)\n",
                      curAxis_ + 1, encCenterRaw_, static_cast<long long>(contactSpan_),
                      measuredSpd, ratio);

    // ---- Chạy về home bằng bước tương đối (rotor đi tự do -> chính xác từng bước);
    //      VERIFY sẽ đối chiếu encoder độc lập và trim nếu steps/deg đo được còn lệch. ----
    int64_t stepsBack;
    if (homeAtMinOffset(curAxis_)) {
        const int64_t offsetSteps = JointModel::degreesToSteps(curAxis_, HOME_OFFSET_FROM_MIN_DEG);
        // firstSide == MIN: cữ chạm thứ hai là MAX, đang đứng ở MAX -> lùi (contactSpan_ - offsetSteps) về MIN + 2.5°.
        // firstSide == MAX: cữ chạm thứ hai là MIN, đang đứng ở MIN -> lùi offsetSteps về MIN + 2.5°.
        stepsBack = (firstSide_ == EndstopWhich::MIN) ? (contactSpan_ - offsetSteps) : offsetSteps;
    } else {
        stepsBack = contactSpan_ / 2;
    }
    if (stepsBack < 0) stepsBack = 0;
    const bool cwBack = !cwApproach_; // quay về phía cữ chạm đầu tiên
    m.setSpeed(DEFAULT_AXIS_HOMING_SPEEDS[curAxis_]);
    if (stepsBack > 0) m.run(cwBack, static_cast<uint32_t>(stepsBack));
    phase_ = HomePhase::CENTERING;
    phaseStartMs_ = millis();
    settleStartMs_ = phaseStartMs_;
    Serial.printf("[HOME] J%u: CENTERING %lld steps (cw=%d, enc_center=%.1f)\n",
                  curAxis_ + 1, static_cast<long long>(stepsBack),
                  static_cast<int>(cwBack), encCenterRaw_);
}

void HomingController::enterVerify() {
    if (jm == nullptr) { finishJoint(true); return; }

    if (!jm->encOK(curAxis_)) {
        Serial.printf("[HOME] J%u: VERIFY encoder mat ket noi — HUY KHOP\n", curAxis_ + 1);
        finishJoint(false);
        return;
    }

    const float raw = jm->rawEncoder(curAxis_);
    const bool hasEndstop = (es != nullptr) && (es->hasPin(curAxis_, EndstopWhich::MIN) ||
                                                es->hasPin(curAxis_, EndstopWhich::MAX));

    if (hasEndstop) {
        Serial.printf("[HOME] J%u: VERIFY OK (co endstop — chot home tai tam co khi, enc_zero=%.1f)\n",
                      curAxis_ + 1, raw);
    } else {
        Serial.printf("[HOME] J%u: VERIFY OK (sensorless hard-stop center, enc_zero=%.1f)\n",
                      curAxis_ + 1, raw);
    }
    finishJoint(true);
}

void HomingController::tickScan(uint32_t now, Motor& m) {
    switch (phase_) {
        case HomePhase::WARMUP: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { m.stop(); finishJoint(false); return; }
            if (m.isRunning()) return; // chờ motor hoàn tất
            const bool minStill = es->hasPin(curAxis_, EndstopWhich::MIN) &&
                                  es->isPressed(curAxis_, EndstopWhich::MIN);
            const bool maxStill = es->hasPin(curAxis_, EndstopWhich::MAX) &&
                                  es->isPressed(curAxis_, EndstopWhich::MAX);
            if (minStill || maxStill) {
                if (minStill && maxStill) {
                    Serial.printf("[HOME] J%u: WARMUP ca 2 endstop nhan — ket cuc/chap mach, HUY KHOP\n",
                                  curAxis_ + 1);
                    finishJoint(false);
                    return;
                }
                Serial.printf("[HOME] J%u: WARMUP tai vi tri endstop (%s) — tiep tuc scan\n",
                              curAxis_ + 1, minStill ? "MIN" : "MAX");
                enterScanMin();
                return;
            }
            // Motor đã dừng, công tắc đã nhả — chuyển sang settle wait non-blocking
            phase_ = HomePhase::WARMUP_SETTLE_WAIT;
            settleStartMs_ = now;
            warmupSettling_ = true;
            warmupSettleStartMs_ = now;
            phaseStartMs_ = now;
            return;
        }
        case HomePhase::WARMUP_SETTLE_WAIT: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { m.stop(); finishJoint(false); return; }
            if (now - settleStartMs_ < WARMUP_ENC_SETTLE_MS) return;
            // Kiểm tra dịch chuyển encoder sau khi motor phát bước warmup (~3°).
            const float encAfter = (jm != nullptr) ? jm->rawEncoder(curAxis_) : 0.0f;
            const float delta = encAfter - encBefore_;
            if (fabsf(delta) < ENC_DIR_DEADZONE_DEG[curAxis_]) {
                if (jm != nullptr && jm->encOK(curAxis_)) {
                    encDirMult_ = static_cast<float>(AXIS_ENC_SIGN[curAxis_]);
                    Serial.printf("[HOME] J%u: WARMUP encoder delta=%.2f < %.2f — fallback config encSign=%+.0f, tiep tuc scan\n",
                                  curAxis_ + 1, delta, ENC_DIR_DEADZONE_DEG[curAxis_], encDirMult_);
                    enterScanMin();
                    return;
                }
                Serial.printf("[HOME] J%u: WARMUP encoder mat ket noi (I2C loi) — HUY KHOP\n", curAxis_ + 1);
                finishJoint(false);
                return;
            }
            const float intendedJointSign = static_cast<float>(AXIS_STEP_SIGN[curAxis_]) *
                                             (warmupCW_ ? 1.0f : -1.0f);
            encDirMult_ = (delta * intendedJointSign >= 0.0f) ? 1.0f : -1.0f;
            Serial.printf("[HOME] J%u: WARMUP encDirMult=%+.0f (delta=%.2f, intent=%.0f, probe=%d)\n",
                          curAxis_ + 1, encDirMult_, delta, intendedJointSign,
                          static_cast<int>(warmupProbed_));
            enterScanMin();
            return;
        }
        case HomePhase::SCAN_MIN:
        case HomePhase::SCAN_MAX: {
            const bool firstLeg = (phase_ == HomePhase::SCAN_MIN);
            const EndstopWhich targetSide = firstLeg
                ? EndstopWhich::MIN
                : ((firstSide_ == EndstopWhich::MIN) ? EndstopWhich::MAX : EndstopWhich::MIN);
            if (now - phaseStartMs_ > SCAN_TIMEOUT_MS) {
                m.stop();
                Serial.printf("[HOME] J%u: SCAN_%s TIMEOUT — khong cham cu trong %us\n",
                              curAxis_ + 1, firstLeg ? "MIN" : "MAX", SCAN_TIMEOUT_MS / 1000);
                finishJoint(false); return;
            }
            if (now - lastPollMs_ > PROGRESS_LOG_MS) {
                lastPollMs_ = now;
                Serial.printf("[HOME] J%u: SCAN_%s progress step=%.1f enc=%.1f\n",
                              curAxis_ + 1, firstLeg ? "MIN" : "MAX",
                              jm->angleFromSteps(curAxis_), jm->rawEncoder(curAxis_));
            }

            // 1. Endstop vật lý (leg 1 chấp nhận cả 2 cữ — lắp ngược có thể chạm MAX trước)
            EndstopWhich hit = targetSide;
            bool hitAny = false;
            if (es != nullptr) {
                if (firstLeg) {
                    for (const auto w : {EndstopWhich::MIN, EndstopWhich::MAX}) {
                        if (es->hasPin(curAxis_, w) &&
                            (es->isLatched(curAxis_, w) || es->isPressed(curAxis_, w))) {
                            if (es->isPressed(curAxis_, w) || !m.isRunning()) {
                                hit = w;
                                hitAny = true;
                                break;
                            }
                            es->clearLatch(curAxis_, w); // latch nhiễu khi motor vẫn quay
                        }
                    }
                } else if (es->hasPin(curAxis_, targetSide) &&
                           (es->isLatched(curAxis_, targetSide) || es->isPressed(curAxis_, targetSide))) {
                    if (es->isPressed(curAxis_, targetSide) || !m.isRunning()) {
                        hitAny = true;
                        es->consumeLatch(curAxis_, targetSide);
                    } else {
                        es->clearLatch(curAxis_, targetSide);
                    }
                }
            }

            // Bảo vệ leg 2: StallGuard/step-lag chỉ bật sau khi đã rời cữ đầu đủ xa (> 10°)
            const int64_t curSteps = m.getAbsoluteSteps();
            bool movedFarEnough = true;
            if (!firstLeg) {
                const float minSpanDeg = 10.0f; // 10° là đủ để rời khỏi cữ chạm đầu tiên
                movedFarEnough = llabs(curSteps) >= JointModel::degreesToSteps(curAxis_, minSpanDeg);
                if (!movedFarEnough) {
                    resetStallWindow(m);
                }
            }

            // 2. TMC2209 StallGuard4 (chỉ pha FAST — ở tốc độ chậm SG không tin cậy)
            if (!hitAny && m.isTmc() && movedFarEnough &&
                (now - phaseStartMs_ > FAST_CONTACT_GRACE_MS) && (now - lastPollMs_ >= 20)) {
                lastPollMs_ = now;
                const uint16_t sg = m.getSGResult();
                if (sg <= STALL_SG_LEVEL && sg != 1023) {
                    ++tmcStallCount_;
                    if (tmcStallCount_ >= 2) {
                        hitAny = true;
                        Serial.printf("[HOME] J%u: SCAN_%s SG stall (sg=%u <= %u)\n",
                                      curAxis_ + 1, firstLeg ? "MIN" : "MAX", sg, STALL_SG_LEVEL);
                    }
                } else if (sg > STALL_SG_LEVEL && sg != 1023) {
                    tmcStallCount_ = 0;
                }
            }

            // 3. Step-lag encoder — CHỈ cho trục không endstop (J4, là sensor dò độc lập).
            //    Với J1..J3, encoder đóng băng (ACK mà góc đứng yên) sẽ tạo stall ảo chặn
            //    trước endstop thật → fake contact + fake span; pha FAST của trục có
            //    endstop chỉ tin endstop latch + StallGuard (2 sensor độc lập với encoder).
            //    Step-lag vẫn giữ ở pha SLOW làm fallback khi endstop đứt dây (span
            //    integrity check ở CROSSCHECK sẽ chặn fake span).
            float encDelta = 0.0f;
            const bool hasEndstop = es->hasPin(curAxis_, EndstopWhich::MIN) ||
                                    es->hasPin(curAxis_, EndstopWhich::MAX);
            if (!hitAny && !hasEndstop && movedFarEnough &&
                (now - phaseStartMs_ > FAST_CONTACT_GRACE_MS) &&
                stallWindowCheck(curAxis_, m, encDelta)) {
                hitAny = true;
                Serial.printf("[HOME] J%u: SCAN_%s stall (encDelta=%.2f < %.2f deg)\n",
                              curAxis_ + 1, firstLeg ? "MIN" : "MAX",
                              encDelta, HOMING_STALL_ENC_DELTA_DEG);
            }

            // 4. Motor dừng sớm mà không có contact nào (ISR dừng nhầm / UART từ chối chạy)
            if (!hitAny && !m.isRunning() && (now - phaseStartMs_ > 500)) {
                m.stop();
                Serial.printf("[HOME] J%u: SCAN_%s motor stopped early — khong xac dinh duoc cu\n",
                              curAxis_ + 1, firstLeg ? "MIN" : "MAX");
                finishJoint(false); return;
            }

            if (hitAny) {
                m.stop();
                if (es != nullptr && es->hasPin(curAxis_, hit)) {
                    es->consumeLatch(curAxis_, hit); // consume ngay, tránh latch cũ gây chạm ảo ở pha SLOW
                }
                approachSide_ = hit;
                Serial.printf("[HOME] J%u: SCAN_%s CONTACT (%s)\n",
                              curAxis_ + 1, firstLeg ? "MIN" : "MAX",
                              (hit == EndstopWhich::MIN) ? "MIN" : "MAX");
                enterScanBackoff();
            }
            return;
        }
        case HomePhase::BACKOFF_SETTLE_WAIT: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { m.stop(); finishJoint(false); return; }
            if (now - settleStartMs_ < HOMING_BACKOFF_SETTLE_MS) return;
            // Settle done — start backoff move non-blocking
            backoffStartEnc_ = (jm != nullptr && jm->encOK(curAxis_)) ? jm->rawEncoder(curAxis_) : 0.0f;
            Motor& m2 = *motors[curAxis_];
            m2.run(pendingBackoffCw_, static_cast<uint32_t>(pendingBackoffSteps_));
            phase_ = HomePhase::SCAN_BACKOFF;
            phaseStartMs_ = now;
            Serial.printf("[HOME] J%u: BACKOFF %lld steps (cw=%d, lan %u)\n",
                          curAxis_ + 1, static_cast<long long>(pendingBackoffSteps_),
                          static_cast<int>(pendingBackoffCw_), backoffExtend_ + 1);
            return;
        }
        case HomePhase::SCAN_BACKOFF: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { m.stop(); finishJoint(false); return; }
            if (m.isRunning()) return;
            // Lùi xong mà cữ vẫn bị đè: chưa chắc kẹt cơ khí — có thể hysteresis nhả của
            // công tắc lớn hơn khoảng lùi. Nới rộng (2.5°→5°→10°→20°) trước khi kết luận fail.
            // FIX #3: Latch đã được xoá trong enterScanBackoff() — chỉ kiểm tra isPressed()
            // (trạng thái thật), không để latch cũ gây false-positive "jammed".
            if (es->hasPin(curAxis_, approachSide_) && es->isPressed(curAxis_, approachSide_)) {
                // Log encoder delta để chẩn đoán: motor thực sự di chuyển (long-travel switch)
                // hay encoder không dịch (motor stall — cần báo cho owner kiểm tra cơ khí).
                if (jm != nullptr && jm->encOK(curAxis_)) {
                    const float encNow = jm->rawEncoder(curAxis_);
                    const float encMoved = fabsf(encNow - backoffStartEnc_);
                    Serial.printf("[HOME] J%u: BACKOFF cong tac van nhan (enc_moved=%.2f deg)%s\n",
                                  curAxis_ + 1, encMoved,
                                  (encMoved < 0.3f) ? " — CANH BAO: motor co the bi ket co hoc!" : "");
                } else {
                    Serial.printf("[HOME] J%u: BACKOFF cong tac van nhan\n", curAxis_ + 1);
                }
                if (backoffExtend_ < HOMING_BACKOFF_MAX_EXTEND) {
                    ++backoffExtend_;
                    Serial.printf("[HOME] J%u: BACKOFF noi rong lan %u\n",
                                  curAxis_ + 1, backoffExtend_ + 1);
                    enterScanBackoff();
                    return;
                }
                Serial.printf("[HOME] J%u: BACKOFF jammed sau %u lan noi rong — ket cuc, HUY KHOP\n",
                              curAxis_ + 1, backoffExtend_ + 1);
                finishJoint(false);
                return;
            }
            enterScanSlow();
            return;
        }
        case HomePhase::SCAN_SLOW: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { m.stop(); finishJoint(false); return; }
            if (now - lastPollMs_ > PROGRESS_LOG_MS) {
                lastPollMs_ = now;
                Serial.printf("[HOME] J%u: SCAN_SLOW progress step=%.1f enc=%.1f\n",
                              curAxis_ + 1, jm->angleFromSteps(curAxis_), jm->rawEncoder(curAxis_));
            }

            // 1. Endstop của cữ đang dò (J1..J3) — nguồn chính xác nhất
            bool hitEndstop = false;
            if (es != nullptr && es->hasPin(curAxis_, approachSide_) &&
                (es->isLatched(curAxis_, approachSide_) || es->isPressed(curAxis_, approachSide_))) {
                if (es->isPressed(curAxis_, approachSide_) || !m.isRunning()) {
                    hitEndstop = true;
                } else {
                    es->clearLatch(curAxis_, approachSide_);
                }
            }

            // 2. Step-lag encoder (primary cho J4; fallback endstop hỏng). Bỏ poll SG ở
            // pha SLOW: SG4 không tin cậy ở tốc độ thấp, dễ false-trip.
            float encDelta = 0.0f;
            bool hitStall = false;
            if (!hitEndstop && (now - phaseStartMs_ > SLOW_CONTACT_GRACE_MS) &&
                stallWindowCheck(curAxis_, m, encDelta)) {
                hitStall = true;
            }

            // 3. Motor dừng sớm không rõ lý do
            if (!hitEndstop && !hitStall && !m.isRunning() && (now - phaseStartMs_ > 500)) {
                Serial.printf("[HOME] J%u: SCAN_SLOW motor stopped early\n", curAxis_ + 1);
                finishJoint(false);
                return;
            }
            if (!hitEndstop && !hitStall) return;

            m.stop();
            const int64_t cur = m.getAbsoluteSteps();
            // Endstop latch: vị trí dừng = điểm chạm (ISR dừng gần tức thì).
            // Stall: bù độ trễ cửa sổ — rotor còn chạy tự do một đoạn encDelta trong
            // cửa sổ trước khi kẹt, điểm chạm thực ≈ mốc cửa sổ + encDelta quy đổi bước.
            int64_t contactStep = cur;
            if (hitStall) {
                const float jointDelta = encDelta;
                int64_t est = lastCheckSteps_ + JointModel::degreesToSteps(curAxis_, jointDelta);
                if (est < lastCheckSteps_) est = lastCheckSteps_;
                if (est > cur) est = cur;
                contactStep = est;
                Serial.printf("[HOME] J%u: SLOW stall (encDelta=%.2f, cur=%lld, contact=%lld)\n",
                              curAxis_ + 1, encDelta, static_cast<long long>(cur),
                              static_cast<long long>(contactStep));
            }
            if (hitEndstop && es != nullptr) es->consumeLatch(curAxis_, approachSide_);

            if (!secondSide_) {
                // Cữ ĐẦU TIÊN — gốc bước mới tại điểm chạm đã bù
                encFirstRaw_ = jm->rawEncoder(curAxis_);
                firstSide_ = approachSide_;
                secondSide_ = true;
                m.setAbsoluteSteps(cur - contactStep);
                Serial.printf("[HOME] J%u: SLOW CONTACT #1 (%s, enc=%.1f, contact=%lld)\n",
                              curAxis_ + 1, (approachSide_ == EndstopWhich::MIN) ? "MIN" : "MAX",
                              encFirstRaw_, static_cast<long long>(contactStep));
                enterScanMax();
            } else {
                // Cữ THỨ HAI — đóng gói quét
                encSecondRaw_ = jm->rawEncoder(curAxis_);
                contactSpan_ = llabs(contactStep);
                Serial.printf("[HOME] J%u: SLOW CONTACT #2 (%s, enc=%.1f, span=%lld)\n",
                              curAxis_ + 1, (approachSide_ == EndstopWhich::MIN) ? "MIN" : "MAX",
                              encSecondRaw_, static_cast<long long>(contactSpan_));
                enterCenteringScan();
            }
            return;
        }
        case HomePhase::CENTERING: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { m.stop(); finishJoint(false); return; }
            if (m.isRunning()) {
                settleStartMs_ = now;
                return;
            }
            // Motor stopped — chuyển sang settle wait non-blocking trước khi VERIFY
            phase_ = HomePhase::VERIFY_SETTLE_WAIT;
            settleStartMs_ = now;
            // phaseStartMs_ giữ nguyên để timeout tổng thể không reset quá sớm; settleStartMs_ dùng cho 350ms
            return;
        }
        case HomePhase::VERIFY_SETTLE_WAIT: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { m.stop(); finishJoint(false); return; }
            if (now - settleStartMs_ < ENC_SETTLE_MS) return;
            enterVerify();
            return;
        }
        case HomePhase::VERIFY: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { m.stop(); finishJoint(false); return; }
            if (jm == nullptr || !jm->encOK(curAxis_)) {
                // Encoder mất giữa chừng: verify bất khả thi và drift-watchdog sẽ false-FAULT
                // với encoder chết → không chấp nhận home theo bước mù.
                Serial.printf("[HOME] J%u: VERIFY encoder mat ket noi — HUY KHOP\n", curAxis_ + 1);
                finishJoint(false);
                return;
            }
            const float raw = jm->rawEncoder(curAxis_);
            const float spanRaw = fabsf(encSecondRaw_ - encFirstRaw_);
            const float tol = HOMING_VERIFY_TOL_BASE_DEG +
                              HOMING_VERIFY_TOL_SPAN_PCT * 0.01f * (spanRaw / 2.0f);
            const float err = targetEncRaw_ - raw;
            if (m.isRunning()) {
                if (fabsf(err) <= tol) {
                    m.stop();          // đã vào biên -> dừng, chờ ổn định rồi chốt
                    settleStartMs_ = now;
                    return;
                }
                // Guard chống trim chạy loạn (encSign sai / encoder đóng băng từng đoạn):
                // err TĂNG vượt mốc đầu lần trim hoặc đi quá HOMING_TRIM_MAX_TRAVEL_DEG
                // mà chưa hội tụ → dừng, hủy khớp thay vì đâm endstop.
                if (fabsf(err) > trimStartErr_ + 2.0f ||
                    llabs(m.getAbsoluteSteps() - trimStartSteps_) >
                        JointModel::degreesToSteps(curAxis_, HOMING_TRIM_MAX_TRAVEL_DEG)) {
                    m.stop();
                    Serial.printf("[HOME] J%u: TRIM RUNAWAY (err=%.2f, start=%.2f) — HUY KHOP\n",
                                  curAxis_ + 1, err, trimStartErr_);
                    finishJoint(false);
                    return;
                }
                return;
            }
            if (now - settleStartMs_ < ENC_SETTLE_MS) return;
            if (fabsf(err) <= tol) {
                Serial.printf("[HOME] J%u: VERIFY OK (enc=%.1f, err=%.2f, tol=%.2f)\n",
                              curAxis_ + 1, raw, err, tol);
                finishJoint(true);
                return;
            }
            if (trimCount_ >= 2) {
                Serial.printf("[HOME] J%u: VERIFY FAIL (err=%.2f > tol=%.2f, %u lan trim)\n",
                              curAxis_ + 1, err, tol, trimCount_);
                finishJoint(false); // -> retry quét lại từ đầu
                return;
            }
            // Trim: chạy chậm về phía target theo encoder (khắc phục sai số steps/deg)
            ++trimCount_;
            const float encSign = jm->encSignOf(curAxis_);
            const bool cw = JointModel::cwForDelta(curAxis_, err / encSign);
            trimStartErr_ = fabsf(err);
            trimStartSteps_ = m.getAbsoluteSteps();
            m.setSpeed(HOMING_SLOW_SCAN_INTERVAL_US);
            m.runContinuous(cw);
            settleStartMs_ = now;
            Serial.printf("[HOME] J%u: TRIM #%u (err=%.2f, cw=%d)\n",
                          curAxis_ + 1, trimCount_, err, static_cast<int>(cw));
            return;
        }
        default:
            break;
    }
}

// ===========================================================================
// CHUNG
// ===========================================================================

void HomingController::tick() {
    if (!active_) return;
    Motor& m = *motors[curAxis_];
    const uint32_t now = millis();
    tickScan(now, m);
}

void HomingController::finishJoint(bool ok) {
    restoreDriverDefaults(curAxis_);
    if (ok && jm != nullptr) {
        jm->setHomeHere(curAxis_);
    } else if (jm != nullptr) {
        jm->resyncFromEncoder(curAxis_);
    }
    if (es != nullptr) {
        es->clearLatch(curAxis_, EndstopWhich::MIN);
        es->clearLatch(curAxis_, EndstopWhich::MAX);
    }
    Serial.printf("[HOME] J%u: %s\n", curAxis_ + 1, ok ? "SETREF OK" : "FAILED");

    if (!ok) {
        retryOrFail();
        return;
    }
    ++seqIdx_;
    attempt_ = 0;
    if (seqIdx_ < seqLen_) {
        beginScan();
    } else {
        active_ = false;
        if (safety_) safety_->assertHoming(false);
        lastOk_ = true;
        phase_ = HomePhase::IDLE;
        Serial.println("[HOME] Toan bo hoan tat");
    }
}

void HomingController::retryOrFail() {
    if (attempt_ + 1 < HOMING_MAX_ATTEMPTS) {
        ++attempt_;
        Serial.printf("[HOME] J%u: FAILED — thu lai lan %u/%u\n",
                      curAxis_ + 1, attempt_ + 1, HOMING_MAX_ATTEMPTS);
        beginScan(); // quét lại từ đầu (SAFE_MODE/WARMUP tự áp dụng lại)
        return;
    }
    active_ = false;
    if (safety_) safety_->assertHoming(false);
    lastOk_ = false;
    phase_ = HomePhase::IDLE;
    Serial.printf("[HOME] J%u: FAILED sau %u lan thu — huy chuoi homing\n",
                  curAxis_ + 1, HOMING_MAX_ATTEMPTS);
}

void HomingController::restoreDriverDefaults(uint8_t axis) {
    Motor& m = *motors[axis];
    if (m.isTmc()) {
        m.setCurrent(DEFAULT_NORMAL_CURRENT);
        m.setChopperMode(true);
    }
    m.setSpeed(DEFAULT_STEP_INTERVAL_US);
}

void HomingController::cancel() {
    if (!active_) return;
    motors[curAxis_]->stop();
    restoreDriverDefaults(curAxis_);
    if (jm != nullptr) jm->resyncFromEncoder(curAxis_);
    if (es != nullptr) {
        es->clearLatch(curAxis_, EndstopWhich::MIN);
        es->clearLatch(curAxis_, EndstopWhich::MAX);
    }
    active_ = false;
    if (safety_) safety_->assertHoming(false);
    lastOk_ = false;
    phase_ = HomePhase::IDLE;
    Serial.println("[HOME] Huy boi nguoi dung");
}

String HomingController::toJson() const {
    static const char* PHASE_NAMES[] = {
        "idle", "warmup", "warmup_settle_wait", "scan_min", "scan_backoff",
        "backoff_settle_wait", "scan_slow", "scan_max", "centering", "verify",
        "verify_settle_wait", "done"
    };
    static_assert(sizeof(PHASE_NAMES) / sizeof(PHASE_NAMES[0]) ==
                      static_cast<uint8_t>(HomePhase::DONE) + 1,
                  "PHASE_NAMES phai khop voi HomePhase");
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"active\":%s,\"lastOk\":%s,\"axis\":%u,\"phase\":\"%s\"}",
             active_ ? "true" : "false",
             lastOk_ ? "true" : "false",
             active_ ? curAxis_ + 1 : 0,
             PHASE_NAMES[static_cast<uint8_t>(phase_)]);
    return String(buf);
}
