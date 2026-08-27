#include "homing.h"
#include "endstop.h"
#include "joint_model.h"
#include "motor.h"

namespace {
// Tốc độ dò lại chậm để định tâm tinh (µs/step)
constexpr uint32_t REAPPROACH_STEP_INTERVAL_US = 3000;
// Sai số encoder khi closed-loop tới tâm (độ)
constexpr float ENCODER_HOME_TOLERANCE_DEG = 0.5f;
// Deadzone encoder không dịch chuyển đáng kể -> giữ dấu mặc định
constexpr float ENC_DIR_DEADZONE_DEG = 0.5f;
// Tỷ số measuredSpd/config chấp nhận được (ngoài khoảng -> giữ config)
constexpr float MEASURED_RATIO_MIN = 0.5f;
constexpr float MEASURED_RATIO_MAX = 2.0f;
// Timeout cho scan phases (quét toàn hành trình, cần dài hơn homing timeout thường)
constexpr uint32_t SCAN_TIMEOUT_MS = 60000;
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
    active_ = true;
    lastOk_ = true;
    if (isScanAxis(seq_[0])) beginScan(); else beginJoint(seq_[0]);
    return true;
}

bool HomingController::startAxis(uint8_t axis) {
    if (active_) return false;
    if (axis >= 4 || motors[axis] == nullptr) return false; // J5/J6: manual only
    seqLen_ = 1;
    seq_[0] = axis;
    seqIdx_ = 0;
    active_ = true;
    lastOk_ = true;
    if (isScanAxis(axis)) beginScan(); else beginJoint(axis);
    return true;
}

bool HomingController::isScanAxis(uint8_t axis) const noexcept {
    return es != nullptr && es->hasPin(axis, EndstopWhich::MIN) &&
           es->hasPin(axis, EndstopWhich::MAX);
}

bool HomingController::homeAtMinOffset(uint8_t axis) const noexcept {
    // J1/J2 (0,1): home = TÂM cơ khí giữa 2 endstop. J3 (2): home = điểm riêng (Min + offset).
    return axis == 2; // J3
}

// ===========================================================================
// SCAN PATH (J1, J2) — kiến trúc 7 giai đoạn
// ===========================================================================

void HomingController::beginScan() {
    curAxis_ = seq_[seqIdx_];
    stallCount_ = 0;
    centeringCoarse_ = false;
    minSide_ = EndstopWhich::MIN;
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
    // Stage 1: Safe Mode — hạ dòng TMC2209 (A4988: Vref cứng, bỏ qua)
    if (m.isTmc()) {
        const uint16_t hc = DEFAULT_AXIS_HOMING_CURRENTS[curAxis_];
        if (hc > 0) m.setCurrent(hc);
    }
    m.setSpeed(DEFAULT_AXIS_HOMING_SPEEDS[curAxis_]);
    // Stage 2: chọn hướng an toàn — tránh BẤT KỲ endstop đang nhấn
    const bool minP = es->hasPin(curAxis_, EndstopWhich::MIN) &&
                      (es->isPressed(curAxis_, EndstopWhich::MIN) ||
                       es->isLatched(curAxis_, EndstopWhich::MIN));
    const bool maxP = es->hasPin(curAxis_, EndstopWhich::MAX) &&
                      (es->isPressed(curAxis_, EndstopWhich::MAX) ||
                       es->isLatched(curAxis_, EndstopWhich::MAX));
    if (minP && !maxP)      warmupCW_ = true;   // lùi khỏi MIN -> chạy dương
    else if (maxP && !minP) warmupCW_ = false;  // lùi khỏi MAX -> chạy âm
    else                    warmupCW_ = false;   // mặc định / cả hai nhấn
    // Số bước đủ lớn để encoder đo được (>2–3° góc khớp) — phụ thuộc gear ratio
    const float targetJointDeg = 3.0f;
    const float spd = JointModel::stepsPerDegree(curAxis_);
    const uint32_t warmupSteps = static_cast<uint32_t>(targetJointDeg * spd) + 1;
    encBefore_ = jm->rawEncoder(curAxis_);
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
    const bool cwToMin = JointModel::cwForDelta(curAxis_, -360.0f);
    m.runContinuous(cwToMin);
    phase_ = HomePhase::SCAN_MIN;
    phaseStartMs_ = millis();
    Serial.printf("[HOME] J%u: SCAN_MIN (cw=%d)\n", curAxis_ + 1, static_cast<int>(cwToMin));
}

void HomingController::enterScanMax() {
    Motor& m = *motors[curAxis_];
    m.setSpeed(DEFAULT_AXIS_HOMING_SPEEDS[curAxis_]);
    const bool cwToMax = JointModel::cwForDelta(curAxis_, +360.0f);
    m.runContinuous(cwToMax);
    phase_ = HomePhase::SCAN_MAX;
    phaseStartMs_ = millis();
    Serial.printf("[HOME] J%u: SCAN_MAX (cw=%d)\n", curAxis_ + 1, static_cast<int>(cwToMax));
}

void HomingController::enterCenteringScan() {
    Motor& m = *motors[curAxis_];
    // J1/J2: home = TÂM cơ khí = step_max/2 bước từ cực đầu (đúng bất kể gear ratio).
    // J3: home = điểm riêng gần MIN = lùi một offset nhỏ khỏi MIN endstop, không phải tâm.
    int64_t stepCenter;
    if (homeAtMinOffset(curAxis_)) {
        stepCenter = JointModel::degreesToSteps(curAxis_, HOME_BACKOFF_DEG + 0.5f);
    } else {
        stepCenter = stepMax_ / 2;
    }
    const float centerDeg = JointModel::stepsToDegrees(curAxis_, stepCenter);
    const bool cwToCenter = JointModel::cwForDelta(curAxis_, -centerDeg); // từ MIN về phía cực đầu
    m.setSpeed(DEFAULT_AXIS_HOMING_SPEEDS[curAxis_]);
    m.run(cwToCenter, static_cast<uint32_t>(stepCenter));
    centeringCoarse_ = false;
    phase_ = HomePhase::CENTERING;
    phaseStartMs_ = millis();
    Serial.printf("[HOME] J%u: CENTERING coarse %lld steps (cw=%d, enc_center=%.1f, min%soffset=%d)\n",
                  curAxis_ + 1, static_cast<long long>(stepCenter),
                  static_cast<int>(cwToCenter), encCenterRaw_,
                  homeAtMinOffset(curAxis_) ? "+" : "/",
                  static_cast<int>(homeAtMinOffset(curAxis_) ? (HOME_BACKOFF_DEG + 0.5f) : 0));
}

void HomingController::tickScan(uint32_t now, Motor& m) {
    switch (phase_) {
        case HomePhase::WARMUP: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { m.stop(); finishJoint(false); return; }
            if (m.isRunning()) return;
            // Đo enc_dir_mult: dấu dịch chuyển encoder so với hướng bước dự định
            const float encAfter = jm->rawEncoder(curAxis_);
            const float delta = encAfter - encBefore_;
            const float intendedJointSign = static_cast<float>(AXIS_STEP_SIGN[curAxis_]) *
                                            (warmupCW_ ? 1.0f : -1.0f);
            if (fabsf(delta) < ENC_DIR_DEADZONE_DEG) {
                encDirMult_ = AXIS_ENC_SIGN[curAxis_]; // encoder không dịch -> giữ mặc định
                Serial.printf("[HOME] J%u: WARMUP enc dead (delta=%.2f) -> giu encSign mac dinh\n",
                              curAxis_ + 1, delta);
            } else {
                encDirMult_ = (delta * intendedJointSign >= 0.0f) ? 1.0f : -1.0f;
                Serial.printf("[HOME] J%u: WARMUP encDirMult=%+.0f (delta=%.2f, intent=%.0f)\n",
                              curAxis_ + 1, encDirMult_, delta, intendedJointSign);
            }
            enterScanMin();
            return;
        }
        case HomePhase::SCAN_MIN: {
            if (now - phaseStartMs_ > SCAN_TIMEOUT_MS) { m.stop();
                Serial.printf("[HOME] J%u: SCAN_MIN TIMEOUT — no endstop reached in %us\n",
                              curAxis_ + 1, SCAN_TIMEOUT_MS / 1000);
                finishJoint(false); return; }
            // Log tiến trình mỗi 2s để thấy motor đang chạy
            if (now - lastPollMs_ > 2000) {
                lastPollMs_ = now;
                Serial.printf("[HOME] J%u: SCAN_MIN progress step=%.1f enc=%.1f\n",
                              curAxis_ + 1, jm->angleFromSteps(curAxis_),
                              jm->rawEncoder(curAxis_));
            }
            // Phát hiện BẤT KỲ endstop nào chạm trước (hướng có thể ngược giả định) -> cực Min.
            EndstopWhich hit = EndstopWhich::MIN;
            bool hitAny = false;
            for (const auto w : {EndstopWhich::MIN, EndstopWhich::MAX}) {
                if (es->hasPin(curAxis_, w) &&
                    (es->isLatched(curAxis_, w) || es->isPressed(curAxis_, w))) {
                    hit = w; hitAny = true; break;
                }
            }
            if (hitAny) {
                m.stop();
                es->consumeLatch(curAxis_, hit);
                minSide_ = hit;
                encMinRaw_ = jm->rawEncoder(curAxis_);
                motors[curAxis_]->setAbsoluteSteps(0); // gốc bước tại cực đầu tiên
                Serial.printf("[HOME] J%u: SCAN_MIN CONTACT (%s, enc_min=%.1f, steps=0)\n",
                              curAxis_ + 1, (hit == EndstopWhich::MIN ? "MIN" : "MAX"), encMinRaw_);
                lastPollMs_ = millis();
                enterScanMax();
            }
            return;
        }
        case HomePhase::SCAN_MAX: {
            // Cực còn lại (khác với cực chạm ở SCAN_MIN)
            const EndstopWhich maxSide = (minSide_ == EndstopWhich::MIN)
                ? EndstopWhich::MAX : EndstopWhich::MIN;
            if (now - phaseStartMs_ > SCAN_TIMEOUT_MS) { m.stop();
                Serial.printf("[HOME] J%u: SCAN_MAX TIMEOUT — %s (pin %d) not reached in %us\n",
                              curAxis_ + 1,
                              (maxSide == EndstopWhich::MIN) ? "MIN" : "MAX",
                              (maxSide == EndstopWhich::MIN) ? AXIS_MIN_PINS[curAxis_] : AXIS_MAX_PINS[curAxis_],
                              SCAN_TIMEOUT_MS / 1000);
                finishJoint(false); return; }
            // Log tiến trình mỗi 2s
            if (now - lastPollMs_ > 2000) {
                lastPollMs_ = now;
                Serial.printf("[HOME] J%u: SCAN_MAX progress step=%.1f enc=%.1f\n",
                              curAxis_ + 1, jm->angleFromSteps(curAxis_),
                              jm->rawEncoder(curAxis_));
            }
            if (es->hasPin(curAxis_, maxSide) &&
                (es->isLatched(curAxis_, maxSide) || es->isPressed(curAxis_, maxSide))) {
                m.stop();
                es->consumeLatch(curAxis_, maxSide);
                encMaxRaw_ = jm->rawEncoder(curAxis_);
                const int64_t absSteps = motors[curAxis_]->getAbsoluteSteps();
                stepMax_ = (absSteps < 0) ? -absSteps : absSteps;
                // Stage 5: cross-check — tính tâm + tỷ số thực tế
                encCenterRaw_ = (encMinRaw_ + encMaxRaw_) / 2.0f;
                const float span = encMaxRaw_ - encMinRaw_;
                float measuredSpd = (span != 0.0f)
                    ? static_cast<float>(stepMax_) / fabsf(span)
                    : JointModel::stepsPerDegree(curAxis_);
                const float cfgSpd = (static_cast<float>(DEFAULT_FULL_STEPS) *
                                      static_cast<float>(DEFAULT_MICROSTEPS) *
                                      DEFAULT_AXIS_GEAR_RATIOS[curAxis_]) / 360.0f;
                const float ratio = (cfgSpd > 0.0f) ? (measuredSpd / cfgSpd) : 1.0f;
                if (ratio < MEASURED_RATIO_MIN || ratio > MEASURED_RATIO_MAX) {
                    measuredSpd = cfgSpd;
                    Serial.printf("[HOME] J%u: measured ratio %.2f ngoai [%g,%g] -> giu config\n",
                                  curAxis_ + 1, ratio, MEASURED_RATIO_MIN, MEASURED_RATIO_MAX);
                }
                jm->applyHomingCalibration(curAxis_, encDirMult_, measuredSpd);
                Serial.printf("[HOME] J%u: CROSSCHECK enc_c=%.1f step_max=%lld spd=%.2f (x%.2f)\n",
                              curAxis_ + 1, encCenterRaw_, static_cast<long long>(stepMax_),
                              measuredSpd, ratio);
                enterCenteringScan();
            }
            return;
        }
        case HomePhase::CENTERING: {
            // Căn tâm theo BƯỚC, chờ coarse xong -> DONE. Không dùng encoder làm mục tiêu tuyệt đối:
            // AS5600 accumulated trên HW này không ổn định (drift/jump giữa phase, log thực tế
            // idle=~39 rồi homing=240..430) khiến vòng encoder phản hồi chạy ngược hướng đâm endstop.
            // - J1/J2: tâm cơ khí = step_max/2 từ cực đầu (bước tỉ lệ trực tiếp với góc, bất kể gear).
            // - J3: home tại điểm riêng gần MIN (Min + offset).
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { m.stop(); finishJoint(false); return; }
            if (m.isRunning()) return;
            Serial.printf("[HOME] J%u: CENTERING DONE (step-center; enc=%.1f)\n",
                          curAxis_ + 1, jm->rawEncoder(curAxis_));
            finishJoint(true);
            return;
        }
        default:
            break;
    }
}

// ===========================================================================
// LEGACY PATH (J3, J4) — giữ nguyên
// ===========================================================================

void HomingController::beginJoint(uint8_t axis) {
    curAxis_ = axis;
    stallCount_ = 0;
    angleEncAtContact_ = 0.0f;
    phaseStartMs_ = millis();
    lastPollMs_ = 0;

    // Xoá latch cũ không phải press thật
    if (es->hasPin(axis, EndstopWhich::MIN)) {
        if (es->isLatched(axis, EndstopWhich::MIN) && !es->isPressed(axis, EndstopWhich::MIN)) {
            es->clearLatch(axis, EndstopWhich::MIN);
        }
    }
    if (es->hasPin(axis, EndstopWhich::MIN) &&
        (es->isPressed(axis, EndstopWhich::MIN) || es->isLatched(axis, EndstopWhich::MIN))) {
        Serial.printf("[HOME] J%u: endstop pressed at start -> BACKOFF\n", axis + 1);
        enterBackoff();
        return;
    }
    enterApproach();
}

void HomingController::enterApproach() {
    Motor& m = *motors[curAxis_];
    if (m.isTmc()) {
        const uint16_t hc = DEFAULT_AXIS_HOMING_CURRENTS[curAxis_];
        if (hc > 0) m.setCurrent(hc);
    }
    m.setSpeed(DEFAULT_AXIS_HOMING_SPEEDS[curAxis_]);
    const bool cwTowardMin = JointModel::cwForDelta(curAxis_, -360.0f);
    m.runContinuous(cwTowardMin);
    phase_ = HomePhase::APPROACH;
    phaseStartMs_ = millis();
    Serial.printf("[HOME] J%u: APPROACH (cw=%d, interval=%uus)\n",
                  curAxis_ + 1, static_cast<int>(cwTowardMin),
                  DEFAULT_AXIS_HOMING_SPEEDS[curAxis_]);
}

void HomingController::contactMade() {
    Motor& m = *motors[curAxis_];
    m.stop();
    if (es != nullptr) {
        es->consumeLatch(curAxis_, EndstopWhich::MIN);
        es->clearLatch(curAxis_, EndstopWhich::MAX);
    }
    if (jm != nullptr && jm->encOK(curAxis_)) {
        angleEncAtContact_ = jm->angleFromEncoder(curAxis_);
    }
    Serial.printf("[HOME] J%u: CONTACT (steps=%lld, enc=%.1f deg)\n", curAxis_ + 1,
                  static_cast<long long>(m.getAbsoluteSteps()), angleEncAtContact_);
    enterBackoff();
}

void HomingController::enterBackoff() {
    Motor& m = *motors[curAxis_];
    const float backoffDeg = HOME_BACKOFF_DEG + 0.5f;
    const int64_t steps = JointModel::degreesToSteps(curAxis_, backoffDeg);
    const bool cwAwayFromMin = JointModel::cwForDelta(curAxis_, +backoffDeg);
    m.run(cwAwayFromMin, static_cast<uint32_t>(steps));
    phase_ = HomePhase::BACKOFF;
    phaseStartMs_ = millis();
}

void HomingController::enterReapproach() {
    Motor& m = *motors[curAxis_];
    m.setSpeed(REAPPROACH_STEP_INTERVAL_US);
    const bool cwTowardMin = JointModel::cwForDelta(curAxis_, -360.0f);
    m.runContinuous(cwTowardMin);
    phase_ = HomePhase::REAPPROACH;
    phaseStartMs_ = millis();
    Serial.printf("[HOME] J%u: REAPPROACH (slow, interval=%uus)\n",
                  curAxis_ + 1, REAPPROACH_STEP_INTERVAL_US);
}

bool HomingController::gotoNearHome() {
    if (jm == nullptr || !jm->encOK(curAxis_)) return false;
    Motor& m = *motors[curAxis_];
    const float encAngle = jm->angleFromEncoder(curAxis_);
    const float stroke = DEFAULT_AXIS_CALIB_RANGE[curAxis_];
    const float targetAngle = angleEncAtContact_ + stroke / 2.0f;
    const float err = targetAngle - encAngle;
    if (fabsf(err) < ENCODER_HOME_TOLERANCE_DEG) {
        Serial.printf("[HOME] J%u: already near home (err=%.2f)\n", curAxis_ + 1, err);
        return true;
    }
    const bool cwPositive = JointModel::cwForDelta(curAxis_, -err);
    m.setSpeed(DEFAULT_AXIS_HOMING_SPEEDS[curAxis_]);
    m.runContinuous(cwPositive);
    phase_ = HomePhase::CENTERING;
    phaseStartMs_ = millis();
    Serial.printf("[HOME] J%u: ENC-CENTERING -> %.1f deg (enc=%.1f, err=%.1f)\n",
                  curAxis_ + 1, targetAngle, encAngle, err);
    return true;
}

void HomingController::enterCentering() {
    Motor& m = *motors[curAxis_];
    const float stroke = DEFAULT_AXIS_CALIB_RANGE[curAxis_];
    const float remainDeg = stroke / 2.0f - (HOME_BACKOFF_DEG + 0.5f);
    const int64_t steps = JointModel::degreesToSteps(curAxis_, remainDeg);
    const bool cwPositive = JointModel::cwForDelta(curAxis_, +remainDeg);
    m.run(cwPositive, static_cast<uint32_t>(steps));
    phase_ = HomePhase::CENTERING;
    phaseStartMs_ = millis();
    Serial.printf("[HOME] J%u: STEP-CENTERING +%.1f deg (%lld steps)\n",
                  curAxis_ + 1, remainDeg, static_cast<long long>(steps));
}

void HomingController::tickLegacy(uint32_t now, Motor& m) {
    switch (phase_) {
        case HomePhase::APPROACH: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) {
                m.stop();
                Serial.printf("[HOME] J%u: TIMEOUT o phan APPROACH\n", curAxis_ + 1);
                finishJoint(false);
                return;
            }
            if (es->hasPin(curAxis_, EndstopWhich::MIN) &&
                (es->isLatched(curAxis_, EndstopWhich::MIN) ||
                 es->isPressed(curAxis_, EndstopWhich::MIN))) {
                contactMade();
            }
            break;
        }
        case HomePhase::BACKOFF: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) {
                if (jm != nullptr) jm->resyncFromEncoder(curAxis_);
                finishJoint(false);
                return;
            }
            if (!m.isRunning()) {
                if (es->hasPin(curAxis_, EndstopWhich::MIN) &&
                    es->isPressed(curAxis_, EndstopWhich::MIN)) {
                    Serial.printf("[HOME] J%u: BACKOFF jammed — endstop still pressed\n", curAxis_ + 1);
                    if (jm != nullptr) jm->resyncFromEncoder(curAxis_);
                    finishJoint(false);
                    return;
                }
                enterReapproach();
            }
            break;
        }
        case HomePhase::REAPPROACH: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { finishJoint(false); return; }
            if (es->hasPin(curAxis_, EndstopWhich::MIN) &&
                (es->isLatched(curAxis_, EndstopWhich::MIN) ||
                 es->isPressed(curAxis_, EndstopWhich::MIN))) {
                m.stop();
                es->consumeLatch(curAxis_, EndstopWhich::MIN);
                if (jm != nullptr && jm->encOK(curAxis_)) {
                    angleEncAtContact_ = jm->angleFromEncoder(curAxis_);
                }
                Serial.printf("[HOME] J%u: REAPPROACH CONTACT (enc=%.1f deg)\n",
                              curAxis_ + 1, angleEncAtContact_);
                if (curAxis_ <= 2) {
                    if (!gotoNearHome()) enterCentering();
                } else {
                    finishJoint(true); // J4: home tại chỗ
                }
            }
            break;
        }
        case HomePhase::CENTERING: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { finishJoint(false); return; }
            if (jm != nullptr && jm->encOK(curAxis_)) {
                const float encAngle = jm->angleFromEncoder(curAxis_);
                const float stroke = DEFAULT_AXIS_CALIB_RANGE[curAxis_];
                const float targetAngle = angleEncAtContact_ + stroke / 2.0f;
                const float err = targetAngle - encAngle;
                if (fabsf(err) < ENCODER_HOME_TOLERANCE_DEG) {
                    m.stop();
                    Serial.printf("[HOME] J%u: ENC-CENTERING DONE (enc=%.1f, target=%.1f)\n",
                                  curAxis_ + 1, encAngle, targetAngle);
                    finishJoint(true);
                    return;
                }
                const bool cwNeeded = JointModel::cwForDelta(curAxis_, -err);
                if (m.isRunning() && cwNeeded != m.getDirCW()) {
                    m.stop();
                    m.runContinuous(cwNeeded);
                }
                if (!m.isRunning()) {
                    if (stallStartMs_ == 0) stallStartMs_ = now;
                    else if (now - stallStartMs_ > 500) {
                        Serial.printf("[HOME] J%u: CENTERING STALL (enc=%.1f, target=%.1f)\n",
                                      curAxis_ + 1, encAngle, targetAngle);
                        stallStartMs_ = 0;
                        finishJoint(false);
                        return;
                    }
                } else {
                    stallStartMs_ = 0;
                }
            }
            break;
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
    if (isScanAxis(curAxis_)) tickScan(now, m);
    else tickLegacy(now, m);
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
    phase_ = HomePhase::DONE;

    if (!ok) {
        active_ = false;
        lastOk_ = false;
        phase_ = HomePhase::IDLE;
        return;
    }
    ++seqIdx_;
    if (seqIdx_ < seqLen_) {
        if (isScanAxis(seq_[seqIdx_])) beginScan(); else beginJoint(seq_[seqIdx_]);
    } else {
        active_ = false;
        lastOk_ = true;
        phase_ = HomePhase::IDLE;
        Serial.println("[HOME] Toan bo hoan tat");
    }
}

void HomingController::restoreDriverDefaults(uint8_t axis) {
    Motor& m = *motors[axis];
    if (m.isTmc()) m.setCurrent(DEFAULT_NORMAL_CURRENT);
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
    lastOk_ = false;
    phase_ = HomePhase::IDLE;
    Serial.println("[HOME] Huy boi nguoi dung");
}

String HomingController::toJson() const {
    static const char* PHASE_NAMES[] = {
        "idle", "safe_mode", "warmup", "scan_min", "scan_max",
        "approach", "backoff", "reapproach", "centering", "done"
    };
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"active\":%s,\"lastOk\":%s,\"axis\":%u,\"phase\":\"%s\"}",
             active_ ? "true" : "false",
             lastOk_ ? "true" : "false",
             active_ ? curAxis_ + 1 : 0,
             PHASE_NAMES[static_cast<uint8_t>(phase_)]);
    return String(buf);
}
