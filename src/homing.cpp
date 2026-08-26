#include "homing.h"
#include "endstop.h"
#include "joint_model.h"
#include "motor.h"

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
    beginJoint(seq_[0]);
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
    beginJoint(axis);
    return true;
}

void HomingController::beginJoint(uint8_t axis) {
    curAxis_ = axis;
    stallCount_ = 0;
    phaseStartMs_ = millis();
    lastPollMs_ = 0;

    // Công tắc đã ở trạng thái nhấn trước khi bắt đầu => coi như đã chạm.
    if (es != nullptr && es->hasPin(axis, EndstopWhich::MIN) &&
        (es->isPressed(axis, EndstopWhich::MIN) || es->isLatched(axis, EndstopWhich::MIN))) {
        Serial.printf("[HOME] J%u: endstop da o trang thai nhat truoc khi bat dau\n", axis + 1);
        enterBackoff();
        return;
    }
    enterApproach();
}

void HomingController::enterApproach() {
    Motor& m = *motors[curAxis_];
    if (m.isTmc()) {
        const uint16_t hc = DEFAULT_AXIS_HOMING_CURRENTS[curAxis_];
        if (hc > 0) m.setCurrent(hc); // A4988: Vref cứng, bỏ qua
        m.setSGThreshold(DEFAULT_STALL_THRESHOLD > 0 ? DEFAULT_STALL_THRESHOLD : 2);
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
    Serial.printf("[HOME] J%u: CONTACT (steps=%lld)\n", curAxis_ + 1,
                  static_cast<long long>(m.getAbsoluteSteps()));
    enterBackoff();
}

void HomingController::enterBackoff() {
    Motor& m = *motors[curAxis_];
    const float backoffDeg = HOME_BACKOFF_DEG + 0.5f; // +margins để chắc chắn rời công tắc
    const int64_t steps = JointModel::degreesToSteps(curAxis_, backoffDeg);
    const bool cwAwayFromMin = JointModel::cwForDelta(curAxis_, +backoffDeg);
    m.run(cwAwayFromMin, static_cast<uint32_t>(steps));
    phase_ = HomePhase::BACKOFF;
    phaseStartMs_ = millis();
}

void HomingController::enterCentering() {
    Motor& m = *motors[curAxis_];
    // Sau backoff ta đang ở min + backoff. Trung tâm hành trình = min + stroke/2.
    const float stroke = DEFAULT_AXIS_CALIB_RANGE[curAxis_];
    const float remainDeg = stroke / 2.0f - (HOME_BACKOFF_DEG + 0.5f);
    const int64_t steps = JointModel::degreesToSteps(curAxis_, remainDeg);
    const bool cwPositive = JointModel::cwForDelta(curAxis_, +remainDeg);
    m.run(cwPositive, static_cast<uint32_t>(steps));
    phase_ = HomePhase::CENTERING;
    phaseStartMs_ = millis();
    Serial.printf("[HOME] J%u: CENTERING +%.1f deg (%lld steps)\n",
                  curAxis_ + 1, remainDeg, static_cast<long long>(steps));
}

void HomingController::finishJoint(bool ok) {
    restoreDriverDefaults(curAxis_);
    if (ok && jm != nullptr) jm->setHomeHere(curAxis_);
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
        beginJoint(seq_[seqIdx_]);
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
    active_ = false;
    lastOk_ = false;
    phase_ = HomePhase::IDLE;
    Serial.println("[HOME] Huy boi nguoi dung");
}

void HomingController::tick() {
    if (!active_) return;
    Motor& m = *motors[curAxis_];
    const uint32_t now = millis();

    switch (phase_) {
        case HomePhase::APPROACH: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) {
                m.stop();
                Serial.printf("[HOME] J%u: TIMEOUT o phan APPROACH\n", curAxis_ + 1);
                finishJoint(false);
                return;
            }
            // 1) Endstop vật lý (J1..J3)
            if (es != nullptr && es->hasPin(curAxis_, EndstopWhich::MIN) &&
                (es->isLatched(curAxis_, EndstopWhich::MIN) ||
                 es->isPressed(curAxis_, EndstopWhich::MIN))) {
                contactMade();
                return;
            }
            // 2) StallGuard (chỉ TMC), poll ~20ms
            if (m.isTmc() && now - lastPollMs_ >= 20) {
                lastPollMs_ = now;
                const uint16_t sg = m.getSGResult(); // 1023 nếu lỗi UART => coi như không stall
                if (sg < STALL_SG_LEVEL) {
                    if (++stallCount_ >= STALL_CONSECUTIVE_POLLS) {
                        Serial.printf("[HOME] J%u: STALL (sg=%u)\n", curAxis_ + 1, sg);
                        contactMade();
                        return;
                    }
                } else {
                    stallCount_ = 0;
                }
            }
            break;
        }
        case HomePhase::BACKOFF: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { finishJoint(false); return; }
            if (!m.isRunning()) {
                if (es != nullptr && es->hasPin(curAxis_, EndstopWhich::MIN) &&
                    es->isPressed(curAxis_, EndstopWhich::MIN)) {
                    // vẫn đè công tắc: lùi thêm một nấc nữa
                    enterBackoff();
                    return;
                }
                if (curAxis_ == 0 || curAxis_ == 1) {
                    enterCentering(); // J1/J2 về giữa hành trình
                } else {
                    finishJoint(true); // J3/J4: home tại chỗ
                }
            }
            break;
        }
        case HomePhase::CENTERING: {
            if (now - phaseStartMs_ > HOMING_JOINT_TIMEOUT_MS) { finishJoint(false); return; }
            if (!m.isRunning()) finishJoint(true);
            break;
        }
        default:
            break;
    }
}

String HomingController::toJson() const {
    static const char* PHASE_NAMES[] = {"idle", "approach", "backoff", "centering", "done"};
    char buf[128];
    snprintf(buf, sizeof(buf),
             "{\"active\":%s,\"lastOk\":%s,\"axis\":%u,\"phase\":\"%s\"}",
             active_ ? "true" : "false",
             lastOk_ ? "true" : "false",
             active_ ? curAxis_ + 1 : 0,
             PHASE_NAMES[static_cast<uint8_t>(phase_)]);
    return String(buf);
}
