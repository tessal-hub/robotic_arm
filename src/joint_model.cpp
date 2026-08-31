#include "joint_model.h"
#include "differential_wrist.h"
#include "motor.h"
#include "nvs_store.h"
#include "sensor.h"

float JointModel::s_encSign[NUM_MOTORS]{};
float JointModel::s_measuredSpd[NUM_MOTORS]{};
bool  JointModel::s_hasMeasured[NUM_MOTORS]{};

JointModel::JointModel() {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        encZeroRef[i] = 0.0f;
        homed[i] = false;
        restored[i] = false;
        driftFault[i] = false;
        lastRunningMs[i] = 0;
        driftFailCount[i] = 0;
        s_encSign[i] = AXIS_ENC_SIGN[i];
        s_measuredSpd[i] = stepsPerDegree(i);
        s_hasMeasured[i] = false;
    }
}

void JointModel::begin(Motor** motors_, Sensor* sensor_) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = motors_[i];
    sensor = sensor_;
}

void JointModel::attachNvs(NvsStore* nvs_) { nvs = nvs_; }

float JointModel::stepsPerDegree(uint8_t axis) {
    if (axis >= NUM_MOTORS) return 1.0f;
    const float stepsPerRev =
        static_cast<float>(DEFAULT_FULL_STEPS) * static_cast<float>(DEFAULT_MICROSTEPS) *
        DEFAULT_AXIS_GEAR_RATIOS[axis];
    return stepsPerRev / 360.0f;
}

int64_t JointModel::degreesToSteps(uint8_t axis, float deg) {
    return static_cast<int64_t>(lroundf(deg * stepsPerDegree(axis)));
}

float JointModel::stepsToDegrees(uint8_t axis, int64_t steps) {
    const float spd = stepsPerDegree(axis);
    return (spd > 0.0f) ? (static_cast<float>(steps) / spd) : 0.0f;
}

float JointModel::wrap180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

bool JointModel::cwForDelta(uint8_t axis, float deltaDeg) {
    // cw=true => absSteps tăng. Góc = SIGN * absSteps / spd.
    return (AXIS_STEP_SIGN[axis] > 0) ? (deltaDeg >= 0.0f) : (deltaDeg < 0.0f);
}

float JointModel::actuatorAngleFromSteps(uint8_t axis) const {
    if (axis >= NUM_MOTORS || motors[axis] == nullptr) return 0.0f;
    return AXIS_STEP_SIGN[axis] * stepsToDegrees(axis, motors[axis]->getAbsoluteSteps());
}

float JointModel::actuatorAngleFromEncoder(uint8_t axis) {
    if (axis >= NUM_MOTORS || sensor == nullptr || !homed[axis]) return 0.0f;
    const float rawDiff = sensor->getAccumulatedAngle(axis) - encZeroRef[axis];
    return s_encSign[axis] * rawDiff;
}

float JointModel::angleFromSteps(uint8_t axis) const {
    if (axis < 4) {
        return actuatorAngleFromSteps(axis);
    }
    // Khớp 5 & 6 qua cơ cấu Vi sai Bánh răng Côn (Bevel Gear Differential)
    const float th5 = actuatorAngleFromSteps(4);
    const float th6 = actuatorAngleFromSteps(5);
    const DifferentialWrist::JointState j = g_diffWrist.forward(th5, th6);
    return (axis == 4) ? j.tiltDeg : j.rollDeg;
}

float JointModel::angleFromEncoder(uint8_t axis) {
    if (axis < 4) {
        return actuatorAngleFromEncoder(axis);
    }
    // Đọc góc vi sai từ 2 encoder E_L (axis 4) và E_R (axis 5)
    const float e5 = actuatorAngleFromEncoder(4);
    const float e6 = actuatorAngleFromEncoder(5);
    const DifferentialWrist::JointState j = g_diffWrist.forward(e5, e6);
    return (axis == 4) ? j.tiltDeg : j.rollDeg;
}

float JointModel::rawEncoder(uint8_t axis) {
    if (axis >= NUM_MOTORS || sensor == nullptr) return 0.0f;
    return sensor->getAccumulatedAngle(axis);
}

void JointModel::setHomeHere(uint8_t axis) {
    if (axis >= NUM_MOTORS) return;
    if (motors[axis] == nullptr) return;

    motors[axis]->setAbsoluteSteps(0);
    if (sensor != nullptr && sensor->isSensorOK(axis)) {
        const float encAngle = sensor->getAccumulatedAngle(axis);
        encZeroRef[axis] = encAngle;
        if (nvs != nullptr) nvs->saveJointHome(axis, sensor->getAngle(axis));
        Serial.printf("[JM] SetHome J%u (enc=ok, zeroRef=%.1f deg)\n",
                      axis + 1, encAngle);
    } else {
        Serial.printf("[JM] SetHome J%u (enc=DEAD, step-only)\n", axis + 1);
    }
    driftFault[axis] = false;
    driftFailCount[axis] = 0;
    lastRunningMs[axis] = millis();
    restored[axis] = false;
    homed[axis] = true;
}

void JointModel::clearHome(uint8_t axis) {
    if (axis >= NUM_MOTORS) return;
    homed[axis] = false;
    restored[axis] = false;
    driftFault[axis] = false;
    driftFailCount[axis] = 0;
    lastRunningMs[axis] = millis();
}

void JointModel::resyncFromEncoder(uint8_t axis) {
    if (axis >= NUM_MOTORS) return;
    if (motors[axis] == nullptr || sensor == nullptr || !sensor->isSensorOK(axis) || !homed[axis]) return;
    const float relEncDeg = (sensor->getAccumulatedAngle(axis) - encZeroRef[axis]) * s_encSign[axis];
    motors[axis]->setAbsoluteSteps(AXIS_STEP_SIGN[axis] * degreesToSteps(axis, relEncDeg));
}

void JointModel::forgetHome(uint8_t axis) {
    clearHome(axis);
    if (axis < NUM_MOTORS) {
        s_hasMeasured[axis] = false;
        s_encSign[axis] = AXIS_ENC_SIGN[axis];
    }
    if (nvs != nullptr) {
        nvs->clearJointHome(axis);
        nvs->clearCalib(axis);
    }
}

void JointModel::applyHomingCalibration(uint8_t axis, float encSign, float stepsPerDeg) {
    if (axis >= NUM_MOTORS) return;
    s_encSign[axis] = AXIS_ENC_SIGN[axis];
    s_measuredSpd[axis] = stepsPerDegree(axis);
    s_hasMeasured[axis] = true;
    if (nvs != nullptr) nvs->saveCalib(axis, s_encSign[axis], s_measuredSpd[axis]);
    Serial.printf("[JM] Calib J%u: encSign=%+.0f, steps/deg=%.2f\n",
                  axis + 1, s_encSign[axis], s_measuredSpd[axis]);
}

void JointModel::resetHomingCalibration(uint8_t axis) {
    if (axis >= NUM_MOTORS) return;
    s_encSign[axis] = AXIS_ENC_SIGN[axis];
    s_measuredSpd[axis] = stepsPerDegree(axis);
    s_hasMeasured[axis] = false;
}

uint8_t JointModel::restoreFromNVS() {
    if (nvs == nullptr || sensor == nullptr) return 0;
    uint8_t okCount = 0;
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        s_encSign[a] = AXIS_ENC_SIGN[a];
        s_measuredSpd[a] = stepsPerDegree(a);
        s_hasMeasured[a] = true;

        if (!sensor->isSensorOK(a) || motors[a] == nullptr) continue;
        const NvsStore::JointHome h = nvs->loadJointHome(a);
        if (!h.valid) continue;

        const float now = sensor->getAngle(a);
        const float delta = s_encSign[a] * wrap180(now - h.rawDeg);

        // Giới hạn an toàn: chỉ khôi phục khi độ lệch <= 30 độ quanh mốc home đã lưu
        const float maxSpan = 30.0f;
        if (fabsf(delta) > maxSpan) {
            Serial.printf("[JM] J%u: restore lech %.1f deg > span %.1f => bo qua (can home lai)\n",
                          a + 1, delta, maxSpan);
            continue;
        }

        motors[a]->setAbsoluteSteps(AXIS_STEP_SIGN[a] * degreesToSteps(a, delta));
        encZeroRef[a] = sensor->getAccumulatedAngle(a) - delta / s_encSign[a];
        driftFault[a] = false;
        driftFailCount[a] = 0;
        lastRunningMs[a] = millis();
        restored[a] = true;
        homed[a] = true;
        ++okCount;
        Serial.printf("[JM] J%u: restore tu NVS %+.2f deg (encZeroRef=%.1f)\n", a + 1, delta, encZeroRef[a]);
    }
    Serial.printf("[JM] Restore xong: %u/%u khop\n", okCount, NUM_MOTORS);
    return okCount;
}

uint8_t JointModel::homedCount() const noexcept {
    uint8_t n = 0;
    for (const bool h : homed) n += h ? 1 : 0;
    return n;
}

bool JointModel::allPositioningHomed() const noexcept {
    for (uint8_t i = 0; i < 4; ++i) { // J1..J4
        if (!homed[i]) return false;
    }
    return true;
}

bool JointModel::updateDriftCheck(uint8_t axis) {
    if (axis >= NUM_MOTORS) return false;
    if (!homed[axis] || sensor == nullptr ||
        motors[axis] == nullptr || !sensor->isSensorOK(axis)) {
        driftFailCount[axis] = 0;
        return false;
    }
    const uint32_t now = millis();
    if (motors[axis]->isRunning()) {
        lastRunningMs[axis] = now;
        driftFailCount[axis] = 0;
        return false;
    }
    // Cho phép thời gian trễ 300ms sau khi motor dừng để cảm biến & bộ lọc EMA ổn định hoàn toàn
    if (now - lastRunningMs[axis] < 300) {
        return false;
    }

    const float diff = angleFromSteps(axis) - angleFromEncoder(axis);
    if (fabsf(diff) > RUNAWAY_ERROR_THRESHOLD) {
        driftFailCount[axis]++;
        if (driftFailCount[axis] >= 3) {
            driftFault[axis] = true;
            Serial.printf("[DRIFT] J%u lech %.2f deg (step=%.2f enc=%.2f) -> FAULT\n",
                          axis + 1, diff, angleFromSteps(axis), angleFromEncoder(axis));
        } else {
            Serial.printf("[DRIFT] J%u nghi lech %.2f deg (step=%.2f enc=%.2f) [%u/3]\n",
                          axis + 1, diff, angleFromSteps(axis), angleFromEncoder(axis),
                          driftFailCount[axis]);
        }
    } else {
        driftFailCount[axis] = 0;
        // Tự động đồng bộ vị trí bước theo encoder sau khi dừng chuyển động
        resyncFromEncoder(axis);
    }
    return driftFault[axis];
}

bool JointModel::hasAnyDriftFault() const noexcept {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (driftFault[i]) return true;
    }
    return false;
}

void JointModel::clearAllDriftFaults() noexcept {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        driftFault[i] = false;
        driftFailCount[i] = 0;
        lastRunningMs[i] = millis();
        if (homed[i] && encOK(i)) {
            resyncFromEncoder(i);
        }
    }
}

bool JointModel::encOK(uint8_t axis) const {
    return (sensor != nullptr) && sensor->isSensorOK(axis);
}

float JointModel::encSignOf(uint8_t axis) const {
    return (axis < NUM_MOTORS) ? s_encSign[axis] : 1.0f;
}

String JointModel::toJson() {
    String j = "[";
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        if (a > 0) j += ",";
        const float actDeg = (homed[a] && encOK(a)) ? angleFromEncoder(a) : angleFromSteps(a);
        char buf[224];
        snprintf(buf, sizeof(buf),
                 "{\"deg\":%.2f,\"encDeg\":%.2f,\"homed\":%s,\"restored\":%s,"
                 "\"encOK\":%s,\"drift\":%s,\"limitMin\":%.1f,\"limitMax\":%.1f}",
                 actDeg, angleFromEncoder(a),
                 homed[a] ? "true" : "false",
                 restored[a] ? "true" : "false",
                 encOK(a) ? "true" : "false",
                 driftFault[a] ? "true" : "false",
                 DEFAULT_AXIS_LIMIT_MIN[a], DEFAULT_AXIS_LIMIT_MAX[a]);
        j += buf;
    }
    j += "]";
    return j;
}
