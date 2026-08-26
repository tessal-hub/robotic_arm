#include "joint_model.h"
#include "motor.h"
#include "nvs_store.h"
#include "sensor.h"

JointModel::JointModel() {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        encZeroRef[i] = 0.0f;
        homed[i] = false;
        restored[i] = false;
        driftFault[i] = false;
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
    // Muốn delta góc dương: nếu SIGN dương cần absSteps tăng (cw), nếu âm thì ngược lại.
    return (AXIS_STEP_SIGN[axis] > 0) ? (deltaDeg >= 0.0f) : (deltaDeg < 0.0f);
}

float JointModel::angleFromSteps(uint8_t axis) const {
    if (axis >= NUM_MOTORS || motors[axis] == nullptr) return 0.0f;
    return AXIS_STEP_SIGN[axis] * stepsToDegrees(axis, motors[axis]->getAbsoluteSteps());
}

float JointModel::angleFromEncoder(uint8_t axis) {
    if (axis >= NUM_MOTORS || sensor == nullptr || !homed[axis]) return 0.0f;
    return AXIS_ENC_SIGN[axis] * (sensor->getAccumulatedAngle(axis) - encZeroRef[axis]);
}

void JointModel::setHomeHere(uint8_t axis) {
    if (axis >= NUM_MOTORS) return;
    if (motors[axis] != nullptr) motors[axis]->setAbsoluteSteps(0);
    bool encHealthy = false;
    if (sensor != nullptr && sensor->isSensorOK(axis)) {
        encZeroRef[axis] = sensor->getAccumulatedAngle(axis);
        encHealthy = true;
        // Lưu raw single-turn để khôi phục vị trí qua nguồn (chỉ tin khi encoder khoẻ)
        if (nvs != nullptr) nvs->saveJointHome(axis, sensor->getAngle(axis));
    }
    driftFault[axis] = false;
    restored[axis] = false;
    homed[axis] = true;
    Serial.printf("[JM] SetHome J%u (enc=%s)\n", axis + 1, encHealthy ? "ok" : "DEAD");
}

void JointModel::clearHome(uint8_t axis) {
    if (axis >= NUM_MOTORS) return;
    homed[axis] = false;
    restored[axis] = false;
    driftFault[axis] = false;
}

void JointModel::forgetHome(uint8_t axis) {
    clearHome(axis);
    if (nvs != nullptr) nvs->clearJointHome(axis);
}

uint8_t JointModel::restoreFromNVS() {
    if (nvs == nullptr || sensor == nullptr) return 0;
    uint8_t okCount = 0;
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        if (!sensor->isSensorOK(a) || motors[a] == nullptr) continue;
        const NvsStore::JointHome h = nvs->loadJointHome(a);
        if (!h.valid) continue;

        const float now = sensor->getAngle(a);
        const float delta = AXIS_ENC_SIGN[a] * wrap180(now - h.rawDeg);

        // Vượt quá hành trình lý thuyết + biên => nam châm dịch chuyển / dữ liệu sai
        const float maxSpan = DEFAULT_AXIS_CALIB_RANGE[a] + 15.0f;
        if (fabsf(delta) > maxSpan) {
            Serial.printf("[JM] J%u: restore BACON lech %.1f deg > span %.1f => bo qua\n",
                          a + 1, delta, maxSpan);
            continue;
        }

        motors[a]->setAbsoluteSteps(degreesToSteps(a, delta));
        encZeroRef[a] = sensor->getAccumulatedAngle(a);
        driftFault[a] = false;
        restored[a] = true;
        homed[a] = true;
        ++okCount;
        Serial.printf("[JM] J%u: restore tu NVS %+.2f deg\n", a + 1, delta);
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
    if (axis >= NUM_MOTORS || !homed[axis] || sensor == nullptr ||
        motors[axis] == nullptr || !sensor->isSensorOK(axis)) {
        return false;
    }
    if (motors[axis]->isRunning()) return false; // đang di chuyển: bỏ qua lần poll này

    const float diff = angleFromSteps(axis) - angleFromEncoder(axis);
    if (fabsf(diff) > RUNAWAY_ERROR_THRESHOLD) {
        driftFault[axis] = true;
        Serial.printf("[DRIFT] J%u lech %.2f deg (step=%.2f enc=%.2f)\n",
                      axis + 1, diff, angleFromSteps(axis), angleFromEncoder(axis));
    }
    return driftFault[axis];
}

bool JointModel::encOK(uint8_t axis) const {
    return (sensor != nullptr) && sensor->isSensorOK(axis);
}

String JointModel::toJson() {
    String j = "[";
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        if (a > 0) j += ",";
        char buf[224];
        snprintf(buf, sizeof(buf),
                 "{\"deg\":%.2f,\"encDeg\":%.2f,\"homed\":%s,\"restored\":%s,"
                 "\"encOK\":%s,\"drift\":%s,\"limitMin\":%.1f,\"limitMax\":%.1f}",
                 angleFromSteps(a), angleFromEncoder(a),
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
