#include "joint_model.h"
#include "drift_policy.h"
#include "differential_wrist.h"
#include "joint_calibration.h"
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
        s_measuredSpd[i] = jointcal::configuredStepsPerDegree(i);
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
    if (s_hasMeasured[axis] && jointcal::isPlausible(axis, s_encSign[axis], s_measuredSpd[axis])) {
        return s_measuredSpd[axis];
    }
    return jointcal::configuredStepsPerDegree(axis);
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
        const float rawDeg = sensor->getAngle(axis);
        encZeroRef[axis] = encAngle;
        bool nvsSaved = false;
        if (nvs != nullptr) nvsSaved = nvs->saveJointHome(axis, rawDeg);
        Serial.printf("[JM] SetHome J%u (enc=ok, raw=%.1f deg, zeroRef=%.1f deg, NVS=%s)\n",
                      axis + 1, rawDeg, encAngle, nvsSaved ? "OK" : "FAIL");
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
        s_measuredSpd[axis] = jointcal::configuredStepsPerDegree(axis);
    }
    if (nvs != nullptr) {
        nvs->clearJointHome(axis);
        nvs->clearCalib(axis);
    }
}

void JointModel::applyHomingCalibration(uint8_t axis, float encSign, float stepsPerDeg) {
    if (axis >= NUM_MOTORS) return;
    if (!jointcal::isPlausible(axis, encSign, stepsPerDeg)) {
        Serial.printf("[JM] Calib J%u bi bo qua: encSign=%+.3f, steps/deg=%.2f khong hop le\n",
                      axis + 1, encSign, stepsPerDeg);
        resetHomingCalibration(axis);
        return;
    }
    s_encSign[axis] = jointcal::normalizedSign(encSign);
    s_measuredSpd[axis] = stepsPerDeg;
    s_hasMeasured[axis] = true;
    if (nvs != nullptr && !nvs->saveCalib(axis, s_encSign[axis], s_measuredSpd[axis])) {
        Serial.printf("[JM] LOI: khong luu duoc calib J%u vao NVS\n", axis + 1);
    }
    Serial.printf("[JM] Calib J%u: encSign=%+.0f, steps/deg=%.2f\n",
                  axis + 1, s_encSign[axis], s_measuredSpd[axis]);
}

void JointModel::resetHomingCalibration(uint8_t axis) {
    if (axis >= NUM_MOTORS) return;
    s_encSign[axis] = AXIS_ENC_SIGN[axis];
    s_measuredSpd[axis] = jointcal::configuredStepsPerDegree(axis);
    s_hasMeasured[axis] = false;
}

uint8_t JointModel::restoreFromNVS() {
    if (nvs == nullptr || sensor == nullptr) return 0;
    uint8_t okCount = 0;
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        s_encSign[a] = AXIS_ENC_SIGN[a];
        s_measuredSpd[a] = jointcal::configuredStepsPerDegree(a);
        s_hasMeasured[a] = false;

        const NvsStore::CalibData calib = nvs->loadCalib(a);
        if (calib.valid && jointcal::isPlausible(a, calib.encSign, calib.stepsPerDeg)) {
            s_encSign[a] = jointcal::normalizedSign(calib.encSign);
            s_measuredSpd[a] = calib.stepsPerDeg;
            s_hasMeasured[a] = true;
            Serial.printf("[JM] J%u: dung calib NVS encSign=%+.0f, steps/deg=%.2f\n",
                          a + 1, s_encSign[a], s_measuredSpd[a]);
        } else if (calib.valid) {
            Serial.printf("[JM] J%u: bo qua calib NVS khong hop le\n", a + 1);
        }

        if (!sensor->isSensorOK(a) || motors[a] == nullptr) {
            Serial.printf("[JM] J%u: khong restore NVS (sensor/motor chua san sang)\n", a + 1);
            continue;
        }
        const NvsStore::JointHome h = nvs->loadJointHome(a);
        if (!h.valid) {
            Serial.printf("[JM] J%u: khong co home hop le trong NVS\n", a + 1);
            continue;
        }

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
        Serial.printf("[JM] J%u: restore NVS rawSaved=%.1f rawNow=%.1f -> %+.2f deg (encZeroRef=%.1f)\n",
                      a + 1, h.rawDeg, now, delta, encZeroRef[a]);
    }
    Serial.printf("[JM] Restore xong: %u/%u khop\n", okCount, NUM_MOTORS);
    return okCount;
}

uint8_t JointModel::homedCount() const noexcept {
    uint8_t n = 0;
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) if (homed[i]) n++;
    return n;
}

bool JointModel::allPositioningHomed() const noexcept {
    for (uint8_t i = 0; i < 4; ++i) { // J1..J4
        if (!homed[i]) return false;
    }
    return true;
}

bool JointModel::updateDriftCheck(uint8_t /*axis*/) {
    // Drift watchdog disabled by user request: open-loop step counting is ground truth
    return false;
}

bool JointModel::hasMeasuredCalibration(uint8_t axis) noexcept {
    return axis < NUM_MOTORS && s_hasMeasured[axis] &&
           jointcal::isPlausible(axis, s_encSign[axis], s_measuredSpd[axis]);
}

bool JointModel::hasAnyDriftFault() const noexcept {
    return false;
}

void JointModel::clearAllDriftFaults() noexcept {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        driftFault[i] = false;
        driftFailCount[i] = 0;
        lastRunningMs[i] = millis();
        if (homed[i] && encOK(i)) {
            resyncFromEncoder(i);
        } else if (!homed[i] && motors[i] != nullptr) {
            motors[i]->setAbsoluteSteps(0);
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
                 "\"encOK\":%s,\"drift\":%s,\"calibMeasured\":%s,\"stepsPerDeg\":%.3f,"
                 "\"limitMin\":%.1f,\"limitMax\":%.1f}",
                 actDeg, angleFromEncoder(a),
                 homed[a] ? "true" : "false",
                 restored[a] ? "true" : "false",
                 encOK(a) ? "true" : "false",
                 driftFault[a] ? "true" : "false",
                 hasMeasuredCalibration(a) ? "true" : "false",
                 stepsPerDegree(a),
                 DEFAULT_AXIS_LIMIT_MIN[a], DEFAULT_AXIS_LIMIT_MAX[a]);
        j += buf;
    }
    j += "]";
    return j;
}
