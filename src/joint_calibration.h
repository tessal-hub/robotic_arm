#ifndef JOINT_CALIBRATION_H
#define JOINT_CALIBRATION_H

#include <cmath>
#include "config.h"

// Calibration data comes from a physical endstop sweep. Keep its acceptance
// envelope close to the homing cross-check so a corrupted NVS record cannot
// change the motion scale at boot.
namespace jointcal {

constexpr float MIN_STEPS_PER_DEGREE_RATIO = 0.5f;
constexpr float MAX_STEPS_PER_DEGREE_RATIO = 2.0f;

inline float configuredStepsPerDegree(uint8_t axis) {
    if (axis >= NUM_MOTORS) return 1.0f;
    const float stepsPerRev =
        static_cast<float>(DEFAULT_FULL_STEPS) * static_cast<float>(DEFAULT_MICROSTEPS) *
        DEFAULT_AXIS_GEAR_RATIOS[axis];
    return stepsPerRev / 360.0f;
}

inline bool isPlausible(uint8_t axis, float encSign, float stepsPerDeg) {
    if (axis >= NUM_MOTORS || !std::isfinite(encSign) || !std::isfinite(stepsPerDeg)) {
        return false;
    }
    if (fabsf(fabsf(encSign) - 1.0f) > 0.01f) return false;

    const float configured = configuredStepsPerDegree(axis);
    const float ratio = stepsPerDeg / configured;
    return ratio >= MIN_STEPS_PER_DEGREE_RATIO && ratio <= MAX_STEPS_PER_DEGREE_RATIO;
}

inline float normalizedSign(float encSign) {
    return encSign < 0.0f ? -1.0f : 1.0f;
}

} // namespace jointcal

#endif // JOINT_CALIBRATION_H
