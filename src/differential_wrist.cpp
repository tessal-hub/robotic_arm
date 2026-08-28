#include "differential_wrist.h"

DifferentialWrist g_diffWrist(1.0f);

DifferentialWrist::DifferentialWrist(float bevelRatio)
    : bevelRatio_((bevelRatio > 0.01f) ? bevelRatio : 1.0f) {}

DifferentialWrist::JointState DifferentialWrist::forward(float leftDeg, float rightDeg) const {
    JointState j;
    // Pure Tilt when theta_L = theta_R: Carrier rotates around tilt axis
    j.tiltDeg = (leftDeg + rightDeg) * 0.5f;
    // Pure Roll when theta_L = -theta_R: Output pinion rotates around roll axis
    j.rollDeg = (leftDeg - rightDeg) / (2.0f * bevelRatio_);
    return j;
}

DifferentialWrist::ActuatorState DifferentialWrist::inverse(float tiltDeg, float rollDeg) const {
    ActuatorState a;
    // Motor Left (M_L): Tilt + (r_bevel * Roll)
    a.leftDeg = tiltDeg + (bevelRatio_ * rollDeg);
    // Motor Right (M_R): Tilt - (r_bevel * Roll)
    a.rightDeg = tiltDeg - (bevelRatio_ * rollDeg);
    return a;
}

DifferentialWrist::ActuatorSteps DifferentialWrist::computeIncrementalSteps(
    float deltaTiltDeg, float deltaRollDeg, float spd5, float spd6) const {
    ActuatorSteps steps;
    const float deltaLeftDeg = deltaTiltDeg + (bevelRatio_ * deltaRollDeg);
    const float deltaRightDeg = deltaTiltDeg - (bevelRatio_ * deltaRollDeg);

    steps.leftSteps = static_cast<int64_t>(lroundf(deltaLeftDeg * spd5));
    steps.rightSteps = static_cast<int64_t>(lroundf(deltaRightDeg * spd6));
    return steps;
}

DifferentialWrist::JointState DifferentialWrist::forwardVelocity(float leftDegS, float rightDegS) const {
    JointState j;
    j.tiltDeg = (leftDegS + rightDegS) * 0.5f;
    j.rollDeg = (leftDegS - rightDegS) / (2.0f * bevelRatio_);
    return j;
}

DifferentialWrist::ActuatorState DifferentialWrist::inverseVelocity(float tiltDegS, float rollDegS) const {
    ActuatorState a;
    a.leftDeg = tiltDegS + (bevelRatio_ * rollDegS);
    a.rightDeg = tiltDegS - (bevelRatio_ * rollDegS);
    return a;
}
