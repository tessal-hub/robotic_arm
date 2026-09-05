// Host tests: JointModel direction logic (cwForDelta, stepsPerDegree).
#include "config.h"
#include "joint_calibration.h"
#include <cmath>
#include <cstdio>

static int g_fail = 0;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("FAIL: %s (line %d)\n", msg, __LINE__);              \
            ++g_fail;                                                        \
        }                                                                    \
    } while (0)

// Mirror JointModel::cwForDelta — must stay in sync with joint_model.cpp
static bool cwForDelta(uint8_t axis, float deltaDeg) {
    return (AXIS_STEP_SIGN[axis] > 0) ? (deltaDeg >= 0.0f) : (deltaDeg < 0.0f);
}

static float stepsPerDegree(uint8_t axis) {
    const float stepsPerRev =
        static_cast<float>(DEFAULT_FULL_STEPS) * static_cast<float>(DEFAULT_MICROSTEPS) *
        DEFAULT_AXIS_GEAR_RATIOS[axis];
    return stepsPerRev / 360.0f;
}

static void testCwForDeltaJ1PositiveSign() {
    CHECK(cwForDelta(0, +10.0f) == true, "J1 +delta => cw");
    CHECK(cwForDelta(0, -10.0f) == false, "J1 -delta => ccw");
    CHECK(cwForDelta(0, 0.0f) == true, "J1 zero delta => cw (STEP_SIGN +1)");
}

static void testCwForDeltaJ2NegativeSign() {
    // J2 AXIS_STEP_SIGN = +1 (config.h: góc dương vươn ra ngoài)
    CHECK(AXIS_STEP_SIGN[1] > 0, "J2 STEP_SIGN positive");
    CHECK(cwForDelta(1, +10.0f) == true, "J2 +delta => cw");
    CHECK(cwForDelta(1, -10.0f) == false, "J2 -delta => ccw");
}

static void testCwForDeltaJ3NegativeSign() {
    CHECK(AXIS_STEP_SIGN[2] > 0, "J3 STEP_SIGN positive");
    CHECK(cwForDelta(2, +5.0f) == true, "J3 +delta => cw");
    CHECK(cwForDelta(2, -5.0f) == false, "J3 -delta => ccw");
}

static void testStepsPerDegreePositive() {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        CHECK(stepsPerDegree(a) > 0.0f, "stepsPerDegree > 0");
    }
}

static void testJ3SoftLimitContract() {
    CHECK(J3_MIN_LIMIT == 0.0f, "J3 minimum limit is zero degrees");
    CHECK(J3_MAX_LIMIT == 90.0f, "J3 maximum limit is positive 90 degrees");
    CHECK(DEFAULT_AXIS_LIMIT_MIN[2] == J3_MIN_LIMIT,
          "J3 minimum in shared soft-limit table");
    CHECK(DEFAULT_AXIS_LIMIT_MAX[2] == J3_MAX_LIMIT,
          "J3 maximum in shared soft-limit table");
    CHECK(DEFAULT_AXIS_CALIB_RANGE[2] == 90.0f,
          "J3 calibration range is 90 degrees");
}

static void testMeasuredCalibrationAcceptance() {
    constexpr uint8_t axis = 2;
    const float configured = jointcal::configuredStepsPerDegree(axis);
    CHECK(jointcal::isPlausible(axis, +1.0f, configured * 0.5f),
          "accept 50% measured steps/degree boundary");
    CHECK(jointcal::isPlausible(axis, -1.0f, configured * 2.0f),
          "accept 200% measured steps/degree boundary");
    CHECK(!jointcal::isPlausible(axis, 0.0f, configured),
          "reject calibration with invalid encoder sign");
    CHECK(!jointcal::isPlausible(axis, +1.0f, configured * 0.49f),
          "reject calibration below physical range");
    CHECK(!jointcal::isPlausible(axis, +1.0f, configured * 2.01f),
          "reject calibration above physical range");
    CHECK(!jointcal::isPlausible(axis, +1.0f, NAN),
          "reject non-finite calibration");
    CHECK(jointcal::normalizedSign(-1.0f) == -1.0f, "preserve negative encoder sign");
    CHECK(jointcal::normalizedSign(+1.0f) == +1.0f, "preserve positive encoder sign");
}

static void testJ5EncoderDirection() {
    CHECK(AXIS_ENC_SIGN[4] == -1, "J5 encoder direction is reversed");
}

int main() {
    testCwForDeltaJ1PositiveSign();
    testCwForDeltaJ2NegativeSign();
    testCwForDeltaJ3NegativeSign();
    testStepsPerDegreePositive();
    testJ3SoftLimitContract();
    testMeasuredCalibrationAcceptance();
    testJ5EncoderDirection();
    if (g_fail == 0) {
        std::printf("ALL PASSED (joint logic)\n");
        return 0;
    }
    std::printf("%d FAILED\n", g_fail);
    return 1;
}
