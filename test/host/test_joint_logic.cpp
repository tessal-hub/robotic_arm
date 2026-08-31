// Host tests: JointModel direction logic (cwForDelta, stepsPerDegree).
#include "config.h"
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

int main() {
    testCwForDeltaJ1PositiveSign();
    testCwForDeltaJ2NegativeSign();
    testCwForDeltaJ3NegativeSign();
    testStepsPerDegreePositive();
    if (g_fail == 0) {
        std::printf("ALL PASSED (joint logic)\n");
        return 0;
    }
    std::printf("%d FAILED\n", g_fail);
    return 1;
}
