#include "differential_wrist.h"

#include <cmath>

namespace wrist {

JointState forward(float leftDeg, float rightDeg) {
    return {(leftDeg + rightDeg) * 0.5f, (leftDeg - rightDeg) * 0.5f};
}

ActuatorState inverse(float tiltDeg, float rollDeg) {
    return {tiltDeg + rollDeg, tiltDeg - rollDeg};
}

ActuatorSteps computeIncrementalSteps(float deltaTiltDeg, float deltaRollDeg,
                                      float spd5, float spd6) {
    return {
        static_cast<int64_t>(lroundf((deltaTiltDeg + deltaRollDeg) * spd5)),
        static_cast<int64_t>(lroundf((deltaTiltDeg - deltaRollDeg) * spd6)),
    };
}

} // namespace wrist
