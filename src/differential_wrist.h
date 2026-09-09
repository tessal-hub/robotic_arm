#ifndef DIFFERENTIAL_WRIST_H
#define DIFFERENTIAL_WRIST_H

#include <cstdint>

namespace wrist {

struct ActuatorState {
    float leftDeg{0.0f};
    float rightDeg{0.0f};
};

struct JointState {
    float tiltDeg{0.0f};
    float rollDeg{0.0f};
};

struct ActuatorSteps {
    int64_t leftSteps{0};
    int64_t rightSteps{0};
};

[[nodiscard]] JointState forward(float leftDeg, float rightDeg);
[[nodiscard]] ActuatorState inverse(float tiltDeg, float rollDeg);
[[nodiscard]] ActuatorSteps computeIncrementalSteps(float deltaTiltDeg, float deltaRollDeg,
                                                     float spd5, float spd6);

} // namespace wrist

#endif // DIFFERENTIAL_WRIST_H
