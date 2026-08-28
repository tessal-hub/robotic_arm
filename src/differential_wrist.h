#ifndef DIFFERENTIAL_WRIST_H
#define DIFFERENTIAL_WRIST_H

#include <cmath>
#include <cstdint>

/**
 * @brief 2-DOF Coupled Bevel Gear Differential Wrist Kinematics Engine (J5 Tilt & J6 Roll).
 * 
 * Mechanical Architecture:
 * - 2 Coaxial Side Gears driven by Motor 5 (Left, M_L / axis 4) and Motor 6 (Right, M_R / axis 5)
 *   with dedicated AS5600 12-bit absolute magnetic encoders (E_L, E_R).
 * - 1 Spider / Output Bevel Pinion Gear mounted perpendicular on the Carrier.
 * 
 * Kinematic Mapping:
 * - Pure Tilt (J5): M_L = M_R (same speed & direction) -> Carrier rotates around pitch/tilt axis.
 * - Pure Roll (J6): M_L = -M_R (same speed, opposite direction) -> Output pinion rotates around tool roll axis.
 * - General Coupled Motion:
 *     J5_Tilt = (theta_L + theta_R) / 2
 *     J6_Roll = (theta_L - theta_R) / (2 * r_bevel)
 * 
 *     theta_L = J5_Tilt + (r_bevel * J6_Roll)
 *     theta_R = J5_Tilt - (r_bevel * J6_Roll)
 */
class DifferentialWrist {
public:
    struct ActuatorState {
        float leftDeg{0.0f};   // Motor/Encoder 5 (M_L / E_L / Axis 4)
        float rightDeg{0.0f};  // Motor/Encoder 6 (M_R / E_R / Axis 5)
    };

    struct JointState {
        float tiltDeg{0.0f};   // Decoupled Joint 5 (Pitch / Tilt)
        float rollDeg{0.0f};   // Decoupled Joint 6 (Tool Roll)
    };

    struct ActuatorSteps {
        int64_t leftSteps{0};
        int64_t rightSteps{0};
    };

    explicit DifferentialWrist(float bevelRatio = 1.0f);

    // Forward Differential Kinematics: (M_L, M_R) or (E_L, E_R) -> (J5_Tilt, J6_Roll)
    [[nodiscard]] JointState forward(float leftDeg, float rightDeg) const;
    [[nodiscard]] JointState forward(const ActuatorState& act) const { return forward(act.leftDeg, act.rightDeg); }

    // Inverse Differential Kinematics: (J5_Tilt, J6_Roll) -> (M_L, M_R)
    [[nodiscard]] ActuatorState inverse(float tiltDeg, float rollDeg) const;
    [[nodiscard]] ActuatorState inverse(const JointState& joint) const { return inverse(joint.tiltDeg, joint.rollDeg); }

    // Incremental Step Translation: (delta_J5, delta_J6) -> (delta_M5_steps, delta_M6_steps)
    [[nodiscard]] ActuatorSteps computeIncrementalSteps(float deltaTiltDeg, float deltaRollDeg,
                                                         float spd5, float spd6) const;

    // Velocity conversions (deg/s)
    [[nodiscard]] JointState forwardVelocity(float leftDegS, float rightDegS) const;
    [[nodiscard]] ActuatorState inverseVelocity(float tiltDegS, float rollDegS) const;

    // Getters and Configuration
    [[nodiscard]] float getBevelRatio() const noexcept { return bevelRatio_; }
    void setBevelRatio(float r) noexcept { if (r > 0.01f) bevelRatio_ = r; }

private:
    float bevelRatio_{1.0f}; // Ratio of output pinion teeth to side gear teeth (default 1.0)
};

extern DifferentialWrist g_diffWrist;

#endif // DIFFERENTIAL_WRIST_H