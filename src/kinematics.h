#ifndef KINEMATICS_H
#define KINEMATICS_H

namespace kin {

// Đơn vị: mm, độ. Nguồn chân lý: docs/ARM_GEOMETRY.md (Modified DH - Craig).
struct Pose {
    float x{0};
    float y{0};
    float z{0};
};

struct JointAngles {
    float j[6]{0, 0, 0, 0, 0, 0}; // góc encoder (độ)
};

struct FkResult {
    Pose tcp;
    Pose wristCenter;
};

// DH constants (bản sao của config.h nhưng thuần C++ để test trên host)
constexpr float D1 = 139.0f;
constexpr float A2 = 138.0f;
constexpr float L_FORE = 153.6863f;              // sqrt(88^2+126^2)
constexpr float DELTA_WRIST = 55.0587f;          // atan2(126, 88) deg
constexpr float D_TOOL = 20.0f;

constexpr float THETA2_OFFSET = -90.0f;

// Giới hạn mềm khớp khi chọn nghiệm IK (độ, theo encoder home)
constexpr float J1_MIN = -90.0f, J1_MAX = 90.0f;
constexpr float J2_MIN = -90.0f, J2_MAX = 90.0f;
constexpr float J3_MIN = 0.0f,   J3_MAX = 90.0f;
constexpr float J5_MIN = -120.0f, J5_MAX = 120.0f;

// FK: encoder(deg) -> TCP + wrist center. Luôn khả thi.
FkResult forward(const float enc[6]);

/**
 * IK closed-form cho chế độ VẼ (bút hướng thẳng xuống, J4 roll giữ mặt phẳng,
 * J6 = 0). Trả về false nếu ngoài vùng với / vi phạm giới hạn mềm.
 * Chọn nhánh khuỷu thoả J3 ∈ [0,90].
 */
bool ikPenDown(const Pose& target, float outEnc[6]);

} // namespace kin

#endif // KINEMATICS_H
