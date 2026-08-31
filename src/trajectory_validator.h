#ifndef TRAJECTORY_VALIDATOR_H
#define TRAJECTORY_VALIDATOR_H

#include <Arduino.h>
#include "kinematics.h"

class WorkPlane;

/**
 * ValidationResult — kết quả pre-flight IK lightweight B.
 * - ok: true nếu toàn bộ quỹ đạo trong tầm với
 * - failIndex: chỉ số điểm fail (0=POINT/target, LINE:0=start/1=mid/2=end, CIRCLE:0=cur/1..4=quadrants), -1 nếu ok
 * - reason: "OK" hoặc "OUT_OF_REACH" / "OUT_OF_REACH mid" / "BAD_RADIUS"
 */
struct ValidationResult {
    bool ok{true};
    int failIndex{-1};
    String reason{"OK"};
};

/**
 * TrajectoryValidator — pure C++ host-testable, lightweight B (§3.4)
 * - POINT: 1 IK (target)
 * - LINE: 3 IK (cur, mid, target)
 * - CIRCLE: 5 IK (cur + 4 quadrants)
 * Áp dụng WorkPlane::toRobotXYZ nếu WorkPlane* đã calibrate+enabled trước khi IK.
 * Không phụ thuộc Arduino ngoài String (stub host-testable), không đụng hardware, không change DH.
 */
class TrajectoryValidator {
public:
    struct Job {
        enum Type : uint8_t { NONE = 0, POINT, LINE, CIRCLE };
        Type type{Type::NONE};
        float x1{0}, y1{0};
        float x2{0}, y2{0};
        float z{0};
        float r{0};
        float feedMmS{20.0f};
        bool drawNow{true};
    };

    explicit TrajectoryValidator(WorkPlane* wp = nullptr) noexcept : wp_(wp) {}

    void setWorkPlane(WorkPlane* wp) noexcept { wp_ = wp; }
    WorkPlane* getWorkPlane() const noexcept { return wp_; }

    ValidationResult validate(const Job& job, const kin::Pose& cur) const;

private:
    bool checkPose(const kin::Pose& p) const;
    WorkPlane* wp_{nullptr};
};

#endif // TRAJECTORY_VALIDATOR_H
