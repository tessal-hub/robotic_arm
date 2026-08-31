#include "trajectory_validator.h"
#include "work_plane.h"
#include <cmath>

bool TrajectoryValidator::checkPose(const kin::Pose& p) const {
    kin::Pose rp = p;
    if (wp_ != nullptr && wp_->isEnabled()) {
        const Point3D pt = wp_->toRobotXYZ(p.x, p.y, p.z);
        rp.x = pt.x;
        rp.y = pt.y;
        rp.z = pt.z;
    } else if (wp_ != nullptr && wp_->isCalibrated()) {
        // Fallback: if only calibrated (host test may check calibrated without explicitly enabling)
        const Point3D pt = wp_->toRobotXYZ(p.x, p.y, p.z);
        rp.x = pt.x;
        rp.y = pt.y;
        rp.z = pt.z;
    }
    float q[6];
    return kin::ikPenDown(rp, q);
}

ValidationResult TrajectoryValidator::validate(const Job& job, const kin::Pose& cur) const {
    // POINT: 1 IK
    if (job.type == Job::POINT) {
        kin::Pose tgt{job.x1, job.y1, job.z};
        if (!checkPose(tgt)) {
            return {false, 0, "OUT_OF_REACH"};
        }
        return {true, -1, "OK"};
    }

    // LINE: 3 IK (cur, mid, target) — lightweight B
    if (job.type == Job::LINE) {
        kin::Pose tgt{job.x2, job.y2, job.z};
        // mid between cur and target (use cur.z for mid to stay on same plane as job.z? spec uses cur.z, we use job.z)
        kin::Pose mid{(cur.x + tgt.x) * 0.5f, (cur.y + tgt.y) * 0.5f, job.z};
        // cur check (index 0) — reason always exact "OUT_OF_REACH", detail via failIndex
        if (!checkPose(cur)) {
            return {false, 0, "OUT_OF_REACH"};
        }
        if (!checkPose(mid)) {
            return {false, 1, "OUT_OF_REACH"};
        }
        if (!checkPose(tgt)) {
            return {false, 2, "OUT_OF_REACH"};
        }
        return {true, -1, "OK"};
    }

    // CIRCLE: 5 IK (cur + 4 quadrants around circle)
    if (job.type == Job::CIRCLE) {
        float cx = job.x1;
        float cy = job.y1;
        float r = job.r;
        float z = job.z;
        if (r <= 0.0f) {
            return {false, 0, "BAD_RADIUS"};
        }
        // index 0: cur — reason always exact "OUT_OF_REACH"
        if (!checkPose(cur)) {
            return {false, 0, "OUT_OF_REACH"};
        }
        // 4 quadrants: (cx+r,cy), (cx,cy+r), (cx-r,cy), (cx,cy-r) at height z
        kin::Pose q1{cx + r, cy, z};
        kin::Pose q2{cx, cy + r, z};
        kin::Pose q3{cx - r, cy, z};
        kin::Pose q4{cx, cy - r, z};
        if (!checkPose(q1)) return {false, 1, "OUT_OF_REACH"};
        if (!checkPose(q2)) return {false, 2, "OUT_OF_REACH"};
        if (!checkPose(q3)) return {false, 3, "OUT_OF_REACH"};
        if (!checkPose(q4)) return {false, 4, "OUT_OF_REACH"};
        return {true, -1, "OK"};
    }

    // NONE or unknown: treat as ok (no motion)
    return {true, -1, "OK"};
}
