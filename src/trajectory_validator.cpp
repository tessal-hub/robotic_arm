#include "trajectory_validator.h"
#include "work_plane.h"
#include <cmath>

bool TrajectoryValidator::checkPose(const kin::Pose& p, float workPlaneLift) const {
    kin::Pose rp = p;
    if (wp_ != nullptr && wp_->isEnabled()) {
        const Point3D pt = wp_->toRobotXYZ(p.x, p.y, workPlaneLift);
        rp.x = pt.x;
        rp.y = pt.y;
        rp.z = pt.z;
    }
    float q[6];
    return kin::ikPenDown(rp, q);
}

ValidationResult TrajectoryValidator::validate(const Job& job, const kin::Pose& cur) const {
    // Current FK pose can be HOME/parked and is not necessarily a pen-down pose.
    // Validate only the pose sequence the planner will command for this job.
    (void)cur;
    // POINT: 1 IK
    if (job.type == Job::POINT) {
        kin::Pose tgt{job.x1, job.y1, job.z};
        if (!checkPose(tgt, 0.0f)) {
            return {false, 0, "OUT_OF_REACH"};
        }
        return {true, -1, "OK"};
    }

    // LINE: start, midpoint, end — lightweight B
    if (job.type == Job::LINE) {
        kin::Pose start{job.x1, job.y1, job.z};
        kin::Pose tgt{job.x2, job.y2, job.z};
        kin::Pose mid{(start.x + tgt.x) * 0.5f, (start.y + tgt.y) * 0.5f, job.z};
        if (!checkPose(start, 0.0f)) {
            return {false, 0, "OUT_OF_REACH"};
        }
        if (!checkPose(mid, 0.0f)) {
            return {false, 1, "OUT_OF_REACH"};
        }
        if (!checkPose(tgt, 0.0f)) {
            return {false, 2, "OUT_OF_REACH"};
        }
        return {true, -1, "OK"};
    }

    // CIRCLE: 4 quadrants around circle
    if (job.type == Job::CIRCLE) {
        float cx = job.x1;
        float cy = job.y1;
        float r = job.r;
        float z = job.z;
        if (r <= 0.0f) {
            return {false, 0, "BAD_RADIUS"};
        }
        // 4 quadrants: (cx+r,cy), (cx,cy+r), (cx-r,cy), (cx,cy-r) at height z
        kin::Pose q1{cx + r, cy, z};
        kin::Pose q2{cx, cy + r, z};
        kin::Pose q3{cx - r, cy, z};
        kin::Pose q4{cx, cy - r, z};
        if (!checkPose(q1, 0.0f)) return {false, 0, "OUT_OF_REACH"};
        if (!checkPose(q2, 0.0f)) return {false, 1, "OUT_OF_REACH"};
        if (!checkPose(q3, 0.0f)) return {false, 2, "OUT_OF_REACH"};
        if (!checkPose(q4, 0.0f)) return {false, 3, "OUT_OF_REACH"};
        return {true, -1, "OK"};
    }

    // SQUARE: 4 corners + midpoint of each side. Planner will
    // still verify every ~1mm segment; this is the fast HTTP pre-flight gate.
    if (job.type == Job::SQUARE) {
        if (job.r <= 0.0f) return {false, 0, "BAD_RADIUS"};
        const float half = job.r * 0.5f;
        const kin::Pose perimeter[] = {
            {job.x1 - half, job.y1 - half, job.z},
            {job.x1,        job.y1 - half, job.z},
            {job.x1 + half, job.y1 - half, job.z},
            {job.x1 + half, job.y1,        job.z},
            {job.x1 + half, job.y1 + half, job.z},
            {job.x1,        job.y1 + half, job.z},
            {job.x1 - half, job.y1 + half, job.z},
            {job.x1 - half, job.y1,        job.z},
        };
        for (uint8_t i = 0; i < sizeof(perimeter) / sizeof(perimeter[0]); ++i) {
            if (!checkPose(perimeter[i], 0.0f)) return {false, static_cast<int>(i), "OUT_OF_REACH"};
        }
        return {true, -1, "OK"};
    }

    // NONE or unknown: treat as ok (no motion)
    return {true, -1, "OK"};
}
