#include "trajectory_validator.h"
#include "kinematics.h"
#include "work_plane.h"
#include <cmath>

namespace {
bool checkPose(const kin::Pose& p, float workPlaneLift, const WorkPlane* workPlane) {
    kin::Pose rp = p;
    if (workPlane != nullptr && workPlane->isEnabled()) {
        const Point3D pt = workPlane->toRobotXYZ(p.x, p.y, workPlaneLift);
        rp.x = pt.x;
        rp.y = pt.y;
        rp.z = pt.z;
    }
    float q[6];
    return kin::ikPenDown(rp, q);
}
} // namespace

ValidationResult validateTrajectory(const Planner::Job& job, const WorkPlane* workPlane) {
    // POINT: 1 IK
    if (job.shape == Planner::Shape::POINT) {
        kin::Pose tgt{job.x1, job.y1, job.z};
        if (!checkPose(tgt, 0.0f, workPlane)) {
            return {false, 0, "OUT_OF_REACH"};
        }
        return {true, -1, "OK"};
    }

    if (workPlane != nullptr && workPlane->isEnabled()) {
        return {false, 0, "WORKPLANE_ENABLED"};
    }
    if (fabsf(job.z - DRAW_PLANE_Z_MM) > 0.01f) {
        return {false, 0, "WRONG_DRAW_PLANE"};
    }

    // LINE: validate the complete draw path plus lifted endpoints before moving.
    if (job.shape == Planner::Shape::LINE) {
        const float dx = job.x2 - job.x1;
        const float dy = job.y2 - job.y1;
        const float len = sqrtf(dx * dx + dy * dy);
        if (len < 1e-3f) return {false, 0, "BAD_LINE"};
        const uint32_t segments = static_cast<uint32_t>(ceilf(len / DRAW_LINE_SEGMENT_MM));
        for (uint32_t i = 0; i <= segments; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(segments);
            const kin::Pose p{job.x1 + dx * t, job.y1 + dy * t, job.z};
            if (!checkPose(p, 0.0f, workPlane)) {
                return {false, static_cast<int>(i), "OUT_OF_REACH"};
            }
        }
        const kin::Pose liftStart{job.x1, job.y1, job.z + PEN_LIFT_MM};
        const kin::Pose liftEnd{job.x2, job.y2, job.z + PEN_LIFT_MM};
        if (!checkPose(liftStart, PEN_LIFT_MM, workPlane) ||
            !checkPose(liftEnd, PEN_LIFT_MM, workPlane))
            return {false, static_cast<int>(segments + 1), "OUT_OF_REACH"};
        return {true, -1, "OK"};
    }

    // CIRCLE: 4 quadrants around circle
    if (job.shape == Planner::Shape::CIRCLE) {
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
        if (!checkPose(q1, 0.0f, workPlane)) return {false, 0, "OUT_OF_REACH"};
        if (!checkPose(q2, 0.0f, workPlane)) return {false, 1, "OUT_OF_REACH"};
        if (!checkPose(q3, 0.0f, workPlane)) return {false, 2, "OUT_OF_REACH"};
        if (!checkPose(q4, 0.0f, workPlane)) return {false, 3, "OUT_OF_REACH"};
        return {true, -1, "OK"};
    }

    // SQUARE: 4 corners + midpoint of each side. Planner will
    // still verify every ~1mm segment; this is the fast HTTP pre-flight gate.
    if (job.shape == Planner::Shape::SQUARE) {
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
            if (!checkPose(perimeter[i], 0.0f, workPlane)) return {false, static_cast<int>(i), "OUT_OF_REACH"};
        }
        return {true, -1, "OK"};
    }

    // NONE or unknown: treat as ok (no motion)
    return {true, -1, "OK"};
}
