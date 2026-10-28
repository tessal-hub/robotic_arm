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

    const bool wpActive = (workPlane != nullptr && workPlane->isEnabled());
    if (!wpActive && (job.z < 0.0f || job.z > 150.0f)) {
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

    // CIRCLE: 16 points around circumference at both draw height and pen-lift height
    if (job.shape == Planner::Shape::CIRCLE) {
        const float cx = job.x1;
        const float cy = job.y1;
        const float r = job.r;
        const float z = job.z;
        if (r <= 0.0f) {
            return {false, 0, "BAD_RADIUS"};
        }
        constexpr uint8_t CIRCLE_SAMPLES = 16;
        constexpr float TWO_PI_F = 6.28318530717958647692f;
        for (uint8_t i = 0; i < CIRCLE_SAMPLES; ++i) {
            const float ang = (TWO_PI_F * static_cast<float>(i)) / static_cast<float>(CIRCLE_SAMPLES);
            const float px = cx + r * cosf(ang);
            const float py = cy + r * sinf(ang);
            const kin::Pose pDraw{px, py, z};
            if (!checkPose(pDraw, 0.0f, workPlane)) return {false, static_cast<int>(i), "OUT_OF_REACH"};
            const kin::Pose pLift{px, py, z + PEN_LIFT_MM};
            if (!checkPose(pLift, PEN_LIFT_MM, workPlane)) return {false, static_cast<int>(CIRCLE_SAMPLES + i), "OUT_OF_REACH"};
        }
        return {true, -1, "OK"};
    }

    // SQUARE: 4 corners + 4 side midpoints at both draw height and pen-lift height
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
        const uint8_t nPoints = sizeof(perimeter) / sizeof(perimeter[0]);
        for (uint8_t i = 0; i < nPoints; ++i) {
            if (!checkPose(perimeter[i], 0.0f, workPlane)) return {false, static_cast<int>(i), "OUT_OF_REACH"};
            const kin::Pose pLift{perimeter[i].x, perimeter[i].y, perimeter[i].z + PEN_LIFT_MM};
            if (!checkPose(pLift, PEN_LIFT_MM, workPlane)) return {false, static_cast<int>(nPoints + i), "OUT_OF_REACH"};
        }
        return {true, -1, "OK"};
    }

    // NONE or unknown: treat as ok (no motion)
    return {true, -1, "OK"};
}
