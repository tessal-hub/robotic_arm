// Host tests: TrajectoryValidator lightweight B (§3.4)
// POINT 1 IK, LINE 3 IK (cur, mid, target), CIRCLE 5 IK (cur + 4 quadrants)
// WorkPlane toRobotXYZ applied if enabled.

#include "trajectory_validator.h"
#include "work_plane.h"
#include "kinematics.h"
#include <cstdio>
#include <cmath>
#include <cstring>

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { std::printf("FAIL: %s (line %d)\n", msg, __LINE__); ++g_fail; } \
} while(0)
#define PASS(msg) do { std::printf("PASS: %s\n", msg); ++g_pass; } while(0)

static void test_point_reachable_pass() {
    TrajectoryValidator v;
    kin::Pose cur{177, 0, 365}; // home reachable
    TrajectoryValidator::Job job;
    job.type = TrajectoryValidator::Job::POINT;
    job.x1 = 120; job.y1 = 20; job.z = 20;
    auto r = v.validate(job, cur);
    if (r.ok && r.failIndex==-1) PASS("point_reachable_pass");
    else { CHECK(false, "point_reachable_pass should be ok"); }
}

static void test_point_out_of_reach_reject() {
    TrajectoryValidator v;
    kin::Pose cur{177, 0, 365};
    TrajectoryValidator::Job job;
    job.type = TrajectoryValidator::Job::POINT;
    job.x1 = 400; job.y1 = 0; job.z = 365; // far outside 291
    auto r = v.validate(job, cur);
    if (!r.ok && r.failIndex==0) PASS("point_out_of_reach_reject");
    else { CHECK(false, "point_out_of_reach_reject should fail at 0"); }
}

static void test_line_mid_out_of_reach_reject() {
    TrajectoryValidator v;
    kin::Pose cur{120, 0, 20}; // reachable
    TrajectoryValidator::Job job;
    job.type = TrajectoryValidator::Job::LINE;
    job.x1 = 120; job.y1 = 0; // start (ignored for cur-based check, but keep consistent)
    job.x2 = 500; job.y2 = 0; // target far
    job.z = 20;
    auto r = v.validate(job, cur);
    // cur (120) reachable, mid (310) out -> failIndex 1
    if (!r.ok && r.failIndex==1) PASS("line_mid_out_of_reach_reject");
    else {
        std::printf("  line_mid: ok=%d failIndex=%d reason=%s\n", r.ok, r.failIndex, r.reason.c_str());
        CHECK(false, "line_mid_out_of_reach_reject expected failIndex 1");
    }
}

static void test_line_all_reachable_pass() {
    TrajectoryValidator v;
    kin::Pose cur{100, 0, 20};
    TrajectoryValidator::Job job;
    job.type = TrajectoryValidator::Job::LINE;
    job.x1 = 100; job.y1 = 0;
    job.x2 = 150; job.y2 = 0;
    job.z = 20;
    auto r = v.validate(job, cur);
    if (r.ok) PASS("line_all_reachable_pass");
    else { CHECK(false, "line_all_reachable_pass should be ok"); }
}

static void test_circle_outside() {
    TrajectoryValidator v;
    kin::Pose cur{100, 0, 20}; // reachable
    TrajectoryValidator::Job job;
    job.type = TrajectoryValidator::Job::CIRCLE;
    job.x1 = 0; job.y1 = 0; // center
    job.r = 300; job.z = 20;
    auto r = v.validate(job, cur);
    if (!r.ok) PASS("circle_outside");
    else { CHECK(false, "circle_outside should fail"); }
}

static void test_workplane_transform() {
    // WorkPlane with origin offset 200mm in X, so UCS (100,0,20) -> robot (300,0,20) which is out of reach
    // Without WorkPlane, UCS (100,0,20) is reachable
    WorkPlane wp;
    Point3D p1{200, 0, 0}, p2{300, 0, 0}, p3{200, 100, 0};
    bool calibOk = wp.setThreePointCalibration(p1, p2, p3);
    CHECK(calibOk, "workplane calib ok");
    // Validator without workplane: point (100,0,20) reachable
    {
        TrajectoryValidator v;
        kin::Pose cur{100,0,20};
        TrajectoryValidator::Job job;
        job.type = TrajectoryValidator::Job::POINT;
        job.x1 = 100; job.y1 = 0; job.z = 20;
        auto r = v.validate(job, cur);
        CHECK(r.ok, "workplane_transform without wp should be ok");
    }
    // Validator with workplane: same UCS (100,0,20) maps to robot (300,0,20) -> out of reach
    {
        TrajectoryValidator v(&wp);
        // For POINT, cur not checked, so any cur ok. Use cur (0,0,20) UCS -> robot (200,0,20) reachable, target UCS (100,0,20) -> robot (300,0,20) out
        kin::Pose cur2{0,0,20};
        TrajectoryValidator::Job job;
        job.type = TrajectoryValidator::Job::POINT;
        job.x1 = 100; job.y1 = 0; job.z = 20;
        auto r = v.validate(job, cur2);
        if (!r.ok) PASS("workplane_transform");
        else { CHECK(false, "workplane_transform should fail with offset"); }
    }
    // Also test that without workplane same point passes, with workplane fails
    {
        TrajectoryValidator v_no;
        TrajectoryValidator v_wp(&wp);
        kin::Pose cur{0,0,20};
        TrajectoryValidator::Job job;
        job.type = TrajectoryValidator::Job::POINT;
        job.x1 = 100; job.y1 = 0; job.z = 20;
        auto r_no = v_no.validate(job, cur);
        auto r_wp = v_wp.validate(job, cur);
        if (r_no.ok && !r_wp.ok) PASS("workplane_transform_comparison");
        else { CHECK(false, "workplane_transform_comparison"); }
    }
}

static void test_line_from_home_park_pose_pass() {
    TrajectoryValidator v;
    // HOME TCP is intentionally not a pen-down pose. It must not reject an
    // otherwise reachable line; Planner stages to the line start at lift height.
    TrajectoryValidator::Job job;
    job.type = TrajectoryValidator::Job::LINE;
    job.x1 = 55; job.y1 = -15; job.x2 = 175; job.y2 = -15; job.z = -10;
    const auto r = v.validate(job, {177, 0, 365});
    if (r.ok) PASS("line_from_home_park_pose_pass");
    else { CHECK(false, "line_from_home_park_pose_pass should be ok"); }
}

static void test_square_reachable_pass() {
    TrajectoryValidator v;
    TrajectoryValidator::Job job;
    job.type = TrajectoryValidator::Job::SQUARE;
    job.x1 = 120; job.y1 = 0; job.z = 20; job.r = 30;
    const auto r = v.validate(job, {100, 0, 20});
    if (r.ok) PASS("square_reachable_pass");
    else { CHECK(false, "square_reachable_pass should be ok"); }
}

static void test_square_bad_side_reject() {
    TrajectoryValidator v;
    TrajectoryValidator::Job job;
    job.type = TrajectoryValidator::Job::SQUARE;
    job.r = 0;
    const auto r = v.validate(job, {100, 0, 20});
    if (!r.ok && std::strcmp(r.reason.c_str(), "BAD_RADIUS") == 0) PASS("square_bad_side_reject");
    else { CHECK(false, "square_bad_side_reject should fail"); }
}

static void test_workplane_target_uses_plane_not_legacy_z() {
    // Planner treats job.z as its legacy paper reference. With UCS enabled,
    // target draw points are w=0 on the calibrated plane, not w=job.z.
    WorkPlane wp;
    CHECK(wp.setThreePointCalibration({0, 0, 20}, {100, 0, 20}, {0, 100, 20}),
          "horizontal workplane calibration");
    TrajectoryValidator v(&wp);
    TrajectoryValidator::Job job;
    job.type = TrajectoryValidator::Job::POINT;
    job.x1 = 120; job.y1 = 20; job.z = 435; // Base target must remain (120,20,20).
    const auto r = v.validate(job, {120, 20, 0});
    if (r.ok) PASS("workplane_target_uses_plane_not_legacy_z");
    else { CHECK(false, "workplane target must validate at w=0, not w=job.z"); }
}

int main() {
    test_point_reachable_pass();
    test_point_out_of_reach_reject();
    test_line_mid_out_of_reach_reject();
    test_line_all_reachable_pass();
    test_line_from_home_park_pose_pass();
    test_circle_outside();
    test_square_reachable_pass();
    test_square_bad_side_reject();
    test_workplane_transform();
    test_workplane_target_uses_plane_not_legacy_z();
    if (g_fail==0) {
        std::printf("ALL PASSED (%d tests)\n", g_pass);
        return 0;
    }
    std::printf("%d FAILED, %d PASSED\n", g_fail, g_pass);
    return 1;
}
