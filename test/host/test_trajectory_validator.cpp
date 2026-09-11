// Host tests: trajectory pre-flight validation
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
    Planner::Job job;
    job.shape = Planner::Shape::POINT;
    job.x1 = 120; job.y1 = 20; job.z = 20;
    auto r = validateTrajectory(job);
    if (r.ok && r.failIndex==-1) PASS("point_reachable_pass");
    else { CHECK(false, "point_reachable_pass should be ok"); }
}

static void test_point_out_of_reach_reject() {
    Planner::Job job;
    job.shape = Planner::Shape::POINT;
    job.x1 = 400; job.y1 = 0; job.z = 365; // far outside 291
    auto r = validateTrajectory(job);
    if (!r.ok && r.failIndex==0) PASS("point_out_of_reach_reject");
    else { CHECK(false, "point_out_of_reach_reject should fail at 0"); }
}

static void test_line_mid_out_of_reach_reject() {
    Planner::Job job;
    job.shape = Planner::Shape::LINE;
    job.x1 = 120; job.y1 = 0; // start (ignored for cur-based check, but keep consistent)
    job.x2 = 500; job.y2 = 0; // target far
    job.z = 20;
    auto r = validateTrajectory(job);
    if (!r.ok && r.failIndex > 0) PASS("line_mid_out_of_reach_reject");
    else {
        std::printf("  line_mid: ok=%d failIndex=%d reason=%s\n", r.ok, r.failIndex, r.reason.c_str());
        CHECK(false, "line path should reject at an unreachable sample");
    }
}

static void test_line_all_reachable_pass() {
    Planner::Job job;
    job.shape = Planner::Shape::LINE;
    job.x1 = 100; job.y1 = 0;
    job.x2 = 150; job.y2 = 0;
    job.z = 20;
    auto r = validateTrajectory(job);
    if (r.ok) PASS("line_all_reachable_pass");
    else { CHECK(false, "line_all_reachable_pass should be ok"); }
}

static void test_circle_outside() {
    Planner::Job job;
    job.shape = Planner::Shape::CIRCLE;
    job.x1 = 0; job.y1 = 0; // center
    job.r = 300; job.z = 20;
    auto r = validateTrajectory(job);
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
        Planner::Job job;
        job.shape = Planner::Shape::POINT;
        job.x1 = 100; job.y1 = 0; job.z = 20;
        auto r = validateTrajectory(job);
        CHECK(r.ok, "workplane_transform without wp should be ok");
    }
    // Validator with workplane: same UCS (100,0,20) maps to robot (300,0,20) -> out of reach
    {
        // For POINT, cur not checked, so any cur ok. Use cur (0,0,20) UCS -> robot (200,0,20) reachable, target UCS (100,0,20) -> robot (300,0,20) out
        Planner::Job job;
        job.shape = Planner::Shape::POINT;
        job.x1 = 100; job.y1 = 0; job.z = 20;
        auto r = validateTrajectory(job, &wp);
        if (!r.ok) PASS("workplane_transform");
        else { CHECK(false, "workplane_transform should fail with offset"); }
    }
    // Also test that without workplane same point passes, with workplane fails
    {
        Planner::Job job;
        job.shape = Planner::Shape::POINT;
        job.x1 = 100; job.y1 = 0; job.z = 20;
        auto r_no = validateTrajectory(job);
        auto r_wp = validateTrajectory(job, &wp);
        if (r_no.ok && !r_wp.ok) PASS("workplane_transform_comparison");
        else { CHECK(false, "workplane_transform_comparison"); }
    }
}

static void test_zero_length_line_reject() {
    Planner::Job job;
    job.shape = Planner::Shape::LINE;
    job.x1 = job.x2 = 120;
    job.y1 = job.y2 = 0;
    job.z = 20;
    const auto r = validateTrajectory(job);
    if (!r.ok && std::strcmp(r.reason.c_str(), "BAD_LINE") == 0) PASS("zero_length_line_reject");
    else { CHECK(false, "zero-length line must be rejected before enqueue"); }
}

static void test_line_from_home_park_pose_pass() {
    // HOME TCP is intentionally not a pen-down pose. It must not reject an
    // otherwise reachable line; Planner stages to the line start at lift height.
    Planner::Job job;
    job.shape = Planner::Shape::LINE;
    job.x1 = 120; job.y1 = -15; job.x2 = 210; job.y2 = -15; job.z = DRAW_PLANE_Z_MM;
    const auto r = validateTrajectory(job);
    if (r.ok) PASS("line_from_home_park_pose_pass");
    else { CHECK(false, "line_from_home_park_pose_pass should be ok"); }
}

static void test_square_reachable_pass() {
    Planner::Job job;
    job.shape = Planner::Shape::SQUARE;
    job.x1 = 120; job.y1 = 0; job.z = 20; job.r = 30;
    const auto r = validateTrajectory(job);
    if (r.ok) PASS("square_reachable_pass");
    else { CHECK(false, "square_reachable_pass should be ok"); }
}

static void test_square_bad_side_reject() {
    Planner::Job job;
    job.shape = Planner::Shape::SQUARE;
    job.z = DRAW_PLANE_Z_MM;
    job.r = 0;
    const auto r = validateTrajectory(job);
    if (!r.ok && std::strcmp(r.reason.c_str(), "BAD_RADIUS") == 0) PASS("square_bad_side_reject");
    else { CHECK(false, "square_bad_side_reject should fail"); }
}

static void test_fixed_draw_plane() {
    Planner::Job job;
    job.shape = Planner::Shape::LINE;
    job.x1 = 100; job.y1 = 0; job.x2 = 150; job.y2 = 0;
    job.z = DRAW_PLANE_Z_MM + 2.0f;
    const auto wrongZ = validateTrajectory(job);
    CHECK(!wrongZ.ok && std::strcmp(wrongZ.reason.c_str(), "WRONG_DRAW_PLANE") == 0,
          "draw rejects every base Z except the fixed plane");

    WorkPlane wp;
    CHECK(wp.setThreePointCalibration({0, 0, DRAW_PLANE_Z_MM}, {100, 0, DRAW_PLANE_Z_MM},
                                      {0, 100, DRAW_PLANE_Z_MM}), "workplane setup");
    job.z = DRAW_PLANE_Z_MM;
    const auto transformed = validateTrajectory(job, &wp);
    CHECK(!transformed.ok && std::strcmp(transformed.reason.c_str(), "WORKPLANE_ENABLED") == 0,
          "draw rejects UCS so only the fixed base plane remains");
    if (g_fail == 0) PASS("fixed_draw_plane");
}

static void test_runtime_segment_spacing() {
    CHECK(Planner::segmentLengthFor(Planner::Shape::LINE) == DRAW_LINE_SEGMENT_MM,
          "line must use the reduced-stop/start spacing");
    CHECK(Planner::segmentLengthFor(Planner::Shape::CIRCLE) == DRAW_SEGMENT_MM,
          "circle must keep fine spacing");
    CHECK(Planner::segmentLengthFor(Planner::Shape::SQUARE) == DRAW_SEGMENT_MM,
          "square must keep fine spacing");
    if (g_fail == 0) PASS("runtime_segment_spacing");
}

static void test_workplane_target_uses_plane_not_legacy_z() {
    // Planner treats job.z as its legacy paper reference. With UCS enabled,
    // target draw points are w=0 on the calibrated plane, not w=job.z.
    WorkPlane wp;
    CHECK(wp.setThreePointCalibration({0, 0, 20}, {100, 0, 20}, {0, 100, 20}),
          "horizontal workplane calibration");
    Planner::Job job;
    job.shape = Planner::Shape::POINT;
    job.x1 = 120; job.y1 = 20; job.z = 435; // Base target must remain (120,20,20).
    const auto r = validateTrajectory(job, &wp);
    if (r.ok) PASS("workplane_target_uses_plane_not_legacy_z");
    else { CHECK(false, "workplane target must validate at w=0, not w=job.z"); }
}

int main() {
    test_point_reachable_pass();
    test_point_out_of_reach_reject();
    test_line_mid_out_of_reach_reject();
    test_line_all_reachable_pass();
    test_zero_length_line_reject();
    test_line_from_home_park_pose_pass();
    test_circle_outside();
    test_square_reachable_pass();
    test_square_bad_side_reject();
    test_runtime_segment_spacing();
    test_fixed_draw_plane();
    test_workplane_transform();
    test_workplane_target_uses_plane_not_legacy_z();
    if (g_fail==0) {
        std::printf("ALL PASSED (%d tests)\n", g_pass);
        return 0;
    }
    std::printf("%d FAILED, %d PASSED\n", g_fail, g_pass);
    return 1;
}
