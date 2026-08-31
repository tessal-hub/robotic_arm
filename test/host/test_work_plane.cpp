// Host tests: WorkPlane 3-point calibration math.
#include "work_plane.h"
#include <cstdio>
#include <cmath>

static int g_fail = 0;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("FAIL: %s (line %d)\n", msg, __LINE__);              \
            ++g_fail;                                                        \
        }                                                                    \
    } while (0)

static bool near(float a, float b, float tol) { return std::fabs(a - b) <= tol; }

static void testIdentityWhenDisabled() {
    WorkPlane wp;
    const Point3D r = wp.toRobotXYZ(10.0f, 20.0f, 5.0f);
    CHECK(near(r.x, 10.0f, 1e-4f), "identity x");
    CHECK(near(r.y, 20.0f, 1e-4f), "identity y");
    CHECK(near(r.z, 5.0f, 1e-4f), "identity z");
}

static void testValidCalibration() {
    WorkPlane wp;
    const Point3D p1{0, 0, 100};
    const Point3D p2{100, 0, 100};
    const Point3D p3{0, 100, 100};
    CHECK(wp.setThreePointCalibration(p1, p2, p3), "calib OK");
    CHECK(wp.isEnabled(), "enabled after calib");
    const Point3D atOrigin = wp.toRobotXYZ(0, 0, 0);
    CHECK(near(atOrigin.x, 0.0f, 0.1f), "origin x");
    CHECK(near(atOrigin.y, 0.0f, 0.1f), "origin y");
    CHECK(near(atOrigin.z, 100.0f, 0.1f), "origin z");
    const Point3D atU = wp.toRobotXYZ(50, 0, 0);
    CHECK(near(atU.x, 50.0f, 0.1f), "u axis x");
}

static void testCollinearRejected() {
    WorkPlane wp;
    const Point3D p1{0, 0, 0};
    const Point3D p2{50, 0, 0};
    const Point3D p3{100, 0, 0};
    CHECK(!wp.setThreePointCalibration(p1, p2, p3), "collinear rejected");
    CHECK(!wp.isEnabled(), "not enabled after bad calib");
}

static void testTooCloseRejected() {
    WorkPlane wp;
    const Point3D p1{0, 0, 0};
    const Point3D p2{5, 0, 0};
    const Point3D p3{0, 30, 0};
    CHECK(!wp.setThreePointCalibration(p1, p2, p3), "too close rejected");
}

int main() {
    testIdentityWhenDisabled();
    testValidCalibration();
    testCollinearRejected();
    testTooCloseRejected();
    if (g_fail == 0) {
        std::printf("ALL PASSED (work plane)\n");
        return 0;
    }
    std::printf("%d FAILED\n", g_fail);
    return 1;
}
