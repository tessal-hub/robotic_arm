// Host-side unit tests cho kinematics.
// Chạy: tools/run_kin_tests.sh  (g++ trực tiếp — PIO test runner native bị lỗi
// "Nothing to build" trên Core 6.1.19, xem docs/IMPLEMENTATION_LOG.md)
#include <cstdio>
#include <cmath>
#include "differential_wrist.h"
#include "kinematics.h"

static int g_fail = 0;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("FAIL: %s (line %d)\n", msg, __LINE__);                   \
            ++g_fail;                                                        \
        }                                                                    \
    } while (0)

static bool near(float a, float b, float tol) { return fabsf(a - b) <= tol; }

static void testFkHome() {
    const float home[6] = {0, 0, 0, 0, 0, 0};
    const kin::FkResult r = kin::forward(home);
    printf("FK home wrist=(%.3f, %.3f, %.3f) tcp=(%.3f, %.3f, %.3f)\n",
           r.wristCenter.x, r.wristCenter.y, r.wristCenter.z,
           r.tcp.x, r.tcp.y, r.tcp.z);
    // docs/ARM_GEOMETRY.md mục 5: wrist center (J5) = (126, 0, 365), tcp = (177, 0, 365)
    CHECK(near(r.wristCenter.x, 126.0f, 1e-3f), "wrist x");
    CHECK(near(r.wristCenter.y, 0.0f, 1e-3f), "wrist y");
    CHECK(near(r.wristCenter.z, 365.0f, 1e-3f), "wrist z");
    CHECK(near(r.tcp.x, 177.0f, 1e-3f), "tcp x");
    CHECK(near(r.tcp.y, 0.0f, 1e-3f), "tcp y");
    CHECK(near(r.tcp.z, 365.0f, 1e-3f), "tcp z");
}

static void testIkRoundTrip() {
    int ok = 0, fail = 0;
    for (int ix = -12; ix <= 12; ++ix) {
        for (int iz = 5; iz <= 40; ++iz) {
            for (int rot = 0; rot < 8; ++rot) {
                const float x = static_cast<float>(ix * 10);
                const float y = 0.0f;
                const float z = static_cast<float>(iz * 5);
                const float a = rot * 45.0f * 0.017453292519943295f;
                const float tx = x * cosf(a) - y * sinf(a);
                const float ty = x * sinf(a) + y * cosf(a);

                float enc[6];
                kin::Pose target;
                target.x = tx; target.y = ty; target.z = static_cast<float>(z);
                if (!kin::ikPenDown(target, enc)) continue;

                const kin::FkResult r = kin::forward(enc);
                const float perr = sqrtf((r.tcp.x - tx) * (r.tcp.x - tx) +
                                         (r.tcp.y - ty) * (r.tcp.y - ty) +
                                         (r.tcp.z - z) * (r.tcp.z - z));
                if (perr <= 0.5f) {
                    ++ok;
                } else if (++fail < 6) {
                    printf("RT FAIL t=(%.1f,%.1f,%.1f) e=[%.1f %.1f %.1f %.1f %.1f %.1f] err=%.2f\n",
                           tx, ty, z, enc[0], enc[1], enc[2], enc[3], enc[4], enc[5], perr);
                }
            }
        }
    }
    printf("IK roundtrip: ok=%d fail=%d\n", ok, fail);
    CHECK(fail == 0, "roundtrip failures");
    CHECK(ok > 500, "enough reachable samples solved");
}

static void testIkRejectsUnreachable() {
    float enc[6];
    CHECK(!kin::ikPenDown({900.0f, 0.0f, 100.0f}, enc), "far target rejected");
    CHECK(!kin::ikPenDown({0.0f, 0.0f, 700.0f}, enc), "high target rejected");
    CHECK(!kin::ikPenDown({50.0f, 0.0f, -80.0f}, enc), "below-floor target rejected");
}

static void testDifferentialWrist() {
    DifferentialWrist dw(1.0f);

    // 1. Pure Tilt (M_L = 30°, M_R = 30°) => Tilt = 30°, Roll = 0°
    DifferentialWrist::JointState j1 = dw.forward(30.0f, 30.0f);
    CHECK(near(j1.tiltDeg, 30.0f, 1e-4f), "diff pure tilt");
    CHECK(near(j1.rollDeg, 0.0f, 1e-4f), "diff pure tilt zero roll");

    // 2. Pure Roll (M_L = 45°, M_R = -45°) => Tilt = 0°, Roll = 45°
    DifferentialWrist::JointState j2 = dw.forward(45.0f, -45.0f);
    CHECK(near(j2.tiltDeg, 0.0f, 1e-4f), "diff pure roll zero tilt");
    CHECK(near(j2.rollDeg, 45.0f, 1e-4f), "diff pure roll");

    // 3. Inverse Kinematics (Tilt = 15°, Roll = -20°) => M_L = -5°, M_R = 35°
    DifferentialWrist::ActuatorState a3 = dw.inverse(15.0f, -20.0f);
    CHECK(near(a3.leftDeg, -5.0f, 1e-4f), "diff inverse left");
    CHECK(near(a3.rightDeg, 35.0f, 1e-4f), "diff inverse right");

    // 4. Roundtrip consistency across sweep
    for (int t = -60; t <= 60; t += 15) {
        for (int r = -180; r <= 180; r += 30) {
            const float tilt = static_cast<float>(t);
            const float roll = static_cast<float>(r);
            const DifferentialWrist::ActuatorState act = dw.inverse(tilt, roll);
            const DifferentialWrist::JointState rec = dw.forward(act.leftDeg, act.rightDeg);
            CHECK(near(rec.tiltDeg, tilt, 1e-4f), "diff roundtrip tilt");
            CHECK(near(rec.rollDeg, roll, 1e-4f), "diff roundtrip roll");
        }
    }
    printf("Differential Wrist: all kinematic transforms and roundtrip tests PASSED\n");
}

int main() {
    testFkHome();
    testIkRoundTrip();
    testIkRejectsUnreachable();
    testDifferentialWrist();
    if (g_fail == 0) {
        printf("ALL KINEMATICS & DIFFERENTIAL WRIST TESTS PASSED\n");
        return 0;
    }
    printf("%d CHECKS FAILED\n", g_fail);
    return 1;
}
