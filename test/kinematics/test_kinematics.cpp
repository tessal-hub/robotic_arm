// Host-side unit tests cho kinematics.
// Chay: tools/run_kin_tests.sh  (g++ truc tiep - PIO native bi loi "Nothing to build")
#include <cstdio>
#include <cmath>
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

// FK tai Home (tat ca encoder = 0):
//   THETA5_OFFSET = 0 -> th5_DH = 0 -> J5 nam ngang -> TCP theo +X tu wrist.
//   Wrist: (125, 0, 365) theo ARM_GEOMETRY.md muc 5.
//   TCP  : (125 + 75, 0, 365) = (200, 0, 365).
static void testFkHome() {
    const float home[6] = {0, 0, 0, 0, 0, 0};
    const kin::FkResult r = kin::forward(home);
    printf("FK home wrist=(%.3f, %.3f, %.3f) tcp=(%.3f, %.3f, %.3f)\n",
           r.wristCenter.x, r.wristCenter.y, r.wristCenter.z,
           r.tcp.x, r.tcp.y, r.tcp.z);
    CHECK(near(r.wristCenter.x, 125.0f, 1e-3f), "wrist x");
    CHECK(near(r.wristCenter.y,   0.0f, 1e-3f), "wrist y");
    CHECK(near(r.wristCenter.z, 365.0f, 1e-3f), "wrist z");
    CHECK(near(r.tcp.x, 200.0f, 1e-3f), "tcp x (J5=0, tool along +X)");
    CHECK(near(r.tcp.y,   0.0f, 1e-3f), "tcp y");
    CHECK(near(r.tcp.z, 365.0f, 1e-3f), "tcp z (J5 horizontal)");
}

// IK pen-down tai (100, 0, 50): wrist=(100,0,225), dist=132mm, trong vung voi.
// THETA5_OFFSET=0: e5 = -q23_deg, ket qua phai nam trong soft-limit [-90, 90].
static void testIkPenDownReachable() {
    float enc[6];
    CHECK(kin::ikPenDown({100.0f, 0.0f, 50.0f}, enc), "pen-down (100,0,50) reachable");
    CHECK(enc[4] >= -90.0f && enc[4] <= 90.0f, "J5 enc within soft limits");
    printf("IK (100,0,50): e=[%.1f %.1f %.1f %.1f %.1f %.1f]\n",
           enc[0],enc[1],enc[2],enc[3],enc[4],enc[5]);
}

// Roundtrip FK(IK(target)) = target voi sai so < 0.5mm.
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
                target.x = tx; target.y = ty; target.z = z;
                if (!kin::ikPenDown(target, enc)) continue;

                const kin::FkResult r = kin::forward(enc);
                const float perr = sqrtf((r.tcp.x - tx) * (r.tcp.x - tx) +
                                         (r.tcp.y - ty) * (r.tcp.y - ty) +
                                         (r.tcp.z -  z) * (r.tcp.z -  z));
                if (perr <= 0.5f) {
                    ++ok;
                } else if (++fail < 6) {
                    printf("RT FAIL t=(%.1f,%.1f,%.1f) e=[%.1f %.1f %.1f %.1f %.1f] err=%.2f\n",
                           tx, ty, z, enc[0],enc[1],enc[2],enc[3],enc[4], perr);
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
    CHECK(!kin::ikPenDown({  0.0f, 0.0f, 700.0f}, enc), "high target rejected");
    CHECK(!kin::ikPenDown({ 50.0f, 0.0f, -80.0f}, enc), "below-floor target rejected");
}

int main() {
    testFkHome();
    testIkPenDownReachable();
    testIkRoundTrip();
    testIkRejectsUnreachable();
    if (g_fail == 0) {
        printf("ALL KINEMATICS TESTS PASSED\n");
        return 0;
    }
    printf("%d CHECKS FAILED\n", g_fail);
    return 1;
}
