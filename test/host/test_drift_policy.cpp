#include "drift_policy.h"

#include <cstdio>

static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        std::printf("FAIL: %s (line %d)\n", msg, __LINE__); \
        ++g_fail; \
    } \
} while (0)

int main() {
    constexpr float threshold = 25.0f;
    constexpr uint8_t required = 3;

    CHECK(!drift::exceedsThreshold(threshold, threshold), "threshold itself is not runaway");
    CHECK(drift::exceedsThreshold(threshold + 0.01f, threshold), "error above threshold is runaway");

    uint8_t failures = 0;
    failures = drift::nextFailureCount(failures, true);
    CHECK(failures == 1 && !drift::shouldLatch(failures, required), "first runaway is only suspect");
    failures = drift::nextFailureCount(failures, true);
    CHECK(failures == 2 && !drift::shouldLatch(failures, required), "second runaway is only suspect");
    failures = drift::nextFailureCount(failures, true);
    CHECK(failures == 3 && drift::shouldLatch(failures, required), "third runaway latches fault");

    failures = drift::nextFailureCount(failures, false);
    CHECK(failures == 0, "normal reading clears suspect count");
    CHECK(drift::nextFailureCount(UINT8_MAX, true) == UINT8_MAX, "failure counter saturates");

    if (g_fail == 0) {
        std::printf("ALL PASSED (drift policy)\n");
        return 0;
    }
    std::printf("%d FAILED\n", g_fail);
    return 1;
}
