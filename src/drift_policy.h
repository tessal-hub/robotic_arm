#ifndef DRIFT_POLICY_H
#define DRIFT_POLICY_H

#include <cstdint>

namespace drift {

inline bool exceedsThreshold(float absoluteErrorDeg, float thresholdDeg) {
    return absoluteErrorDeg > thresholdDeg;
}

inline uint8_t nextFailureCount(uint8_t current, bool exceedsThreshold) {
    if (!exceedsThreshold) return 0;
    return current == UINT8_MAX ? UINT8_MAX : static_cast<uint8_t>(current + 1);
}

inline bool shouldLatch(uint8_t failures, uint8_t requiredConsecutiveFailures) {
    return failures >= requiredConsecutiveFailures;
}

} // namespace drift

#endif // DRIFT_POLICY_H
