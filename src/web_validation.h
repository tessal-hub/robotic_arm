#ifndef WEB_VALIDATION_H
#define WEB_VALIDATION_H

#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <limits>

namespace webval {

inline bool parseFiniteFloat(const char* text, float& value) {
    if (text == nullptr || *text == '\0') return false;
    errno = 0;
    char* end = nullptr;
    const float parsed = strtof(text, &end);
    if (end == text || *end != '\0' || errno == ERANGE || !std::isfinite(parsed)) return false;
    value = parsed;
    return true;
}

inline bool parseInt(const char* text, int& value) {
    if (text == nullptr || *text == '\0') return false;
    errno = 0;
    char* end = nullptr;
    const long parsed = strtol(text, &end, 10);
    if (end == text || *end != '\0' || errno == ERANGE ||
        parsed < std::numeric_limits<int>::min() || parsed > std::numeric_limits<int>::max()) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

inline bool validFeed(float feedMmS) {
    return std::isfinite(feedMmS) && feedMmS > 1.0f && feedMmS < 200.0f;
}

inline bool parseBool01(const char* text, bool& value) {
    int parsed = 0;
    if (!parseInt(text, parsed) || (parsed != 0 && parsed != 1)) return false;
    value = parsed == 1;
    return true;
}

} // namespace webval

#endif // WEB_VALIDATION_H
