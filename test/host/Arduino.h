#pragma once
// Minimal Arduino stub for host-side unit tests (g++ on PC).
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <cstdint>
#include <cstddef>

using uint8_t  = std::uint8_t;
using uint16_t = std::uint16_t;
using uint32_t = std::uint32_t;
using uint64_t = std::uint64_t;
using int8_t   = std::int8_t;
using int16_t  = std::int16_t;
using int32_t  = std::int32_t;
using int64_t  = std::int64_t;
using BaseType_t = int32_t;
using UBaseType_t = uint32_t;
using TickType_t = uint32_t;

constexpr uint8_t LOW = 0;
constexpr uint8_t HIGH = 1;
constexpr float TWO_PI = 6.283185307179586476925286766559f;

class String {
public:
    String() = default;
    String(const char* s) : data_(s ? s : "") {}
    String& operator=(const char* s) {
        data_ = s ? s : "";
        return *this;
    }
    [[nodiscard]] const char* c_str() const { return data_.c_str(); }

private:
    std::string data_;
};

struct SerialStub {
    template<typename... Args>
    void printf(const char* fmt, Args... args) {
        std::printf(fmt, args...);
    }
};

inline SerialStub Serial;
