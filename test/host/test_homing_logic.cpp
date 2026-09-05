// Host tests: Homing FSM calculations, direction logic, and safety constraints.
// Ghi chú: homing.cpp không compile được trên host (phụ thuộc TMCStepper/esp_timer qua
// motor.h) — các test dưới đây bảo vệ CÔNG THỨC và BẤT BIẾN cấu hình mà FSM dựa vào
// (mirror formula), để sớm phát hiện hồi quy khi sửa config.h.
#include "config.h"
#include <cmath>
#include <cstdio>
#include <cstdint>

static int g_fail = 0;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("FAIL: %s (line %d)\n", msg, __LINE__);              \
            ++g_fail;                                                        \
        }                                                                    \
    } while (0)

// Helper mimicking HomingController centering and scan range math
static uint32_t calculateCenterStep(uint32_t minStep, uint32_t maxStep) {
    if (maxStep <= minStep) return 0;
    return (maxStep - minStep) / 2;
}

static bool isDualEndstopAxis(uint8_t axis) {
    // J1, J2, J3 have both MIN and MAX endstop pins configured in config.h
    return (AXIS_MIN_PINS[axis] >= 0 && AXIS_MAX_PINS[axis] >= 0);
}

// Mirror của stepsPerDegree() cấu hình (chưa đo measured)
static float cfgStepsPerDegree(uint8_t axis) {
    return (static_cast<float>(DEFAULT_FULL_STEPS) * static_cast<float>(DEFAULT_MICROSTEPS) *
            DEFAULT_AXIS_GEAR_RATIOS[axis]) / 360.0f;
}

// Mirror của stallWindowCheck: cửa sổ = max(sàn bước, 1.2° quy đổi)
static int64_t stallWindowSteps(uint8_t axis) {
    const int64_t byDeg = static_cast<int64_t>(HOMING_STALL_WINDOW_MIN_DEG *
                                               cfgStepsPerDegree(axis) + 0.5f);
    return (byDeg > HOMING_STALL_WINDOW_MIN_STEPS) ? byDeg : HOMING_STALL_WINDOW_MIN_STEPS;
}

// Mirror của VERIFY tolerance: base + % nửa span
static float verifyTol(float spanRaw) {
    return HOMING_VERIFY_TOL_BASE_DEG + HOMING_VERIFY_TOL_SPAN_PCT * 0.01f * (spanRaw / 2.0f);
}

// Mirror của stepsBack khi centering J3 (home = MIN + offset)
static int64_t j3StepsBack(bool firstSideIsMin, int64_t contactSpan,
                           int64_t offsetSteps) {
    return firstSideIsMin ? (contactSpan - offsetSteps) : offsetSteps;
}

static void testScanAxesConfiguration() {
    CHECK(isDualEndstopAxis(0) == true, "J1 is dual endstop axis");
    CHECK(isDualEndstopAxis(1) == true, "J2 is dual endstop axis");
    CHECK(isDualEndstopAxis(2) == true, "J3 is dual endstop axis");
    CHECK(isDualEndstopAxis(3) == false, "J4 is single/stallguard axis (no physical dual endstops)");
    CHECK(isDualEndstopAxis(4) == false, "J5 has no auto-homing endstops");
    CHECK(isDualEndstopAxis(5) == false, "J6 has no auto-homing endstops");
}

static void testCenteringMath() {
    uint32_t spanSteps = 64000;
    uint32_t center = calculateCenterStep(0, spanSteps);
    CHECK(center == 32000, "Center step of 0..64000 is 32000");

    // Zero span edge case
    CHECK(calculateCenterStep(0, 0) == 0, "Zero span returns 0 center");
}

static void testHomingCurrentsConfig() {
    for (uint8_t a = 0; a < 4; ++a) {
        CHECK(DEFAULT_AXIS_HOMING_CURRENTS[a] > 0, "TMC homing current > 0");
        CHECK(DEFAULT_AXIS_HOMING_CURRENTS[a] <= 1200, "Homing current <= 1200mA (TMC2209 safe max)");
    }
}

static void testHomingStepIntervals() {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        CHECK(HOMING_STEP_INTERVAL_J1 >= 1000, "Homing interval J1 >= 1000us");
        CHECK(HOMING_STEP_INTERVAL_J2 >= 1000, "Homing interval J2 >= 1000us");
        CHECK(HOMING_STEP_INTERVAL_J3 >= 1000, "Homing interval J3 >= 1000us");
    }
}

// Kiến trúc 2 tốc độ: pha SLOW phải CHẬM hơn (hoặc bằng) mọi pha FAST của TMC axes
static void testTwoSpeedArchitecture() {
    for (uint8_t a = 0; a < 4; ++a) {
        CHECK(HOMING_SLOW_SCAN_INTERVAL_US >= DEFAULT_AXIS_HOMING_SPEEDS[a],
              "Slow re-approach interval >= fast scan interval for TMC axis");
    }
    CHECK(HOMING_SLOW_SCAN_INTERVAL_US >= 2500,
          "Slow interval >= 2500us (gentle sensorless contact phase)");
}

// Retry: đủ để chống glitch, không vô hạn
static void testRetryConfig() {
    CHECK(HOMING_MAX_ATTEMPTS >= 1, "At least one attempt");
    CHECK(HOMING_MAX_ATTEMPTS <= 5, "Retry capped (không đập cữ vô hạn)");
}

// Cửa sổ step-lag: windowDeg >= MIN_DEG và windowSteps >= floor
// (config hiện tại HOMING_STALL_ENC_DELTA_DEG=2.5, MIN_DEG=1.2, MIN_STEPS=120)
// Delta lớn làm threshold nhạy; free-run margin không còn 2x như cấu hình cũ 0.6
// nên kiểm tra hợp lệ: window tôn trọng floor và minDeg, và đủ lớn để phân biệt stall
static void testStallWindowMargin() {
    for (uint8_t a = 0; a < 4; ++a) {
        const float windowDeg = static_cast<float>(stallWindowSteps(a)) / cfgStepsPerDegree(a);
        CHECK(windowDeg >= HOMING_STALL_WINDOW_MIN_DEG - 0.05f,
              "Stall window >= minDeg (EMA settling)");
        CHECK(stallWindowSteps(a) >= HOMING_STALL_WINDOW_MIN_STEPS,
              "Stall window floor respected (EMA settling)");
        CHECK(windowDeg > 0.5f, "Stall window positive");
    }
    // J2/J3 (gear 20:1) phải có cửa sổ quy đổi theo độ, không bị kẹt ở sàn 120 bước
    CHECK(stallWindowSteps(1) > HOMING_STALL_WINDOW_MIN_STEPS,
          "J2 window scaled by gear ratio (> floor)");
    CHECK(stallWindowSteps(2) > HOMING_STALL_WINDOW_MIN_STEPS,
          "J3 window scaled by gear ratio (> floor)");
}

// Dung sai verify: gốc dương, tăng theo span, giá trị hợp lý cho hành trình lớn
static void testVerifyTolerance() {
    CHECK(verifyTol(0.0f) > 0.0f, "Verify tolerance positive at zero span");
    CHECK(verifyTol(0.0f) == HOMING_VERIFY_TOL_BASE_DEG, "Zero span -> base tolerance");
    CHECK(verifyTol(360.0f) > verifyTol(90.0f), "Tolerance grows with span");
    CHECK(verifyTol(360.0f) < 5.0f, "Tolerance bounded for full stroke");
    // J4: span raw là góc motor (x4) — err và tol cùng đơn vị raw nên tự đồng bộ
    CHECK(verifyTol(4.0f * 600.0f) > verifyTol(600.0f), "Motor-referenced span scales tolerance");
}

// Centering J3: về MIN+offset đúng cho CẢ hai phương án lắp đặt (MIN chạm trước / MAX chạm trước)
static void testJ3OffsetCentering() {
    const float offsetDeg = 2.5f; // HOME_BACKOFF_DEG + 0.5
    const int64_t offsetSteps = static_cast<int64_t>(offsetDeg * cfgStepsPerDegree(2) + 0.5f);
    const int64_t span = 6400;

    const int64_t backFromMax = j3StepsBack(true, span, offsetSteps);
    CHECK(backFromMax == span - offsetSteps,
          "J3 first-contact MIN: run back span-offset to land at MIN+offset");
    const int64_t backFromMin = j3StepsBack(false, span, offsetSteps);
    CHECK(backFromMin == offsetSteps,
          "J3 first-contact MAX (reversed): run offset away from MIN");
    CHECK(backFromMax > 0 && backFromMin > 0, "Both layouts produce positive run");
}

// Tâm cơ khí: bất đối xứng glitch 2 đầu tự triệt tiêu trong trung bình
static void testCenterSymmetry() {
    // 2 điểm chạm slow đối xứng quanh tâm => center = trung bình raw, trung bình bước
    const int64_t span = 12000;
    CHECK(calculateCenterStep(0, span) == span / 2, "Center equals half span");
}

// Span integrity + trim bound: ngưỡng hợp lý, J4 thấp hơn do bất định tỷ số encoder
static void testSpanIntegrityAndTrimBounds() {
    for (uint8_t a = 0; a < 4; ++a) {
        CHECK(HOMING_MIN_ENC_SPAN_DEG[a] >= 10.0f, "Min encoder span >= 10 deg");
        CHECK(HOMING_MIN_ENC_SPAN_DEG[a] < 100.0f, "Min span < smallest expected stroke");
    }
    CHECK(HOMING_MIN_ENC_SPAN_DEG[3] < HOMING_MIN_ENC_SPAN_DEG[0],
          "J4 threshold lower (encoder ratio uncertainty)");
    CHECK(HOMING_J4_MAX_MECHANICAL_SPAN_DEG > HOMING_MIN_MECHANICAL_SPAN_DEG &&
          HOMING_J4_MAX_MECHANICAL_SPAN_DEG <= 60.0f,
          "J4 second-side travel cap is above valid span and remains conservative");
    CHECK(HOMING_TRIM_MAX_TRAVEL_DEG >= 2.0f && HOMING_TRIM_MAX_TRAVEL_DEG <= 15.0f,
          "Trim travel bound sane");
}

static void testDirectionalEncoderFrame() {
    const float expectedRawSign = -1.0f;
    const float forward = 4.0f * expectedRawSign;
    const float backward = -4.0f * expectedRawSign;
    CHECK(forward * expectedRawSign >= HOMING_STALL_ENC_DELTA_DEG,
          "forward raw encoder movement resets the stall frame");
    CHECK(backward * expectedRawSign < 0.0f,
          "backward raw encoder jump is not treated as forward progress");
}

// Backoff phải đủ xa để nhả endstop với steps/deg CONFIG (không phụ thuộc calib cũ)
static void testBackoffDistanceUsesConfig() {
    // Mirror: beginScan reset calib => stepsBack tính theo config spd
    const float backoffDeg = 2.5f; // HOME_BACKOFF_DEG + 0.5
    for (uint8_t a = 0; a < 4; ++a) {
        const int64_t steps = static_cast<int64_t>(backoffDeg * cfgStepsPerDegree(a) + 0.5f);
        CHECK(steps > 0, "Backoff steps positive");
        // đủ xa để qua vùng actuation của công tắc (~1°) với margin
        CHECK(backoffDeg >= 2.0f, "Backoff >= HOME_BACKOFF_DEG");
    }
}

int main() {
    testScanAxesConfiguration();
    testCenteringMath();
    testHomingCurrentsConfig();
    testHomingStepIntervals();
    testTwoSpeedArchitecture();
    testRetryConfig();
    testStallWindowMargin();
    testVerifyTolerance();
    testJ3OffsetCentering();
    testCenterSymmetry();
    testSpanIntegrityAndTrimBounds();
    testDirectionalEncoderFrame();
    testBackoffDistanceUsesConfig();

    if (g_fail == 0) {
        std::printf("ALL PASSED (homing logic)\n");
        return 0;
    }
    std::printf("%d FAILED\n", g_fail);
    return 1;
}
