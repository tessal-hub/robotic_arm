#pragma once
#include "Arduino.h"
#include "endstop.h"
#include "joint_calibration.h"
#include <algorithm>
#include <cstdlib>

inline uint32_t fakeNow = 0;
inline uint32_t millis() { return fakeNow; }
inline bool fakeHasEndstop[NUM_MOTORS][2]{};
inline bool fakeEndstopPressed[NUM_MOTORS][2]{};
inline bool fakeEndstopEnabled[NUM_MOTORS][2]{};

// Hardware boundary only. The test executes production homing.cpp unchanged.
class Motor {
public:
    int64_t steps = 0;
    uint32_t remaining = 0, speed = 2000;
    uint16_t sg = 200;
    bool running = false, cw = false, continuous = false;
    bool uartAvailable = true;
    int uartChecks = 0, runCalls = 0;
    bool testUART() { ++uartChecks; return uartAvailable; }
    bool isTmc() const { return true; }
    bool isRunning() const { return running; }
    int64_t getAbsoluteSteps() const { return steps; }
    void setAbsoluteSteps(int64_t value) { steps = value; }
    uint16_t getSGResult() const { return sg; }
    void setCurrent(uint16_t) {}
    void setChopperMode(bool) {}
    void setSpeed(uint32_t value) { speed = value; }
    void run(bool dir, uint32_t count) { ++runCalls; cw = dir; remaining = count; continuous = false; running = count > 0; }
    void runContinuous(bool dir) { cw = dir; continuous = true; running = true; }
    void stop() { running = false; }
};

class JointModel {
public:
    float raw = 0, sign = 1;
    bool healthy = true;
    int homes = 0, calibrations = 0;
    static float stepsPerDegree(uint8_t axis) { return jointcal::configuredStepsPerDegree(axis); }
    static int64_t degreesToSteps(uint8_t axis, float deg) { return std::lround(deg * stepsPerDegree(axis)); }
    static bool cwForDelta(uint8_t axis, float delta) { return AXIS_STEP_SIGN[axis] > 0 ? delta >= 0 : delta < 0; }
    float rawEncoder(uint8_t) const { return raw; }
    bool encOK(uint8_t) const { return healthy; }
    float angleFromSteps(uint8_t) const { return 0; }
    float encSignOf(uint8_t) const { return sign; }
    void resetHomingCalibration(uint8_t) {}
    void clearHome(uint8_t) {}
    bool resyncFromEncoder(uint8_t) { return true; }
    void applyHomingCalibration(uint8_t, float s, float) { sign = s; ++calibrations; }
    void setHomeHere(uint8_t) { ++homes; }
};
class SafetyManager { public: void assertHoming(bool) {} };

inline Endstops::Endstops() = default;
inline bool Endstops::hasPin(uint8_t axis, EndstopWhich w) const noexcept { return fakeHasEndstop[axis][static_cast<uint8_t>(w)]; }
inline bool Endstops::isPressed(uint8_t axis, EndstopWhich w) const noexcept { return fakeEndstopEnabled[axis][static_cast<uint8_t>(w)] && fakeEndstopPressed[axis][static_cast<uint8_t>(w)]; }
inline bool Endstops::isPhysicallyPressed(uint8_t axis, EndstopWhich w) const noexcept { return fakeEndstopPressed[axis][static_cast<uint8_t>(w)]; }
inline bool Endstops::isLatched(uint8_t, EndstopWhich) const noexcept { return false; }
inline bool Endstops::isrPending(uint8_t, EndstopWhich) const noexcept { return false; }
inline bool Endstops::isPinEnabled(uint8_t axis, EndstopWhich w) const noexcept { return fakeEndstopEnabled[axis][static_cast<uint8_t>(w)]; }
inline void Endstops::setPinEnabled(uint8_t axis, EndstopWhich w, bool enabled) noexcept { fakeEndstopEnabled[axis][static_cast<uint8_t>(w)] = enabled; }
inline void Endstops::clearLatch(uint8_t, EndstopWhich) noexcept {}
inline bool Endstops::consumeLatch(uint8_t, EndstopWhich) noexcept { return false; }
inline void Endstops::clearAllLatches() noexcept {}
