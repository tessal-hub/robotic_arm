#pragma once
// Host mocks for Endstops / JointModel used by SafetyManager tests.
// Host-only header: do NOT include in firmware build (ARDUINO).
#include <cstdint>
#include "config.h"
#include "safety_manager.h"

// MockEndstops mimics Endstops API needed by SafetyManager:
// - setGpio(axis, which, high)  high=true => not pressed (HIGH), high=false => pressed (LOW)
// - isPressed(axis, which)       true if pressed (LOW)
// - anyPressed(), anyLatched(), clearAllLatches()
class MockEndstops {
public:
  MockEndstops() {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
      for (uint8_t w = 0; w < 2; ++w) {
        gpioHigh[a][w] = true; // HIGH = not pressed
        latched[a][w] = false;
      }
    }
  }
  void setGpio(uint8_t axis, EndstopWhich which, bool high) {
    if (axis >= NUM_MOTORS) return;
    uint8_t idx = (which == EndstopWhich::MIN) ? 0 : 1;
    gpioHigh[axis][idx] = high;
  }
  bool isPressed(uint8_t axis, EndstopWhich which) const {
    if (axis >= NUM_MOTORS) return false;
    uint8_t idx = (which == EndstopWhich::MIN) ? 0 : 1;
    // Active LOW: pressed when gpio is LOW (high==false)
    return !gpioHigh[axis][idx];
  }
  bool anyPressed() const {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
      for (uint8_t w = 0; w < 2; ++w) {
        if (!gpioHigh[a][w]) return true;
      }
    }
    return false;
  }
  bool isLatched(uint8_t axis, EndstopWhich which) const {
    if (axis >= NUM_MOTORS) return false;
    uint8_t idx = (which == EndstopWhich::MIN) ? 0 : 1;
    return latched[axis][idx];
  }
  bool anyLatched() const {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a)
      for (uint8_t w = 0; w < 2; ++w)
        if (latched[a][w]) return true;
    return false;
  }
  void clearAllLatches() {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a)
      for (uint8_t w = 0; w < 2; ++w) latched[a][w] = false;
  }
  // helper for internal test inspection (not used by SafetyManager)
  void setLatched(uint8_t axis, EndstopWhich which, bool v) {
    if (axis >= NUM_MOTORS) return;
    uint8_t idx = (which == EndstopWhich::MIN) ? 0 : 1;
    latched[axis][idx] = v;
  }
private:
  bool gpioHigh[NUM_MOTORS][2];
  bool latched[NUM_MOTORS][2];
};

class MockJointModel {
public:
  bool hasAnyDriftFault() const { return drift; }
  void clearAllDriftFaults() { drift = false; }
  void setDriftFault(bool v) { drift = v; }
  // alias used by some tests
  void setDrift(bool v) { drift = v; }
private:
  bool drift{false};
};

// Compatibility aliases expected by plan example
using MockEndstopsAlias = MockEndstops;
using MockJointModelAlias = MockJointModel;
