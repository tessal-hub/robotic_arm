#pragma once
#include <cstdint>
#include <atomic>
#include <array>
#include <functional>
#include "config.h"

#ifdef ARDUINO
#include "endstop.h"
#else
// Host build: define EndstopWhich locally to avoid pulling Arduino heavy headers
enum class EndstopWhich : uint8_t { MIN = 0, MAX = 1 };
#endif

enum class SafetyState { NORMAL = 0, E_STOP = 1, FAULT = 2, HOMING = 3 };

class Endstops;
class JointModel;

class SafetyManager {
public:
#ifdef ARDUINO
  // Firmware constructor with real types
  SafetyManager(Endstops* es, JointModel* jm);
#endif

  // Generic constructor for host mocks (and also works for firmware via template)
  template<typename E, typename J>
  SafetyManager(E* es, J* jm) {
    // Capture isPressed
    isPressed_ = [es](uint8_t axis, EndstopWhich w) -> bool {
      return es->isPressed(axis, w);
    };
    // anyPressed derived via isPressed_ loop (works even if mock lacks anyPressed)
    anyPressed_ = [this]() -> bool {
      for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        for (auto w : {EndstopWhich::MIN, EndstopWhich::MAX}) {
          if (isPressed_(a, w)) return true;
        }
      }
      return false;
    };
    clearLatches_ = [es]() { es->clearAllLatches(); };
    hasDrift_ = [jm]() -> bool {
      if (!jm) return false;
      return jm->hasAnyDriftFault();
    };
    clearDrift_ = [jm]() {
      if (jm) jm->clearAllDriftFaults();
    };
    state_.store(SafetyState::NORMAL, std::memory_order_release);
    homingActive_ = false;
    for (auto &row : pending_) row.fill(false);
    for (auto &row : pendingTime_) row.fill(0);
    for (auto &row : latched_) row.fill(false);
  }

  void isrNotify(uint8_t axis, EndstopWhich which, int64_t isrTimeUs);
  void pollEndstops(uint64_t nowUs); // test-injectable
  void pollEndstops(); // on Arduino calls esp_timer_get_time()
  void assertHoming(bool active);
  void assertEStop(const char* reason);
  bool tryClearFault();
  bool isMotionAllowed() const;
  bool isEStop() const;
  SafetyState state() const;
  bool anyLatched() const;

private:
  std::function<bool(uint8_t, EndstopWhich)> isPressed_;
  std::function<bool()> anyPressed_;
  std::function<void()> clearLatches_;
  std::function<bool()> hasDrift_;
  std::function<void()> clearDrift_;
  std::atomic<SafetyState> state_{SafetyState::NORMAL};
  bool homingActive_{false};
  std::array<std::array<bool,2>, NUM_MOTORS> pending_{};
  std::array<std::array<int64_t,2>, NUM_MOTORS> pendingTime_{};
  std::array<std::array<bool,2>, NUM_MOTORS> latched_{};
  static constexpr int64_t DEBOUNCE_US = 50000; // 50ms
};
