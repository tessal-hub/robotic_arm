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

// SafetyState single owner (Spec §3.3): NORMAL=idle/jog/cart, HOMING=homing active,
// E_STOP=endstop latch (ISR debounce, 50ms+poll), FAULT=drift/power failure.
// Both E_STOP and FAULT block motion (isMotionAllowed() false). CLEAR_FAULT clears both.
// isEStop() is true only for E_STOP (fast ISR stop); FAULT is via isFault()/drift.
enum class SafetyState { NORMAL = 0, E_STOP = 1, FAULT = 2, HOMING = 3 };

class Endstops;
class JointModel;

class SafetyManager {
public:
#ifdef ARDUINO
  // Firmware constructor with real types
  SafetyManager(Endstops* es, JointModel* jm);
#else
  // Generic constructor for host mocks (host only — avoids overload ambiguity on firmware)
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
    for (auto &row : pending_) for (auto &v : row) v.store(false, std::memory_order_relaxed);
    for (auto &row : pendingTime_) for (auto &v : row) v.store(0, std::memory_order_relaxed);
    for (auto &row : latched_) for (auto &v : row) v.store(false, std::memory_order_relaxed);
  }
#endif

  void isrNotify(uint8_t axis, EndstopWhich which, int64_t isrTimeUs);
  void pollEndstops(uint64_t nowUs); // test-injectable, also checks drift→FAULT
  void pollEndstops(); // on Arduino calls esp_timer_get_time()
  void assertHoming(bool active);
  // Release là thao tác tay khi driver đã tắt mô-men: bỏ qua endstop ISR/latch cho tới khi resume.
  void assertManualRelease(bool active) noexcept;
  void assertEStop(const char* reason);
  void notifyFault(const char* reason); // sets FAULT (drift/power) if not already FAULT/E_STOP
  bool tryClearFault();
  bool isMotionAllowed() const;
  bool isEStop() const; // true only for E_STOP, not FAULT
  bool isFault() const; // true for FAULT (drift)
  SafetyState state() const;
  bool anyLatched() const;
  bool isLatched(uint8_t axis, EndstopWhich which) const;
  void clearLatched(uint8_t axis, EndstopWhich which) noexcept;
  bool consumeLatched(uint8_t axis, EndstopWhich which) noexcept;
  void clearPending(uint8_t axis, EndstopWhich which) noexcept;
  void forceClear() noexcept;

private:
  std::function<bool(uint8_t, EndstopWhich)> isPressed_;
  std::function<bool()> anyPressed_;
  std::function<void()> clearLatches_;
  std::function<bool()> hasDrift_;
  std::function<void()> clearDrift_;
  std::atomic<SafetyState> state_{SafetyState::NORMAL};
  bool homingActive_{false};
  bool manualReleaseActive_{false};
  std::array<std::array<std::atomic<bool>,2>, NUM_MOTORS> pending_{};
  std::array<std::array<std::atomic<int64_t>,2>, NUM_MOTORS> pendingTime_{};
  std::array<std::array<std::atomic<bool>,2>, NUM_MOTORS> latched_{};
  static constexpr int64_t DEBOUNCE_US = 50000; // 50ms
};
