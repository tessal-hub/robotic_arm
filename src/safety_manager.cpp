#include "safety_manager.h"

#ifdef ARDUINO
#include "endstop.h"
#include "joint_model.h"
#include <esp_timer.h>
#include <Arduino.h>

// Firmware explicit constructor
SafetyManager::SafetyManager(Endstops* es, JointModel* jm) {
  isPressed_ = [es](uint8_t axis, EndstopWhich w) -> bool {
    return es->isPressed(axis, w);
  };
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

void SafetyManager::pollEndstops() {
  pollEndstops(static_cast<uint64_t>(esp_timer_get_time()));
}

#else
// Host build: no esp_timer, provide dummy no-arg poll
void SafetyManager::pollEndstops() {
  pollEndstops(0);
}
#endif

void SafetyManager::isrNotify(uint8_t axis, EndstopWhich which, int64_t isrTimeUs) {
  if (axis >= NUM_MOTORS) return;
  uint8_t idx = (which == EndstopWhich::MIN) ? 0 : 1;
  pending_[axis][idx].store(true, std::memory_order_relaxed);
  pendingTime_[axis][idx].store(isrTimeUs, std::memory_order_relaxed);
}

void SafetyManager::pollEndstops(uint64_t nowUs) {
  if (manualReleaseActive_) {
    for (auto &row : pending_) for (auto &v : row) v.store(false, std::memory_order_relaxed);
    for (auto &row : latched_) for (auto &v : row) v.store(false, std::memory_order_relaxed);
    return;
  }
  for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
    for (uint8_t wi = 0; wi < 2; ++wi) {
      if (!pending_[a][wi].load(std::memory_order_relaxed)) continue;
      int64_t pt = pendingTime_[a][wi].load(std::memory_order_relaxed);
      int64_t delta = static_cast<int64_t>(nowUs) - pt;
      if (delta < DEBOUNCE_US) continue; // not yet debounced, keep pending
      EndstopWhich w = (wi == 0) ? EndstopWhich::MIN : EndstopWhich::MAX;
      bool pressed = false;
      if (isPressed_) pressed = isPressed_(a, w);
      if (pressed) {
        latched_[a][wi].store(true, std::memory_order_relaxed);
        if (!homingActive_) {
          SafetyState cur = state_.load(std::memory_order_acquire);
          if (cur != SafetyState::E_STOP) {
            state_.store(SafetyState::E_STOP, std::memory_order_release);
          }
        } else {
          // homing active: keep HOMING state (ensure it is HOMING if was NORMAL)
          SafetyState cur = state_.load(std::memory_order_acquire);
          if (cur == SafetyState::NORMAL) {
            state_.store(SafetyState::HOMING, std::memory_order_release);
          }
        }
      } else {
        // GPIO HIGH at poll time -> glitch, no latch
      }
      pending_[a][wi].store(false, std::memory_order_relaxed);
    }
  }
  // Drift -> FAULT mapping (Spec §3.3 single owner): if drift latched and we are
  // in NORMAL/HOMING, promote to FAULT. Both E_STOP and FAULT block motion
  // (isMotionAllowed false). isEStop() stays false for FAULT; use isFault().
  // FAULT covers drift/power, E_STOP covers endstop latch.
  if (hasDrift_ && hasDrift_()) {
    SafetyState cur = state_.load(std::memory_order_acquire);
    if (cur == SafetyState::NORMAL || cur == SafetyState::HOMING) {
      state_.store(SafetyState::FAULT, std::memory_order_release);
    }
  }
}

void SafetyManager::assertHoming(bool active) {
  homingActive_ = active;
  if (active) {
    SafetyState cur = state_.load(std::memory_order_acquire);
    if (cur == SafetyState::NORMAL) {
      state_.store(SafetyState::HOMING, std::memory_order_release);
    }
  } else {
    SafetyState cur = state_.load(std::memory_order_acquire);
    if (cur == SafetyState::HOMING) {
      state_.store(SafetyState::NORMAL, std::memory_order_release);
    }
  }
}

void SafetyManager::assertManualRelease(bool active) noexcept {
  manualReleaseActive_ = active;
  if (!active) return;
  for (auto &row : pending_) for (auto &v : row) v.store(false, std::memory_order_relaxed);
  for (auto &row : latched_) for (auto &v : row) v.store(false, std::memory_order_relaxed);
  if (state_.load(std::memory_order_acquire) == SafetyState::E_STOP) {
    state_.store(SafetyState::NORMAL, std::memory_order_release);
  }
}

void SafetyManager::assertEStop(const char* reason) {
  (void)reason;
  state_.store(SafetyState::E_STOP, std::memory_order_release);
}

void SafetyManager::notifyFault(const char* reason) {
  (void)reason;
  SafetyState cur = state_.load(std::memory_order_acquire);
  if (cur != SafetyState::FAULT && cur != SafetyState::E_STOP) {
    state_.store(SafetyState::FAULT, std::memory_order_release);
  } else if (cur == SafetyState::E_STOP) {
    // Keep E_STOP if already latched; drift will be visible via hasDrift_ but motion already blocked.
    // Optionally promote to FAULT if needed; keep E_STOP precedence per spec.
  }
}

bool SafetyManager::tryClearFault() {
  bool pressed = false;
  if (anyPressed_) pressed = anyPressed_();
  // A drift latch is precisely what CLEAR_FAULT acknowledges. Rejecting it here
  // made the recovery path unreachable: clearDrift_() below could never run.
  // A physically pressed endstop remains a hard gate for restart.
  if (pressed) return false;
  for (auto &row : latched_) for (auto &v : row) v.store(false, std::memory_order_relaxed);
  for (auto &row : pending_) for (auto &v : row) v.store(false, std::memory_order_relaxed);
  // pendingTime not needed clear but for completeness
  for (auto &row : pendingTime_) for (auto &v : row) v.store(0, std::memory_order_relaxed);
  if (clearLatches_) clearLatches_();
  if (clearDrift_) clearDrift_();
  state_.store(SafetyState::NORMAL, std::memory_order_release);
  homingActive_ = false;
  return true;
}

bool SafetyManager::isMotionAllowed() const {
  SafetyState s = state_.load(std::memory_order_acquire);
  return s == SafetyState::NORMAL || s == SafetyState::HOMING;
}

bool SafetyManager::isEStop() const {
  return state_.load(std::memory_order_acquire) == SafetyState::E_STOP;
}

bool SafetyManager::isFault() const {
  return state_.load(std::memory_order_acquire) == SafetyState::FAULT;
}

SafetyState SafetyManager::state() const {
  return state_.load(std::memory_order_acquire);
}

bool SafetyManager::anyLatched() const {
  for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
    for (uint8_t wi = 0; wi < 2; ++wi) {
      if (latched_[a][wi].load(std::memory_order_relaxed)) return true;
    }
  }
  return false;
}

bool SafetyManager::isLatched(uint8_t axis, EndstopWhich which) const {
  if (axis >= NUM_MOTORS) return false;
  uint8_t idx = (which == EndstopWhich::MIN) ? 0 : 1;
  return latched_[axis][idx].load(std::memory_order_relaxed);
}

void SafetyManager::clearLatched(uint8_t axis, EndstopWhich which) noexcept {
  if (axis >= NUM_MOTORS) return;
  uint8_t idx = (which == EndstopWhich::MIN) ? 0 : 1;
  latched_[axis][idx].store(false, std::memory_order_relaxed);
  // Also clear pending for this channel to avoid stale ISR pending after explicit clear
  pending_[axis][idx].store(false, std::memory_order_relaxed);
  pendingTime_[axis][idx].store(0, std::memory_order_relaxed);
}

bool SafetyManager::consumeLatched(uint8_t axis, EndstopWhich which) noexcept {
  if (axis >= NUM_MOTORS) return false;
  uint8_t idx = (which == EndstopWhich::MIN) ? 0 : 1;
  bool was = latched_[axis][idx].exchange(false, std::memory_order_acq_rel);
  // Clear pending as well — consume means handled, no re-latch from old ISR
  pending_[axis][idx].store(false, std::memory_order_relaxed);
  pendingTime_[axis][idx].store(0, std::memory_order_relaxed);
  return was;
}

void SafetyManager::clearPending(uint8_t axis, EndstopWhich which) noexcept {
  if (axis >= NUM_MOTORS) return;
  uint8_t idx = (which == EndstopWhich::MIN) ? 0 : 1;
  pending_[axis][idx].store(false, std::memory_order_relaxed);
  pendingTime_[axis][idx].store(0, std::memory_order_relaxed);
}

void SafetyManager::forceClear() noexcept {
  for (auto &row : pending_) for (auto &v : row) v.store(false, std::memory_order_relaxed);
  for (auto &row : pendingTime_) for (auto &v : row) v.store(0, std::memory_order_relaxed);
  for (auto &row : latched_) for (auto &v : row) v.store(false, std::memory_order_relaxed);
  // Do NOT call clearLatches_ — Endstops::clearAllLatches() already cleared local latched/pending
  // and then calls forceClear() to clear manager side. Calling back would recurse infinitely.
  if (clearDrift_) clearDrift_();
  state_.store(SafetyState::NORMAL, std::memory_order_release);
  homingActive_ = false;
}
