# Task 3 Report — Motor + Arm integration — remove globals

**Status:** DONE

**Commit:** `HEAD` — `refactor(safety): remove g_* globals, inject SafetyManager into Motor/Arm`

- `src/motor.h` — Removed `extern std::atomic<bool> g_emergencyStop/g_homingActive` (lines 12-14). Added `class SafetyManager;` forward decl. Added `void setSafetyManager(SafetyManager* s) noexcept { safety_=s; }` public and `SafetyManager* safety_{nullptr};` private member (with `uartMutex`).
- `src/motor.cpp` — Deleted global definitions `std::atomic<bool> g_emergencyStop{false}; g_homingActive{false};` (lines 5-6). Added `#ifdef ARDUINO #include "safety_manager.h" #endif`. In `onStepTimer` replaced first-line `g_emergencyStop` check with `if (safety_ && safety_->isEStop()) { running=false; gpio_set_level 0; return; }` (<20us fail-fast, first check before any step logic). Second late check replaced with `bool estop = (safety_ && safety_->isEStop()); if (!running || estop)`.
- `src/arm.h` — Added `#include "safety_manager.h"` (complete type for `unique_ptr`), changed `~ArmController() = default` to `~ArmController();` (defined in cpp with complete type), added `std::unique_ptr<SafetyManager> safety_{nullptr};` and `SafetyManager* safety() noexcept` accessor. Added `#include <memory>`.
- `src/arm.cpp` — Added `#include "safety_manager.h"`, defined `~ArmController() = default;` in TU with complete type. In `begin()` creates `safety_ = std::make_unique<SafetyManager>(es, jm)` and injects `es->setSafetyManager(safety_.get()); for each motor setSafetyManager(safety_.get()); hc->setSafetyManager(safety_.get());` In `taskLoop()` first line after `vTaskDelayUntil`/`esp_task_wdt_reset` calls `if (safety_) safety_->pollEndstops();` then replaces `g_emergencyStop.load`/`es->anyLatched` with `safety_->isEStop()`/`safety_->anyLatched()` and backup `g_emergencyStop.store(true)` with `safety_->assertEStop(...)`. `motionAllowed()` now checks `if (safety_ && !safety_->isMotionAllowed()) return false;` `execute(CLEAR_FAULT)` delegates to `if (safety_) ok = safety_->tryClearFault(); else fallback; if(ok) IDLE else rejected`.
- `src/homing.h` — Added `class SafetyManager;` forward, `void setSafetyManager(SafetyManager* s) noexcept` and `SafetyManager* safety_{nullptr};` member.
- `src/homing.cpp` — Added `#include "safety_manager.h"`, removed anonymous `setHomingActive(bool)` helper and its `g_homingActive.store`. All 5 call sites (`startAll`, `startAxis`, `finishJoint` 2 places, `cancel`) replaced with `if (safety_) safety_->assertHoming(true/false);` No remaining `g_homingActive`.
- `src/endstop.cpp/h` — No additional change (Task 2 already removed `g_*` from endstop, now verifies still clean).
- `src/main.cpp` — No change required: `ArmController::begin` owns injection, wiring order `g_endstops.begin` before `g_arm.begin` still valid because `setSafetyManager` propagates to IsrCtx after attach.

**Test Summary:**

- Command: `Get-ChildItem -Recurse -Path "src" -Include "*.cpp","*.h" | Select-String -Pattern "g_emergencyStop|g_homingActive"` 
- Output: `(no output)` — `globals removed PASS`

- Command: `python -m platformio run`
- Output (tail):
```
RAM:   [==        ]  15.2% (used 49732 bytes from 327680 bytes)
Flash: [===       ]  27.0% (used 901085 bytes from 3342336 bytes)
========================= [SUCCESS] Took 50.96 seconds =========================
```
Exit 0. No warnings.

- Command: `g++ -std=c++17 -I src -I test/host test/host/test_safety_manager.cpp src/safety_manager.cpp -o C:/tmp/test_safety.exe && C:/tmp/test_safety.exe`
- Output:
```
PASS: pending_high_no_latch
PASS: low_before_50ms_no_latch
PASS: low_after_50ms_latches_estop
PASS: homing_no_estop
PASS: tryClear_reject_when_pressed
PASS: tryClear_reject_when_drift
PASS: tryClear_success_clears
PASS: isMotionAllowed_matrix
PASS: anyLatched_and_assertEStop
PASS: debounce_exact_boundary_and_glitch_recovery
ALL PASSED (10 tests)
```

- Command: `g++ -std=c++17 -I src -I test/host test/host/test_isr_latency.cpp src/safety_manager.cpp src/endstop.cpp -o C:/tmp/test_isr.exe && C:/tmp/test_isr.exe`
- Output:
```
PASS: isr_no_delay
  avg ISR latency: 57 ns (0 us) <5us OK
PASS: isr_delegates_to_safety_manager
PASS: isr_homing_no_estop
ALL PASSED (3 tests)
```

- Command: `g++ -std=gnu++17 -Wall -Wextra -I test/host -I src src/kinematics.cpp src/differential_wrist.cpp test/kinematics/test_kinematics.cpp -o /tmp/kin_test && /tmp/kin_test`
- Output:
```
FK home wrist=(126.000, 0.000, 365.000) tcp=(177.000, 0.000, 365.000)
IK roundtrip: ok=2230 fail=0
Differential Wrist: all kinematic transforms and roundtrip tests PASSED
ALL KINEMATICS & DIFFERENTIAL WRIST TESTS PASSED
```

- Additional host suite `test_work_plane` PASS; `test_joint_logic` (6 FAILED) and `test_homing_logic` (4 FAILED) are pre-existing failures due to config-vs-test mismatch (AXIS_STEP_SIGN J2/J3 +1 vs test expects -1; stall window constants) — not introduced by this task (verified `git diff` shows no config.h change, failures reproduce on base `a3766a0`).

- Verification: `python -m platformio run` SUCCESS with RAM delta +64B (49668→49732) within 1KB, Flash +1928B; no `g_*` symbols.

**Self-Review Notes:**
- Globals fully removed: `grep -rn g_emergencyStop|g_homingActive src/` empty (Task 3 Step 1 failing test now passes, Step 4 gates satisfied). Checked both `std::atomic<bool>` definitions deleted and all 8 call sites replaced.
- `Motor::onStepTimer` fail-fast is first check after `running` — <20us abort, uses `SafetyManager::isEStop()` with `acquire` semantics, clears step pin via direct `GPIO.out*_w1tc` as before. Guarded with `#ifdef ARDUINO` to keep host compile clean (host `endstop.cpp` already compiles without Arduino; `motor.cpp` host not used).
- `SafetyManager` injection ownership: `ArmController` owns `unique_ptr<SafetyManager>` created in `begin()` after `es`/`jm` valid, injected to `Endstops` (propagates to IsrCtx `safety` + `self->safety_`), each `Motor`, and `HomingController`. Destructor defined in `arm.cpp` TU with complete type (fixes `sizeof incomplete SafetyManager` error from `unique_ptr` in `main.cpp`/`web_server.cpp` TUs — verified fix by adding `#include "safety_manager.h"` to `arm.h` so header TUs see complete type, still keeping destructor out-of-line). `main.cpp` needs no change; wiring order preserved.
- `pollEndstops()` called every 10ms at top of `taskLoop` before E_STOP handling ensures 50ms debounce + `isPressed()==LOW` check remains single source (spec §4.1 ≤10ms + 50ms = ≤60ms). Backup `isPressed`+`isRunning`+`movingAway` check preserved but now calls `assertEStop()` instead of `g_emergencyStop.store`, keeping immediate E_STOP for jog-into-endstop while debounce handles ISR path.
- `HomingController::assertHoming` via `safety_` replaces `g_homingActive`; null-check guards host tests (no safety injected → no-op, homing still functional in host `test_homing_logic` which doesn't set safety). `forceClear`/`tryClearFault` semantics unchanged from Task 1/2 (clears latched/pending + drift + state, rejects when pressed).
- `motionAllowed()` now delegates to `safety_->isMotionAllowed()` (NORMAL|HOMING true) plus `mode_!=FAULT`, preserving `STOP_ALL` always allowed and `HOME_*` gated. `CLEAR_FAULT` now returns rejected when SafetyManager still sees pressed/drift (spec §4.4), instead of blindly clearing — verified via `test_safety_manager` case `tryClear_reject_when_pressed`.
- No DH/geometry change, no EN pin, debounce 50ms preserved, timeouts unchanged, Task 2 ISR <3us still holds (second poll now centralized).
- Concerns: `arm.h` now includes `safety_manager.h` (complete type) to satisfy `unique_ptr` — increases header coupling vs pure forward decl + pimpl, but acceptable for Domain Clean approach; alternative pimpl would add indirection. No circular include because `endstop.h` only forward-declares `SafetyManager`. Host build with `ARDUINO` undefined still uses host branch of `safety_manager.h` (template ctor) — `arm.cpp` host compile would still succeed (not required for firmware gate).

**Base:** `a3766a0` → **Head:** `HEAD`
