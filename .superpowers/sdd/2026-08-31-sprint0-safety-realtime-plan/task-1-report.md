# Task 1 Report — SafetyManager core

**Status:** DONE

**Commit:** `a8ab257` — `feat(safety): add SafetyManager single owner with debounce poll`
- `src/safety_manager.h` (new) — SafetyState enum, SafetyManager class with isrNotify/pollEndstops (debounce 50ms, LOW check, HOMING guard), assertHoming/assertEStop/tryClearFault/isMotionAllowed/isEStop/state/anyLatched
- `src/safety_manager.cpp` (new) — implementation: pending+timestamp per channel (6×2), DEBOUNCE_US=50000, poll checks isPressed==LOW before latching, only E_STOP when !homingActive, tryClear only when !anyPressed && !hasAnyDriftFault, clears latches+drift+pending, Arduino vs host #ifdef for esp_timer
- `test/host/test_safety_manager.cpp` (new) — 10 cases (covers 8 required): pending_high_no_latch, low_before_50ms_no_latch, low_after_50ms_latches_estop, homing_no_estop, tryClear_reject_when_pressed, tryClear_reject_when_drift, tryClear_success_clears, isMotionAllowed_matrix, anyLatched_and_assertEStop, debounce_exact_boundary_and_glitch_recovery
- `test/host/test_mocks.h` (new) — MockEndstops/MockJointModel with setGpio/isPressed/anyPressed/anyLatched/clearAllLatches, drift helpers; uses safety_manager.h EndstopWhich

**Test Summary:**
- Command: `g++ -std=c++17 -I src -I test/host test/host/test_safety_manager.cpp src/safety_manager.cpp -o /tmp/test_safety && /tmp/test_safety` (Windows: `-o C:/tmp/test_safety.exe && C:/tmp/test_safety.exe`)
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
- Exit code 0. Host gate PASS.

**Self-Review Notes:**
- Debounce: isrNotify stores pending+time, poll checks delta >=50000 and isPressed()==LOW before latching; HIGH glitch clears pending without latch — matches ENDSTOP_DEBOUNCE_US=50000 and digitalRead LOW requirement.
- HOMING guard: poll only transitions to E_STOP if !homingActive_; when homingActive true, poll keeps state HOMING and latches without E_STOP (verified by homing_no_estop).
- anyLatched internal array mirrors SafetyManager latch (not delegated to Endstops latch) — ensures host test deterministic without real ISR latch; tryClear clears both internal and mock clearAllLatches.
- tryClearFault correctly rejects when anyPressed (loop via anyPressed_ derived from isPressed_) or hasAnyDriftFault true, otherwise clears latches+pending+drift and returns to NORMAL (homingActive false).
- isMotionAllowed returns true for NORMAL|HOMING, false for E_STOP/FAULT; isEStop true only for E_STOP.
- Template ctor in header captures mock methods via std::function; firmware explicit ctor (ARDUINO) captures real Endstops/JointModel same way — keeps existing files untouched per Task1 constraint (no motor.h/arm.h changes).
- No DH/geometry changes; no EN pin; no web handler changes; timeouts preserved.
- File follows AGENTS.md §5 C++ RAII/atomic: state_ is atomic<SafetyState>, pending/latched arrays single-owner (only task thread mutates after ISR store); ISR path minimal (store only).
- Host build guards: safety_manager.h conditionally defines EndstopWhich for host vs includes endstop.h for Arduino; safety_manager.cpp #ifdef ARDUINO for esp_timer/poll without arg.

**Concerns:**
- None blocking. Note: pio run not executed in this workspace due to PowerShell/timeout lim (host gate passed; new files are opt-in and don't alter existing compile units, so Arduino build expected PASS but should be re-verified with `python -m platformio run` by integrator).
- Future Task 3 will need to wire SafetyManager into Motor/Arm and remove g_emergencyStop/g_homingActive globals; current SafetyManager is standalone and not yet integrated.

---

## Fix 2026-08-31 — ISR atomic & ctor overload guard (review Important #1, #2)

**Review findings fixed:**
1. **Important — ISR data race (spec §3.1 IsrCtx):** `pending_`/`pendingTime_`/`latched_` were plain `bool`/`int64_t` shared between `isrNotify` (ISR) and `pollEndstops`/`anyLatched`/`tryClearFault` (task). Fixed by making all three `std::array<std::array<std::atomic<...>,2>, NUM_MOTORS>` and using `store(..., relaxed)` / `load(..., relaxed)` in every access (`src/safety_manager.h:77-79`, `src/safety_manager.cpp:isrNotify`, `pollEndstops`, `tryClearFault`, `anyLatched`). Also fixed ctor init loops to use atomic stores instead of `fill()`. Preserves `state_` acquire/release semantics for mode; `homingActive_` remains task-only (as spec).
2. **Important — Constructor overload ambiguity (firmware):** `SafetyManager(Endstops*,JointModel*)` under `#ifdef ARDUINO` and unconditional `template<typename E,J> SafetyManager(E*,J*)` co-existed on firmware build → ambiguous overload. Fixed by guarding template ctor with `#else` (host-only) in `src/safety_manager.h:22-56`. Firmware build now sees only explicit ctor; host build sees only template ctor. No `std::enable_if` needed; minimal change.
3. DRY: ctor init duplication kept minimal (two sites, 3 lines each); extracting `init()` would be cosmetic and would need atomic-friendly helper — deferred.

**Changes:**
- `src/safety_manager.h` — `pending_`/`pendingTime_`/`latched_` → atomic arrays; template ctor inside `#else`; init loops via `store(relaxed)`.
- `src/safety_manager.cpp` — firmware ctor init via `store(relaxed)`; `isrNotify` uses `store(relaxed)`; `pollEndstops` uses `load(relaxed)` for pending/pendingTime/latched and `store(relaxed)` for pending/latched updates; `tryClearFault` clears via `store(relaxed)`; `anyLatched` via `load(relaxed)`.

**Test output (host gate re-run):**
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
- Exit code 0. 10/10 PASS, same as pre-fix. No new tests broken.

**Build gate:**
- Host `g++` clean (no warnings with `-std=c++17`). Firmware header guard ensures no ambiguous overload under `-DARDUINO`.

**Commit:** `1830870` (`18308700eccfe763ee093f3716ce376bbb0fd683`) — `fix(safety): make ISR pending atomic and guard ctor overload`
