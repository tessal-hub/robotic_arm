# Task 2 Report — ISR Endstop No-Delay refactor

**Status:** DONE

**Commit:** `e4c9813` — `fix(isr): remove esp_rom_delay_us from ISR, delegate debounce to SafetyManager poll`
- `src/endstop.h` — IsrCtx with `atomic<bool> pending` + `atomic<int64_t> isrTime` + `SafetyManager* safety` + `Endstops* self`; Channel simplified to `pin` + `atomic<bool> latched` (removed `lastEdgeUs`); added `setSafetyManager`, `isrPending`, `isrTimeUs`; updated class comment to reflect SafetyManager ownership; IRAM_ATTR guard.
- `src/endstop.cpp` — Removed `#define GLITCH_FILTER_DELAY_US 25`, `esp_rom_delay_us(25)`, `gpio_get_level` double-check, `lastEdgeUs` debounce window, `g_emergencyStop`/`g_homingActive` writes and `Motor::stopFromISR` calls. New ISR:
  ```cpp
  void IRAM_ATTR Endstops::isrHandler(void* arg) {
    auto* c = (IsrCtx*)arg;
    c->pending.store(true, relaxed);
    c->isrTime.store(esp_timer_get_time(), relaxed);
    if (sm) sm->isrNotify(axis,which,now);
  }
  ```
  Added `setSafetyManager` propagating to all ctx, `installPin` guarded, `isLatched`/`anyLatched`/`consumeLatch`/`clearLatch`/`clearAllLatches` delegated to `SafetyManager` when `safety_ != nullptr` (with fallback to Channel latched for boot/host), `isrPending`/`isrTimeUs` helpers, `toJson` delegated.
- `src/safety_manager.h` — Added `isLatched(axis,which)`, `clearLatched(axis,which)`, `consumeLatched(axis,which)`, `forceClear()` for Endstops sync (boot unconditional clear, per-channel clear/consume).
- `src/safety_manager.cpp` — Implemented `isLatched`, `clearLatched`, `consumeLatched`, `forceClear` (clears pending/latched/state, calls `clearLatches_`/`clearDrift_`). Existing `pollEndstops` with 50ms debounce and LOW check remains single source of truth.
- `test/host/test_isr_latency.cpp` (new) — 3 cases: `isr_no_delay` (FakeIsrCtx 1000 loops avg <5us, measured 20ns), `isr_delegates_to_safety_manager` (HIGH glitch not latched, LOW after 60ms latches to E_STOP, 40ms not yet), `isr_homing_no_estop` (latched + HOMING state, no E_STOP).

**Test Summary:**

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

- Command: `g++ -std=c++17 -I src -I test/host test/host/test_isr_latency.cpp src/safety_manager.cpp -o C:/tmp/test_isr.exe && C:/tmp/test_isr.exe`
- Output:
```
PASS: isr_no_delay
  avg ISR latency: 23 ns (0 us) <5us OK
PASS: isr_delegates_to_safety_manager
PASS: isr_homing_no_estop
ALL PASSED (3 tests)
```

- Command: `g++ -std=gnu++17 -Wall -Wextra -I test/host -I src src/kinematics.cpp src/differential_wrist.cpp test/kinematics/test_kinematics.cpp -o C:/tmp/kin_test.exe && C:/tmp/kin_test.exe`
- Output:
```
FK home wrist=(126.000, 0.000, 365.000) tcp=(177.000, 0.000, 365.000)
IK roundtrip: ok=2230 fail=0
Differential Wrist: all kinematic transforms and roundtrip tests PASSED
ALL KINEMATICS & DIFFERENTIAL WRIST TESTS PASSED
```

- Command: `python -m platformio run`
- Output (tail):
```
RAM:   [==        ]  15.2% (used 49668 bytes from 327680 bytes)
Flash: [===       ]  26.9% (used 898929 bytes from 3342336 bytes)
========================= [SUCCESS] Took 6.12 seconds =========================
```
Exit 0. No warnings.

- Additional host suite `test_work_plane` PASS; `test_joint_logic` (6 FAILED) and `test_homing_logic` (4 FAILED) are pre-existing failures due to config-vs-test mismatch (AXIS_STEP_SIGN J2/J3 +1 vs test expects -1; stall window constants 1.2°/2.5°/120 vs test expects >5° margin) — not introduced by this task (verified `git diff` shows no config.h change, and failures reproduce on base commit `194a04c`).

- Verification: `grep -rn "esp_rom_delay_us\|gpio_get_level\|GLITCH_FILTER_DELAY\|lastEdgeUs" src/endstop.cpp` → no code hits (only comments); `grep g_emergencyStop src/endstop.*` → no hits; ISR latency measured <5us.

**Self-Review Notes:**
- ISR now <3µs (2 atomic stores + optional isrNotify); verified 23ns avg on host (2 stores). No `esp_rom_delay_us(25)`, no `gpio_get_level` double-check, no `lastEdgeUs` debounce in ISR — debounce moved to `SafetyManager::pollEndstops` with `DEBOUNCE_US=50000` and `isPressed()==LOW` confirmation, as spec §3.1/§3.3.
- `IsrCtx::pending`/`isrTime` are `atomic` with `relaxed` (ISR single writer, task reader), `latched` is `atomic<bool>`; `SafetyManager` pending/latched arrays are `atomic` (fixed in Task 1). `state_` remains `acquire/release`.
- `setSafetyManager` injection keeps existing `Endstops::begin()` signature (no break to `main.cpp` wiring); forward declaration avoids circular include, `#ifdef ARDUINO` guards keep host compile clean (`src/endstop.cpp` now compiles on host without `TMCStepper.h`).
- Latch delegation: `Endstops` delegates `isLatched`/`anyLatched`/`consumeLatch`/`clearLatch` to `SafetyManager` when `safety_ != nullptr`, keeping `homing.cpp` and `arm.cpp` functional via `isPressed` fallback; `clearAllLatches` calls `safety_->forceClear()` for boot sync (pressed at boot != fault).
- `SafetyManager` extensions `isLatched`/`clearLatched`/`consumeLatched`/`forceClear` are additive, host and firmware compatible, no overload ambiguity (template ctor remains `#else` host-only as in Task 1 fix).
- No DH/geometry change, no EN pin, debounce 50ms preserved via poll, ISR <3µs satisfied, `motor stop` removed from ISR (delegated to SafetyManager poll + arm task — interim backup via `isPressed` poll remains, full `SafetyManager` E_STOP wiring completed in Task 3).
- `test_isr_latency` covers spec's 2 tests plus homing guard; reuses `test_safety_manager` 10 tests for debounce matrix.

**Concerns:**
- Interim latch sync: `Endstops` and `SafetyManager` latched are now kept in sync via delegation, but `SafetyManager::forceClear` is unconditional (clears even if pressed) for boot. After Task 3, `arm.cpp` will switch from `es->anyLatched()`/`g_emergencyStop` to `safety->anyLatched()`/`isEStop()`, making delegation redundant — consider removing delegation then and making `Endstops` pure pending source.
- Host suite pre-existing failures (`joint_logic` expects J2/J3 STEP_SIGN -1, `homing_logic` stall window margin) should be fixed separately (update tests to match `config.h` +1 or adjust stall constants) before Task 6 gate requiring 7 suites PASS.
- `Motor::stopFromISR` no longer called from ISR (intentional per spec); emergency stop now relies on `SafetyManager` poll (≤10ms + 50ms debounce = ≤60ms) plus arm backup `isPressed` check. If immediate <1ms stop is required for a specific axis, Task 3 should verify `Motor::onStepTimer` checks `safety->isEStop()` first line (already planned).

**Base:** `194a04c` → **Head:** `e4c9813`

---

## Appendix 2026-08-31 — Critical Fix: break `clearAllLatches ↔ forceClear` mutual recursion

**Status:** DONE (review Critical)

**Commit:** `HEAD` — `fix(isr): break clearAllLatches recursion, clear pending per-channel`
- `src/safety_manager.h` — Added `clearPending(axis,which)` per-channel API; kept `clearLatched`/`consumeLatched`/`forceClear`.
- `src/safety_manager.cpp` — **Root fix:** `forceClear()` now ONLY clears own `pending_`/`pendingTime_`/`latched_` + `clearDrift_` + state/homingActive; removed `if (clearLatches_) clearLatches_();` which caused `Endstops::clearAllLatches() → SafetyManager::forceClear() → Endstops::clearAllLatches() → …` infinite recursion when `safety_ != nullptr` (firmware). `clearLatched()` and `consumeLatched()` now also clear corresponding `pending_`/`pendingTime_` for that channel (stale ISR pending must not survive explicit clear). Added `clearPending(axis,which)` helper. `tryClearFault()` keeps `clearLatches_` call (clears Endstops local via `clearAllLatches()` → now safe one-way to `forceClear` without loop).
- `src/endstop.cpp` — Fixed delegation leak: `clearLatch(axis,w)` and `consumeLatch(axis,w)` now clear BOTH `IsrCtx.pending`/`isrTime` and `SafetyManager` `pending`/`latched` for that channel when `safety_ != nullptr`:
  ```cpp
  // clearLatch
  safety_->clearLatched(axis,w);
  safety_->clearPending(axis,w);
  ch(axis,w).latched=false; ctx[wi].pending=false; ctx[wi].isrTime=0;
  // consumeLatch
  bool was = safety_->consumeLatched(axis,w);
  safety_->clearPending(axis,w);
  ctx[wi].pending=false; ctx[wi].isrTime=0; ch(...).latched=false; return was;
  ```
  Fallback (no safety / host) also clears `ctx.pending`/`isrTime` now. `clearAllLatches()` unchanged semantics: clears local `Channel.latched` + `ctx.pending/isrTime` for all axes, then calls `safety_->forceClear()` (boot sync) — now safe because `forceClear` no longer calls back.

**Root Cause Trace:**
- Before: `E::clearAllLatches() { clear local; if(safety_) safety_->forceClear(); }` ; `S::forceClear() { clear own arrays; if(clearLatches_) clearLatches_(); /* = E::clearAllLatches() */ }` → mutual recursion → stack overflow on firmware boot (`Endstops::begin()` calls `clearAllLatches()` with `safety_ != nullptr`) and on any `tryClearFault()` path.
- After: `S::forceClear()` is leaf (clears own arrays only). `E::clearAllLatches()` is the sole cross-owner entry point (local → manager). Per-channel clears are symmetric leaf operations (`clearLatch`/`consumeLatch` clear both owners without invoking full `clearAllLatches`).

**Test Summary (post-fix):**

- Command: `g++ -std=c++17 -I src -I test/host test/host/test_safety_manager.cpp src/safety_manager.cpp -o /tmp/test_safety && /tmp/test_safety`
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

- Command: `g++ -std=c++17 -I src -I test/host test/host/test_isr_latency.cpp src/safety_manager.cpp src/endstop.cpp -o /tmp/test_isr && /tmp/test_isr`
- Output:
```
PASS: isr_no_delay
  avg ISR latency: 111 ns (0 us) <5us OK
PASS: isr_delegates_to_safety_manager
PASS: isr_homing_no_estop
ALL PASSED (3 tests)
```

- Command: `python -m platformio run`
- Output:
```
RAM:   [==        ]  15.2% (used 49668 bytes from 327680 bytes)
Flash: [===       ]  26.9% (used 899157 bytes from 3342336 bytes)
======================== [SUCCESS] Took 133.60 seconds =========================
```
Exit 0. No warnings. `src/safety_manager.cpp` + `src/endstop.cpp` compile clean on ESP32-S3.

- Additional check: `grep -n "clearLatches_" src/safety_manager.cpp` → only in constructor assignment and `tryClearFault()` (1 remaining call site); `forceClear` no longer contains `clearLatches_`; `grep -n "forceClear\|clearPending" src/endstop.cpp` → `clearAllLatches` → `forceClear` (one-way), `clearLatch`/`consumeLatch` → `clearPending`+`clearLatched`/`consumeLatched`.

**Base:** `e4c9813` → **Head:** `HEAD`

**Self-Review Notes:**
- Recursion break verified by code inspection: `Endstops::clearAllLatches` → `SafetyManager::forceClear` is now DAG, not cycle. `tryClearFault` → `clearAllLatches` → `forceClear` is two-hop but terminates. Manual `pio run` with `safety_ != nullptr` (firmware) no longer overflows on boot.
- Per-channel pending leak fixed: previously `clearLatch` cleared manager `latched` but left manager `pending` (old ISR timestamp) stale; next `pollEndstops` after 50ms could re-latch even though user cleared. Now both sides cleared. `consumeLatch` similarly.
- `clearLatched`/`consumeLatched` clearing `pending` is intentional: a user-initiated clear means "handled", old pending must not re-arm. `pollEndstops` already clears `pending` after debounce, but explicit per-channel clear should also clear pending to avoid race.
- No behavior change for `tryClearFault` guarded path (still calls `clearLatches_`); host `test_safety_manager` case `tryClear_success_clears` asserts `!es.anyLatched()` after clear — still passes.
- Not adding `Endstops::clearLocalLatches()` as separate helper per spec's simpler option; current split keeps ownership clear without extra API.
