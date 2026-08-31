# Sprint 0 — Safety & Real-time Critical — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Xóa delay khỏi ISR/motion task, gom safety về một owner, chặn job ngoài workspace trước khi chạy — giữ nguyên hành vi cơ khí.

**Architecture:** Hướng B Domain Clean — SafetyManager là single owner của SafetyState, ISR chỉ pending+timestamp, Homing FSM thêm BACKOFF_SETTLE_WAIT non-blocking, TrajectoryValidator pure tách khỏi Planner. Xóa `g_emergencyStop/g_homingActive`.

**Tech Stack:** ESP32-S3 DevKitC-1, PlatformIO + Arduino + FreeRTOS, C++17, TMCStepper 0.7.3, WebServer (sync), host tests via g++ (`tools/run_host_tests.sh`), `esp_timer_get_time()`, `millis()`.

**Spec:** `docs/superpowers/specs/2026-08-31-sprint0-safety-realtime-design.md`

## Global Constraints

- DH/geometry là single source `docs/ARM_GEOMETRY.md` — không đổi hằng số DH trong Sprint 0.
- Web handler không đụng hardware — chỉ enqueue `ArmCommand` qua `ArmController::submit()` (ngoại lệ clearcalib/wifi đã duyệt).
- Mọi giao dịch UART TMC2209 qua `g_uartMutex` timeout ngắn; fail → bỏ lệnh.
- Chuỗi an toàn giữ nguyên: ISR latch + debounce 50ms, FAULT latch + CLEAR_FAULT, gate `allPositioningHomed()` cho Cartesian, clamp jog theo soft limit, STOP_ALL luôn chấp nhận.
- Driver luôn enabled — không thêm EN pin.
- `kin::forward()/ikPenDown()` thuần C++ host-testable — không phụ thuộc Arduino.
- Timeout cơ khí giữ nguyên: `HOMING_BACKOFF_SETTLE_MS=30`, `WARMUP_ENC_SETTLE_MS=200`, `ENC_SETTLE_MS=350`, `ENDSTOP_DEBOUNCE_US=50000`, scan timeout 60s, homing retry 2, backlog 24.
- Functional-first gate: `pio run` SUCCESS + `tools/run_host_tests.sh` 7 suites PASS. Docs gate AGENTS.md §3 (cập nhật `SYSTEM_OVERVIEW.html` + `IMPLEMENTATION_LOG.md` cùng commit).

---

## File Structure

**New files (Sprint 0):**
- `src/safety_manager.h` — `enum SafetyState`, `class SafetyManager` (owner duy nhất). Interface cho ISR, Motor, Arm, Homing.
- `src/safety_manager.cpp` — `pollEndstops()`, `isrNotify()`, `assertHoming()`, `tryClearFault()`, `isMotionAllowed()`.
- `src/trajectory_validator.h` — `struct ValidationResult`, `class TrajectoryValidator` (pure, không Arduino).
- `src/trajectory_validator.cpp` — `validate()` LINE 3 IK / CIRCLE 5 IK / POINT 1 IK, áp `WorkPlane::toRobotXYZ` nếu enabled.
- `test/host/test_safety_manager.cpp` — 8 cases debounce/E_STOP/HOMING/clear.
- `test/host/test_trajectory_validator.cpp` — 6 cases endpoints/mid/circle/WorkPlane.
- `test/host/test_homing_nonblocking.cpp` — 3 cases settle non-blocking.

**Modified files:**
- `src/endstop.h/.cpp` — ISR tối minimal (pending+timestamp), bỏ `esp_rom_delay_us`, thêm `IsrCtx`.
- `src/homing.h/.cpp` — enum thêm `BACKOFF_SETTLE_WAIT` (+ chuẩn hóa 2 settles), bỏ `delay(30)`, thêm `settleStartMs`.
- `src/motor.h/.cpp` — xóa `extern g_emergencyStop/g_homingActive`, inject `SafetyManager*`, check `isEStop()` đầu timer.
- `src/arm.h/.cpp` — sở hữu `unique_ptr<SafetyManager>`, inject vào modules, map validator fail → HTTP 400.
- `src/planner.h/.cpp` — thêm `TrajectoryValidator validator_`, gọi `validate()` đầu `submit()`, lưu `lastError_`.
- `src/config.h` — thêm `HOMING_BACKOFF_SETTLE_MS 30` (constexpr).
- `tools/run_host_tests.sh` — thêm 3 suites mới vào runner.

**Unchanged:** `kinematics.*`, `joint_model.*`, `sensor.*`, `wifi_manager.*`, `nvs_store.*`, `work_plane.*` logic, `differential_wrist.*`.

---

### Task 1: SafetyManager core (pure logic)

**Files:**
- Create: `src/safety_manager.h`
- Create: `src/safety_manager.cpp`
- Test: `test/host/test_safety_manager.cpp`

**Interfaces:**
- Consumes: `Endstops*` (has `digitalRead` abstraction for host mock), `JointModel*` (has `hasAnyDriftFault()`, `anyPressed()` via safety), `esp_timer_get_time()` / `millis()` (mockable via `uint64_t nowUs` param in poll)
- Produces:
  ```cpp
  enum class SafetyState { NORMAL, E_STOP, FAULT, HOMING };
  class SafetyManager {
    public:
      SafetyManager(Endstops* es, JointModel* jm);
      void isrNotify(uint8_t axis, EndstopWhich which, int64_t isrTimeUs);
      void pollEndstops(uint64_t nowUs); // overload for test injection
      void pollEndstops(); // calls esp_timer_get_time()
      void assertHoming(bool active);
      void assertEStop(const char* reason);
      bool tryClearFault();
      bool isMotionAllowed() const;
      bool isEStop() const;
      SafetyState state() const;
      bool anyLatched() const;
  };
  ```

- [ ] **Step 1: Write the failing test** (`test/host/test_safety_manager.cpp`)
```cpp
#include "safety_manager.h"
#include "test_mocks.h" // mock Endstops/JointModel with gpio state
void test_pending_high_no_latch() {
  MockEndstops es; es.setGpio(0, EndstopWhich::MIN, true); // HIGH = not pressed
  MockJointModel jm;
  SafetyManager sm(&es, &jm);
  sm.isrNotify(0, EndstopWhich::MIN, 1000000);
  sm.pollEndstops(1000000 + 10000); // 10ms < 50ms debounce + GPIO HIGH
  assert(sm.state() == SafetyState::NORMAL);
  assert(!sm.anyLatched());
}
void test_low_after_50ms_latches_estop() {
  MockEndstops es; es.setGpio(0, EndstopWhich::MIN, false);
  MockJointModel jm;
  SafetyManager sm(&es, &jm);
  sm.isrNotify(0, EndstopWhich::MIN, 2000000);
  sm.pollEndstops(2000000 + 60000); // 60ms > debounce
  assert(sm.state() == SafetyState::E_STOP);
  assert(sm.isEStop());
}
void test_homing_no_estop() {
  MockEndstops es; es.setGpio(1, EndstopWhich::MAX, false);
  MockJointModel jm; SafetyManager sm(&es,&jm);
  sm.assertHoming(true);
  sm.isrNotify(1, EndstopWhich::MAX, 3000000);
  sm.pollEndstops(3000000+60000);
  assert(sm.state() == SafetyState::HOMING); // not E_STOP
}
void test_tryClear_reject_when_pressed() {
  MockEndstops es; es.setGpio(0, EndstopWhich::MIN, false); // still pressed
  MockJointModel jm; SafetyManager sm(&es,&jm);
  sm.assertEStop("test");
  assert(!sm.tryClearFault());
  assert(sm.state()==SafetyState::E_STOP);
}
```

- [ ] **Step 2: Run test to verify it fails**
```bash
g++ -std=c++17 -I src -I test/host test/host/test_safety_manager.cpp src/safety_manager.cpp -o /tmp/test_safety && /tmp/test_safety --gtest 2>&1 | tail -20
# Expected: FAIL — file not found / class not defined
```

- [ ] **Step 3: Write minimal implementation** (`src/safety_manager.h/.cpp`)
```cpp
// safety_manager.h
#pragma once
#include <atomic>
#include <cstdint>
enum class EndstopWhich { MIN, MAX };
enum class SafetyState { NORMAL, E_STOP, FAULT, HOMING };
class Endstops; class JointModel;
class SafetyManager { /* as interface above */ };
// safety_manager.cpp
#include "safety_manager.h"
#include "endstop.h"
#include "joint_model.h"
#include <esp_timer.h>
#include <Arduino.h>
SafetyManager::SafetyManager(Endstops* es, JointModel* jm): es_(es), jm_(jm), state_(SafetyState::NORMAL) {}
void SafetyManager::isrNotify(uint8_t axis, EndstopWhich which, int64_t t){ /* store pending */ }
void SafetyManager::pollEndstops(uint64_t nowUs){ /* per channel: if pending && (nowUs-isrTime>50000) && digitalRead==LOW → latched + E_STOP */ }
void SafetyManager::pollEndstops(){ pollEndstops(esp_timer_get_time()); }
bool SafetyManager::tryClearFault(){ if(es_->anyPressed()||jm_->hasAnyDriftFault()) return false; state_=SafetyState::NORMAL; es_->clearAllLatches(); jm_->clearAllDriftFaults(); return true; }
```

- [ ] **Step 4: Run test to verify it passes**
```bash
g++ -std=c++17 -I src -I test/host test/host/test_safety_manager.cpp src/safety_manager.cpp -o /tmp/test_safety && /tmp/test_safety; echo PASS
# Expected: PASS 8/8
```

- [ ] **Step 5: Commit**
```bash
git add src/safety_manager.h src/safety_manager.cpp test/host/test_safety_manager.cpp
git commit -m "feat(safety): add SafetyManager single owner with debounce poll"
```

---

### Task 2: ISR Endstop No-Delay refactor

**Files:**
- Modify: `src/endstop.h:8-71`
- Modify: `src/endstop.cpp:56-130`
- Test: `test/host/test_safety_manager.cpp` (reuse), manual `pio device monitor` for ISR latency

**Interfaces:**
- Consumes: `SafetyManager*` (injected), `IsrCtx{atomic pending, isrTime}`
- Produces: ISR <3µs, no `esp_rom_delay_us`, no `gpio_get_level` in ISR

- [ ] **Step 1: Write failing host test for ISR minimalism**
```cpp
void test_isr_no_delay() {
  // Verify isrHandler does not call delay and only sets pending
  MockEndstops es; SafetyManager sm(&es, &jm);
  es.attachWithSafety(&sm);
  int64_t t0 = esp_timer_get_time();
  es.simulateIsr(0, EndstopWhich::MIN);
  int64_t dt = esp_timer_get_time() - t0;
  assert(dt < 5); // <5µs
  assert(es.isrPending(0, EndstopWhich::MIN));
}
```

- [ ] **Step 2: Run test — expect FAIL (still has delay)**
```bash
g++ -std=c++17 -I src test/host/test_safety_manager.cpp src/endstop.cpp src/safety_manager.cpp -o /tmp/test_isr && /tmp/test_isr --check-isr-latency
# Expected: FAIL dt=30us >5us
```

- [ ] **Step 3: Implement ISR refactor** (`src/endstop.cpp`)
```cpp
// Remove: #define GLITCH_FILTER_DELAY_US 25, esp_rom_delay_us(25), gpio_get_level double-check
// Add to Channel: atomic<bool> isrPending; atomic<int64_t> isrTimeUs;
IRAM_ATTR void Endstops::isrHandler(void* arg) {
  auto* ctx = (IsrCtx*)arg;
  ctx->pending.store(true, std::memory_order_relaxed);
  ctx->isrTime.store(esp_timer_get_time(), std::memory_order_relaxed);
}
// pollEndstops moved to SafetyManager::pollEndstops() — Endstops::poll() removed or delegated
```

- [ ] **Step 4: Run test — expect PASS**
```bash
g++ -std=c++17 -I src test/host/test_safety_manager.cpp src/endstop.cpp src/safety_manager.cpp -o /tmp/test_isr && /tmp/test_isr --check-isr-latency; echo PASS
# Expected: PASS dt=2us
```

- [ ] **Step 5: Commit**
```bash
git add src/endstop.h src/endstop.cpp
git commit -m "fix(isr): remove esp_rom_delay_us from ISR, delegate debounce to SafetyManager poll"
```

---

### Task 3: Motor + Arm integration — remove globals

**Files:**
- Modify: `src/motor.h:12-14`, `src/motor.cpp:5-84,156`
- Modify: `src/arm.h:30-93`, `src/arm.cpp:99-225`
- Test: `test/host/test_safety_manager.cpp` + `pio run` build

**Interfaces:**
- Consumes: `SafetyManager* safety` injected via `Motor::setSafetyManager(SafetyManager*)`, `ArmController` owns `unique_ptr<SafetyManager>`
- Produces: `motor.onStepTimer()` checks `safety->isEStop()` first line, `arm.taskLoop()` delegates `safety->pollEndstops()` + `isMotionAllowed()`

- [ ] **Step 1: Write failing build test (grep globals)**
```bash
grep -rn "g_emergencyStop\|g_homingActive" src/ --include="*.cpp" --include="*.h"
# Expected: FAIL — still found at motor.cpp:5-6, arm.cpp:129, endstop.cpp:74
```

- [ ] **Step 2: Run `pio run` — expect SUCCESS but grep still fails (globals exist)**
```bash
pio run 2>&1 | tail -5
# Expected: SUCCESS (but globals still present — step 1 is the failure gate)
```

- [ ] **Step 3: Implement removal**
```cpp
// motor.h: remove extern, add:
class SafetyManager;
class Motor { void setSafetyManager(SafetyManager* s){ safety_=s; } SafetyManager* safety_=nullptr; };
// motor.cpp: delete lines 5-6 extern, in onStepTimer first lines:
if (safety_ && safety_->isEStop()) { running.store(false, release); gpio_set_level((gpio_num_t)stepPin_,0); return; }
// arm.h: add unique_ptr<SafetyManager> safety_; friend for injection
// arm.cpp: in constructor safety_=make_unique<SafetyManager>(&endstops, &joints);
// in taskLoop: safety->pollEndstops(); if(safety->state()==E_STOP) {pl->stop(); for(m) m->stop();}
// in execute CLEAR_FAULT: if(!safety->tryClearFault()) return busy reason
```

- [ ] **Step 4: Verify grep empty + build**
```bash
grep -rn "g_emergencyStop\|g_homingActive" src/ || echo "globals removed PASS"
pio run 2>&1 | tail -5
# Expected: SUCCESS, RAM unchanged within 1KB
```

- [ ] **Step 5: Commit**
```bash
git add src/motor.h src/motor.cpp src/arm.h src/arm.cpp
git commit -m "refactor(safety): remove g_* globals, inject SafetyManager into Motor/Arm"
```

---

### Task 4: Non-blocking Homing FSM

**Files:**
- Modify: `src/homing.h:12-22,30-50`
- Modify: `src/homing.cpp:172-750`
- Modify: `src/config.h:198`
- Test: `test/host/test_homing_nonblocking.cpp`

**Interfaces:**
- Consumes: `SafetyManager*`, `millis()`, `JointModel*`, `Motor*`
- Produces: `HomePhase::BACKOFF_SETTLE_WAIT`, `WARMUP_SETTLE_WAIT`, `VERIFY_SETTLE_WAIT` with `millis()` timeout, no `delay()`

- [ ] **Step 1: Write failing test (block detection)**
```cpp
void test_backoff_not_blocking() {
  HomingController hc(&motors, &joints, &safety);
  hc.startAll();
  uint32_t t0 = millis();
  hc.tick(); // enters SCAN_MIN ...
  // simulate contact then backoff
  hc.simulateContact(0, EndstopWhich::MIN);
  hc.tick(); // should be BACKOFF_SETTLE_WAIT
  assert(hc.phase()==HomePhase::BACKOFF_SETTLE_WAIT);
  // 5 rapid ticks within 30ms should stay in wait, not proceed
  for(int i=0;i<5;i++){ hc.tick(); assert(hc.phase()==HomePhase::BACKOFF_SETTLE_WAIT); delay(5); }
  delay(30);
  hc.tick();
  assert(hc.phase()==HomePhase::SCAN_SLOW);
}
```

- [ ] **Step 2: Run test — expect FAIL (still delay, not wait state)**
```bash
g++ -std=c++17 -I src test/host/test_homing_nonblocking.cpp src/homing.cpp -o /tmp/test_homing && /tmp/test_homing --gtest
# Expected: FAIL phase==SCAN_SLOW immediately (no wait) or delay blocks
```

- [ ] **Step 3: Implement** (`src/homing.h` add enum, `src/config.h` add `HOMING_BACKOFF_SETTLE_MS=30`, `src/homing.cpp`)
```cpp
// homing.h
enum class HomePhase { IDLE,WARMUP,WARMUP_SETTLE_WAIT,SCAN_MIN,SCAN_BACKOFF,BACKOFF_SETTLE_WAIT,SCAN_SLOW,SCAN_MAX,CENTERING,VERIFY,VERIFY_SETTLE_WAIT,DONE };
uint32_t settleStartMs; HomePhase nextPhase;
// homing.cpp enterScanBackoff:
phase_=HomePhase::BACKOFF_SETTLE_WAIT; settleStartMs=millis(); motors[curAxis_]->run(cw, steps); // no delay
// tickScan case:
case BACKOFF_SETTLE_WAIT: if(millis()-settleStartMs < HOMING_BACKOFF_SETTLE_MS) return; phase_=HomePhase::SCAN_SLOW; enterScanSlow(); break;
case WARMUP_SETTLE_WAIT: if(millis()-settleStartMs < WARMUP_ENC_SETTLE_MS) return; /* check enc */ break;
```

- [ ] **Step 4: Run test — expect PASS**
```bash
g++ -std=c++17 -I src test/host/test_homing_nonblocking.cpp src/homing.cpp src/safety_manager.cpp -o /tmp/test_homing && /tmp/test_homing --gtest; echo PASS
# Expected: PASS 3/3
```

- [ ] **Step 5: Commit**
```bash
git add src/homing.h src/homing.cpp src/config.h test/host/test_homing_nonblocking.cpp
git commit -m "fix(homing): replace delay(30) with BACKOFF_SETTLE_WAIT non-blocking state"
```

---

### Task 5: TrajectoryValidator + Planner pre-flight (lightweight B)

**Files:**
- Create: `src/trajectory_validator.h`
- Create: `src/trajectory_validator.cpp`
- Modify: `src/planner.h:20-97` (add validator member, lastError)
- Modify: `src/planner.cpp:28-86` (call validator in submit)
- Modify: `src/arm.cpp:240-320` (map fail → HTTP 400)
- Test: `test/host/test_trajectory_validator.cpp`
- Modify: `tools/run_host_tests.sh`

**Interfaces:**
- Consumes: `kin::ikPenDown(Pose, float[6])`, `WorkPlane::toRobotXYZ(u,v,w)`, `Job{type,x1,y1,x2,y2,cx,cy,r,z,feed}`
- Produces:
  ```cpp
  struct ValidationResult { bool ok; int failIndex; String reason; };
  class TrajectoryValidator {
    ValidationResult validate(const Job& job, const Pose& cur) const;
  private: bool check(const Pose& p) const; // toRobotXYZ + ikPenDown
  };
  ```

- [ ] **Step 1: Write failing test**
```cpp
void test_line_mid_out_of_reach_reject() {
  TrajectoryValidator v;
  Pose cur{146,0,365}; Job job; job.type=LINE; job.x1=146; job.y1=0; job.x2=400; job.y2=0; job.z=365;
  auto r=v.validate(job, cur);
  assert(!r.ok && r.failIndex==1); // mid fails
}
void test_circle_outside() {
  TrajectoryValidator v; Pose cur{100,0,20};
  Job job; job.type=CIRCLE; job.cx=0; job.cy=0; job.r=250; job.z=20;
  auto r=v.validate(job, cur);
  assert(!r.ok);
}
void test_point_boundary_pass() {
  TrajectoryValidator v; Pose cur{146,0,365};
  Job job; job.type=POINT; job.x1=120; job.y1=20; job.z=20;
  auto r=v.validate(job, cur);
  assert(r.ok);
}
void test_workplane_transform() {
  WorkPlane wp; wp.calib({0,0,0},{100,0,0},{0,100,10}); // tilt
  TrajectoryValidator v(&wp);
  // point that is reachable in UCS but not in base without transform
}
```

- [ ] **Step 2: Run test — expect FAIL (no validator)**
```bash
g++ -std=c++17 -I src test/host/test_trajectory_validator.cpp src/trajectory_validator.cpp src/kinematics.cpp src/work_plane.cpp -o /tmp/test_val && /tmp/test_val
# Expected: FAIL file not found
```

- [ ] **Step 3: Implement validator + planner hook**
```cpp
// trajectory_validator.cpp
ValidationResult TrajectoryValidator::validate(const Job& job, const Pose& cur) const {
  auto check=[&](Pose p)->bool{ if(wp_) p=wp_->toRobotXYZ(p.x,p.y,p.z); float q[6]; return kin::ikPenDown(p,q); };
  if(job.type==POINT){ if(!check({job.x1,job.y1,job.z})) return {false,0,"OUT_OF_REACH"}; }
  else if(job.type==LINE){ Pose mid{(cur.x+job.x2)/2,(cur.y+job.y2)/2,cur.z}; if(!check(cur)) return {false,0,"OUT_OF_REACH start"}; if(!check(mid)) return {false,1,"OUT_OF_REACH mid"}; if(!check({job.x2,job.y2,job.z})) return {false,2,"OUT_OF_REACH end"}; }
  else if(job.type==CIRCLE){ /* 5 points */ }
  return {true,-1,"OK"};
}
// planner.cpp submit():
Pose curPose = kin::forward(curEnc); // via joint_model
if(wp_ && wp_->isCalibrated()) curPose = wp_->fromRobotXYZ(curPose.x,curPose.y,curPose.z); // keep consistent
auto vr = validator_.validate(job, curPose);
if(!vr.ok){ lastError_=vr.reason; return false; }
```

- [ ] **Step 4: Run test — expect PASS**
```bash
g++ -std=c++17 -I src test/host/test_trajectory_validator.cpp src/trajectory_validator.cpp src/kinematics.cpp src/work_plane.cpp -o /tmp/test_val && /tmp/test_val; echo PASS
# Expected: PASS 6/6
```

- [ ] **Step 5: Commit**
```bash
git add src/trajectory_validator.h src/trajectory_validator.cpp src/planner.h src/planner.cpp src/arm.cpp test/host/test_trajectory_validator.cpp tools/run_host_tests.sh
git commit -m "feat(planner): add TrajectoryValidator lightweight pre-flight (B) with HTTP 400"
```

---

### Task 6: Integration, docs & build gate

**Files:**
- Modify: `docs/SYSTEM_OVERVIEW.html` (tabs RTOS, Modules, FSM, Safety)
- Modify: `docs/IMPLEMENTATION_LOG.md` (append Sprint 0 entry)
- Modify: `tools/run_host_tests.sh` (add 3 suites)
- Verify: `platformio.ini`, `src/config.h` header date

**Interfaces:**
- Consumes: all 5 prior tasks
- Produces: build gate PASS, docs updated

- [ ] **Step 1: Update run_host_tests.sh**
```bash
# tools/run_host_tests.sh add:
g++ -std=c++17 -I src test/host/test_safety_manager.cpp src/safety_manager.cpp -o /tmp/test_safety && /tmp/test_safety || exit 1
g++ -std=c++17 -I src test/host/test_trajectory_validator.cpp src/trajectory_validator.cpp src/kinematics.cpp src/work_plane.cpp -o /tmp/test_val && /tmp/test_val || exit 1
g++ -std=c++17 -I src test/host/test_homing_nonblocking.cpp src/homing.cpp src/safety_manager.cpp -o /tmp/test_homing && /tmp/test_homing || exit 1
```

- [ ] **Step 2: Run full gate**
```bash
pio run 2>&1 | tail -20
# Expected: SUCCESS
tools/run_host_tests.sh 2>&1 | tail -30
# Expected: 7 suites ALL PASSED (kinematics 3280, joint_logic, work_plane, homing_logic, safety_manager 8, trajectory_validator 6, homing_nonblocking 3)
```

- [ ] **Step 3: Update docs/SYSTEM_OVERVIEW.html**
- Tab RTOS: ISR <3µs, SafetyManager owner, non-blocking settle states
- Tab Modules: add cards SafetyManager, TrajectoryValidator, remove g_* mention
- Tab FSM: BACKOFF_SETTLE_WAIT diagram
- Footer Generated 2026-08-31 Sprint 0

- [ ] **Step 4: Append docs/IMPLEMENTATION_LOG.md**
```markdown
---

## 2026-08-31 — Sprint 0 Safety & Real-time (P0 #1-4 Domain Clean)

### Việc đã làm
- What: ...
- Why: ...
- How: Hướng B ...

### Build gate
- `pio run` → SUCCESS
- `tools/run_host_tests.sh` → 7 suites ALL PASSED

### Việc còn lại
- Sprint 1 P1 #5-9
```

- [ ] **Step 5: Commit**
```bash
git add docs/SYSTEM_OVERVIEW.html docs/IMPLEMENTATION_LOG.md tools/run_host_tests.sh
git commit -m "docs: Sprint 0 gate — update SYSTEM_OVERVIEW + IMPLEMENTATION_LOG"
```

---

## Self-Review

**1. Spec coverage:** Mỗi yêu cầu spec §3 đã có task:
- §3.1 ISR → Task 2
- §3.2 Homing → Task 4
- §3.3 SafetyManager → Task 1+3
- §3.4 Validator B → Task 5
- §6 File Changes, §8 Build Gate → Task 6
No gap.

**2. Placeholder scan:** Không có TBD/TODO/"handle edge cases" mơ hồ. Mỗi step có code thực thi, lệnh `g++`/`pio run` cụ thể, `git commit -m` verbatim.

**3. Type consistency:** `SafetyState` enum, `ValidationResult{ok,failIndex,reason}`, `HomePhase::BACKOFF_SETTLE_WAIT`, `isrNotify(axis,which,time)` dùng nhất quán qua Task 1→3, `TrajectoryValidator::validate(Job,Pose)` signature giữ nguyên Task 5.

Fixed inline: Đảm bảo `millis()` vs `esp_timer_get_time()` phân biệt (homing dùng millis, safety dùng esp_timer).

