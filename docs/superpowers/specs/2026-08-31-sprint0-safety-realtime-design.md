# Sprint 0 — Safety & Real-time Critical — Design Spec

**Date:** 2026-08-31
**Sprint:** 0/4 (P0 #1-4 trong Roadmap V3.0 — 21 nâng cấp)
**Scope:** ISR Endstop No-Delay (#1) + Non-blocking Homing FSM (#2) + Dynamic Safety Manager (#3) + Kinematic Pre-flight Validation lightweight B (#4)
**Approach:** Hướng B — Domain Clean (làm sạch kiến trúc, realtime-first)
**Status:** Approved — awaiting implementation plan

---

## 1. Context & Goals

### 1.1 Why Sprint 0
Codebase hiện tại có 2 vi phạm real-time nghiêm trọng đã verify bằng code:
- `src/endstop.cpp:65` `esp_rom_delay_us(25)` trong `IRAM_ATTR isrHandler()` — ISR giữ interrupt disabled, miss step timer ticks.
- `src/homing.cpp:221` `delay(30)` trong `enterScanBackoff()` — block toàn bộ `arm_motion` task 100Hz 3 ticks, nguy cơ WDT 5s nếu homing 4 khớp liên tiếp.
- Safety phân tán: `g_emergencyStop`/`g_homingActive` định nghĩa ở `motor.cpp:5-6`, đọc/ghi rải rác `endstop.cpp:74/79`, `motor.cpp:81/156`, `arm.cpp:99/129`.
- Planner không pre-check: `planner.cpp:28-86` `submit()` không gọi `ikPenDown()`, chỉ check per-segment `110/209` → dừng giữa quỹ đạo.

Sprint 0 là hotfix bắt buộc trước khi chạm Foundation (Sprint 1) hay Architecture (Sprint 2).

### 1.2 Goals (functional-first)
- Xóa mọi `delay()` / `esp_rom_delay_us` khỏi ISR và motion task.
- Gom safety về một owner, xóa globals, semantics `tryClearFault()` rõ ràng.
- Chặn job ngoài vùng với trước khi chạy (lightweight B), không dừng giữa đường.
- Giữ nguyên hành vi cơ khí: timeout 30/200/350ms, debounce 50ms, gear/DH/pinout.

### 1.3 Non-Goals
- Không đổi hằng số DH, gear, current, soft limit (thuộc NVS Config Sprint 1).
- Không tách HAL/DI full (Sprint 2), không WebSocket/LittleFS (Sprint 2), không OTA/G-code (Sprint 3 bỏ qua).
- Không tune timeout cơ khí trong Sprint 0 — giữ giá trị cũ, chỉ đổi cơ chế.

---

## 2. Architecture Overview

```
[GPIO] ──FALLING──→ [ISR 3µs] pending+timestamp ──→ [SafetyManager::pollEndstops() @100Hz] ──→ state
                                           ↑                         ↓
                                     Motor::onStepTimer      ArmController::taskLoop HomingController
                                     isEStop() → stop        isMotionAllowed()       assertHoming()

[Web] POST /api/move ─→ ArmController::execute() ─→ TrajectoryValidator::validate() ─→ Planner::submit()
                              (kin::forward curPose)    (3-5 ikPenDown)  ──→ HTTP 400 nếu fail
                                                                             ──→ active nếu pass
[Homing] tickScan() non-blocking: BACKOFF_SETTLE_WAIT dùng millis(), không delay()
```

**New modules:** `SafetyManager` (single owner), `TrajectoryValidator` (pure, host-testable).
**Modified:** `endstop.*`, `homing.*`, `motor.*`, `arm.*`, `planner.*`.
**Unchanged values:** `config.h` timeouts, DH, pins.

---

## 3. Components

### 3.1 ISR Endstop No-Delay (#1)
**File:** `src/endstop.h`, `src/endstop.cpp`

Before:
```cpp
IRAM_ATTR isrHandler(void* arg) {
  if (now - lastEdgeUs < 50000) return;
  esp_rom_delay_us(25); // VIOLATION
  if (gpio_get_level(pin)==HIGH) return;
  // ... latch + g_emergencyStop
}
```

After:
```cpp
struct IsrCtx { atomic<bool> pending; atomic<int64_t> isrTime; uint8_t axis; EndstopWhich which; };
IRAM_ATTR isrHandler(void* arg) {
  auto* c = (IsrCtx*)arg;
  c->pending.store(true, memory_order_relaxed);
  c->isrTime.store(esp_timer_get_time(), memory_order_relaxed);
}
struct Channel { atomic<bool> pending; atomic<int64_t> isrTime; volatile bool latched; /* ... */ };
```

- Xóa `GLITCH_FILTER_DELAY_US`, `esp_rom_delay_us`, `gpio_get_level` khỏi ISR.
- Latency <3µs (chỉ 2 atomic store).
- Debounce + confirm chuyển sang `SafetyManager::pollEndstops()` (Section 3.3).

### 3.2 Non-blocking Homing FSM (#2)
**File:** `src/homing.h`, `src/homing.cpp`

Changes:
- Enum `HomePhase` thêm `BACKOFF_SETTLE_WAIT` (và chuẩn hóa `WARMUP_SETTLE_WAIT`, `VERIFY_SETTLE_WAIT` nếu chưa).
- Members: `uint32_t settleStartMs; HomePhase nextAfterSettle;`
- `enterScanBackoff()`:
  ```cpp
  void HomingController::enterScanBackoff() {
    // xóa latch, ghi backoffStartEnc_, v.v.
    motors[curAxis_]->run(cw, steps);
    phase_ = HomePhase::BACKOFF_SETTLE_WAIT;
    settleStartMs = millis();
    // KHÔNG delay(30)
  }
  ```
- `tickScan()` thêm:
  ```cpp
  case BACKOFF_SETTLE_WAIT:
    if (millis() - settleStartMs < HOMING_BACKOFF_SETTLE_MS) return; // 30ms, non-blocking
    // poll safety vẫn chạy ở arm_motion tick cha
    phase_ = HomePhase::SCAN_SLOW;
    enterScanSlow();
    break;
  ```
- Tương tự cho `WARMUP → WARMUP_SETTLE_WAIT (200ms)` và `CENTERING → VERIFY_SETTLE (350ms)`.
- Constants giữ nguyên: `HOMING_BACKOFF_SETTLE_MS=30`, `WARMUP_ENC_SETTLE_MS=200`, `ENC_SETTLE_MS=350`. Để trong `config.h` để Sprint 1 có thể NVS-hóa.

### 3.3 Dynamic Safety Manager (#3)
**File mới:** `src/safety_manager.h`, `src/safety_manager.cpp`

```cpp
enum class SafetyState { NORMAL, E_STOP, FAULT, HOMING };

class SafetyManager {
public:
  SafetyManager(Endstops* es, JointModel* jm);
  void isrNotify(uint8_t axis, EndstopWhich which, int64_t t); // ISR gọi
  void pollEndstops(); // 100Hz, debounce 50ms
  void assertHoming(bool active);
  void assertEStop(const char* reason); // log + state=E_STOP, stop motors via callback
  bool tryClearFault(); // true chỉ khi !anyPressed() && !jm->hasAnyDriftFault()
  bool isMotionAllowed() const; // NORMAL|HOMING
  bool isEStop() const;
  SafetyState state() const;
  bool anyLatched() const;
private:
  Endstops* es_; JointModel* jm_;
  atomic<SafetyState> state_{SafetyState::NORMAL};
  // per-channel pending/isrTime mirror
};
```

- `pollEndstops()`: với mỗi `pending==true`, đọc `digitalRead(pin)`, nếu `LOW` và `now-isrTime>50000` → `latched=true`, `state=E_STOP` nếu `!homing`. Clear `pending`. Nếu `HIGH` → chỉ clear pending (glitch).
- Xóa `motor.cpp:5-6` `g_emergencyStop/g_homingActive`. Thay mọi `if(g_emergencyStop)` → `safety.isEStop()`, `if(g_homingActive)` → `safety.state()==HOMING`.
- `ArmController` sở hữu `unique_ptr<SafetyManager> safety;` inject vào `Endstops`, `HomingController`, `Motor` (qua raw ptr hoặc reference).
- `tryClearFault()` được `POST /api/jog fault_clear=1` gọi. Trước đó `SafetyManager::anyPressed()` check.

Ownership rule: chỉ `SafetyManager` được set/clear `E_STOP`/`FAULT`.

### 3.4 Kinematic Pre-flight Validation B (#4)
**File mới:** `src/trajectory_validator.h`, `src/trajectory_validator.cpp`
**File sửa:** `src/planner.cpp`, `src/arm.cpp`, `src/work_plane.h`

```cpp
struct ValidationResult { bool ok; int failIndex; const char* reason; };
class TrajectoryValidator {
public:
  ValidationResult validate(const Job& job, const Pose& curPose) const;
private:
  bool checkPose(const Pose& p) const; // kin::ikPenDown + WorkPlane::toRobotXYZ nếu enabled
};
```

- `validate()`:
  - `POINT`: check `target` 1 lần.
  - `LINE`: check `curPose`, `mid=(cur+target)/2`, `target` (3 IK).
  - `CIRCLE`: check `curPose`, `90°`, `180°`, `270°`, `end` (5 IK). Tính điểm trên circle qua `ang = startAng + prog/r`, áp `workPlane->toRobotXYZ` nếu enabled, rồi `ikPenDown()`.
- `Planner::submit()` gọi đầu tiên:
  ```cpp
  auto vr = validator_.validate(job, curPose);
  if (!vr.ok) { lastError_=vr.reason; return false; }
  ```
- `ArmController::execute()` map `false` → `HTTP 400 {"error":"OUT_OF_REACH","segment":vr.failIndex}`.
- Giữ `startMoveTo()` per-segment guard `ikPenDown()` như defense-in-depth (nếu validator pass nhưng segment fail do sai số).

Pure C++ (chỉ `kinematics.h`, `work_plane.h`), host-testable không cần Arduino.

---

## 4. Data Flow & Error Handling

### 4.1 Endstop → E_STOP (≤10ms)
`GPIO FALLING → ISR pending → 100Hz pollEndstops() debounce 50ms → latched → state=E_STOP → ArmController::taskLoop() stop planner+motors → Motor::onStepTimer early return`. EMI ngắn <50ms hoặc GPIO HIGH khi poll → không latch.

### 4.2 Homing settle
`SCAN_BACKOFF → BACKOFF_SETTLE_WAIT (30ms non-blocking) → SCAN_SLOW`. Trong wait, `arm_motion` vẫn poll safety + drift mỗi tick. Jammed → `backoffExtend++`, hết quota → `finishJoint(false)` → retry (`HOMING_MAX_ATTEMPTS=2`).

### 4.3 Pre-flight reject
`POST /api/move x=400 y=0 z=365` (ngoài reach) → validator fail tại segment 0 → HTTP 400 ngay, robot không nhúc nhích. Validator pass nhưng per-segment fail → `stop()` + `FAULT` (an toàn).

### 4.4 FAULT clear
`tryClearFault()` chỉ true khi `!anyPressed() && !hasAnyDriftFault()`. Web handler không đụng hardware, chỉ enqueue `ArmCommand` (FHIR).

---

## 5. Testing

### 5.1 Host tests (`tools/run_host_tests.sh`)
- `test/host/test_safety_manager.cpp` (8 cases): pending+GPIO HIGH→no latch, LOW+50ms→latch+E_STOP, HOMING mode→no E_STOP, tryClearFault reject khi pressed, isMotionAllowed matrix.
- `test/host/test_trajectory_validator.cpp` (6 cases): LINE mid OUT_OF_REACH→reject, CIRCLE radius 250→reject seg 3, WorkPlane tilt 15° transform, POINT boundary z=-20/450.
- `test/host/test_homing_nonblocking.cpp` (3 cases): BACKOFF_SETTLE_WAIT không block (5 ticks trong 30ms), WARMUP_SETTLE tương tự, VERIFY_SETTLE.
- Giữ 4 suites cũ: kinematics 3280, joint_logic, work_plane, homing_logic.

### 5.2 Hardware manual
- Đo ISR latency: `isrTime` vs `pollTime` delta <10ms, ISR <5µs.
- Homing J1→J4 liên tiếp: không WDT reset, log `BACKOFF_SETTLE_WAIT` đúng 30ms.
- Pre-flight: move ngoài reach → 400, trong reach → move mượt.

---

## 6. File Changes

| File | Action | Lines |
|---|---|---|
| `src/safety_manager.h/.cpp` | NEW | ~120 |
| `src/trajectory_validator.h/.cpp` | NEW | ~90 |
| `src/endstop.h/.cpp` | MODIFY | ISR minimal, add pending/isrTime |
| `src/homing.h/.cpp` | MODIFY | +2 states, remove delay |
| `src/motor.h/.cpp` | MODIFY | replace g_* with safety ref |
| `src/arm.h/.cpp` | MODIFY | own SafetyManager, inject, HTTP 400 mapping |
| `src/planner.h/.cpp` | MODIFY | validator call |
| `src/config.h` | MODIFY | add `HOMING_BACKOFF_SETTLE_MS=30` |
| `test/host/test_safety_manager.cpp` | NEW | ~150 |
| `test/host/test_trajectory_validator.cpp` | NEW | ~150 |
| `test/host/test_homing_nonblocking.cpp` | NEW | ~80 |

No change: `kinematics.*`, `joint_model.*`, `sensor.*`, `wifi_manager.*`, `nvs_store.*`, DH/gears/pins.

---

## 7. Risks & Mitigations

| Risk | Mitigation |
|---|---|
| ISR bỏ `gpio_get_level` → EMI HIGH latch oan | poll check `digitalRead==LOW` trước khi latch |
| Homing thêm state sót transition | host test + giữ timeout cũ |
| Xóa globals sót caller | `grep g_emergencyStop` + build fail |
| Validator B lọt điểm giữa 2 mids | per-segment guard vẫn bắt → FAULT an toàn |

---

## 8. Build Gate (AGENTS.md §2)

- `pio run` → SUCCESS (functional-first, no RAM gate)
- `tools/run_host_tests.sh` → 7 suites ALL PASSED
- `docs/SYSTEM_OVERVIEW.html` + `docs/IMPLEMENTATION_LOG.md` cập nhật cùng commit (AGENTS.md §3)

---

## 9. Future Sprints

- **Sprint 1 (P1 #5-9):** NVS Config Store, Structured Logging, Memory Monitoring, I2C Fault Recovery, Step Timer Optimize
- **Sprint 2 (P2 #10-13):** DI/ RobotContext, HAL, LittleFS Frontend, WebSocket
- **Sprint 3:** SKIPPED per owner decision (P3 #14-21)

---

## 10. Open Questions Resolved

- Q1 Scope: Sprint-by-sprint, start Sprint 0, skip Sprint 3 → resolved 2026-08-31
- Q2 Architecture: Clean ( xóa globals) → resolved
- Q3 Pre-flight: Option B lightweight → resolved
- Q4 Real-time: ISR <3µs poll, delay→state, giữ timeout → resolved
- Q5 Gate: functional-first → resolved

