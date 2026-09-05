# Kế hoạch refactor đầy đủ — NEMA-6AXIS-ARM-CONTROLLER

Generated: 2026-09-01

## Mục tiêu và nguyên tắc

Mục tiêu là firmware điều khiển 6 trục an toàn, có hành vi xác định và có thể kiểm chứng. Mọi thay đổi giữ nguyên `docs/ARM_GEOMETRY.md` là nguồn hình học duy nhất; J3 chỉ dùng dải 0°…+90°; WiFi credential và homing calibration chỉ ở NVS; driver luôn enabled.

Một phase chỉ được chuyển tiếp sau `pio run` thành công, host regression phù hợp pass, và commissioning hardware của phase có bằng chứng log/scope. Không dùng host test để tuyên bố đã đo được timing hoặc cơ khí thật.

## Trạng thái đã hoàn thành

| Ưu tiên | Hạng mục | Evidence |
|---|---|---|
| P0 | J3 soft-limit 0°…+90° | config chung + host contract |
| P0 | Endstop cắt STEP trong ISR, fault latch sau debounce 50 ms | `Endstops` + SafetyManager + ISR contract |
| P0 | STOP_ALL preempt queue | mailbox atomic + queue purge |
| P1 | Calibration homing runtime/NVS | validation ±1, 0.5–2.0×; restore trước home |
| P1 | Drift watchdog | >25° trong 3×500ms latch FAULT |
| P1 | WorkPlane preflight | validator cùng UCS semantics với Planner |
| P2 | API mutation | stop/home/sethome/clearcalib chuyển POST |
| P2 | Sensor read integrity | atomic fallback snapshot khi dataMutex timeout |

## P2 — Hoàn thiện ranh giới điều khiển và dữ liệu

1. **Xác thực web và AP provisioning**
   - Owner chọn một UX: mật khẩu AP theo MAC in Serial, mật khẩu owner-provided trong NVS, hoặc Basic Auth khi STA.
   - Bỏ `DEFAULT_AP_PASS` có giá trị dự đoán được; chỉ endpoint status/asset được read-only theo policy đã chọn.
   - Gate: test HTTP method/auth matrix; kết nối AP, provision và STA fallback trên board.

2. **Schema command/API**
   - Chuẩn hóa parse `float` (finite/range/feed) thành helper thuần C++; endpoint chỉ map request → `ArmCommand`.
   - Bổ sung versioned response/error codes để SPA không dựa chuỗi tự do.
   - Gate: host tests bad input, queue-full, fault/busy matrix.

3. **NVS integrity và observability**
   - Bổ sung schema version và migration-forward; commit marker cho nhóm home+calibration nếu owner cần atomic pair khi mất nguồn.
   - Xuất trạng thái calibration validated/restored trong status JSON, không lộ WiFi password.
   - Gate: power-cut simulation/record corrupt fixtures; clear calibration and restore tests.

## P3 — Đồng bộ pulse thời gian thực (thay đổi kiến trúc có rủi ro cao)

Vấn đề còn lại: mỗi motor hiện có timer riêng; Planner scale interval độc lập nên các trục không có master tick/DDA chung. Đây là phase riêng vì sai pulse có thể gây chuyển động thực.

1. Đo baseline bằng logic analyzer: jitter STEP, độ lệch điểm kết thúc 6 trục, feed và CPU load ở jog/draw/homing.
2. Tạo `MotionScheduler` single owner trên core 1: profile segment immutable, master tick hardware timer, accumulator Bresenham/DDA cho 6 axes, không UART/I2C/heap trong callback.
3. Chạy scheduler shadow mode chỉ đếm pulse và đối chiếu với planner hiện hữu; thêm host deterministic tests (ratio, stop, profile endpoint).
4. Chuyển JOG rồi Cartesian từng phần; homing có mode velocity riêng. Giữ chuỗi ISR endstop → `stopFromISR` và STOP mailbox nguyên vẹn.
5. Gate hardware: scope tất cả STEP, stop latency, 10k line segments, max feed, nóng driver/motor, repeatability endpoint. Chỉ bỏ timers cũ khi số đo đạt tiêu chí owner phê duyệt.

## P4 — Robustness và vận hành

1. Telemetry: heap còn lại, stack high-water từng task, WDT reset reason, I2C/UART timeout counters và calibration source.
2. Fault policy: phân loại sensor stale, TMC UART degraded, NVS write failure; fault codes rõ ràng trong API/UI.
3. Commissioning checklist có chữ ký kết quả: endstop, J3 0/+90, home restore, calibration NVS, drift, WorkPlane, E-stop, STOP_ALL queue full.

## Trình tự thực hiện

P2.1 authentication/provisioning (cần lựa chọn owner) → P2.2 schema API → P2.3 NVS versioning → P3 measurement/shadow DDA → P3 cutover từng mode → P4 telemetry/commissioning.

Không tự thay DH, gear ratio, theta offset hoặc model J3 trong các phase này.
