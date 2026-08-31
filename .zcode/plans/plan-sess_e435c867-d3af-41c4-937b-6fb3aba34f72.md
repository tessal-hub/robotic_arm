# Nâng cấp kiến trúc homing J4 + hoàn thiện homing J1–J3

## Chẩn đoán hiện trạng
Cả J1–J4 đã chạy chung scan path, nhưng còn 4 khiếm khuyết kiến trúc:
1. **Chỉ 1 tốc độ quét** — chạm cữ ở tốc độ fast (J4 đập cữ + trượt bước; J1–J3 mất chính xác do bounce cơ khí + độ trễ dừng).
2. **Phát hiện stall J4 yếu** — 1 cửa sổ 150 bước/Δenc<0.6° quyết định ngay (1 glitch I2C = dừng sai); `encStallCount_` khai báo nhưng chưa dùng.
3. **Không verify sau căn tâm** (home sai không phát hiện), **không retry** (1 glitch = fail toàn chuỗi).
4. **~350 dòng dead code legacy** (APPROACH/BACKOFF/REAPPROACH) + comment header `homing.h` lỗi thời.

## Kiến trúc mới — FSM scan 2 tốc độ, áp dụng chung J1–J4

Mỗi khớp chạy chuỗi: `SAFE_MODE → WARMUP → SCAN_MIN(fast) → SCAN_BACKOFF → SCAN_SLOW → SCAN_MAX(fast) → SCAN_BACKOFF → SCAN_SLOW → CROSSCHECK → CENTERING → VERIFY → SETREF`

- **Fast seek** (giữ tốc độ hiện tại): tìm cữ thô. Sai glitch ở pha này tự hồi phục vì slow phase tiếp cận lại đúng cữ đó.
- **Backoff + Slow re-approach** (mới): lùi `HOME_BACKOFF_DEG + 0.5°`, chạm lại ở tốc độ chậm `HOMING_SLOW_SCAN_INTERVAL_US = 3000µs`. Điểm chạm chậm là mốc chính xác:
  - J1–J3: endstop latch ISR (chính xác ±1 bước).
  - J4: stall step-lag; **ghi nhận vị trí = `lastCheckSteps_`** (đầu cửa sổ stall) thay vì vị trí hiện tại → triệt tiêu overshoot của cửa sổ phát hiện; StallGuard giữ làm tín hiệu phụ (kém tin cậy ở tốc độ thấp).
- **VERIFY (mới)**: sau CENTERING + settle 350ms, đối chiếu `rawEncoder()` (góc tích lũy AS5600, độc lập với steps) với vị trí mong đợi — J1/J2/J4: `encCenterRaw_` (tâm đo được); J3: `enc_min + encSign × offsetDeg`. Tolerance = 0.5° + 1% nửa hành trình. FAIL → retry khớp.
- **Retry (mới)**: `HOMING_MAX_ATTEMPTS = 2` — khi 1 attempt fail (timeout/stall sai/verify fail) thì `beginScan()` lại khớp đó thay vì hủy cả chuỗi; hết lượt mới abort.
- **encStallCount_**: pha FAST yêu cầu 2 cửa sổ liên tiếp (chống glitch); pha SLOW 1 cửa sổ + bù vị trí.

## Thay đổi file
1. **`src/config.h`**: thêm `HOMING_SLOW_SCAN_INTERVAL_US = 3000`, `HOMING_MAX_ATTEMPTS = 2` (+ hằng tolerance verify).
2. **`src/homing.h`**: enum mới (`SCAN_MIN, SCAN_MAX, SCAN_BACKOFF, SCAN_SLOW, CENTERING, VERIFY, DONE` — bỏ `APPROACH/REAPPROACH`); thêm state `attempt_`, `slowTowardFirst_`, context cữ đang tiếp cận; xoá method legacy; cập nhật comment class.
3. **`src/homing.cpp`**: viết lại tickScan theo chuỗi trên (dùng lại phần detect đã hoạt động: endstop latch → SG → step-lag, minSpan arm, crosscheck, centering); xoá sạch đường legacy; retry trong `finishJoint()`; VERIFY mới. **API công khai giữ nguyên** (`startAll/startAxis/cancel/tick/toJson`) → `arm.cpp`/web không phải sửa.
4. **`test/host/test_homing_logic.cpp`**: thêm test thuần toán — mapping phase↔tên JSON, homeAtMinOffset, công thức tâm/offset J3, công thức tolerance verify, số attempt.
5. **`docs/SYSTEM_OVERVIEW.html`** (AGENTS §3a): cập nhật sơ đồ FSM homing tab "FSM & Luồng dữ liệu", invariant mới (2 tốc độ + verify + retry) tab "An toàn", footer ngày.
6. **`docs/IMPLEMENTATION_LOG.md`** (AGENTS §3b): append entry mới cuối file.
7. **`docs/HW_REGRESSION_CHECKLIST.md`**: thêm mục kiểm chứng slow re-approach / verify / retry trên hardware.

## Build gate
- `pio run` → SUCCESS (0 errors/warnings)
- `tools/run_host_tests.sh` → ALL 4 HOST TESTS PASSED

## Rủi ro đã tính
- Không đổi DH/kinematics — không cần `run_kin_tests.sh` (vẫn chạy để chắc chắn).
- J4 slow contact dựa step-lag encoder (speed-independent), không phụ thuộc StallGuard ở tốc độ thấp.
- Overshoot cửa sổ stall đối xứng 2 đầu → tâm cơ khí không lệch; J3 dùng endstop latch nên không chịu overshoot.
- Tên phase JSON thay đổi (thêm `scan_backoff/scan_slow/verify`) — chỉ là chuỗi hiển thị trong status API.