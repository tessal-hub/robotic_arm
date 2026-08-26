# Implementation Log — Timeline

Project: NEMA-6AXIS-ARM-CONTROLLER (ESP32-S3, PlatformIO, Arduino framework)
Goal: Web app (STA+AP) · Homing (stallguard+encoder+endstop+steps) · Joint jog (FK) · IK control → draw line/circle on paper.

Decisions locked with owner (see plan): 4x TMC2209 UART addr 0-3 (no DIR pin, direction via `shaft()`), J5/J6 A4988 STEP+DIR, no EN pins anywhere (drivers always on), J1/J2 home at travel center via min-stop + half-stroke move, J3 home at min stop + backoff, J4 stallguard bump home, J5/J6 manual Set-Home. Pen lift = Z-raise. IK = closed-form pen-down, theta6=0. WebServer polling 250ms. secrets.h for STA creds.

---

## 2026-08-26 — Session start

### DESIGN CHANGE (owner request): NVS persistence
- WiFi credentials KHÔNG dùng secrets.h nữa -> lưu NVS, cấp phát qua trang provisioning ở chế độ AP (`/wifi`). Mang robot sang WiFi mới không cần compile/flash lại.
- Vị trí zero từng khớp cũng lưu NVS. Khi bật nguồn: đọc góc RAW tuyệt đối của AS5600 (single-turn), so với raw đã lưu lúc home -> khôi phục vị trí mà KHÔNG cần home lại. Chỉ home khi muốn re-calib.
- Hệ quả: xóa src/secrets.template.h; JointModel thêm restore/persist; WifiManager thêm provision().
- Giới hạn đã ghi chú: khôi phục chính xác trong phạm vi ±180° của encoder (đủ cho J1-J5; J6 ±360 chỉ khôi phục mod 360 — vô hại vì roll).

### P0 — Config & pins (DONE)
- [x] `src/config.h`: UART1 RX=15/TX=16; STEP: 1/2/41/42/38/40; DIR: 39/47 (J5/J6); TMC J1-J4 addr 0b00-0b11 không DIR; thêm AXIS_STEP_SIGN/AXIS_ENC_SIGN; HOME_BACKOFF_DEG, STALL_SG_LEVEL, PEN_LIFT_MM, DRAW_FEED_MM_S, DRAW_SEGMENT_MM, PLANNER_QUEUE_DEPTH; sửa `J3_MIN_LIMIT = 0.0f`.
- [x] Motor: thêm `absSteps` (atomic int64) cập nhật trong step timer cả 2 nhánh bounded/continuous; API getAbsoluteSteps/setAbsoluteSteps; toJson có absSteps; getter isTmc().
- [x] Module mới: `endstop.*` (ISR FALLING + debounce 50ms + latch, abort stopFromISR), `joint_model.*` (deg<->step theo gear/microstep, setHomeHere, drift watchdog), `homing.*` (FSM APPROACH->BACKOFF->CENTERING->SETREF, stallguard+endstop fusion, timeout), `arm.*` (arbiter + motion task core1 @100Hz, queue lệnh, FAULT khi endstop ngoài homing, statusJson tổng hợp), `wifi_manager.*` v1.

### Assembly checklist (owner to fill physical values)
- [ ] MS1/MS2 address jumpers: J1=0b00, J2=0b01, J3=0b10, J4=0b11 (all four TMC2209 share Serial1).
- [ ] A4988 J5/J6: measure Vref trimpots, record coil current here: J5 ____ A, J6 ____ A (I_max ≈ Vref/0.8 @ Rs=0.1Ω).
- [ ] Verify each axis step sign vs encoder sign at commissioning; flip AXIS_STEP_SIGN/AXIS_ENC_SIGN if inverted.
- [ ] StallGuard thresholds tuned per joint after first homing attempt (record values in config.h).
- [ ] Xác nhận encoder gắn trực tiếp trên trục ra khớp (1:1) để khôi phục NVS đúng.

### Next
- [x] nvs_store.* (creds + joint homes)
- [x] WifiManager: load NVS -> STA -> AP fallback; provision() lưu NVS
- [x] JointModel: restoreFromNVS() (encoder raw delta) + persist trong setHomeHere
- [x] web_server.* (tabs: Dashboard/Joints/Homing/WiFi) + main.cpp wiring

---

## 2026-08-26 — P0/P1 hoàn tất, build firmware xanh lần 1

### Build gate
- `pio run` → **SUCCESS**. RAM 15.0% (49160 B), Flash 24.4% (815993 B).
- Fix dọc đường: `AXIS_STEP_SIGN` dùng NUM_MOTORS trước khi khai báo; `endstop.h` thứ tự khai báo Channel; Arduino dùng `<ESPmDNS.h>`; chuỗi NS trong nvs_store.cpp; DIR_PIN_0..3 thay bằng PIN_UNSET.

### P2 — Kinematics (DONE)
- Tái tạo ground truth vì `fk_verify.py` KHÔNG tồn tại trong repo (docs chỉ dẫn file chưa từng commit). Dùng script Python nội bộ (/tmp) xác định quy ước Modified DH và rút ra IK đóng.
- **FK xác nhận đúng docs/ARM_GEOMETRY.md mục 5**: home wrist center = (126.000, 0, 365.000); TCP = (146, 0, 365), tool z-axis nằm ngang +x.
- **IK pen-down closed-form** (đã verify số học 273/273 pose random trước khi port):
  - C = TCP + (0,0,D6=20); t1 = atan2(Cy,Cx)
  - Phẳng 2 khâu: (r,h) = 138∠φ₂ + 153.69∠φψ với φ₂=−t2, φψ=−(q23+δ), δ=atan2(126,88)=55.06°
  - β law-of-cosines, γ=atan2(h,r), nhánh φ₂=γ±β chọn theo J3∈[0,90]
  - Wrist: t4=0 giữ mặt phẳng tay, t5_DH=−q23 (bút thẳng đứng), t6=0
- File: `src/kinematics.h/.cpp` (thuần C++, host-test được), test host `test/kinematics/test_kinematics.cpp`, runner `tools/run_kin_tests.sh`.
- **Lỗi bắt được nhờ test**: FK thiếu Tz(D6_TOOL=20); nhầm đơn vị độ/radian ở DELTA_WRIST.
- Lý do không dùng `pio test -e native`: PIO Core 6.1.19 lỗi "Nothing to build" với env native dự án này → chạy g++ trực tiếp qua script.
- Kết quả: FK home chính xác tuyệt đối; IK roundtrip **3280/3280 OK, 0 fail**, từ chối đúng các điểm ngoài vùng với.

### P2/P3 — Planner + vẽ (DONE)
- `src/planner.h/.cpp`: bộ sinh waypoint on-the-fly (LINE/POINT/CIRCLE CCW đầy vòng), không buffer lớn. Mỗi segment ~1mm: IK → di chuyển ĐỒNG THỜI 6 trục thời gian bằng nhau (trục chủ đạo đặt interval = T/steps_max, trục khác scale). Feed mm/s chuẩn hoá: interval_us = 1e6·segLen/(feed·steps_max) — bản đầu bị nghịch đảo/thiếu 1e6, đã fix.
- Chuỗi an toàn nét vẽ: LIFTING (+PEN_LIFT_MM) → TRAVELING (ngang ở độ cao nâng) → DROPPING (xuống z giấy) → DRAWING → FINISHED_LIFT. POINT kết thúc tại đích.
- Gate an toàn: MOVE/DRAW chỉ chạy khi J1-J4 đã homed (allPositioningHomed); ngoài vùng với → huỷ job + log.
- arm.*: thêm lệnh MOVE_CART/DRAW_LINE/DRAW_CIRCLE (p[8] tham số), STOP_ALL huỷ planner, statusJson có "pose" (FK realtime) + "planner".
- web: tabs Cartesian (input X/Y/Z/feed, hiển thị pose live) + Draw (form line/circle, canvas preview top-view, START/ABORT).

### Build gate cuối
- `pio run` → **SUCCESS**. RAM 15.0% (49268 B), Flash 24.9% (833077 B).
- Kinematics host tests → **ALL PASSED** (3280 roundtrip).

### Việc còn lại khi có hardware (commissioning)
1. Nạp firmware, xác nhận UART 4 driver (log `[TMC2209 OK]... Ver 0x21` cho cả 4 addr).
2. Flip AXIS_STEP_SIGN/AXIS_ENC_SIGN nếu trục nào ngược chiều encoder (jog mù khi chưa home để kiểm tra).
3. Tune STALL_SG_LEVEL theo tải thực (xem SG_RESULT trên /api/status).
4. Home All → jog → Move Cartesian → Draw circle thử. Đo Vref A4988 J5/J6 rồi Set-Home chúng.
5. Xác nhận offset θ4-θ6 (docs TODO) ảnh hưởng độ chính xác nét vẽ.

---


