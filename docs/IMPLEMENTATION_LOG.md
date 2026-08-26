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

## 2026-08-26 — Kiểm chứng mô hình động học (phản hồi review của owner)

### Thắc mắc
Owner hỏi: δ dùng cạnh 126mm (atan2(126,88)=55.06°) trong khi bản kinematics cũ của owner dùng
φ=atan2(110,88)=51.34° — nghi ngờ việc gộp offset ngang 16mm vào d4 làm thay đổi cấu trúc DH.

### Bằng chứng trong repo → mô hình hiện tại là ĐÚNG theo tài liệu gốc
1. `ARM_GEOMETRY.md:35` — "**126mm tổng cộng, nối tiếp nhau trên cùng một trục** (không phải hai
   hướng riêng biệt)" — khẳng định thẳng hàng 16+110, ghi rõ là đã xác nhận thực tế.
2. `ARM_GEOMETRY.md:66` — bảng DH dòng 4: **d4 = 126 (=16+110)**.
3. `ARM_GEOMETRY.md:97-104` — mục 5: đo đạc vật lý xác nhận wrist center home = (126, 0, 365),
   "khớp chính xác tuyệt đối... không cần nghi ngờ hay đo lại". FK ma trận 4×4 trong
   `kinematics.cpp` tái tạo đúng (126.000, 0.000, 365.000).
4. `config.h:196` — `DH_D4_MM = 126.0f // (16mm + 110mm)`.
5. Công thức đóng IK (L=153.69=√(88²+126²), δ=atan2(126,88)) được **rút ra từ bảng DH này** và
   đối chiếu chéo với implementation ma trận độc lập: 3280/3280 pose khớp <0.5mm.
   → Không có lẫn lộn 110/126 bên trong firmware: toàn bộ hệ thống nhất quán Model A.

### Giới hạn được thừa nhận
Roundtrip chỉ chứng minh closed-form ≡ bảng DH. Nó KHÔNG chứng minh bảng DH ≡ cơ khí thật —
điều đó phụ thuộc vào việc 16mm có thực sự thẳng hàng với 110mm hay bị gắt L.

### Định lượng rủi ro nếu sai mô hình (script so sánh phẳng)
- Biến thể "gắt L" (16 vuông góc rồi 110): lệch tới **~155mm**, kể cả vị trí home.
- Biến thể tinh vi hơn (16 vuông góc nhưng trùng phương ngang lúc home — thỏa mãn kiểm tra
  (126,0,365)): khớp ở home NHƯNG lệch tăng dần theo góc khuỷu, cỡ **vài mm → ~16mm** khi
  e3 gập sâu. Đây mới là kẻ ngụy trang nguy hiểm cho nét vẽ.

### Thí nghiệm phân biệt (chỉ cần 1 phép đo, làm ngay khi có hardware)
1. Home all. Đặt bút chạm giấy tại home, đánh dấu điểm mực M0.
2. Giữ J1/J2 khóa, jog J3 đúng +30°. Bút sẽ nhấc khỏi giấy — đo khoảng cách ngang thực tế
   mà đầu bút dịch chuyển, so với `/api/status` "pose" trước/sau.
3. Khớp ≤ ~2mm → Model A đúng, vẽ được ngay. Lệch hệ thống lớn hơn và tỉ lệ với e3 → báo
   lại số đo, đổi sang biến thể phù hợp (thay đổi chỉ nằm trong `kinematics.cpp`: hằng số
   L/δ hoặc thêm 1 khâu offset + cập nhật test — ~30 phút).
4. Ghi kết quả đo tại đây: ____ mm (thực) vs ____ mm (firmware dự đoán).

### Cảnh báo
Hằng số cũ atan2(110,88)=51.34° thuộc mô hình khác (d4=110). KHÔNG import lại vào firmware
hiện tại trừ khi thí nghiệm trên buộc phải đổi mô hình.

---

## 2026-08-26 — Tài liệu sống: SYSTEM_OVERVIEW.html + AGENTS.md

### Việc đã làm
- What: thêm 2 file mới:
  - `docs/SYSTEM_OVERVIEW.html` — bản đồ hệ thống sống tự chứa (8 tab: Tổng quan / Phần cứng /
    RTOS & Tasks / Modules / FSM & Luồng / Kinematics / REST API / An toàn & TODO), tiếng Việt,
    offline, nội dung rút trực tiếp từ code (pin map config.h, task table, 13 module cards,
    FSM homing/planner/arm-mode, SVG hình học + bảng DH Modified Craig, API reference, invariants).
  - `AGENTS.md` (root) — hợp đồng bảo trì cho AI agent: lệnh build/test gate, giao thức bắt buộc
    cập nhật SYSTEM_OVERVIEW.html + IMPLEMENTATION_LOG.md sau MỖI thay đổi, quy tắc bất biến
    (DH constants, NVS-only creds, web-handler không đụng hardware, uartMutex, chuỗi an toàn...).
- Why: owner yêu cầu tài liệu demo toàn bộ logic/core/task + cơ chế giữ tài liệu đồng bộ với code.
- How: HTML thuần CSS/JS một file (không CDN) để mở offline; AGENTS.md đặt ở root theo convention
  để agent tự đọc khi vào repo. Nội dung chỉ ghi thứ code thật sự làm (đối chiếu từng module).

### Build gate
- Chỉ thay đổi docs — firmware không đụng tới.
- Validate: HTML parse OK (balanced tags), inline JS syntax OK (node --check), 46 KB.

### Việc còn lại
- Owner review nội dung 2 file; duy trì theo giao thức trong AGENTS.md từ giờ.

---

## 2026-08-26 — AGENTS.md: thêm mục tri thức kỹ thuật rút từ skill/

### Việc đã làm
- What: AGENTS.md thêm mục mới "5. Tri thức kỹ thuật cô đọng (rút từ skill/)" — tinh gọn các
  nguyên tắc áp dụng được cho firmware từ skill/embedded-systems, skill/embedded-agent-skills
  (ESP32 GPIO) và skill/cpp-pro; chia 5 nhóm: ISR & thời gian thực, ESP32-S3 GPIO, FreeRTOS,
  C++, giao tiếp bus. Đổi số mục cũ: Quy trình 5→6, Cấu trúc repo 6→7.
- Why: owner yêu cầu nhúng tri thức tham khảo vào hợp đồng bảo trì để agent không phải đọc
  toàn bộ thư viện skill trước mỗi thay đổi.
- How: chỉ giữ những quy tắc có ảnh hưởng trực tiếp đến code hiện tại (ISR pattern khớp
  Endstops::isrHandler, bảng cấm GPIO khớp config.h, atomic memory order khớp motor.*, ...),
  mỗi nhóm dẫn mẫu chuẩn trong repo làm điểm tham chiếu.

### Build gate
- Docs-only — không đụng firmware. Kiểm tra cấu trúc heading AGENTS.md §1–§7 tuần tự đúng.

---

## 2026-08-26 — Refine UI: web app nhúng + SYSTEM_OVERVIEW.html

### Việc đã làm
- What: tinh chỉnh UI cả 2 bề mặt, không đổi REST API:
  - `src/web_server.cpp` (INDEX_HTML PROGMEM):
    - FIX BUG: nút jog nhúng giá trị stepSize cũ vào onclick lúc buildCards() — đổi bước jog
      không có tác dụng đến khi reload. Giờ jog(axis,dir) đọc stepSize toàn cục tại thời điểm click.
    - Nút hành động (home/move/draw/sethome) tự disable khi `busy` hoặc mode `fault` (class need-idle).
    - Toast phản hồi mọi lệnh (OK/busy/queue full/lỗi mạng) qua api()/post() gom chung; bỏ alert().
    - Chỉ báo MẤT KẾT NỐI khi poll /api/status lỗi ≥3 lần liên tiếp; tự hồi phục khi kết nối lại.
    - Thanh progress homing fake 50% → chuỗi chip J1→J4 sáng theo axis đang home + phase (trung thực).
    - Clear Calib thêm confirm() (hành động phá huỷ data NVS).
    - Tab WiFi: card hiển thị kết nối hiện tại (mode/IP/SSID/RSSI từ status).
    - E-STOP to hơn + pulse nhẹ (tôn trọng prefers-reduced-motion); canvas Draw thêm marker home (146,0);
      input số thêm inputmode=decimal cho bàn phím mobile; focus-visible cho tab.
  - `docs/SYSTEM_OVERVIEW.html`: tab deep-link qua location.hash (#kin, #api...), bảng bọc wrapper
    cuộn ngang mobile (JS tự bọc), print stylesheet (mở hết pane, nền trắng), hover highlight dòng
    bảng, nút scroll-to-top.
- Why: owner yêu cầu "refine UI related things" cho cả hai bề mặt.
- How: giữ nguyên kiến trúc polling 300ms + PROGMEM single-file; mọi feedback client-side đều dựa
  trên dữ liệu /api/status sẵn có, không thêm endpoint.

### Build gate
- `pio run` → **SUCCESS**. RAM 15.0% (49,268 B), Flash 25.0% (836,245 B) (+3.2 KB so với trước).
- HTML parse + JS syntax check pass cho cả 2 file (python html.parser + node --check).

---



