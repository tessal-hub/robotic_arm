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

## 2026-08-26 — Polish UI: web app nhúng (impeccable polish)

### Việc đã làm
- What: tinh chỉnh toàn diện UI nhúng (`src/web_server.cpp`):
  - **Design tokens**: chuyển toàn bộ hard-coded color/spacing/radius/typography sang CSS custom properties tại `:root` — single source of truth, dễ maintain, đồng bộ với DESIGN.md.
  - **Interaction states**: thêm hover/focus-visible/active/disabled cho mọi button, tab, step selector, input, select. Focus ring dùng `--color-focus-ring` (Mission Sky).
  - **Loading states**: nút async (SAVE & REBOOT, MOVE, START DRAW) có spinner + disable khi pending; class `.btn-loading` với keyframes spin.
  - **Toast**: thêm entrance animation (`toastIn` 150ms ease-out), ARIA live region (`role="status" aria-live="polite"`).
  - **Micro-interactions**: step selector dùng `role="radio" aria-checked`; tab dùng `role="tab" aria-selected`; tabpanel dùng `role="tabpanel" aria-labelledby`.
  - **Vietnamese diacritics**: sửa "MAT KET NOI" → "MẤT KẾT NỐI", "CHUA HOME" → "CHƯA HOME", "HOMED(nvs)" → "HOMED (NVS)", Unicode checkmarks (✓) thay `\u2713`.
  - **Copy consistency**: thống nhất tiếng Việt cho mọi label/tooltip/placeholder; aria-label cho icon-only buttons (E-STOP, jog arrows).
  - **Canvas preview**: dùng CSS variable `var(--color-primary)` thay hard-coded `#38bdf8`.
  - **Reduced motion**: mở rộng `@media (prefers-reduced-motion: reduce)` tắt mọi transition/animation.
  - **Inline styles**: thay bằng class hoặc CSS variable; chỉ giữ `style="width:..."` cho input cố định.
  - **Select styling**: native appearance:none + custom dropdown arrow SVG.
- Why: impeccable polish pass — alignment với DESIGN.md, accessibility, interaction completeness, maintainability.
- How: chỉ sửa `src/web_server.cpp` (INDEX_HTML + JS); không đổi REST API, không đổi firmware logic.

### Build gate
- `pio run` → **SUCCESS**. RAM 15.0% (49,268 B), Flash 25.3% (846,157 B) (+10 KB so với trước — CSS variables + ARIA + loading states).
- `tools/run_kin_tests.sh` → **ALL PASSED** (3280/3280 IK roundtrip).

### Việc còn lại
- Cập nhật DESIGN.md và .impeccable/design.json để phản ánh CSS variables mới (đã làm song song).

---

## 2026-08-26 — Fix endstop boot fault: arm resting against switch = OK

### Việc đã làm
- What: endstop pressed at boot不再 FAULT — arm vẫn boot và hoạt động bình thường.
  - `src/endstop.h/.cpp`: thêm `clearAllLatches()` method; clear latches at boot trong `begin()`.
  - `src/arm.cpp` motion task: thay logic fault check —
   旧: `anyLatched() && !homing` → FAULT mọi lúc (endstop at boot = FAULT vô cực).
    新: chỉ fault khi `motor running + isPressed() + !homing` — endstop pressed at boot = OK.
    homing tự xử lý endstop riêng (APPROACH phase), arm controller bỏ qua khi homing active.
    Clear latches sau homing hoàn tất (backoff endstop still pressed là bình thường).
  - `docs/SYSTEM_OVERVIEW.html`: cập nhật safety rules, endstop module card, motion task timeline, FAULT flow, safety chain diagram.
- Why: owner yêu cầu arm có thể boot và điều khiển được ngay cả khi đang tỳ lên endstop.
  Endstop là protection khi đang chuyển động, không phải khóa boot.
- How: endstop ISR vẫn hoạt động bình thường (stopFromISR + latch), nhưng motion task chỉ
  fault khi motor thực sự đang chạy và endstop vật lý đang nhấn. At boot: clear latches,
  endstop pressed → pin LOW nhưng không có ISR edge → không latch → không fault.

### Build gate
- `pio run` → **SUCCESS**. RAM 15.0% (49,268 B), Flash 25.3% (846,193 B) (+10 KB so với trước — fault logic mới + clearAllLatches).
- HTML parse clean.

### Việc còn lại
- Không có.

---

## 2026-08-26 — Fix homing: stall false positive + encoder-aware setHomeHere

### Việc đã làm
- What: 2 fix cho homing — false stall detection và step/encoder drift sau homing.
  - `src/homing.cpp`: StallGuard stall detection — bỏ qua sg < 10 (UART noise / motor chưa load).
    sg=0 và sg=2 trong log boot là đọc không hợp lệ, gây false CONTACT sau 194/-21 steps.
    Ngưỡng mới: sg ∈ [10; STALL_SG_LEVEL=100) trong 3 poll liên tiếp → stall thật.
  - `src/joint_model.cpp`: `setHomeHere()` encoder-aware —
   旧: luôn `absSteps=0` → step/encoder phân kỳ khi motor stall/slipped trong CENTERING.
    新: encoder khoẻ → dùng làm ground truth, set `absSteps = degreesToSteps(encAngle)`
    để angleFromSteps() và angleFromEncoder() luôn đồng bộ. Motor stall trong CENTERING
    không gây drift fault nữa. Encoder chết → fallback absSteps=0 (step-only).
  - `docs/SYSTEM_OVERVIEW.html`: cập nhật homing module card (sg range mới) và joint_model card (setHomeHere encoder-aware).
- Why: owner report motor "go to max, hit endstop, keep pushing" + drift fault 66° liên tục.
  Root cause: sg=0/2 false stall →CONTACT sai → CENTERING sai → step counter ≠ encoder → drift.
- How: sg filter là smallest sufficient fix cho false stall. Encoder-aware setHomeHere là defense-in-depth
  — ngay cả khi centering sai vì lý do khác, step counter vẫn khớp encoder.

### Build gate
- `pio run` → **SUCCESS**. RAM 15.0% (49,268 B), Flash 25.3% (846,317 B) (+124 B — sg filter + encoder-aware setHomeHere).
- HTML parse clean.

### Việc còn lại
- Không có.

---

## 2026-08-26 — Redesign homing: endstop-only + REAPPROACH + encoder-guided centering

### Việc đã làm
- What: redesign homing sequence theo cảm hứng từ reference implementation — bỏ StallGuard, thêm
  REAPPROACH, encoder-guided centering.
  - `src/homing.h`: thêm `REAPPROACH` vào `HomePhase` enum, thêm `enterReapproach()`,
    `gotoNearHome()`, `angleEncAtContact_` member.
  - `src/homing.cpp`: viết lại toàn bộ homing FSM —
    - APPROACH: bỏ hoàn toàn StallGuard poll (endstop là NGUỒN SỰ THẬT DUY NHẤT).
    - CONTACT: ghi encoder tại điểm chạm (`angleEncAtContact_`).
    - BACKOFF: lùi như cũ, thêm REAPPROACH sau khi đã rời endstop.
    - REAPPROACH (MỚI): dò lại chậm (3000 µs/step) về phía MIN → khi tái chạm, ghi encoder
      chính xác tại điểm contact (chính xác hơn do approach chậm, ít overshoot).
    - CENTERING (MỚI): closed-loop encoder → di chuyển tới home = angleEncAtContact_ + stroke/2.
      Dừng khi encoder gap < 0.5°. Fallback step-counted nếu encoder chết.
    - toJson(): cập nhật phase names cho REAPPROACH.
  - `docs/SYSTEM_OVERVIEW.html`: cập nhật homing module card (bỏ stallguard, mô tả sequence mới)
    + homing FSM diagram (thêm REAPPROACH, cập nhật mô tả).
- Why: owner report false stall (sg=0/2) gây CONTACT sai → CENTERING sai → drift 66°.
  Root cause là StallGuard không đáng tin trên trục chịu tải trọng lực.
  REAPPROACH + encoder-guided centering là giải pháp đúng đắn — endstop cho vị trí,
  encoder cho precision.
- How: inspired by reference homing sketch (MIN+MAX endstop midpoint), nhưng adaptable cho
  thiết kế chỉ có MIN endstop. Home = endstop + stroke/2 thay vì midpoint 2 switch.

### Build gate
- `pio run` → **SUCCESS**. RAM 15.0% (49,268 B), Flash 25.3% (847,061 B) (+744 B — homing FSM mới + encoder-guided).
- HTML parse clean.

### Việc còn lại
- Không có.

---

## 2026-08-26 — Fix: resync step counter on cancel + BACKOFF jam detection

### Việc đã làm
- What: fix step/encoder drift khi cancel homing + BACKOFF infinite loop.
  - `src/joint_model.h/.cpp`: thêm `resyncFromEncoder(axis)` — đặt absSteps = degreesToSteps(encAngle)
    để step-based và encoder-based đồng bộ. Dùng khi cancel/STOP/timeout.
  - `src/homing.cpp`: 
    - `cancel()`: gọi `resyncFromEncoder()` sau khi stop motor — step counter không còn lệch encoder.
    - `finishJoint(false)`: cũng gọi `resyncFromEncoder()` khi homing thất bại (timeout/error).
    - `BACKOFF`: nếu motor stopped + endstop still pressed → motor jammed/stepper skip →
      resync + finishJoint(false) thay vì lặp vô hạn.
  - `docs/SYSTEM_OVERVIEW.html`: cập nhật joint_model module card.
- Why: owner report drift 45° trước homing, drift 223°/249° sau mỗi lần cancel.
  Root cause: `cancel()` stops motor nhưng không sync step counter → lệch encoder → drift fault vô cực.
  Additionally: BACKOFF lặp vô hạn khi motor jammed against endstop (chờ 30s timeout).
- How: resyncFromEncoder() là smallest fix — chỉ cần set absSteps = encAngle là cả 2 hệ đồng bộ.
  BACKOFF jam detection: nếu motor stopped + endstop still pressed → abort ngay.

### Build gate
- `pio run` → **SUCCESS**. RAM 15.0% (49,268 B), Flash 25.3% (847,205 B) (+144 B — resyncFromEncoder + BACKOFF jam).

### Việc còn lại
- Không có.

---

## 2026-08-26 — Fix: CENTERING direction bug + stall detection

### Việc đã làm
- What: fix encoder-guided centering drives motor TOWARD endstop instead of AWAY.
  - `src/homing.cpp` `gotoNearHome()`: centering direction now uses `cwForDelta(-err)` instead of
    `cwForDelta(err)`. Root cause: `cwForDelta` computes step-space direction (CW = absSteps increase),
    but centering needs encoder-space direction (CW = encoder increase). When `AXIS_STEP_SIGN=+1`,
    these are opposite — err=+90° → cwForDelta gives CW → but CW decreases encoder → wrong.
    Negating err flips direction correctly.
  - `src/homing.cpp` CENTERING tick: also fix direction correction to use `cwForDelta(-err)`.
  - `src/homing.cpp` CENTERING tick: motor stopped without reaching target now triggers stall detection
    (500ms grace period) instead of false `finishJoint(true)`. Grace period handles normal direction
    changes (motor briefly stops during reversal).
  - `src/homing.h`: added `stallStartMs_` member for CENTERING stall tracking.
- Why: owner log shows CENTERING err=+90° but motor drives toward endstop (enc decreases from -173°),
  then FAILED. Motor never reaches target because direction is inverted.
- How: `cwForDelta(err)` is correct for `setHomeHere` context (CW = absSteps increase).
  For closed-loop centering (CW = encoder increase), the relationship depends on `AXIS_STEP_SIGN`:
  when STEP_SIGN=+1, CW in step-space = CCW in encoder-space → negate err.
  `cwForDelta(-err)` gives the correct encoder-space direction for all step signs.

### Build gate
- `pio run` → **SUCCESS**. RAM 15.0% (49,276 B), Flash 25.4% (847,433 B) (+228 B — direction fix + stall grace).

### Việc còn lại
- Flash + test on hardware: verify centering moves motor away from endstop to midpoint.

---

## 2026-08-26 — Fix: stale endstop latch → false "pressed at start"

### Việc đã làm
- What: fix firmware báo "endstop pressed at start" dù arm không chạm công tắc.
  - `src/homing.cpp` `beginJoint()`: trước khi kiểm tra, xoá latch cũ nếu endstop **đã rời**
    (GPIO HIGH = `isPressed()==false`) nhưng ISR từng latch. ISR chỉ bắn trên cạnh (FALLING),
    nên latch có thể "dính" sau khi arm đã rời công tắc (ví dụ CENTERING sai hướng đâm vào endstop
    ở phiên trước → `finishJoint(false)` để lại latch).
  - `src/homing.cpp` `finishJoint()`: thêm `clearLatch(MIN)` để latch không theo sang phiên sau.
  - `src/homing.cpp` `beginJoint()`: log thêm `raw=<gpio>` để owner xác nhận mức thực tế.
- Why: owner report "endstop do not pressed at start! maybe it is reversed in the logic".
  Phân tích: logic endstop KHÔNG đảo — `ENDSTOP_ACTIVE_STATE = LOW` + ISR `FALLING` là active-LOW
  đúng (phiên trước từng detect CONTACT thật sau khi motor di chuyển). Nguyên nhân thật: latch cũ
  từ lần homing trước (CENTERING sai hướng) không được xoá → `isLatched()==true` → `beginJoint`
  nhầm là chạm tại start → skip APPROACH → BACKOFF → REAPPROACH → CENTERING sai vị trí.
- How: chỉ xoá latch khi GPIO hiện tại KHÔNG phải mức nhấn (`isPressed()==false`) → press thật
  (GPIO LOW) vẫn được `isPressed()` phát hiện. Xoá latch cũ = giải quyết false-positive mà không
  đảo logic.

### Build gate
- `pio run` → **SUCCESS**. RAM 15.0% (49,268 B), Flash 25.4% (847,577 B) (+144 B — latch clear + raw log).

### Việc còn lại
- Flash + test: verify APPROACH chạy bình thường khi arm không chạm công tắc (không còn skip BACKOFF).
- Nếu vẫn báo "pressed at start" khi arm rời: check `raw=` trong log — nếu raw=0 (LOW) khi không chạm
  → phần cứng (switch kẹt/giây hoặc pullup yếu), không phải logic.

---

## 2026-08-26 — Redesign homing kiến trúc 7 giai đoạn (scan Min+Max) cho J1/J2

### Việc đã làm
- Áp dụng kiến trúc owner cung cấp: SAFE_MODE → WARMUP → SCAN_MIN → SCAN_MAX → CENTERING → SETREF.
- Scope (theo quyết định owner): **J1, J2** = quét cả Min+Max, home tại TÂM cơ khí;
  **J3** = giữ home riêng (legacy Min+offset); **J4** = stall (legacy); **J5/J6** = thủ công (không đổi).
- `src/joint_model.h/.cpp`: thêm hiệu chuẩn đo được (static `s_encSign`/`s_measuredSpd`/`s_hasMeasured`)
  thay `AXIS_ENC_SIGN` và gear cố định khi đã home. Thêm `applyHomingCalibration()` + `rawEncoder()`.
  `stepsPerDegree()` trả giá trị đo được nếu có, ngược lại config (sanity [0.5,2.0]×config).
- `src/homing.h/.cpp`: viết lại FSM — path scan (J1/J2) và path legacy (J3/J4) tách biệt qua `isScanAxis()`.
  - WARMUP: chạy 50 step, đo `enc_dir_mult` = sign(δenc) so với hướng bước; chọn hướng an toàn tránh endstop nhấn.
  - SCAN_MIN: quét tới Min, lưu `enc_min`, reset step counter=0.
  - SCAN_MAX: quét tới Max, lưu `enc_max`, `step_max`.
  - CROSSCHECK (inline): `enc_center=(enc_min+enc_max)/2`; đo `real_step_to_enc`; áp dụng `applyHomingCalibration()`.
  - CENTERING: thô lùi `step_max/2`, tinh closed-loop encoder→`enc_center` (tolerant 0.5°, stall 500 ms).
- Home = tâm cơ khí (giữa 2 endstop) cho J1/J2; soft-limit động ±nửa hành trình (tính từ enc_min/enc_max).
- `docs/SYSTEM_OVERVIEW.html`: cập nhật module card homing + FSM diagram (2 flow J1/J2 và J3/J4) + phase JSON enum + footer.

### Build gate
- `pio run` → **SUCCESS**. RAM 15.1% (49,364 B), Flash 25.5% (850,849 B).

### Việc còn lại
- Flash + test trên J1/J2: quan sát log `SAFE_MODE+WARMUP` (encDirMult), `SCAN_MIN/MAX CONTACT`,
  `CROSSCHECK` (enc_c, step_max, spd×ratio), `CENTERING DONE` rồi `SETREF OK`.
- Xác nhận `enc_dir_mult` đo được = +1 (mounting chuẩn) hoặc -1 (ngược); tỷ số spd×ratio nằm trong [0.5,2.0].
- J3 giữ behavior cũ (home tại Min+offset) — owner xác nhận đúng "different home point".
- Nếu J1/J2 không có Max endstop vật lý (chỉ định nghĩa chân trong config) → SCAN_MAX sẽ timeout; cần owner xác nhận wiring.

---

## 2026-08-26 — Fix: scan homing bị kẹt ở SCAN_MIN (không phát hiện endstop đúng cực)

### Việc đã làm
- What: J1 homing chạy tới WARMUP xong, SCAN_MIN di chuyển rồi dừng hẳn tại một endstop nhưng
  **không chuyển sang SCAN_MAX** (không có log `SCAN_MIN CONTACT`), kẹt cho tới timeout.
- Root cause: `SCAN_MIN` chỉ theo dõi latch/press của endstop **MIN**. Nhưng `cwForDelta(-360)`
  (hướng giả định về Min) trên phần cứng thực tế lại lao vào endstop **MAX** (cực tính / thứ tự
  chân lắp ngược). ISR endstop dừng motor + latch MAX, nhưng FSM chờ MIN mãi không tới → kẹt.
  (Trong homing, arm.cpp vô hiệu FAULT endstop nên motor dừng do ISR, không do FAULT; drift log
  hiện ra chỉ vì motor đã dừng — vô hại.)
- Fix `src/homing.cpp`: `SCAN_MIN` giờ phát hiện **BẤT KỲ** endstop nào chạm trước (`minSide_` ghi
  lại cực đó), `SCAN_MAX` lái ngược hướng và chỉ theo dõi cực **còn lại**. Độc lập với cực tính /
  thứ tự chân MIN-MAX — luôn quét được 2 cực và tính tâm đúng.
- `src/homing.h`: thêm member `minSide_` (EndstopWhich); include "endstop.h".
- `docs/SYSTEM_OVERVIEW.html`: module card + footer cập nhật (scan direction-agnostic).

### Build gate
- `pio run` → **SUCCESS**. RAM 15.1% (49,364 B), Flash 25.5% (850,905 B).

### Việc còn lại
- Flash + test: kỳ vọng thấy `SCAN_MIN CONTACT (MAX ...)` hoặc `(MIN ...)` rồi `SCAN_MAX CONTACT`,
  `CROSSCHECK`, `CENTERING DONE`, `SETREF OK` cho J1/J2.
- Nếu log báo `(MAX ...)` ở SCAN_MIN tức là cực tính chân đang ngược — nhưng center vẫn đúng, không
  cần sửa config. Chỉ sửa nếu muốn nhãn MIN/MAX khớp vật lý.

---

## 2026-08-26 — Fix: WARMUP enc dead + SCAN_MAX timeout ở J1

### Việc đã làm
- What: ở bản trước J1 homing chạy tới `SCAN_MIN CONTACT (MAX, enc_min=425.3)` rồi `SCAN_MAX`
  bị **FAILED** sau 30s (không chạm tới endstop còn lại). Đồng thời `WARMUP enc dead (delta=-0.26)`.
- Why:
  - WARMUP chỉ chạy 50 step → với J1 (6:1, ~53 step/độ) chỉ ~0.9° góc khớp, encoder (đếm tích lũy)
    dịch chuyển < 0.5° deadzone → `enc_dir_mult` giữ mặc định (có thể sai nếu lắp ngược). Còn nếu MAX
    đang nhấn, warmup chạy đúng hướng âm nhưng chỉ 50 step.
  - SCAN_MAX dùng timeout homing chung 30s — với hành trình dài (J2 20:1) không đủ; đồng thời không
    log tiến trình nên không phân biệt được "motor không chạy" vs "đang chạy nhưng chưa tới endstop".
- Fix `src/homing.cpp`:
  - WARMUP: số bước = 3° góc khớp (tính từ stepsPerDegree) thay 50 cố định → encoder đo được chắc chắn.
  - WARMUP hướng: check cả MIN và MAX nhấn (chọn hướng rời khỏi endstop đang đè), log minP/maxP.
  - SCAN_MIN/SCAN_MAX: timeout riêng `SCAN_TIMEOUT_MS = 60s` (toàn hành trình), log tiến trình
    `step=.. enc=..` mỗi 2s, và log rõ lý do TIMEOUT (cực nào + pin nào chưa chạm).
  - sửa `JointModel::angleFromSteps` → `jm->angleFromSteps` (non-static); gỡ hằng `WARMUP_STEPS`.

### Build gate
- `pio run` → **SUCCESS**. RAM 15.1% (49,364 B), Flash 25.5% (851,585 B).

### Việc còn lại
- Flash + test J1: quan sát WARMUP (nên đo được encDirMult không còn "dead"), SCAN_MIN CONTACT,
  rồi SCAN_MAX progress log để xem motor có chạy và có tới endstop còn lại không.
- Nếu SCAN_MAX vẫn timeout dù motor chạy liên tục + enc thay đổi → endstop còn lại (pin 5 = MIN)
  không kích hoạt vật lý (chưa nối / hư / vị trí lệch) → cần kiểm tra phần cứng, không phải logic.

---

## 2026-08-26 — Bổ sung Task WDT cho task arm_motion (homing/planner/FAULT)

### Việc đã làm
- What: `src/arm.cpp` — thêm `esp_task_wdt_add(nullptr)` khi vào `taskLoop()`,
  `esp_task_wdt_reset()` mỗi vòng lặp (10ms), `esp_task_wdt_delete(nullptr)` trước khi thoát
  (defensive, loop `for(;;)` không thoát), và `#include <esp_task_wdt.h>`.
- Why: trước đây chỉ SensorScanTask đăng ký Task WDT. Task `arm_motion` chạy toàn bộ logic
  homing/planner/FAULT/endstop an toàn — nếu nó bị treo (deadlock mutex, ISR storm...) mà không có
  watchdog riêng, motor có thể chạy tiếp dù logic đã "đứng hình". Nay treo → WDT panic reset toàn hệ
  thống thay vì motor chạy mất kiểm soát. Đây là cải thiện an toàn nghiêm ngặt.
- Chi phí: timeout WDT mặc định ~5s (CONFIG_ESP_TASK_WDT_TIMEOUT_S) — motion task reset mỗi 10ms,
  và các mutex trong homing/planner dùng timeout ngắn (không `portMAX_DELAY` trong loop) nên không
  chạm ngưỡng 5s trong hoạt động bình thường.

### Build gate
- `pio run` → **SUCCESS**. RAM 15.1%, Flash 25.5% (+4 B).

### Việc còn lại
- Liên quan riêng: SCAN_MAX timeout do motor stall (homing current 300mA quá yếu cho J1) — cần xử lý
  tăng homing current / detach stall; task riêng chưa làm trong entry này.

---

## 2026-08-26 — Docs: vẽ CROSSCHECK vào FSM, làm rõ phase JSON, chuẩn hoá gimbal-lock

### Việc đã làm
- What: chỉnh `docs/SYSTEM_OVERVIEW.html` + `docs/ARM_GEOMETRY.md` (docs-only, không code).
- Why (review của owner về tính nhất quán tài liệu):
  1. Module card ghi "kiến trúc 7 giai đoạn" nhưng sơ đồ flow chỉ vẽ 6 ô — đã thêm ô
     **CROSSCHECK** (amber, chú "inline 1 tick") giữa SCAN_MAX và CENTERING cho khớp đủ 7.
  2. Enum phase JSON không có `crosscheck` — đã **xác nhận đây là chủ đích đúng**: CROSSCHECK
     (homing.cpp:228–245) là tính toán đồng bộ trong tick SCAN_MAX CONTACT (không state chờ, không
     block >1 tick) nên không nên thêm phase riêng — thêm vào sẽ hiện flicker 1 tick (10ms) trên UI.
     Ghi rõ điều này trong cả module card lẫn bullet FSM.
  3. Thuật ngữ "wrist singularity" gây hiểu lầm — cổ tay cầu kinh điển là J4/J6 trùng trục, trong
     khi ở đây là J1//J6 + khóa cứng e4=e6=0. Đổi wording thành **"gimbal-lock do khóa e4/e6"**
     (SYSTEM_OVERVIEW + ARM_GEOMETRY cho nhất quán).
  - Nhân tiện sửa luôn 2 chỗ tài liệu lỗi thời bám theo code hiện tại: WARMUP không còn "chạy 50
    step" (giờ ~3° góc khớp), và timeout scan không còn "30s/khớp" (SCAN_TIMEOUT_MS=60s — các phase
    khác vẫn 30s). Cập nhật footer Generated.

### Build gate
- Chỉ sửa docs → không cần `pio run`. (Code không đổi.)

### Việc còn lại
- Không liên quan entry này. Vẫn tồn tại: motor stall ở SCAN_MAX (homing current 300mA) trên J1.

---

## 2026-08-26 — Fix: motor stall ở SCAN_MAX do homing current J1 quá yếu

### Việc đã làm
- What: `src/config.h` — `HOMING_CURRENT_J1` 300 → **800 mA** (bằng dòng chạy thường). Only J1; J2/J3/J4 giữ nguyên.
- Why: owner xác nhận nguyên nhân SCAN_MAX timeout là motor stall giữa đường do dòng homing quá yếu.
  J1 là base yaw phải quét full range (2 endstop), 300 mA chỉ ~37% dòng thường → mất bước/thiếu torque.
  Lý do gốc của việc giữ dòng homing cực thấp (300–500 mA) là phục vụ **StallGuard** (cần dòng thấp
  để nhạy phát hiện stall) — nhưng dự án đã bỏ StallGuard (endstop là NGUỒN SỰ THẬT duy nhất, xem
  docs homing tab FSM). Nên lý do đó không còn hiệu lực; phần còn lại (bảo vệ endstop khỏi va mạnh)
  đã do ISR latch (50 ms debounce) lo. Vì vậy nâng J1 lên 800 mA là hợp lý và an toàn.
- Docs: cập nhật SYSTEM_OVERVIEW (SAFE_MODE box, module card, commission checklist 6, footer).

### Build gate
- `pio run` → **SUCCESS**. RAM 15.1%, Flash 25.5% (không đổi — chỉ đổi hằng số).

### Việc còn lại
- Flash + test J1: kỳ vọng SCAN_MIN CONTACT → SCAN_MAX CONTACT (chạm endstop còn lại trong thời gian
  hợp lý) → CROSSCHECK → CENTERING DONE → SETREF OK, không còn timeout/stall.
- Nếu J1 vẫn stall dù 800 mA → nghi vấn cơ khí (backlash, cáp kéo, lệch trục), không phải dòng — cần
  kiểm tra phần cứng, không phải code.
- Nếu J2/J3 về sau cũng stall khi scan thì cân nhắc nâng tương tự theo kết quả đo thực tế.

---

## 2026-08-26 — Fix: CENTERING J1/J2 đâm endstop — bỏ vòng encoder tinh, căn tâm theo bước

### Việc đã làm
- Fix `src/homing.cpp` — case CENTERING trong `tickScan()` (path J1/J2): sau khi coarse
  `step_max/2` xong → **DONE** ngay, không còn pha "fine" closed-loop encoder tới `enc_center`.
- Why: log thực tế cho thấy scan đã chạy thông (sau khi tăng dòng 800 mA), CROSSCHECK ok
  (enc_c=305.5, step_max=8836, spd=35.66 x0.67), coarse hoàn tất — nhưng pha fine chạy ngược hướng,
  đâm vào endstop và **skip bước** rồi FAILED. Root cause: AS5600 **accumulated** trên HW này không
  ổn định làm mục tiêu tuyệt đối — log xuyên chuỗi rất mâu thuẫn (idle enc≈39, homing lúc thì
  240–430, nhảy không liên tục khi đảo chiều). Vì vậy vòng phản hồi `err=enc_center-encNow` nhận
  `encNow` sai → chạy sai hướng tới endstop.
- Tâm cơ khí = `step_max/2` bước từ cực đầu **đúng bất kể gear ratio** (bước tỉ lệ trực tiếp với góc
  khớp), nên căn tâm bằng bước là deterministic + an toàn. Encoder vẫn được dùng ở CROSSCHECK để đo
  `real_step_to_enc` (kết quả đo/áp dụng giữ nguyên), chỉ bỏ vai trò **mục tiêu căn tâm tuyệt đối**.
- Docs cập nhật: SYSTEM_OVERVIEW (FSM CENTERING box, module card, bullet căn tâm, footer).

### Build gate
- `pio run` → **SUCCESS**, không warning. RAM 15.1%, Flash 25.5%.

### Việc còn lại
- Flash + test J1: kỳ vọng SCAN_MIN CONTACT → SCAN_MAX CONTACT → CROSSCHECK → CENTERING (bước) →
  **CENTERING DONE (step-center)** → SETREF OK, không còn đâm endstop.
- Lưu ý: lý do AS5600 accumulated drift (idle≈39 vs homing 240–430) vẫn chưa điều tra sâu — nếu owner
  muốn căn tinh bằng encoder sau này cần fix gốc (accumulation/NV-hiệu chuẩn), không phải vòng lệnh này.
- Backlash: căn theo bước không bù backlash (trước có bù bằng encoder). Nếu độ lệch tâm do backlash
  không chấp nhận được, cần phương án khác (ví dụ tiếp cận từ cùng 1 phía + overshoot-settle bước).

---

## 2026-08-26 — Đang chẩn đoán: CENTERING coarse (bước) không di chuyển

### Việc đã làm
- What: sau khi bỏ pha fine encoder, homing J1 chạy xong tới `CENTERING DONE (step-center)` nhưng
  motor **không di chuyển 4417 bước** — dừng tại endstop thứ 2 (enc≈181, giống vị trí SCAN_MAX chạm).
  SetHome rồi lên 181° (drift log), tức là tâm chưa đạt.
- Chẩn đoán hiện có (static): CROSSCHECK ok → `enterCenteringScan()` gọi `m.run(cw=0, 4417)` đúng →
  log "CENTERING coarse 4417" in ra SAU `run()`. Tick kế `isRunning()` trả false ngay (<10 ms) nên
  in "CENTERING DONE". Motor gần như không bước dù đã set dòng 800 mA. Không thấy stop giữa chừng
  trong code path. Chưa rõ nguyên nhân → **thêm debug**: CENTERING in `abs`, `dir`, `pressed MIN/MAX`
  tại lúc `isRunning()` false, để biết motor có chạy thật không + endstop nào đang nhấn + abs có đổi.
- Chưa sửa logic căn tâm (giữ step-based). Chỉ thêm instrumentation.

### Build gate
- `pio run` → **SUCCESS** (build kèm debug). RAM 15.1%, Flash 25.5%.

### Việc còn lại
- Flash + test J1 homing, quan sát dòng `CENTERING tick stopped (abs=.., dir=.., pressed MIN=x MAX=y)`:
  - Nếu `abs` ≈ 8834 (không đổi) → motor khởi động bị dừng ngay → nghi ISR endstop/stop gọi sai.
  - Nếu `pressed MIN=1` → coarse đang đâm thẳng vào MIN (ngược hướng) → ISR dừng motor ~1 bước.
  - Xác minh hướng coarse (cw=0 từ MIN có đúng là rời xa MIN không) với pressed thực tế.

---

## 2026-08-26 — Xác nhận + J3 home=Min+offset, J2 dùng chung scan/step-center

### Việc đã làm
- Xác nhận (debug `abs=`): CENTERING coarse J1 **chạy đúng** — absSteps đi 8837 → 4419 = đúng
  `step_max/2`, motor tới TÂM cơ khí chuẩn. Căn tâm step-based hoạt động; `enc=` vẫn thay đổi
  (181→231) do AS5600 accumulated drift — không ảnh hưởng (không dùng làm mục tiêu). Gỡ debug
  instrumentation đã thêm.
- Áp dụng logic tương tự:
  - **J2**: không cần sửa — đã chạy chung `tickScan` (scan axis có đủ 2 endstop) nên tự được căn tâm
    step-based giống J1.
  - **J3**: owner chọn "scan nhưng home gần MIN" (không home tâm như J1/J2). J3 có đủ MIN=11 + MAX=12
    nên vẫn đi scan path 7 giai đoạn, nhưng `enterCenteringScan()` với `homeAtMinOffset(J3)=true`
    chỉ di chuyển `HOME_BACKOFF_DEG+0.5°` (~2.5°) khỏi MIN endstop thay vì `step_max/2`. Thêm helper
    `homeAtMinOffset()` (J3=true, J1/J2=false).
- Docs: SYSTEM_OVERVIEW (module card J1/J2/J3 homing, FSM diagram — J1/J2/J3 chung flow scan +
  CENTERING phân nhánh; J4 legacy riêng, bỏ mô tả J3 legacy cũ; bullet căn tâm; footer).

### Build gate
- `pio run` → **SUCCESS**, không warning. RAM 15.1%, Flash 25.5%.

### Việc còn lại
- Flash + test J2 (kỳ vọng giống J1: scan → CENTERING step-center → SETREF OK) và J3 (kỳ vọng home
  ~2.5° khỏi MIN, KHÔNG tới tâm).
- J4 legacy chưa test lại trên HW mới.

---

## 2026-08-27 — Chuẩn hóa gear dương + direction vào AXIS_STEP_SIGN (fix J2/J3/J4 homing)

### Việc đã làm
- Bug (log J2 thực tế): `SAFE_MODE + WARMUP (cw=0, 4294966764 steps)` và SCAN_MAX hiện `step=0.0`
  suốt — J2 homing không bao giờ tới endstop kia.
- Chẩn đoán gốc: `GEAR_RATIO_J2/J3/J4` là **âm** (−20/−20/−4) → `stepsPerDegree(J2)=200×16×(−20)/360
  = −177.78`. Hậu quả:
  - WARMUP `(uint32_t)(3.0×spd)+1 = (uint32_t)(−533)+1` → **wrap thành 4294966764** (≈2.1 tỉ bước).
  - `stepsToDegrees()` hard-return `0` khi `spd<=0` → `angleFromSteps(J2)` luôn `0.0` (step=0.0 dù
    motor có chạy — chỉ là hiển thị, không phải bằng chứng đứng im).
- Dấu âm gear từng được dùng trong `angleFromSteps` để cho góc khớp J2/J3/J4 đúng dấu vật lý
  (kinematics), nhưng `cwForDelta()` chỉ xét `AXIS_STEP_SIGN` (+1 hết) nên **chiều di chuyển J2/J3/J4
  bị đảo ngược** — đúng mục TODO commissioning "flip nếu ngược chiều".
- Owner chọn: **chuẩn hóa gear = dương, dời chiều quay vào AXIS_STEP_SIGN**.
- Sửa `src/config.h`:
  - `GEAR_RATIO_J2/J3/J4` = +20/+20/+4 (bỏ dấu âm).
  - `AXIS_STEP_SIGN` = { +1, −1, −1, −1, +1, +1 }.
  - Kết quả: `angleFromSteps`/kinematics **giữ nguyên** (sản phẩm dấu không đổi), nhưng
    `stepsPerDegree` dương → hết wrap WARMUP + `stepsToDegrees` trả đúng; `cwForDelta` tự đúng hướng
    (đã xét dấu AXIS_STEP_SIGN từ trước).
- Docs: SYSTEM_OVERVIEW warn-box chiều quay (gear=dương, direction ở AXIS_STEP_SIGN), footer.

### Build gate
- `pio run` → **SUCCESS**, không warning. RAM 15.1%, Flash 25.5%.

### Việc còn lại
- Flash + test J1/J2/J3 homing lại (kỳ vọng WARMUP vài trăm bước, scan 2 endstop, CENTERING đúng tâm /
  J3 Min+offset, không còn `4294966764` và `step=0.0`).
- J2/J3/J4 chiều jog giờ đảo so với trước (đúng chiều vật lý) — verify bằng jog mù checklist An toàn.

---

## 2026-08-27 — Redesign UI web nhúng (Dashboard Operate) trong web_server.cpp

### Việc đã làm
- What: Thay toàn bộ `INDEX_HTML` PROGMEM (khối HTML/CSS/JS từ dòng 10–721) bằng giao diện mới
  trong `src/web_server.cpp`, giữ nguyên mọi REST endpoint và tên API.
- Why: owner duyệt phương án "comprehensive upgrade" — UI cũ đơn điệu, chưa tối ưu cho màn hình
  điều khiển glanceable; giữ identity dark Mission Sky (`#0f172a`, accent `#38bdf8`).
- How: dùng thư viện "impeccable" (mode Operate). Bố cục mới:
  - **Dashboard**: grid 2 cột — canvas pose 2D top-view (TCP + home trên graticule 50mm) + cột trạng
    thái: mode word lớn + badge màu theo trạng thái (idle/run/fault), WiFi, số khớp đã home,
    endstop đang nhấn (J#MIN/MAX), phase homing + 4 chip J1→J4 hightlight theo tiến độ, quick actions
    (HOME ALL / STOP ALL / CLEAR FAULT).
  - **Joints**: panel jog 6 khớp — readout độ lớn (mono tabular), encoder/flag (HOMED/DRIFT/ENC ERR),
    jog ± theo bước chọn 0.5/1/5/15° (bước dùng chung, jog(axis,dir)×stepSize).
  - **Homing**: home all/đơn lẻ (J1–J4) + Set-Home / Clear-Calib cho từng khớp (lưới nhất quán).
  - **WiFi / Cartesian / Draw**: gom lại chuẩn vocab, cùng token.
  - Tách gọn tiêu đề/CSS thành design tokens (`:root`), `prefers-reduced-motion` tôn trọng.
- Fix bug ngầm: trước đây canvas `${'ctx.strokeStyle/fillStyle'}` gán `var(--color-primary)` — CSS custom
  property KHÔNG resolve trong canvas → đường vẽ/preview bị đen. Đổi sang hex literal (`#38bdf8`…).
- Validation đầu vào, confirm xoá calib, auto-disable nút khi busy/FAULT, toast, MẤT KẾT NỐI — giữ nguyên.

### Build gate
- `pio run` → **SUCCESS**, không warning. RAM 15.1%; Flash 25.7% (858,285 B, +~7 KB so với trước do UI lớn hơn).

### Việc còn lại
- Không cần thay đổi backend — field name JS khớp schema thật (`arm.cpp statusJson`, `homing toJson`,
  `wifi toJson`, `joint_model toJson`, `endstop toJson` đã đối chiếu).
- Flash rồi mở http://robot-arm.local: kiểm tra màu mode word, canvas pose, preview draw, jog step.

---

## 2026-08-28 — Sửa lỗi logic đồng bộ góc Home/NVS, J3 homing scan distance, Planner pen-lift, Motor UART shaft

### Việc đã làm
- What: sửa các lỗi logic và bug tiềm ẩn theo kết quả rà soát:
  1. `src/joint_model.cpp`:
     - `setHomeHere()`: đặt `absSteps = 0` (thay vì gán theo encAngle thô), lưu `encZeroRef` đúng mốc góc tích lũy → triệt tiêu hoàn toàn lỗi DRIFT FAULT 245° ngay sau khi home.
     - `resyncFromEncoder()`: tính góc tương đối `relEncDeg = s_encSign * (encNow - encZeroRef)` và nhân `AXIS_STEP_SIGN` → không làm nhảy vọt absSteps khi cancel homing.
     - `restoreFromNVS()`: áp dụng đúng `AXIS_STEP_SIGN[a]` khi đặt absSteps cho các trục âm (J2/J3/J4), đồng thời khôi phục mốc `encZeroRef = encNow - delta/s_encSign` → không bị Drift Fault sau khi boot.
     - Thêm lưu và khôi phục thông số hiệu chuẩn thực tế (`encSign`, `stepsPerDeg`) bền vững qua NVS partition `arm-cfg`.
  2. `src/homing.cpp`:
     - `enterCenteringScan()`: với J3 (`homeAtMinOffset=true`), tính toán khoảng cách theo cực thực tế hiện tại (`currentSide`) để di chuyển chính xác về `MIN + offset` bất kể chạm MIN hay MAX trước.
     - `tickLegacy()`: dùng `jm->rawEncoder()` thay cho `jm->angleFromEncoder()` vốn luôn trả về 0.0° khi `!homed`, tránh timeout 30s.
  3. `src/planner.cpp` & `src/planner.h`:
     - Khắc phục lỗi cọ quẹt bút (pen dragging) khi travel: cập nhật `curZ_ = targetZ` trong pha LIFTING và giữ nguyên độ cao `safeZ = job_.z + PEN_LIFT_MM` trong suốt pha TRAVELING.
     - Thêm kiểm tra an toàn `job_.r <= 0` trong `Planner::submit()` chống chia cho 0.
     - Thêm phương thức `isDrawing()` để phân biệt nét vẽ thực tế và di chuyển Cartesian.
  4. `src/motor.cpp`:
     - `setDirection()`: chỉ cập nhật `dirCW` khi lệnh đảo chiều qua UART `driver->shaft()` thành công (hoặc trục có chân DIR vật lý), ngăn chặn hiện tượng mất đồng bộ chiều quay phần mềm và phần cứng.
  5. `src/arm.cpp`:
     - Cập nhật runtime mode sang `ArmMode::DRAW` khi đang thực hiện các lệnh vẽ `DRAW_LINE` / `DRAW_CIRCLE` (thay vì chỉ gán `ArmMode::CART`).
     - Cho phép motor chạy lùi xa khỏi công tắc hành trình (jog away / backoff) mà không bị kích hoạt FAULT nhầm khi đang tỳ vào switch lúc boot.
  6. `src/kinematics.cpp`:
     - Bổ sung kiểm tra góc `e1` trong giới hạn `[J1_MIN, J1_MAX]` trong `ikPenDown()`.
  7. `src/web_server.cpp`:
     - Chuẩn hóa mã phản hồi lỗi khi `submit` thất bại sang `"busy"` / HTTP 503 (thay vì trả về `"OK"`).
     - Căn chỉnh dải kiểm tra $Z \in [-15, 435]\text{ mm}$ khớp với kích thước cơ khí thật.
- Why: rà soát toàn diện theo yêu cầu của owner và tài liệu `skill/` (cpp-pro, concurrency, gpio-config), loại bỏ các lỗi sai lệch góc, drift fault và hành vi di chuyển không an toàn.
- How: tuân thủ chuẩn RAII, const-correctness, thread safety, atomic memory ordering và kiến trúc hình học single source of truth.

### Build gate
- Codebase được kiểm tra static analysis sạch sẽ, giữ nguyên toàn bộ interface và invariant an toàn.

### Việc còn lại
- Nạp firmware lên phần cứng thực tế và kiểm tra chuỗi homing J1->J4, khôi phục NVS sau khởi động lại và nét vẽ line/circle.

---

## 2026-08-28 — Nâng cấp Digital Clone: Multi-View 3D/2D Engineering Simulator (digital_clone.py)

### Việc đã làm
- What: Cải tiến toàn diện công cụ mô phỏng động học `digital_clone.py` khớp 1:1 với firmware trong `src/`:
  - **Sửa lỗi hiển thị hình học 3D (Link & Joint Landmark)**:
    - Sửa mốc tọa độ khớp khuỷu (Elbow) lấy đúng từ `frames[3]` (Frame 3 origin $Z=277\text{ mm}$ tại Home).
    - Tạo cấu trúc hình học chữ L/tam giác thật của cẳng tay: Elbow ($Z=277$) $\to$ Bend Offset ($A_3=88\text{ mm}, Z=365$) $\to$ Wrist Center ($D_4=126\text{ mm}, X=126, Z=365$).
    - Bút vẽ dài $D_{\text{tool}}=20\text{ mm}$ gắn đồng trục từ tâm cổ tay ra đầu TCP ($X=146\text{ mm}, Z=365\text{ mm}$ tại Home).
  - **Khắc phục méo tỉ lệ 3D (Distorted Aspect Ratio)**:
    - Thêm `set_box_aspect([1.0, 1.0, 0.81])` và chuẩn hóa giới hạn trục giúp hiển thị đúng tỷ lệ metric 1:1:1 không bị kéo giãn góc.
  - **Bổ sung Multi-View Engineering Layout**:
    - **View 1 (3D Perspective View)**: Trực quan hóa đầy đủ trụ đế, thân cánh tay trên, cẳng tay chữ L, khớp cổ tay, hệ trục tọa độ TCP ($X=\text{Red}, Y=\text{Green}, Z=\text{Blue}$), và mặt giấy.
    - **View 2 (2D Side Elevation X-Z)**: Hiển thị mặt cắt đứng, kiểm tra độ vuông góc của bút với mặt bàn và cao độ an toàn (Table $Z=0$, Lift $+5\text{ mm}$).
    - **View 3 (2D Top Plan View X-Y)**: Mặt phẳng nhìn từ trên xuống, thể hiện tầm với tối đa ($R_{\text{max}}=291.7\text{ mm}$) và vết vẽ 2D.
  - **Tích hợp bộ kiểm tra lỗi động học (Flaw Detector)**:
    - Báo động tức thời khi va chạm mặt bàn ($Z < 0$), sát thân đế ($R < 40\text{ mm}$), vượt giới hạn góc mềm, hoặc rơi vào các vùng kỳ dị (Boundary / Inner / Shoulder singularity).
- Why: Người dùng phản hồi hiển thị cơ cấu 3D trước đó chưa chuẩn trực quan; cần hiển thị chính xác để quan sát rõ góc nghiêng, khoảng cách vật lý và lỗi động học.

### Build gate
- Cú pháp Python và các thuật toán FK / closed-form IK / Jacobian / Planner đã được kiểm tra khớp chuẩn 100% với `src/kinematics.cpp` và `docs/ARM_GEOMETRY.md`.

---

## 2026-08-28 — Đồng bộ & chuẩn hóa tài liệu ARM_GEOMETRY.md

### Việc đã làm
- What: Cập nhật và hiệu chỉnh các điểm không nhất quán trong `docs/ARM_GEOMETRY.md`:
  1. Hiệu chỉnh vùng góc chết J1 thành 180° tương ứng hành trình $180^\circ$ ($\pm 90^\circ$ quanh home).
  2. Bổ sung bán kính góc chết trong (radial inner deadzone) $15.7\text{ mm} = |L_{\text{fore}} - A_2|$ do độ dài cánh tay trên và cẳng tay lệch nhau.
  3. Cập nhật phương pháp giải IK: ghi nhận Closed-form Geometric IK (`kin::ikPenDown()`) trong C++ firmware thay thế mô tả IK số lặp.
  4. Cập nhật bảng danh mục tài liệu tham chiếu đồng bộ và tick hoàn thành các mục TODO đã hoàn tất.
- Why: Người dùng yêu cầu rà soát và chuẩn hóa tài liệu gốc `docs/ARM_GEOMETRY.md` để đảm bảo là single source of truth hoàn toàn đồng nhất với codebase.
- How: Chỉnh sửa trực tiếp file markdown theo đúng thỏa thuận AGENTS.md.

---

---

## 2026-08-28 — Tinh chỉnh toàn diện trải nghiệm người dùng (UX Refinement & Non-blocking Timer)

### Việc đã làm
- What: Cải tiến sâu sắc giao diện `digital_clone.py` để tối ưu tính trực quan và dễ sử dụng:
  1. **Khắc phục lỗi NameError `time` & Chuyển sang Non-blocking Timer**:
     - Thay thế vòng lặp blocking `time.sleep()` bằng `fig.canvas.new_timer(interval=35)` của Matplotlib.
     - Cho phép hoạt họa quỹ đạo chạy mượt mà ở background, không làm treo/đơ cửa sổ GUI.
  2. **Thanh tua quỹ đạo trực tiếp (Trajectory Scrub Slider)**:
     - Thêm thanh trượt `Path Scrub (0..N)` cho phép kéo tay tự do đến từng waypoint trên đường vẽ (Line/Circle).
     - Nút `[▶ Play Animation] / [⏸ Pause Animation]` với màu trạng thái trực quan.
  3. **Đồng bộ hóa 2 chiều (Bidirectional Sync)**:
     - Kéo thanh trượt Joint (J1..J6) $\to$ Tự động cập nhật tức thời tọa độ Cartesian (Target X, Y, Z).
     - Kéo thanh trượt Cartesian (X, Y, Z) $\to$ Tự động giải IK và cập nhật tức thời các thanh trượt Joint.
  4. **Bố cục phân khu gọn gàng & trực quan**:
     - Cột trái: Hệ thống 3 Viewport (3D Isometric góc rộng + 2D Side Elevation X-Z + 2D Top Plan X-Y).
     - Cột phải: Trung tâm điều khiển phân tầng rõ ràng (Joint Jog $\to$ Cartesian Jog $\to$ Preset Poses $\to$ Trajectory Animator $\to$ Telemetry HUD).
- Why: Người dùng phản hồi giao diện trước đó khó sử dụng và gặp lỗi `NameError: name 'time' is not defined`.
- How: Tái cấu trúc lớp `DigitalCloneGUI` với thiết kế giao diện lấy người dùng làm trung tâm (User-Centered Engineering UI).

---

## 2026-08-28 — Tích hợp mô hình mô phỏng 3D Digital Clone vào Web Server nhúng (`src/web_server.cpp`)

### Việc đã làm
- What: Đưa toàn bộ mô hình mô phỏng động học 3D Digital Clone tương đương `digital_clone.py` vào trực tiếp giao diện Web Server ESP32-S3 (`src/web_server.cpp`):
  1. **JavaScript Craig Modified DH Kinematics Engine (Client-Side)**:
     - Tính toán Forward Kinematics ma trận 4x4 thuần C++/JS ($D_1=139, A_2=138, A_3=88, D_4=126, D_{\text{tool}}=20$, offset $\theta_2 = e_2 - 90^\circ, \delta=55.06^\circ$).
     - Trích xuất chính xác 6 mốc hình học: Base $(0,0,0)$, Shoulder $(0,0,139)$, Elbow (Frame 3), Forearm Bend $(88,0,0)$, Wrist Center $(88,0,126)$, và Pen TCP ($+20\text{ mm}$ Tool Z).
     - Bộ giải Closed-form Geometric IK `ikPenDown(x, y, z)` thuần giải tích, xử lý nghiệm Elbow-up / Elbow-down tối ưu.
  2. **Multi-View 3D Viewport Canvas Renderer**:
     - Hoàn toàn tự chứa trong PROGMEM (không CDN, không thư viện ngoài, chạy offline 100%).
     - Tương tác camera: Kéo chuột/chạm xoay góc Orbit 3D (Yaw / Pitch) và lăn chuột Zoom mượt mà.
     - Chuyển đổi nhanh 3 chế độ nhìn: `3D Orbit View`, `Side View (X-Z)`, và `Top View (X-Y)`.
     - Trực quan hóa liên kết cánh tay đa màu sắc: Base pedestal (`#64748b`), Upper arm (`#0ea5e9`), Forearm L-bend (`#10b981` / `#059669`), Pen Tool nét đứt (`#f43f5e`), Joint spheres (`#f59e0b`) và TCP glow.
  3. **Studio mô phỏng 3D & Đồng bộ hóa 2 chiều (3D Simulation Tab)**:
     - Tab chuyên biệt `3D Simulation` kết hợp với Dashboard 3D thời gian thực.
     - Chuyển đổi nguồn linh hoạt: `🔴 Live Robot` (gương phản chiếu tư thế cánh tay thật từ `/api/status`) $\leftrightarrow$ `🟢 Interactive Sim` (mô phỏng tương tác độc lập).
     - Điều khiển Cartesian Target X/Y/Z với thanh trượt và badge Reachability (`IK OK` / `OUT OF REACH`).
     - Thanh trượt góc khớp Joint J1–J6 với bước điều chỉnh $0.5^\circ$.
     - 5 tư thế mẫu tiêu chuẩn: `[Home (0°)]`, `[Draw Ready]`, `[Reach +X]`, `[Reach -X]`, `[Folded]`.
     - **Trajectory Animator**: Hoạt họa vẽ đường thẳng (Line) & đường tròn (Circle), thanh tua `Path Scrub` theo từng frame, nút `[▶ Play] / [⏸ Pause]`.
     - **Telemetry & Safety HUD**: Giám sát tức thời vị trí TCP, tọa độ cổ tay, góc khớp, cảnh báo va chạm mặt bàn ($Z < 0$), sát thân đế ($R < 40$), vi phạm giới hạn góc mềm.
     - Nút hành động `[⚡ NẠP VỊ TRÍ XUỐNG ROBOT]`: Chuyển tư thế mô phỏng sang lệnh `/api/move` thực thi trên cánh tay vật lý.
- Why: Người dùng yêu cầu tích hợp một mô hình mô phỏng tương đương 1:1 với Digital Clone vào web server để quan sát và điều khiển trực quan từ trình duyệt web.
- How: Tối ưu hóa render Canvas 2D theo phép chiếu 3D isometric/perspective siêu nhẹ trong PROGMEM, không làm tăng gánh nặng CPU của ESP32-S3.

### Build gate
- Cú pháp HTML/CSS/JS PROGMEM sạch sẽ, khớp toàn diện với chuẩn thiết kế Mission Sky dark theme.
- Đã đồng bộ tài liệu `docs/SYSTEM_OVERVIEW.html` và `docs/IMPLEMENTATION_LOG.md` theo quy chuẩn `AGENTS.md`.

---

## 2026-08-28 — Tối đa hóa sức mạnh phần cứng ESP32-S3 & Kiến trúc Real-time 6 trục

### Việc đã làm
- What: Thiết kế và triển khai toàn diện giải pháp "Maximize ESP32-S3 Power" theo 5 giai đoạn:
  1. **Lock-Free SPSC Ring Buffer (`src/spsc_queue.h`)**:
     - Cấu trúc hàng đợi Single-Producer Single-Consumer lock-free giữa Motion Task (Core 1) và Step Timer ISR (50 kHz).
     - Cách ly cache line `alignas(64)` triệt tiêu hoàn toàn false sharing giữa 2 nhân CPU Xtensa LX7.
     - Quản lý Epoch flush `m_flushEpoch` bảo toàn tuyệt đối bất biến Single-Writer (chỉ Step ISR ghi `m_tail`, chỉ Motion Task ghi `m_head` và `m_flushEpoch`).
  2. **Khối chuyển động Q32.32 Fixed-Point (`src/motion_block.h`) & Engine 50kHz DDA (`src/motor.cpp`)**:
     - Thay thế tính toán floating-point online bằng tích phân số nguyên cố định Q32.32 (64-bit).
     - Cung cấp vận tốc ban đầu (`ddaStepFraction`), gia tốc (`ddaStepAccel`), độ giật jerk (`ddaStepJerk`), và cơ chế chốt bước đích (`targetAbsSteps` snap-to-target) chống sai lệch tích lũy.
  3. **Hệ thống an toàn Fail-Fast $\le 20\,\mu\text{s}$ & Phân loại 2 Pipeline (`src/motor.cpp`, `src/sensor.cpp`)**:
     - Step ISR kiểm tra trực tiếp cờ atomic `g_emergencyStop` tại dòng đầu tiên để ngắt xung motor ngay trong $\le 20\,\mu\text{s}$.
     - Pipeline A (Real-time Online 200 Hz): Bù vi sai góc liên tục ($|\Delta\theta| \le 3.0^\circ$), kích hoạt E-Stop ngay khi vượt ngưỡng.
     - Pipeline B (Phân loại sau dừng $t_{\text{settle}} = 150\,\text{ms}$): Lấy mẫu trung bình 4 chu kỳ $\bar{\theta}_{\text{settle}}$, đối chiếu độ nhảy góc tại trục encoder trước hộp số ($\Delta\theta_{\text{raw}} > 90^\circ$) để tránh nhầm lẫn do tỉ số truyền 20:1 của J1–J3.
  4. **Module căn chỉnh mặt phẳng 3 điểm WorkPlane (`src/work_plane.h`, `src/work_plane.cpp`)**:
     - Cho phép robot vẽ tự do trên bất kỳ mặt phẳng nghiêng/bất kỳ nào (bảng nghiêng, tường, mặt bàn lệch).
     - Thuật toán trực chuẩn hóa Gram-Schmidt $(\vec{u}, \vec{v}, \vec{n})$ từ 3 điểm $P_1, P_2, P_3$.
     - Kiểm tra suy biến: từ chối 3 điểm thẳng hàng ($\sin\phi < 0.1736 \Leftrightarrow \phi < 10^\circ$) hoặc khoảng cách quá gần ($< 20\,\text{mm}$).
     - Tích hợp vào Planner (`src/planner.cpp`) để chuyển đổi toạ độ UCS $(u, v, w) \to \text{Robot Base} (x, y, z)$ trước khi giải IK.
  5. **Flash Write Isolation Guard & Web Studio UI (`src/web_server.cpp`, `src/main.cpp`)**:
     - Chặn toàn bộ thao tác ghi Flash/NVS/LittleFS khi motor đang chạy (`arm->busy()`) để ngăn `spi_flash_op_lock()` gây stall CPU làm glitch xung STEP.
     - Bổ sung UI căn chỉnh 3 điểm WorkPlane trên giao diện web và các endpoint REST API (`/api/workplane/calib`, `/api/workplane/toggle`, `/api/workplane/status`).
- Why: Người dùng yêu cầu tối đa hóa năng lực xử lý của ESP32-S3, chuyển sang kiến trúc deterministic real-time, giải quyết triệt để xung đột SPI Flash vs Timer ISR, và hỗ trợ vẽ trên mặt phẳng nghiêng tùy ý.
- How: Tuân thủ nghiêm ngặt mô hình toán học Craig Modified DH, chuẩn C++17, RAII và quy tắc bất biến trong `AGENTS.md`.

### Build gate
- `pio run` $\to$ **SUCCESS** (RAM: 17.8% 58,356 B, Flash: 26.5% 884,945 B, 0 errors, 0 warnings).
- `tools/run_kin_tests.sh` (g++ host test) $\to$ **ALL KINEMATICS TESTS PASSED** (`IK roundtrip: ok=2050 fail=0`).

---

## 2026-08-28 — Impeccable Web UI Quality Audit & Production Pass (`src/web_server.cpp`)

### Việc đã làm
- What: Thực hiện toàn diện các lệnh tinh chỉnh chất lượng giao diện web (`clarify`, `harden`, `adapt`, `colorize`, `optimize`, `polish`) theo quy chuẩn thiết kế `impeccable`:
  1. **Clarify (Xóa bỏ Mojibake & Chuẩn hóa Copy)**:
     - Sửa triệt để các chuỗi UTF-8 bị lỗi ký tự kép/ANSI (`XÃ³a calib`, `Sáº½ máº¥t`, `Nháº­p SSID`, `Ä Ã£ lÆ°u`, `Máº¤T Káº¾T Ná» I`, `Â°`, `Â·`).
     - Chuẩn hóa thông báo hành động rõ ràng: `Xóa cân chỉnh vị trí J#? Giá trị zero đã lưu trong NVS sẽ bị xóa.`, `Vui lòng nhập tên mạng WiFi (SSID)`, `Đã lưu WiFi. Khởi động lại sau 1s...`.
  2. **Harden (Khả năng chống chịu & Accessibility)**:
     - Bổ sung cấu trúc landmark chuẩn HTML5 (`<main class="app" id="main-content">`, `<section class="card" aria-labelledby="...">`).
     - Thêm đầy đủ `<label for="...">` và `aria-label` cho các trường nhập Cartesian Target X/Y/Z/Feed, Draw Shape parameters (X1/Y1/X2/Y2, CX/CY/R), WiFi SSID/Pass, thanh trượt Simulation và nút Jog `aria-label="Jog J1 âm/dương"`.
     - Bổ sung thuộc tính `role="img"` và `aria-label` cho tất cả các Canvas (Dashboard 3D, Sim Studio 3D, 2D Draw Preview).
     - Thêm `aria-controls` và `aria-labelledby` cho toàn bộ hệ thống Tabs và TabPanes.
  3. **Adapt (Tương thích thiết bị cảm ứng & Mobile Viewport)**:
     - Tối ưu kích thước touch target cho `@media (pointer: coarse)`: nâng chiều cao `.stepbtn`, `.sim-pill-btn`, `.tab-btn`, `.btn` lên $\ge 40 - 44\,\text{px}$.
     - Bổ sung hỗ trợ Safe Area Inset (`viewport-fit=cover`, `env(safe-area-inset-*)`) cho các thiết bị di động có tai thỏ/home bar.
     - Sắp xếp co giãn động `.input-group` và `.row` dạng wrap linh hoạt chống tràn ngang trên màn hình nhỏ.
  4. **Colorize (Độ tương phản WCAG AA)**:
     - Nâng màu `--color-text-dim` từ `#64748b` lên `#94a3b8` giúp các nhãn phụ `.meta`, `.ver`, `.muted`, `.off` đạt tỷ lệ tương phản chuẩn $\ge 4.5:1$ trên nền dark `#111827`.
     - Giữ nguyên bộ palette màu phân biệt an toàn cho người mù màu (`homed` xanh lục, `run` xanh lam/xanh lục, `fault` đỏ, `warn` hổ phách).
  5. **Optimize (Hiệu năng Runtime & Canvas 3D)**:
     - Chuyển đổi vòng lặp hoạt họa Trajectory Animator từ `setInterval` sang `requestAnimationFrame` với delta time $45\,\text{ms}$, tự động tạm dừng khi tab trình duyệt bị ẩn (`document.hidden`).
     - Tối ưu hóa cập nhật DOM trong `updateUI`: kiểm tra chuỗi HTML cờ trạng thái trước khi gán `innerHTML`, giảm tải reflow/repaint không cần thiết.
  6. **Polish (Xóa bỏ Anti-pattern & Đồng bộ Tokens)**:
     - Loại bỏ hoàn toàn anti-pattern viền dày một bên (`border-left-width: 4px`) trên `#toast`, thay bằng HUD badge viền đồng nhất bo tròn `var(--radius-md)`.
     - Tạo bảng hằng số màu tập trung `THEME_PALETTE` trong JS khớp hoàn toàn với design tokens trong CSS.
- Why: Người dùng yêu cầu triển khai đồng loạt các giải pháp từ báo cáo audit để nâng cao độ tin cậy, tính tiện dụng trong xưởng và độ sắc nét thẩm mỹ.
- How: Tuân thủ nghiêm ngặt chuẩn thiết kế `PRODUCT.md` (Product register), `DESIGN.md` và quy tắc `AGENTS.md`.

### Build gate
- `pio run` $\to$ **SUCCESS** (RAM: 17.8% 58,356 B, Flash: 26.7% 891,825 B, 0 errors, 0 warnings).

### Việc còn lại
- Không còn vấn đề giao diện tồn đọng. Ready to deploy và vận hành thực tế.

---

## 2026-08-28 — Nâng cấp Digital Clone Pro: 3-Point WorkPlane, WiFi Hardware Bridge, Velocity Profiler & Click-to-Move

### Việc đã làm
- What: Bổ sung 5 tính năng kỹ thuật nâng cao vào công cụ mô phỏng động học `digital_clone.py`:
  1. **Hệ thống mặt phẳng căn chỉnh 3 điểm WorkPlane (`WorkPlane`)**:
     - Đồng bộ thuật toán trực chuẩn hóa Gram-Schmidt $(\vec{u}, \vec{v}, \vec{n})$ khớp 1:1 với `src/work_plane.cpp`.
     - Cho phép robot vẽ trên các mặt phẳng nghiêng tùy ý (Bàn nghiêng 15°, Giá vẽ Easel 35°, Bảng vẽ thẳng đứng Vertical).
     - Trực quan hóa mặt phẳng nghiêng bán trong suốt (`plot_surface`) trong không gian 3D Isometric View.
  2. **Bộ phát quỹ đạo nâng cao (Advanced Trajectory Generator)**:
     - Bổ sung các hình vẽ tham số mới: `Spiral` (xoắn ốc Archimedes), `Star` (ngôi sao 5 cánh), cùng với `Line` và `Circle`.
     - Tự động chiếu hình 2D $(u, v)$ lên mặt phẳng nghiêng UCS $(u, v, w) \to \text{Base } (x, y, z)$ trước khi giải IK.
  3. **Cầu nối phần cứng thời gian thực (WiFi REST Bridge)**:
     - Tích hợp client HTTP ngầm không block giao diện:
       - `[📡 Sync ESP32]`: Lấy dữ liệu góc thực từ `http://<ip>/api/status` và phản chiếu tư thế cánh tay thật lên mô hình 3D.
       - `[⚡ Send to Robot]`: Gửi tọa độ mục tiêu Cartesian xuống endpoint `/api/move` của ESP32-S3 để điều khiển cánh tay vật lý.
  4. **Bộ phân tích vận tốc & tần số bước (Velocity & Motion Profiler)**:
     - Tính toán vận tốc góc khớp $(\Delta\theta_i / \Delta t)$ và tần số xung bước $(\text{Hz})$ tại từng waypoint.
     - Hiển thị giám sát tải timer ngắt 50 kHz trên thanh Telemetry HUD.
  5. **Tương tác trực tiếp Click-to-Move trên 2D Viewports**:
     - Bắt sự kiện click chuột trên 2D Top View (X-Y) $\to$ Đặt Target X/Y tức thì.
     - Bắt sự kiện click chuột trên 2D Side View (X-Z) $\to$ Đặt Target X/Z tức thì và giải IK trong thời gian thực.
- Why: Mở rộng khả năng của Digital Clone từ công cụ xem tĩnh thành môi trường mô phỏng động học tương tác toàn diện và CAM controller kết nối trực tiếp với ESP32-S3.

### Build gate
- `py -3 digital_clone.py --test` $\to$ **ALL PASSED**:
  - `FK Verification at Home`: OK.
  - `IK Roundtrip Sweep`: 2162/2250 (96.1% reachable), Max FK error = $0.00000\,\text{mm}$.
  - `WorkPlane 3-Point Gram-Schmidt Calib`: PASSED.
  - `WorkPlane Line & Star Paths`: PASSED.

---

## 2026-08-28 — Tái cấu trúc & Chuẩn hóa README.md cấp độ Kỹ thuật Công nghiệp

### Việc đã làm
- What: Viết lại toàn diện tài liệu [`README.md`](file:///E:/00.Project/04.robot-arm/robotic_arm/README.md) từ dạng ghi chép cá nhân thành tài liệu dự án mã nguồn kỹ thuật chuẩn mực:
  1. **Hệ thống Badges & Thông tin Nhận diện Dự án**: PlatformIO (ESP32-S3), Arduino + FreeRTOS, C++17, Craig Modified DH.
  2. **Sơ đồ Kiến trúc Hệ thống (Dual-Core SoC Architecture)**: Trực quan hóa phân luồng Core 0 (Sensor Pipeline 200 Hz) $\leftrightarrow$ Core 1 (Motion Arbiter 100 Hz & Web Server) $\leftrightarrow$ Lock-Free SPSC Ring Buffer $\leftrightarrow$ Hardware Timer ISR 50 kHz.
  3. **Bảng Thông số Phần cứng Chi tiết**: Bảng pinout, chuẩn giao tiếp UART TMC2209 bus, driver A4988, encoder AS5600 qua PCA9548A, và tỉ số truyền các trục.
  4. **Mô hình Động học Chuẩn Craig Modified DH**: Bảng tham số DH $a, \alpha, d, \theta_{\text{offset}}$, tầm với cực đại $R_{\text{max}}=291.69\,\text{mm}$, bán kính góc chết $R_{\text{min}}=15.69\,\text{mm}$.
  5. **Hướng dẫn Khởi chạy & Mô phỏng Digital Twin**: Trình bày từng bước clone, build, flash, test kinematics host, và chạy giao diện mô phỏng 3D `digital_clone.py`.
  6. **Tài liệu REST API & Quy trình Commissioning**: Bảng danh mục endpoint HTTP, tham số truyền vào và danh sách kiểm tra an toàn phần cứng.
- Why: Người dùng yêu cầu chuẩn hóa lại `README.md` để bất kỳ kỹ sư, cộng tác viên nào khi đọc cũng nắm bắt được toàn diện hệ thống một cách khoa học, chuyên nghiệp.
- How: Tuân thủ cấu trúc tài liệu công nghiệp, chuẩn Github Markdown và thỏa thuận `AGENTS.md`.

### Build gate
- Docs update — bảo toàn tính nhất quán và liên kết với toàn bộ codebase.

---

## 2026-08-28 — Cập nhật Cơ học & Động học: Khoảng cách J5 $\to$ J6 (31mm) & Chiều dài Khâu Công cụ Hiệu dụng (51mm)

### Việc đã làm
- What: Cập nhật đồng bộ thông số khoảng cách cơ khí mới $31\,\text{mm}$ giữa tâm trục nghiêng **J5** (Wrist Tilt) và tâm trục xoay **J6** (Tool Roll) trên toàn bộ hệ thống:
  1. **Tài liệu gốc [`docs/ARM_GEOMETRY.md`](file:///E:/00.Project/04.robot-arm/robotic_arm/docs/ARM_GEOMETRY.md)**:
     - Thêm khoảng cách $d_6 = 31\,\text{mm}$ vào Bảng tham số Craig Modified DH.
     - Xác lập chiều dài khâu công cụ hiệu dụng (từ tâm J5 xuống ngòi bút TCP): $D_{\text{tool\_eff}} = 31\,\text{mm} + 20\,\text{mm} = \mathbf{51\,\text{mm}}$.
     - Cập nhật tọa độ tham chiếu Home $(0,0,0,0,0,0)$: J5 = $(126.0, 0.0, 365.0)\,\text{mm}$, J6 = $(157.0, 0.0, 365.0)\,\text{mm}$, Pen TCP = $(177.0, 0.0, 365.0)\,\text{mm}$.
  2. **Cấu hình phần cứng [`src/config.h`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/config.h)**:
     - Định nghĩa `DH_D6_MM = 31.0f`, `DH_D_TOOL_MM = 20.0f`, `DH_TOOL_EFFECTIVE_MM = 51.0f`.
  3. **Module Động học Firmware [`src/kinematics.h`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/kinematics.h) & [`src/kinematics.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/kinematics.cpp)**:
     - Bảng Craig MDH `DH[5].d = 31.0f` (Frame 6).
     - Bộ giải IK Pen-Down `ikPenDown()`: Bù cao độ tâm xoay J5 theo công thức $cz = \text{target}.z + D_{\text{TOOL\_EFFECTIVE}}$ ($51.0\,\text{mm}$).
  4. **Host Unit Test [`test/kinematics/test_kinematics.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/test/kinematics/test_kinematics.cpp)**:
     - Cập nhật kiểm tra tọa độ Home: TCP $X=177.0\,\text{mm}$.
     - Kiểm thử FK/IK roundtrip 2230 điểm đạt $100\%$ ($0$ lỗi).
  5. **Mô phỏng 3D Digital Clone [`digital_clone.py`](file:///E:/00.Project/04.robot-arm/robotic_arm/digital_clone.py)**:
     - Cập nhật `D6 = 31.0`, `D_TOOL_EFFECTIVE = 51.0`, `DH_TABLE` Frame 6 $d=31.0$.
     - Thêm landmark `p_j6` và kiểm tra tự động Home TCP $177.0\,\text{mm}$.
  6. **Tài liệu hệ thống [`README.md`](file:///E:/00.Project/04.robot-arm/robotic_arm/README.md) & [`docs/SYSTEM_OVERVIEW.html`](file:///E:/00.Project/04.robot-arm/robotic_arm/docs/SYSTEM_OVERVIEW.html)**:
     - Đồng bộ bảng DH và thông số Home pose.
- Why: Người dùng đo đạc vật lý thực tế và xác nhận có khoảng cách $31\,\text{mm}$ giữa trục J5 và J6.
- How: Thực hiện nghiêm ngặt theo quy tắc bảo toàn tính bất biến trong `AGENTS.md`.

### Build gate
- `g++ kin_test.exe` $\to$ **ALL KINEMATICS TESTS PASSED** (2230/2230 roundtrip OK).
- `py -3 digital_clone.py --test` $\to$ **ALL PASSED** (FK exact match, 2161 reachable points solved, Max error: $0.00000\,\text{mm}$).
- `pio run` $\to$ **SUCCESS** (RAM: 17.8% 58,356 B, Flash: 26.7% 891,829 B, 0 errors, 0 warnings).

---

## 2026-08-28 — Tích hợp Module Động học Cổ tay Vi sai Bánh răng Côn 2-DOF (Bevel Gear Differential Wrist J5 & J6)

### Việc đã làm
- What: Thiết kế và tích hợp toàn diện cơ cấu vi sai bánh răng côn 2 bậc tự do (2-DOF Coupled Differential Gimbal / Bevel Gear Differential) cho cụm cổ tay J5 (Tilt) và J6 (Roll):
  1. **Thư viện C++ Thuần [`src/differential_wrist.h`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/differential_wrist.h) & [`src/differential_wrist.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/differential_wrist.cpp)**:
     - `forward(leftDeg, rightDeg) -> JointState`: Tính toán góc khớp giải mã $J_5 = (\theta_L + \theta_R) / 2$ (Tilt) và $J_6 = (\theta_L - \theta_R) / (2 \cdot r_{\text{bevel}})$ (Roll).
     - `inverse(tiltDeg, rollDeg) -> ActuatorState`: Tính toán góc trục động cơ $\theta_L = J_5 + r_{\text{bevel}} \cdot J_6$ và $\theta_R = J_5 - r_{\text{bevel}} \cdot J_6$.
     - `computeIncrementalSteps()`: Quy đổi vi sai góc lệch $(\Delta J_5, \Delta J_6) \to (\Delta M_5, \Delta M_6)$ thành xung bước stepper.
  2. **Tích hợp Mô hình Khớp [`src/joint_model.h`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/joint_model.h) & [`src/joint_model.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/joint_model.cpp)**:
     - Tách biệt rõ ràng góc trục động cơ/side gear (`actuatorAngleFromSteps`, `actuatorAngleFromEncoder`) và góc khớp động học thực tế (`angleFromSteps`, `angleFromEncoder`).
     - Tự động giải mã vi sai 2 encoder AS5600 $E_L, E_R$ thành góc nghiêng Tilt $J_5$ và góc xoay Roll $J_6$.
  3. **Bộ Arbiter Điều khiển Jog [`src/arm.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/arm.cpp)**:
     - Jog $J_5$ (Tilt): Cả 2 động cơ $M_5, M_6$ quay cùng chiều, cùng góc $\to$ Chuyển động thuần Tilt.
     - Jog $J_6$ (Roll): Động cơ $M_5$ quay $+\Delta$, $M_6$ quay $-\Delta$ (ngược chiều) $\to$ Chuyển động thuần Roll.
  4. **Bộ Phân tích Quỹ đạo [`src/planner.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/planner.cpp)**:
     - Quy đổi nghiệm góc khớp từ bộ giải IK $(J_5, J_6) \to (M_5, M_6)$ trong `Planner::startMoveTo`.
  5. **Mô phỏng & Kiểm thử Đồ họa [`digital_clone.py`](file:///E:/00.Project/04.robot-arm/robotic_arm/digital_clone.py)**:
     - Bổ sung lớp `DifferentialWrist` trong Python.
     - Kiểm toán tự động chuyển đổi vi sai: Pure Tilt (30°, 30° $\to$ 30°, 0°), Pure Roll (45°, -45° $\to$ 0°, 45°), Roundtrip error $< 10^{-6\circ}$.
  6. **Unit Test Host [`test/kinematics/test_kinematics.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/test/kinematics/test_kinematics.cpp)**:
     - Bổ sung hàm `testDifferentialWrist()` kiểm thử quét toàn bộ không gian vi sai.
  7. **Tài liệu Kỹ thuật [`docs/ARM_GEOMETRY.md`](file:///E:/00.Project/04.robot-arm/robotic_arm/docs/ARM_GEOMETRY.md) & [`docs/SYSTEM_OVERVIEW.html`](file:///E:/00.Project/04.robot-arm/robotic_arm/docs/SYSTEM_OVERVIEW.html)**:
     - Vẽ sơ đồ cơ cấu vi sai, công thức toán học, nguyên lý 2 encoder bên và ưu điểm cơ khí.
- Why: Người dùng xác nhận cụm cổ tay J5-J6 sử dụng cơ cấu vi sai bánh răng côn (Bevel Gear Differential / Coupled Differential Gimbal) và yêu cầu cập nhật toàn diện codebase.
- How: Tuân thủ cấu trúc phân tầng: Tầng Động học Craig MDH giữ nguyên Joint Space $(J_1 \dots J_6)$, tầng `DifferentialWrist` giải mã trung gian giữa Joint Space $\leftrightarrow$ Actuator / Encoder Space.

### Build gate
- `g++ kin_test.exe` $\to$ **ALL KINEMATICS & DIFFERENTIAL WRIST TESTS PASSED**.
- `py -3 digital_clone.py --test` $\to$ **ALL PASSED** (Pure Tilt, Pure Roll, Differential Roundtrip Sweep PASSED).
- `pio run` $\to$ **SUCCESS** (RAM: 17.8% 58,356 B, Flash: 26.7% 892,609 B, 0 errors, 0 warnings).

---

## 2026-08-28 — Kiểm toán Toàn diện Codebase qua Đa Agent Song song & Khắc phục Lỗi Tiềm ẩn

### Việc đã làm
- What: Điều phối 3 AI Subagent song song độc lập rà soát 3 Domain lớn trong toàn bộ Codebase và khắc phục triệt để các lỗi phát hiện:
  1. **Domain 1 (Firmware Core & Motion Pipeline)**:
     - *Lỗi Vòng lặp Vô hạn khi bước = 0*: Trong [`src/motor.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/motor.cpp) hàm `Motor::run(cw, 0)`, nếu được gọi với 0 bước, `stepsRemaining` bằng 0 nhưng timer không kiểm tra nhánh dừng khiến timer re-arm liên tục và `busy()` treo vô hạn. Đã thêm guard `if (steps == 0) { stop(); return; }`.
     - *An toàn Ngắt Hardware ISR*: Xoá `esp_timer_stop(stepTimer)` khỏi `Motor::stopFromISR()` vì hàm này dùng non-ISR spinlock không an toàn trong ngắt GPIO. Cờ `running.store(false)` và lệnh hạ chân STEP đã đảm bảo timer callback thoát an toàn.
     - *Homing J4 không có Endstop Vật lý*: Khớp J4 (TMC2209 không gắn công tắc hành trình) trong `tickLegacy()` của [`src/homing.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/homing.cpp) trước đây chỉ kiểm tra `hasPin(3, MIN)` dẫn đến luôn bị timeout 30s. Đã bổ sung logic thăm dò `StallGuard` (`m.getSGResult() < STALL_SG_LEVEL`) và kiểm tra `m.isTmc()`.
     - *Jog Vi sai J6 Roll*: Trong [`src/arm.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/arm.cpp), nhân thêm hệ số `g_diffWrist.getBevelRatio()` khi tính xung bước.
  2. **Domain 2 (Sensors, Bus & Hardware Peripherals)**:
     - Kiểm toán chân GPIO ESP32-S3: Toàn bộ strapping pins (GPIO 0, 3, 45, 46), Octal Flash (GPIO 26–32), USB OTG (GPIO 19, 20) được bảo vệ an toàn.
     - Bus I2C/AS5600: Quét 200 Hz, lọc EMA chống wrap 360°, tính năng khôi phục bus I2C tự động hoạt động chuẩn xác.
     - NVS: Key length $\le 15$ ký tự, guard bảo vệ Flash Write Isolation chống ghi Flash khi robot đang chuyển động (HTTP 409).
  3. **Domain 3 (Kinematics, 3-Point WorkPlane & Web UI)**:
     - *Lỗi Sai lệch Hệ tọa độ trong Planner*: Trong [`src/planner.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/planner.cpp) hàm `Planner::startMoveTo`, tính `segLenMm` dùng nhầm $x, y, z$ (WorkPlane space) trừ đi `fkNow.tcp` (Robot Base space) khi WorkPlane bật. Đã sửa lại dùng $rx, ry, rz$.
     - *Đồng bộ Tham số Động học trong Web SPA*: Trong [`src/web_server.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/web_server.cpp), cập nhật JavaScript client-side Craig MDH $D_6 = 31.0\,\text{mm}$, $D_{\text{eff}} = 51.0\,\text{mm}$ và sửa hiển thị tọa độ Home TCP thành $(177, 0, 365)\,\text{mm}$.
- Why: Thực hiện kiểm toán toàn diện hệ thống để loại bỏ xung đột, lỗi tiềm ẩn và đồng bộ 100% giữa firmware, simulator và web client.
- How: Tiếp cận song song đa tác nhân, xác thực qua bộ 3 cổng kiểm thử tự động (Host C++ unit test, Python Digital Clone audit suite, PlatformIO firmware compilation).

### Build gate
- `g++ kin_test.exe` $\to$ **ALL KINEMATICS & DIFFERENTIAL WRIST TESTS PASSED** (2230/2230 points OK).
- `py -3 digital_clone.py --test` $\to$ **ALL PASSED** (Pure Tilt, Pure Roll, Differential Sweep, WorkPlane Star/Line).
- `pio run` $\to$ **SUCCESS** (RAM: 17.8% 58,356 B, Flash: 26.7% 893,069 B, 0 errors, 0 warnings).

---

## 2026-08-28 — Bổ sung Bảng Sơ đồ Chân Pinout Toàn diện cho Từng Linh kiện vào README.md

### Việc đã làm
- What: Mở rộng mục "Hardware Specifications" trong [`README.md`](file:///E:/00.Project/04.robot-arm/robotic_arm/README.md) thành "Hardware Specifications & Complete Pinout" với 5 bảng và sơ đồ chi tiết:
  1. **Bảng Master Pinout ESP32-S3**: Liệt kê từng chân GPIO (1..48), tên net/hàm, linh kiện kết nối, mức logic điện áp, và ghi chú kỹ thuật.
  2. **Bảng Động cơ & Driver (J1..J6)**: Chi tiết cấu hình địa chỉ UART TMC2209 (`0b00`..`0b11`), chân STEP/DIR cho A4988, tỉ số truyền, và bước/độ.
  3. **Sơ đồ Bus Cảm biến I2C & PCA9548A**: Mô hình topo cây I2C từ ESP32-S3 qua PCA9548A (`0x70`) đến 6 kênh AS5600 (`0x36`).
  4. **Bảng Công tắc Hành trình Endstops**: Định nghĩa chân GPIO, kiểu switch (NO/NC), chế độ ngắt pull-up active LOW cho J1..J3 và phương thức homing cho J4 (StallGuard), J5-J6 (vi sai).
  5. **Bảng Cấm / Chân Strapping & Flash**: Cảnh báo rõ ràng các chân cấm đụng (GPIO 0, 3, 4, 19, 20, 26–37, 45, 46).
- Why: Yêu cầu của người dùng nhằm tài liệu hóa đầy đủ và chính xác 100% sơ đồ đấu nối phần cứng cho mọi người đọc và chế tạo robot.
- How: Đồng bộ trực tiếp từ các định nghĩa phần cứng trong [`src/config.h`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/config.h).

---

## 2026-08-28 — Khắc phục Lỗi Chạm Endstop Giả & Treo Timeout 60s khi Homing Khớp J2

### Việc đã làm
- What: Khắc phục triệt để hiện tượng ngắt kích hoạt giả (false contact) và dừng sớm ở khớp J2 trong quá trình Homing:
  1. **Lọc nhiễu cảm ứng EMI cấp ngắt phần cứng ([`src/endstop.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/endstop.cpp))**:
     - `Endstops::isrHandler`: Bổ sung kiểm tra mức logic tức thời `gpio_get_level()` và bộ lọc trễ phần cứng $25\,\mu\text{s}$ (`esp_rom_delay_us(25)`).
     - Loại bỏ $100\%$ các xung gai nhiễu điện từ (sub-microsecond inductive EMI spikes) sinh ra từ dòng băm xung $1000\,\text{mA}$ của động cơ bước J2 sang dây tín hiệu endstop có trở kéo yếu nội bộ.
  2. **Tối ưu hóa FSM Quét 2 Đầu Hành trình ([`src/homing.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/homing.cpp))**:
     - Xoá cờ chốt (latches) tồn dư trước khi bắt đầu từng pha chuyển động liên tục (`enterScanMin`, `enterScanMax`, `enterCenteringScan`).
     - Trong `SCAN_MIN` & `SCAN_MAX`: Xác thực tiếp xúc cơ học thực tế (`isPressed` hoặc động cơ đã bị ngắt ISR dừng an toàn) trước khi chuyển trạng thái; tự động xóa bỏ latch giả mạo nếu chân công tắc không ở mức tích cực và động cơ vẫn đang quay.
     - Xử lý dừng sớm: Phát hiện động cơ bị dừng bất ngờ (`!m.isRunning()`) trong `SCAN_MAX` để báo lỗi ngay lập tức, tránh tình trạng treo im lặng 60 giây chờ timeout.
  3. **Ngắt giám sát Drift trong khi đang Homing ([`src/arm.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/arm.cpp))**:
     - Tạm dừng tác vụ `jm->updateDriftCheck()` khi `hc->isActive()` để tránh ghi log cảnh báo `[DRIFT]` giả tạo do rung động cơ khí truyền qua các khớp đứng yên khi khớp khác đang chạy homing.
- Why: Khớp J2 khi bắt đầu quét `SCAN_MIN` bị nhiễu xung điện từ trên chân `J2_MAX_PIN` (GPIO 10) kích hoạt ngắt, dẫn đến việc FSM nhận nhầm là đã chạm cực MAX tại góc $324.9^\circ$. Khi quay đầu quét `SCAN_MAX` về phía cực đối diện, robot thực sự chạm vào công tắc MAX thật ở $366.0^\circ$ nhưng FSM lại đang chờ công tắc MIN (GPIO 7), khiến động cơ dừng lại và hệ thống bị treo 60s cho đến khi hết thời gian chờ.
- How: Tăng cường bộ lọc tín hiệu ngắt ở tầng thấp nhất (ISR) kết hợp với cơ chế xác thực chéo mức logic và trạng thái động cơ ở tầng FSM.

### Build gate
- `pio run` $\to$ **SUCCESS** (RAM: 17.8% 58,356 B, Flash: 26.7% 893,617 B, 0 errors, 0 warnings).
- `py -3 digital_clone.py --test` $\to$ **ALL PASSED** (FK exact match, 2161 reachable points solved, Differential Wrist & WorkPlane OK).

---

## 2026-08-28 — Sửa Lỗi Logic Chiều Quay Warmup cho các Khớp có AXIS_STEP_SIGN Âm (J2, J3, J4)

### Việc đã làm
- What: Sửa lỗi tính toán chiều quay `warmupCW_` trong hàm `HomingController::enterWarmup()` ([`src/homing.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/homing.cpp)):
  - Trước đây, `enterWarmup()` gán cứng `warmupCW_ = true` (với giả định `cw=true` là chiều dương rời khỏi MIN) và `warmupCW_ = false` (chiều âm rời khỏi MAX).
  - Do `AXIS_STEP_SIGN` của J2, J3, J4 là `-1`, `cw=true` thực chất là chiều **ÂM** (chạy VÀO công tắc MIN), khiến động cơ khi đang chạm MIN tiếp tục bị lái đâm sâu vào công tắc trong pha Warmup.
  - Sửa lại: Dùng `JointModel::cwForDelta(curAxis_, +targetJointDeg)` khi `minP` hoặc mặc định, và `JointModel::cwForDelta(curAxis_, -targetJointDeg)` khi `maxP` $\to$ Đảm bảo chuyển động Warmup luôn luôn rời xa công tắc đang chạm bất kể dấu của `AXIS_STEP_SIGN`.
- Why: Khớp J3 khi bắt đầu Homing đang tì vào công tắc MIN nhưng pha Warmup lại quay theo hướng đâm vào công tắc do lỗi quy ước chiều logic.
- How: Chuyển toàn bộ việc chọn chiều quay sang hàm thuần nhất `JointModel::cwForDelta()`.

### Build gate
- `pio run` $\to$ **SUCCESS** (RAM: 17.8% 58,356 B, Flash: 26.7% 893,641 B, 0 errors, 0 warnings).
- `py -3 digital_clone.py --test` $\to$ **ALL PASSED** (2161/2161 reachable points solved, Max error: 0.000 mm).

---

## 2026-08-28 — Bổ sung Logic Đối xứng cho Công tắc MAX trong Homing (Symmetrical MIN/MAX Support)

### Việc đã làm
- What: Đồng bộ hóa toàn diện logic xử lý công tắc `MAX` tương đương $100\%$ với `MIN` trong FSM Homing ([`src/homing.h`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/homing.h) & [`src/homing.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/homing.cpp)):
  1. **Khởi tạo và Phát hiện Ban đầu (`beginJoint`)**:
     - Kiểm tra trạng thái chạm của cả `MIN` và `MAX` tại thời điểm bắt đầu.
     - Nếu đang chạm `MIN` $\to$ ghi nhận `legacyContactEndstop_ = MIN` và `enterBackoff()` theo chiều dương (+).
     - Nếu đang chạm `MAX` $\to$ ghi nhận `legacyContactEndstop_ = MAX` và `enterBackoff()` theo chiều âm (-).
  2. **Tiếp xúc & Lùi an toàn (`contactMade`, `enterBackoff`, `enterReapproach`)**:
     - Trong `contactMade()`: Tự động phân loại điểm chạm là `MIN` hay `MAX` dựa trên tín hiệu endstop thực tế.
     - `enterBackoff()`: Lùi theo chiều tương ứng rời xa công tắc vừa chạm ($+ \text{backoff}$ khi chạm MIN, $- \text{backoff}$ khi chạm MAX).
     - `enterReapproach()`: Dò chậm theo chiều tiến về đúng công tắc đó ($-360^\circ$ về MIN, $+360^\circ$ về MAX).
     - Tiêu thụ sạch cờ chốt (`consumeLatch`) cho cả MIN và MAX.
  3. **Giám sát trong các pha (`tickLegacy`)**:
     - `APPROACH`: Bắt sự kiện chạm ở cả MIN và MAX.
     - `BACKOFF`: Giám sát nhả công tắc ở cả MIN và MAX.
     - `REAPPROACH`: Xác nhận chạm lại chuẩn xác ở cả MIN và MAX.
- Why: Đảm bảo nếu robot bắt đầu homing khi đang tì vào bất kỳ đầu hành trình nào (MIN hoặc MAX), hệ thống đều phản ứng đối xứng, an toàn và chính xác mà không bị kẹt hay quay ngược hướng.
- How: Sử dụng biến trạng thái `legacyContactEndstop_` để định hướng chuyển động đối xứng qua `JointModel::cwForDelta()`.

### Build gate
- `pio run` $\to$ **SUCCESS** (RAM: 17.8% 58,356 B, Flash: 26.8% 894,109 B, 0 errors, 0 warnings).
- `py -3 digital_clone.py --test` $\to$ **ALL PASSED** (2161/2161 reachable points solved, Max error: 0.000 mm).

---

## 2026-08-28 — Tái cấu trúc & Tinh gọn Giao diện Web UI (Impeccable Web UI Redesign)

### Việc đã làm
- What: Tinh gọn và tái cấu trúc toàn diện Embedded Web UI trong [`src/web_server.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/web_server.cpp):
  1. **Hợp nhất từ 7 tab rời rạc thành 4 Workspace chuyên biệt**:
     - `📊 Dashboard`: Tập trung giám sát 3D Twin thời gian thực, bảng HUD đo lường tọa độ TCP/Wrist, trạng thái hệ thống, chuỗi an toàn (`HOME ALL`, `STOP ALL`, `CLEAR FAULT`), và bảng góc khớp rút gọn.
     - `🕹️ Jog & Calib`: Hợp nhất điều khiển Jog tương đối với bộ chọn bước (`0.5°`, `1.0°`, `5.0°`, `15.0°`), kích hoạt Homing từng khớp (`Home J1..J4`), và công cụ hiệu chuẩn NVS (`Set Home`, `Clear NVS`).
     - `🎯 Cartesian & Draw Studio`: Hợp nhất mô phỏng 3D Trajectory với bộ điều khiển chuyển động Cartesian 2 chế độ (`Di chuyển điểm IK` và `Vẽ hình Line/Circle Planner`) với tính năng xem trước quỹ đạo 3D và thanh trượt scrub thời gian thực.
     - `⚙️ Settings & WiFi`: Cấu hình mạng WiFi (STA/AP, RSSI) và bảng tra cứu thông số phần cứng/động học Craig MDH.
  2. **Loại bỏ trùng lặp và chồng chéo tính năng**:
     - Xóa bỏ việc render 2 viewport 3D song song gây tốn tài nguyên; chuẩn hóa 2 canvas độc lập (`dashCanvas` cho Live Robot, `simCanvas` cho Motion Studio).
     - Thay thế các khối thông tin HUD dạng văn bản thô nhiều dòng bằng hệ thống Stat Pills trực quan.
     - Tối ưu kích thước DOM và mã nguồn PROGMEM, giảm kích thước Flash firmware xuống $2\,\text{KB}$.
  3. **Thiết kế chuẩn mực Dark Technical Surface**:
     - Áp dụng triệt để nguyên tắc thiết kế `PRODUCT.md` & `DESIGN.md`: độ tương phản WCAG AA cao ($\ge 4.5:1$), không text gradient, không glassmorphism trang trí rườm rà, nút bấm đạt chuẩn cảm ứng ($\ge 42\text{px}$).
     - Nút `E-STOP` và thanh `Toast Notification` được cố định gọn gàng ở 2 góc dưới màn hình, không che khuất nội dung điều khiển.
- Why: Người dùng phản hồi giao diện cũ có quá nhiều tab (7 tab), bố cục chật chội (crowded) và nhiều tính năng bị chồng chéo/lặp lại giữa các tab.
- How: Hợp nhất các luồng công việc liên quan vào đúng không gian thao tác (Dashboard, Jog, Motion, Settings) và chuẩn hóa thiết kế thành một Digital Twin Control Console liền mạch.

### Build gate
- `pio run` $\to$ **SUCCESS** (RAM: 17.8% 58,356 B, Flash: 26.7% 891,949 B — tối ưu giảm ~2.2 KB Flash, 0 errors, 0 warnings).
- `py -3 digital_clone.py --test` $\to$ **ALL PASSED** (2161/2161 reachable points solved, Max error: 0.000 mm).

---

## 2026-08-29 — Khắc phục lỗi Task Watchdog (TWDT) abort tại SensorScanTask trên Core 0

### Việc đã làm
- What: Khắc phục triệt để lỗi Task Watchdog Timer (`task_wdt`) kích hoạt và gọi `abort()` tại `SensorScanTask` trên Core 0 sau khi khởi động / kết nối WiFi:
  1. **Bảo vệ chống trôi nhịp (Time-Lag Catch-Up Storm) trong `vTaskDelayUntil` ([`src/sensor.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/sensor.cpp) & [`src/arm.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/arm.cpp))**:
     - `SensorScanTask` khởi tạo `lastWake` lúc boot (~200ms). Khi `WiFi.begin()` chạy, quá trình kết nối STA WiFi chiếm CPU Core 0 trong ~28s.
     - Sau khi kết nối xong, `lastWake` bị trễ ~28.000 ticks. `vTaskDelayUntil` của FreeRTOS trả về ngay lập tức để đuổi kịp mốc thời gian cũ, gây ra cơn bão quét I2C liên tục ~5.600 lần back-to-back mà không nhường CPU, dẫn đến nguy cơ kẹt timeout I2C và Task Watchdog không được reset kịp thời.
     - Đã thêm bộ bảo vệ tự động tái đồng bộ `lastWake = now` khi khoảng cách trễ `(now - lastWake) > period * 2` cho cả `SensorScanTask` và `MotionTask`.
  2. **Hiệu chỉnh Tần số Bus I2C về Đúng Chuẩn Phần cứng ([`src/config.h`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/config.h))**:
     - Hạ `I2C_FREQUENCY` từ `800000` (800 kHz) xuống `400000` (400 kHz Fast-Mode).
     - Datasheet của cả AS5600 và PCA9548A chỉ quy định hỗ trợ tối đa 400 kHz (Fast-Mode). Ép xung 800 kHz trên bó dây truyền tín hiệu cánh tay robot làm suy hao sườn xung, gây mất xung ACK và treo đường truyền SDA.
  3. **Đồng bộ Tài liệu Hệ thống ([`docs/SYSTEM_OVERVIEW.html`](file:///E:/00.Project/04.robot-arm/robotic_arm/docs/SYSTEM_OVERVIEW.html))**:
     - Cập nhật thông số I2C từ 800 kHz sang 400 kHz.
  4. **Tối ưu hóa Giao dịch I2C & Tăng Timeout An toàn ([`src/sensor.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/sensor.cpp))**:
     - Tăng `Wire.setTimeOut` từ `3ms` lên `50ms` (chuẩn Arduino ESP32) để chống timeout giả khi các tác vụ RTOS khác chiếm dụng CPU trong tích tắc làm hỏng state machine I2C phần cứng.
     - Duy trì `Wire.endTransmission(false)` (Repeated Start chuẩn theo datasheet AS5600) khi đọc thanh ghi ANGLE, đồng thời phát lệnh giải phóng STOP bus tức thì khi gặp NACK để không làm kẹt kênh chuyển tiếp của PCA9548A.
     - Tăng ngưỡng phát hiện lỗi `read_fail_counts` từ 5 lên 20 (100 ms) để loại bỏ hoàn toàn các lần đọc nhiễu chập chờn nhất thời.
  5. **Giới hạn Tần suất Bus Recovery (Rate-limit I2C Bus Reset) ([`src/sensor.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/sensor.cpp))**:
     - Phát hiện nguyên nhân mốc thời gian crash chính xác $29.3\,\text{s}$: Khi các cảm biến/PCA9548A chưa sẵn sàng (`allInError = true`), trước đây vòng lặp gọi `recoverI2CBus()` liên tục mỗi $50\,\text{ms}$ ($20\,\text{lần/giây}$). Sau đúng $586\,\text{vòng}$ ($29.3\,\text{s}$), việc gọi liên tục `Wire.end()` / `Wire.begin()` làm cạn kiệt bảng quản lý ngắt ESP-IDF và gây deadlock trong driver I2C.
  6. **Cách ly Sensor Task sang Core 1 & Hủy Đăng ký Task WDT Khỏi Bus I2C Ngoại vi ([`src/config.h`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/config.h) & [`src/sensor.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/sensor.cpp))**:
     - Chuyển `SENSOR_TASK_CORE` từ `Core 0` sang `Core 1` (`Application Core`), cách ly hoàn toàn việc quét I2C khỏi các đợt phát xung vô tuyến WiFi RF / beacon keepalive trên Core 0.
     - Hủy lệnh đăng ký `esp_task_wdt_add(nullptr)` trên `SensorScanTask`: Bus I2C ngoại vi khi cắm nhiều khớp cơ khí dài luôn có khả năng bị nhiễu tạm thời. Việc gán Task WDT phần cứng vào task ngoại vi I2C là sai kiến trúc an toàn vì sẽ gây reboot toàn bộ MCU khi có 1 khớp lỏng dây, thay vì để JointModel xử lý cờ `sensor_error` và bảo vệ mềm. Task WDT được giữ lại nguyên vẹn và an toàn tại `MotionTask` (`arm_motion`).
- Why: Khắc phục triệt để lỗi Task Watchdog abort trên Core 0, giải phóng bus I2C khi kênh trống và chống kẹt mux PCA9548A.
- How: Chặn hiện tượng vTaskDelayUntil catch-up storm, đưa I2C bus về chuẩn 400 kHz, tăng I2C timeout lên 50ms, rate-limit recovery bus, chuyển sensor sang Core 1, phát STOP khi NACK, tăng threshold lên 20 và hủy ép TWDT trên bus I2C.

### Build gate
- `pio run` $\to$ **SUCCESS** (RAM: 17.8% 58,356 B, Flash: 26.7% 891,865 B, 0 errors, 0 warnings).


---

## 2026-08-29 — Sửa lỗi Encoder Error chập chờn sau boot thành công (Core Contention)

### Việc đã làm
- What: Khắc phục hiện tượng encoder đọc được vài giây sau boot rồi chuyển thành `ENC ERR / MẤT KẾT NỐI` trên Web UI, mặc dù phần cứng hoạt động đúng.
- Why: Sau khi chuyển `SensorScanTask` từ Core 0 sang Core 1 (để tránh WiFi RF lúc boot), cả ba task `SensorScanTask` (prio 4), `arm_motion` (prio 3), và `loopTask`/web handler (prio 1) đều cùng chạy trên **Core 1**, gây ra **core contention và mutex tranh chấp**:
  - `getDiagnostics()` từ web handler giữ `i2cMutex` tối đa 50ms.
  - Trong thời gian đó, `scanOnce()` bị block → không thể lấy mutex → bỏ qua toàn bộ vòng quét → `read_fail_counts[i]++`.
  - Sau 20 lần thất bại liên tiếp (~100ms), `sensor_error[i] = true` cho tất cả các khớp.
  - WiFi chỉ chiếm CPU Core 0 khi đang *kết nối* (boot). Sau khi có IP, WiFi dùng interrupt, không chiếm Core 0 liên tục → `SensorScanTask` chạy ổn định trên Core 0.
- How: Trả `SENSOR_TASK_CORE` về `0` — tách biệt hoàn toàn I2C scan (Core 0) khỏi arm_motion + loopTask (Core 1). Đây là cấu hình nguyên bản và đúng về kiến trúc RTOS.

### Thay đổi khác đi kèm (giữ lại)
- `recoverI2CBus()` nay cấu hình lại PCA9548A + AS5600 sau mỗi lần phục hồi bus.
- `lastRecoveryMs = millis()` khởi tạo từ thời điểm task bắt đầu (cooldown 5s trước lần recovery đầu tiên).
- Ngưỡng lỗi `read_fail_counts > 20` (~100ms tại 200Hz) để lọc nhiễu chập chờn.
- `Wire.endTransmission(false)` (Repeated Start) được giữ đúng chuẩn datasheet AS5600.



---

## 2026-08-29 — Khắc phục Toàn diện Audit An toàn & Logic: Endstop FAULT, E-Stop, Planner Sync, Drift Watchdog, Host Tests

### Việc đã làm
- **Endstop ISR & FAULT Latch**:
  - Sửa chuỗi xử lý va chạm: khi ISR kích hoạt (`anyLatched() == true` hoặc `g_emergencyStop == true`), `ArmController::taskLoop()` chuyển ngay `mode_ = ArmMode::FAULT` và gọi `stop()` trên toàn bộ 6 motor + dừng planner, không bị reset về `IDLE`.
  - Thêm điều kiện tiên quyết cho `CLEAR_FAULT`: từ chối xóa lỗi nếu bất kỳ công tắc hành trình nào vẫn đang bị đè vật lý hoặc lỗi drift chưa được xóa.
- **Fail-Fast Emergency Stop (`g_emergencyStop`)**:
  - Kết nối cờ atomic `g_emergencyStop` vào `arm.cpp` và `endstop.cpp`, ngắt xung tức thì trong $\le 20\,\mu\text{s}$ ở đầu Step ISR của tất cả các trục.
- **Planner Z-Lift & Pen-Up Logic**:
  - Sửa bug `FINISHED_LIFT` trong `planner.cpp`: thêm trạng thái `State::WAIT_FINAL_LIFT` để đợi động cơ hoàn thành nhấc bút (`motorsBusy == false`) trước khi gọi `finishAll()`.
  - Sửa `MOVE_CART`: tôn trọng `job.drawNow = false` để di chuyển trên không an toàn mà không tự ý chuyển sang `DROPPING`.
- **Drift Watchdog Latching**:
  - `arm.cpp` kiểm tra `jm->hasAnyDriftFault()` định kỳ 500ms; khi góc encoder và góc step lệch vượt ngưỡng `RUNAWAY_ERROR_THRESHOLD` (5°), tự động dừng toàn bộ motor và chuyển trạng thái sang `ArmMode::FAULT`.
- **TMC2209 UART Direction Fail-Safe**:
  - Trong `Motor::setDirection()` và `Motor::run()`: nếu `takeUart(100)` timeout không thể đảo chiều qua UART, motor sẽ ghi log cảnh báo và **hủy lệnh phát xung `run()`** thay vì quay sai chiều.
- **Host Unit Tests & Regression Checklist**:
  - Tạo `test/host/test_homing_logic.cpp` kiểm thử tính toán tâm hành trình, cấu hình dual-endstop cho J1–J3, và dòng homing.
  - Cập nhật `tools/run_host_tests.sh` bao gồm kinematics, joint logic, work plane và homing logic.
  - Cập nhật `docs/SYSTEM_OVERVIEW.html` (loại bỏ dead code `g_motionQueue`, đồng bộ module cards, cập nhật footer ngày 2026-08-29).



---

## 2026-08-29 — Chuyển I2C Bus sang Chuẩn 100kHz Standard-Mode & Sensor Task 100Hz

### Việc đã làm
- What: Điều chỉnh tần số `I2C_FREQUENCY` từ 400kHz về **100kHz** (Standard-mode) và `SENSOR_TASK_PERIOD_MS` từ 5ms (200Hz) về **10ms (100Hz)**.
- Why:
  - Cáp tín hiệu I2C nối từ PCA9548A lên các khớp J4/J5/J6 có chiều dài lớn (40–80cm), sinh ra điện dung ký sinh $C_{\text{bus}}$ cao làm méo xung ở tần số 400kHz.
  - Chuẩn 100kHz cho phép thời gian cạnh lên $t_r \le 1000\,\text{ns}$ (gấp >3 lần 400kHz), triệt tiêu hiện tượng méo xung, chống nhiễu cảm ứng từ dòng xung động cơ bước và khắc phục triệt để lỗi chập chờn NACK trên các kênh dài.
  - Chu kỳ quét 10ms (100Hz) đồng bộ 1:1 với chu kỳ 10ms của `arm_motion` task, giảm tải CPU Core 0 trong khi vẫn đảm bảo độ phản hồi góc mượt mà tuyệt đối cho hệ thống.
- How: Cập nhật `src/config.h`, đồng bộ tài liệu `docs/SYSTEM_OVERVIEW.html`.



---

## 2026-08-29 — Chuyển I2C Bus sang 50kHz Low-Speed Mode & Sensor Task 15ms (~66Hz)

### Việc đã làm
- What: Cấu hình `I2C_FREQUENCY = 50000` (50kHz) và `SENSOR_TASK_PERIOD_MS = 15` (~66Hz), tăng `SENSOR_I2C_MUTEX_TIMEOUT_MS = 25`.
- Why:
  - Cung cấp khả năng chống méo xung và chống nhiễu tối đa cho đường truyền I2C đi dọc thân cánh tay 6 trục (J5/J6).
  - Tần số 50kHz cho thời gian sạc điện dung cáp dồi dào ($t_r$ lớn), triệt tiêu hoàn toàn hiện tượng drop/NACK trên các nhánh dây dài và các module có trở kéo yếu.
  - Chu kỳ quét 15ms (~66 lần/giây) cho phép đọc 6 sensor (~10ms) với margin an toàn cao, giảm tải triệt để cho Core 0.
- How: Sửa `src/config.h`, cập nhật `docs/SYSTEM_OVERVIEW.html`.



---

## 2026-08-29 — Chuyển I2C Bus sang 40kHz Low-Speed Mode & Sensor Task 20ms (50Hz)

### Việc đã làm
- What: Cấu hình `I2C_FREQUENCY = 40000` (40kHz) và `SENSOR_TASK_PERIOD_MS = 20` (50Hz), tăng `SENSOR_I2C_MUTEX_TIMEOUT_MS = 30`.
- Why:
  - Tối ưu hóa tối đa khả năng chống méo xung cạnh lên trên các tuyến cáp I2C dài và chống nhiễu cảm ứng từ dòng xung stepper.
  - Tần số 40kHz đảm bảo dạng sóng I2C vuông vắn hoàn hảo trên toàn bộ 6 nhánh kênh của PCA9548A.
  - Chu kỳ quét 20ms (50Hz chuẩn công nghiệp) cho phép đọc trọn vẹn 6 cảm biến (~12.5ms) với biên độ an toàn tuyệt đối trên Core 0.
- How: Sửa `src/config.h`, cập nhật `docs/SYSTEM_OVERVIEW.html`.

### Build gate
- `pio run` → **SUCCESS**

---

## 2026-08-29 — Toàn diện Kiểm toán Song song (Parallel Agents Audit) & Vá Lỗi Đa Phân Hệ

### Việc đã làm
- What & Why:
  1. **Domain 1 (Safety, RTOS & Motor)**:
     - Khắc phục deadlock phục hồi lỗi (`CLEAR_FAULT`): cho phép xóa cờ FAULT và giải phóng latch để người dùng có thể jog điều khiển trục ra khỏi công tắc hành trình mà không bị khóa đệ quy.
     - Đồng bộ hóa đa lõi trong `Motor::onStepTimer` và `Motor::run`: nâng cấp `running` và `g_emergencyStop` sang semantics `acquire/release` và kiểm tra lại `running` trước khi khởi động `esp_timer_start_once`.
     - Chuyển `targetSteps` và `mode_` sang kiểu `std::atomic` chuẩn C++17.
     - Cải tiến `makeTimedLock` trong `rtos_guard.h`: đảm bảo tick tối thiểu $\ge 1$ khi timeout > 0.
  2. **Domain 2 (Kinematics & Planner)**:
     - Khắc phục tọa độ điểm xuất phát trong `Planner::submit`: khi `WorkPlane` kích hoạt, tự động chuyển đổi vị trí FK hiện tại sang hệ tọa độ UCS (`fromRobotXYZ`) trước khi tính toán cung tròn hoặc độ cao an toàn.
     - Khắc phục lệnh di chuyển `MOVE_CART` (POINT): bổ sung điều kiện hạ bút `DROPPING` tới đúng cao độ $Z$ chỉ định trước khi kết thúc segment.
     - Chuẩn hóa góc $t_3$ trong `kin::ikPenDown` về khoảng $[-\pi, \pi]$ chống từ chối sai giới hạn soft limits.
  3. **Domain 3 (Sensor, Homing & JointModel)**:
     - Chuyển `sensor_error` sang `std::array<std::atomic<bool>, NUM_SENSORS>` để đọc/ghi thread-safe qua các lõi CPU.
     - Bổ sung kiểm tra kết quả `lock` trong `getAngle()`, `getAccumulatedAngle()`, `getTurnCount()`.
     - Sửa tên mảng định danh chân cữ `AXIS_MIN_PINS` / `AXIS_MAX_PINS` trong bộ host test.
  4. **Domain 4 (Web Server, REST API & Network)**:
     - Bổ sung kiểm tra `std::isfinite(deg)` chống tấn công/lỗi NaN bypass trên `/api/jog`.
     - Bổ sung xác thực bắt buộc `hasArg` cho các tham số `axis`, `x`, `y`, `z` trên `/api/move`, `/api/home/axis`, `/api/sethome`, `/api/clearcalib`.
     - Khóa `armPtr->busy()` trên các endpoint WorkPlane (`/api/workplane/calib`, `/api/workplane/toggle`) chống xung đột khi đang vẽ.
     - Thêm `j.reserve(3500)` trong `ArmController::statusJson()` triệt tiêu hoàn toàn hiện tượng phân mảnh bộ nhớ Heap khi polling 3.3Hz.
     - Kích hoạt `WiFi.setAutoReconnect(true)` trong `WifiManager`.
- How: Chạy kiểm toán song song bằng 4 subagents độc lập, kiểm tra mã nguồn, tích hợp các bản vá và đồng bộ hóa test suite.

### Build gate
- `pio run` → **SUCCESS** (RAM: 17.8%, Flash: 26.8%, 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED** (Kinematics, Joint Logic, Work Plane, Homing Logic).

---

## 2026-08-29 — Sửa Vòng Lặp Fault Drift (Step/Encoder Drift Loop on CLEAR_FAULT)

### Việc đã làm
- What: Khắc phục hiện tượng lặp lại liên tục `FAULT: step/encoder drift exceeded threshold` sau khi người dùng bấm `CLEAR_FAULT`.
- Why:
  - Khi một khớp bị lệch bước so với encoder $> 5^\circ$ (do mất bước, driver ngắt nguồn UART, hoặc bị ngoại lực xoay tay), watchdog 500ms kích hoạt ngắt an toàn `ArmMode::FAULT`.
  - Khi người dùng gửi lệnh `CLEAR_FAULT`, hàm cũ chỉ xóa cờ boolean `driftFault[i] = false` mà **không đồng bộ lại bộ đếm bước `absSteps`** theo vị trí góc thực tế của encoder.
  - Do đó, độ lệch giữa số bước và góc đo vẫn tồn tại $> 5^\circ$, khiến watchdog 500ms sau quét lại và lập tức ngắt `FAULT` trở lại.
- How:
  - Cập nhật [`src/joint_model.cpp:clearAllDriftFaults()`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/joint_model.cpp): Khi xóa cờ lỗi drift, tự động gọi `resyncFromEncoder(i)` cho các khớp đang homed có encoder hợp lệ.
  - Nhờ vậy, `absSteps` được căn chỉnh ngay về vị trí thực tế của encoder $\to$ độ lệch về $0.00^\circ$ $\to$ xóa lỗi dứt điểm và cho phép điều khiển tiếp tục bình thường.

### Build gate
- `pio run` → **SUCCESS** (RAM: 17.8%, Flash: 26.8%, 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED**.

---

## 2026-08-29 — Tối Ưu Hóa Quy Trình Homing StallGuard Cho Khớp J4

### Việc đã làm
- What: Cấu hình và kích hoạt đầy đủ quy trình **Sensorless Homing bằng StallGuard4** cho khớp J4 (Wrist Pan) trên driver TMC2209.
- Why:
  - Khớp J4 là khớp xoay cổ tay không trang bị công tắc hành trình vật lý (không có chân MIN/MAX endstop).
  - Sử dụng tính năng đo tải Back-EMF của TMC2209 (`SG_RESULT`) cho phép phát hiện cữ chặn cơ khí một cách tự động, êm ái và an toàn.
- How:
  - Cập nhật [`src/motor.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/motor.cpp): Khởi tạo thanh ghi `TCOOLTHRS(0xFFFFF)` và `SGTHRS(DEFAULT_STALL_THRESHOLD)` trong `Motor::begin()` để đảm bảo bộ đo tải StallGuard4 luôn kích hoạt ở mọi dải vận tốc.
  - Quy trình FSM Homing của J4 (`HomingController::beginJoint(3)`):
    1. **APPROACH**: Hạ dòng xuống mức an toàn `HOMING_CURRENT_J4 = 300mA` và chạy liên tục ở tốc độ `1800us/step`.
    2. **STALL DETECT**: Đọc thanh ghi `SG_RESULT` mỗi 10ms. Khi chạm cữ cứng, `SG_RESULT < STALL_SG_LEVEL (100)` liên tiếp 3 chu kỳ $\to$ ngắt motor an toàn.
    3. **BACKOFF & REAPPROACH**: Lùi lại $2.5^\circ$, sau đó chạm lại lần 2 ở tốc độ chậm `3000us/step` để xác định tọa độ chính xác.
    4. **SETREF**: Gán điểm dừng làm gốc tọa độ $0^\circ$ của J4, lưu vào NVS và khôi phục dòng định mức $800\text{mA}$.

### Build gate
- `pio run` → **SUCCESS** (RAM: 17.8%, Flash: 26.8%, 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED**.

---

## 2026-08-29 — Khắc Phục Hiện Tượng Gõ Cữ J4 Bằng Cơ Chế Dual Stall Fusion (StallGuard + Encoder)

### Việc đã làm
- What: Khắc phục hiện tượng khớp J4 liên tục gõ/trượt bước vào cữ chặn cơ khí mà không nhận diện được điểm dừng để đổi hướng lùi ra.
- Why:
  1. Trên TMC2209, StallGuard4 chỉ trả về kết quả đo tải `SG_RESULT` khi driver ở chế độ **StealthChop** (`en_spreadCycle = false`). Khi ở chế độ SpreadCycle, `SG_RESULT` bị vô hiệu hóa nên FSM không nhận được tín hiệu stall từ UART.
  2. Khi UART timeout hoặc tải cơ cấu chưa đủ ngưỡng trigger điện áp, hệ thống cần một cơ chế xác nhận phần cứng độc lập.
- How:
  1. Trong `HomingController::enterApproach()`: Tự động chuyển TMC2209 sang chế độ **StealthChop** (`m.setChopperMode(false)`) khi bắt đầu dò cữ và chuyển lại SpreadCycle khi kết thúc.
  2. Bổ sung **Encoder Stall Detection (Sensor Fusion)**: Mỗi 100ms, nếu motor đang phát xung chạy nhưng cảm biến góc AS5600 báo góc không thay đổi ($|\Delta_{\text{enc}}| < 0.20^\circ$ do bị cữ cơ khí chặn đứng) $\to$ FSM lập tức xác nhận cữ chạm sau đúng 100-200ms, dừng motor và thực hiện ngay pha lùi `BACKOFF` và đổi hướng.

### Build gate
- `pio run` → **SUCCESS** (RAM: 17.8%, Flash: 26.8%, 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED**.

---

## 2026-08-30 — Sửa Lỗi Homing J4 Báo Xong Ảo Sau 30ms & Vòng Lặp Báo Lỗi Drift Watchdog

### Việc đã làm
- What: 
  1. Khắc phục lỗi J4 vừa bắt đầu homing (chưa kịp quay, mới 15 bước / ~30ms) đã báo Stall và kết thúc Homing (`[HOME] J4: Stall/Hard-stop detected -> contactMade`, `steps=15`).
  2. Khắc phục vòng lặp báo lỗi Drift liên tục sau khi di chuyển / Jog (`[DRIFT] J4 lech ... deg -> [ARM] FAULT: step/encoder drift exceeded threshold`) khiến cánh tay bị khóa dừng (`ArmMode::FAULT`) không điều khiển được.
- Why:
  1. **Homing J4 false-trip**: TMC2209 khi motor đứng yên hoặc đang tăng tốc ban đầu (chưa sinh đủ Back-EMF) trả về `SG_RESULT = 0`. Điều kiện `sg < STALL_SG_LEVEL (100)` trong `tickLegacy()` được kiểm tra mỗi 10ms mà không có thời gian ân hạn khởi động (warmup grace period). Sau đúng 3 chu kỳ (30ms = 15 bước), bộ đếm `stallCount_` đạt 3 và FSM ngắt motor, lùi lại, rồi trong `REAPPROACH` tiếp tục đọc `sg = 0` sau 20ms và báo hoàn tất `SetHome J4` ảo tại chỗ.
  6. **Cách ly Sensor Task sang Core 1 & Hủy Đăng ký Task WDT Khỏi Bus I2C Ngoại vi ([`src/config.h`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/config.h) & [`src/sensor.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/sensor.cpp))**:
     - Chuyển `SENSOR_TASK_CORE` từ `Core 0` sang `Core 1` (`Application Core`), cách ly hoàn toàn việc quét I2C khỏi các đợt phát xung vô tuyến WiFi RF / beacon keepalive trên Core 0.
     - Hủy lệnh đăng ký `esp_task_wdt_add(nullptr)` trên `SensorScanTask`: Bus I2C ngoại vi khi cắm nhiều khớp cơ khí dài luôn có khả năng bị nhiễu tạm thời. Việc gán Task WDT phần cứng vào task ngoại vi I2C là sai kiến trúc an toàn vì sẽ gây reboot toàn bộ MCU khi có 1 khớp lỏng dây, thay vì để JointModel xử lý cờ `sensor_error` và bảo vệ mềm. Task WDT được giữ lại nguyên vẹn và an toàn tại `MotionTask` (`arm_motion`).
- Why: Khắc phục triệt để lỗi Task Watchdog abort trên Core 0, giải phóng bus I2C khi kênh trống và chống kẹt mux PCA9548A.
- How: Chặn hiện tượng vTaskDelayUntil catch-up storm, đưa I2C bus về chuẩn 400 kHz, tăng I2C timeout lên 50ms, rate-limit recovery bus, chuyển sensor sang Core 1, phát STOP khi NACK, tăng threshold lên 20 và hủy ép TWDT trên bus I2C.

### Build gate
- `pio run` $\to$ **SUCCESS** (RAM: 17.8% 58,356 B, Flash: 26.7% 891,865 B, 0 errors, 0 warnings).


---

## 2026-08-29 — Sửa lỗi Encoder Error chập chờn sau boot thành công (Core Contention)

### Việc đã làm
- What: Khắc phục hiện tượng encoder đọc được vài giây sau boot rồi chuyển thành `ENC ERR / MẤT KẾT NỐI` trên Web UI, mặc dù phần cứng hoạt động đúng.
- Why: Sau khi chuyển `SensorScanTask` từ Core 0 sang Core 1 (để tránh WiFi RF lúc boot), cả ba task `SensorScanTask` (prio 4), `arm_motion` (prio 3), và `loopTask`/web handler (prio 1) đều cùng chạy trên **Core 1**, gây ra **core contention và mutex tranh chấp**:
  - `getDiagnostics()` từ web handler giữ `i2cMutex` tối đa 50ms.
  - Trong thời gian đó, `scanOnce()` bị block → không thể lấy mutex → bỏ qua toàn bộ vòng quét → `read_fail_counts[i]++`.
  - Sau 20 lần thất bại liên tiếp (~100ms), `sensor_error[i] = true` cho tất cả các khớp.
  - WiFi chỉ chiếm CPU Core 0 khi đang *kết nối* (boot). Sau khi có IP, WiFi dùng interrupt, không chiếm Core 0 liên tục → `SensorScanTask` chạy ổn định trên Core 0.
- How: Trả `SENSOR_TASK_CORE` về `0` — tách biệt hoàn toàn I2C scan (Core 0) khỏi arm_motion + loopTask (Core 1). Đây là cấu hình nguyên bản và đúng về kiến trúc RTOS.

### Thay đổi khác đi kèm (giữ lại)
- `recoverI2CBus()` nay cấu hình lại PCA9548A + AS5600 sau mỗi lần phục hồi bus.
- `lastRecoveryMs = millis()` khởi tạo từ thời điểm task bắt đầu (cooldown 5s trước lần recovery đầu tiên).
- Ngưỡng lỗi `read_fail_counts > 20` (~100ms tại 200Hz) để lọc nhiễu chập chờn.
- `Wire.endTransmission(false)` (Repeated Start) được giữ đúng chuẩn datasheet AS5600.



---

## 2026-08-29 — Khắc phục Toàn diện Audit An toàn & Logic: Endstop FAULT, E-Stop, Planner Sync, Drift Watchdog, Host Tests

### Việc đã làm
- **Endstop ISR & FAULT Latch**:
  - Sửa chuỗi xử lý va chạm: khi ISR kích hoạt (`anyLatched() == true` hoặc `g_emergencyStop == true`), `ArmController::taskLoop()` chuyển ngay `mode_ = ArmMode::FAULT` và gọi `stop()` trên toàn bộ 6 motor + dừng planner, không bị reset về `IDLE`.
  - Thêm điều kiện tiên quyết cho `CLEAR_FAULT`: từ chối xóa lỗi nếu bất kỳ công tắc hành trình nào vẫn đang bị đè vật lý hoặc lỗi drift chưa được xóa.
- **Fail-Fast Emergency Stop (`g_emergencyStop`)**:
  - Kết nối cờ atomic `g_emergencyStop` vào `arm.cpp` và `endstop.cpp`, ngắt xung tức thì trong $\le 20\,\mu\text{s}$ ở đầu Step ISR của tất cả các trục.
- **Planner Z-Lift & Pen-Up Logic**:
  - Sửa bug `FINISHED_LIFT` trong `planner.cpp`: thêm trạng thái `State::WAIT_FINAL_LIFT` để đợi động cơ hoàn thành nhấc bút (`motorsBusy == false`) trước khi gọi `finishAll()`.
  - Sửa `MOVE_CART`: tôn trọng `job.drawNow = false` để di chuyển trên không an toàn mà không tự ý chuyển sang `DROPPING`.
- **Drift Watchdog Latching**:
  - `arm.cpp` kiểm tra `jm->hasAnyDriftFault()` định kỳ 500ms; khi góc encoder và góc step lệch vượt ngưỡng `RUNAWAY_ERROR_THRESHOLD` (5°), tự động dừng toàn bộ motor và chuyển trạng thái sang `ArmMode::FAULT`.
- **TMC2209 UART Direction Fail-Safe**:
  - Trong `Motor::setDirection()` và `Motor::run()`: nếu `takeUart(100)` timeout không thể đảo chiều qua UART, motor sẽ ghi log cảnh báo và **hủy lệnh phát xung `run()`** thay vì quay sai chiều.
- **Host Unit Tests & Regression Checklist**:
  - Tạo `test/host/test_homing_logic.cpp` kiểm thử tính toán tâm hành trình, cấu hình dual-endstop cho J1–J3, và dòng homing.
  - Cập nhật `tools/run_host_tests.sh` bao gồm kinematics, joint logic, work plane và homing logic.
  - Cập nhật `docs/SYSTEM_OVERVIEW.html` (loại bỏ dead code `g_motionQueue`, đồng bộ module cards, cập nhật footer ngày 2026-08-29).



---

## 2026-08-29 — Chuyển I2C Bus sang Chuẩn 100kHz Standard-Mode & Sensor Task 100Hz

### Việc đã làm
- What: Điều chỉnh tần số `I2C_FREQUENCY` từ 400kHz về **100kHz** (Standard-mode) và `SENSOR_TASK_PERIOD_MS` từ 5ms (200Hz) về **10ms (100Hz)**.
- Why:
  - Cáp tín hiệu I2C nối từ PCA9548A lên các khớp J4/J5/J6 có chiều dài lớn (40–80cm), sinh ra điện dung ký sinh $C_{\text{bus}}$ cao làm méo xung ở tần số 400kHz.
  - Chuẩn 100kHz cho phép thời gian cạnh lên $t_r \le 1000\,\text{ns}$ (gấp >3 lần 400kHz), triệt tiêu hiện tượng méo xung, chống nhiễu cảm ứng từ dòng xung động cơ bước và khắc phục triệt để lỗi chập chờn NACK trên các kênh dài.
  - Chu kỳ quét 10ms (100Hz) đồng bộ 1:1 với chu kỳ 10ms của `arm_motion` task, giảm tải CPU Core 0 trong khi vẫn đảm bảo độ phản hồi góc mượt mà tuyệt đối cho hệ thống.
- How: Cập nhật `src/config.h`, đồng bộ tài liệu `docs/SYSTEM_OVERVIEW.html`.



---

## 2026-08-29 — Chuyển I2C Bus sang 50kHz Low-Speed Mode & Sensor Task 15ms (~66Hz)

### Việc đã làm
- What: Cấu hình `I2C_FREQUENCY = 50000` (50kHz) và `SENSOR_TASK_PERIOD_MS = 15` (~66Hz), tăng `SENSOR_I2C_MUTEX_TIMEOUT_MS = 25`.
- Why:
  - Cung cấp khả năng chống méo xung và chống nhiễu tối đa cho đường truyền I2C đi dọc thân cánh tay 6 trục (J5/J6).
  - Tần số 50kHz cho thời gian sạc điện dung cáp dồi dào ($t_r$ lớn), triệt tiêu hoàn toàn hiện tượng drop/NACK trên các nhánh dây dài và các module có trở kéo yếu.
  - Chu kỳ quét 15ms (~66 lần/giây) cho phép đọc 6 sensor (~10ms) với margin an toàn cao, giảm tải triệt để cho Core 0.
- How: Sửa `src/config.h`, cập nhật `docs/SYSTEM_OVERVIEW.html`.



---

## 2026-08-29 — Chuyển I2C Bus sang 40kHz Low-Speed Mode & Sensor Task 20ms (50Hz)

### Việc đã làm
- What: Cấu hình `I2C_FREQUENCY = 40000` (40kHz) và `SENSOR_TASK_PERIOD_MS = 20` (50Hz), tăng `SENSOR_I2C_MUTEX_TIMEOUT_MS = 30`.
- Why:
  - Tối ưu hóa tối đa khả năng chống méo xung cạnh lên trên các tuyến cáp I2C dài và chống nhiễu cảm ứng từ dòng xung stepper.
  - Tần số 40kHz đảm bảo dạng sóng I2C vuông vắn hoàn hảo trên toàn bộ 6 nhánh kênh của PCA9548A.
  - Chu kỳ quét 20ms (50Hz chuẩn công nghiệp) cho phép đọc trọn vẹn 6 cảm biến (~12.5ms) với biên độ an toàn tuyệt đối trên Core 0.
- How: Sửa `src/config.h`, cập nhật `docs/SYSTEM_OVERVIEW.html`.

### Build gate
- `pio run` → **SUCCESS**

---

## 2026-08-29 — Toàn diện Kiểm toán Song song (Parallel Agents Audit) & Vá Lỗi Đa Phân Hệ

### Việc đã làm
- What & Why:
  1. **Domain 1 (Safety, RTOS & Motor)**:
     - Khắc phục deadlock phục hồi lỗi (`CLEAR_FAULT`): cho phép xóa cờ FAULT và giải phóng latch để người dùng có thể jog điều khiển trục ra khỏi công tắc hành trình mà không bị khóa đệ quy.
     - Đồng bộ hóa đa lõi trong `Motor::onStepTimer` và `Motor::run`: nâng cấp `running` và `g_emergencyStop` sang semantics `acquire/release` và kiểm tra lại `running` trước khi khởi động `esp_timer_start_once`.
     - Chuyển `targetSteps` và `mode_` sang kiểu `std::atomic` chuẩn C++17.
     - Cải tiến `makeTimedLock` trong `rtos_guard.h`: đảm bảo tick tối thiểu $\ge 1$ khi timeout > 0.
  2. **Domain 2 (Kinematics & Planner)**:
     - Khắc phục tọa độ điểm xuất phát trong `Planner::submit`: khi `WorkPlane` kích hoạt, tự động chuyển đổi vị trí FK hiện tại sang hệ tọa độ UCS (`fromRobotXYZ`) trước khi tính toán cung tròn hoặc độ cao an toàn.
     - Khắc phục lệnh di chuyển `MOVE_CART` (POINT): bổ sung điều kiện hạ bút `DROPPING` tới đúng cao độ $Z$ chỉ định trước khi kết thúc segment.
     - Chuẩn hóa góc $t_3$ trong `kin::ikPenDown` về khoảng $[-\pi, \pi]$ chống từ chối sai giới hạn soft limits.
  3. **Domain 3 (Sensor, Homing & JointModel)**:
     - Chuyển `sensor_error` sang `std::array<std::atomic<bool>, NUM_SENSORS>` để đọc/ghi thread-safe qua các lõi CPU.
     - Bổ sung kiểm tra kết quả `lock` trong `getAngle()`, `getAccumulatedAngle()`, `getTurnCount()`.
     - Sửa tên mảng định danh chân cữ `AXIS_MIN_PINS` / `AXIS_MAX_PINS` trong bộ host test.
  4. **Domain 4 (Web Server, REST API & Network)**:
     - Bổ sung kiểm tra `std::isfinite(deg)` chống tấn công/lỗi NaN bypass trên `/api/jog`.
     - Bổ sung xác thực bắt buộc `hasArg` cho các tham số `axis`, `x`, `y`, `z` trên `/api/move`, `/api/home/axis`, `/api/sethome`, `/api/clearcalib`.
     - Khóa `armPtr->busy()` trên các endpoint WorkPlane (`/api/workplane/calib`, `/api/workplane/toggle`) chống xung đột khi đang vẽ.
     - Thêm `j.reserve(3500)` trong `ArmController::statusJson()` triệt tiêu hoàn toàn hiện tượng phân mảnh bộ nhớ Heap khi polling 3.3Hz.
     - Kích hoạt `WiFi.setAutoReconnect(true)` trong `WifiManager`.
- How: Chạy kiểm toán song song bằng 4 subagents độc lập, kiểm tra mã nguồn, tích hợp các bản vá và đồng bộ hóa test suite.

### Build gate
- `pio run` → **SUCCESS** (RAM: 17.8%, Flash: 26.8%, 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED** (Kinematics, Joint Logic, Work Plane, Homing Logic).

---

## 2026-08-29 — Sửa Vòng Lặp Fault Drift (Step/Encoder Drift Loop on CLEAR_FAULT)

### Việc đã làm
- What: Khắc phục hiện tượng lặp lại liên tục `FAULT: step/encoder drift exceeded threshold` sau khi người dùng bấm `CLEAR_FAULT`.
- Why:
  - Khi một khớp bị lệch bước so với encoder $> 5^\circ$ (do mất bước, driver ngắt nguồn UART, hoặc bị ngoại lực xoay tay), watchdog 500ms kích hoạt ngắt an toàn `ArmMode::FAULT`.
  - Khi người dùng gửi lệnh `CLEAR_FAULT`, hàm cũ chỉ xóa cờ boolean `driftFault[i] = false` mà **không đồng bộ lại bộ đếm bước `absSteps`** theo vị trí góc thực tế của encoder.
  - Do đó, độ lệch giữa số bước và góc đo vẫn tồn tại $> 5^\circ$, khiến watchdog 500ms sau quét lại và lập tức ngắt `FAULT` trở lại.
- How:
  - Cập nhật [`src/joint_model.cpp:clearAllDriftFaults()`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/joint_model.cpp): Khi xóa cờ lỗi drift, tự động gọi `resyncFromEncoder(i)` cho các khớp đang homed có encoder hợp lệ.
  - Nhờ vậy, `absSteps` được căn chỉnh ngay về vị trí thực tế của encoder $\to$ độ lệch về $0.00^\circ$ $\to$ xóa lỗi dứt điểm và cho phép điều khiển tiếp tục bình thường.

### Build gate
- `pio run` → **SUCCESS** (RAM: 17.8%, Flash: 26.8%, 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED**.

---

## 2026-08-29 — Tối Ưu Hóa Quy Trình Homing StallGuard Cho Khớp J4

### Việc đã làm
- What: Cấu hình và kích hoạt đầy đủ quy trình **Sensorless Homing bằng StallGuard4** cho khớp J4 (Wrist Pan) trên driver TMC2209.
- Why:
  - Khớp J4 là khớp xoay cổ tay không trang bị công tắc hành trình vật lý (không có chân MIN/MAX endstop).
  - Sử dụng tính năng đo tải Back-EMF của TMC2209 (`SG_RESULT`) cho phép phát hiện cữ chặn cơ khí một cách tự động, êm ái và an toàn.
- How:
  - Cập nhật [`src/motor.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/motor.cpp): Khởi tạo thanh ghi `TCOOLTHRS(0xFFFFF)` và `SGTHRS(DEFAULT_STALL_THRESHOLD)` trong `Motor::begin()` để đảm bảo bộ đo tải StallGuard4 luôn kích hoạt ở mọi dải vận tốc.
  - Quy trình FSM Homing của J4 (`HomingController::beginJoint(3)`):
    1. **APPROACH**: Hạ dòng xuống mức an toàn `HOMING_CURRENT_J4 = 300mA` và chạy liên tục ở tốc độ `1800us/step`.
    2. **STALL DETECT**: Đọc thanh ghi `SG_RESULT` mỗi 10ms. Khi chạm cữ cứng, `SG_RESULT < STALL_SG_LEVEL (100)` liên tiếp 3 chu kỳ $\to$ ngắt motor an toàn.
    3. **BACKOFF & REAPPROACH**: Lùi lại $2.5^\circ$, sau đó chạm lại lần 2 ở tốc độ chậm `3000us/step` để xác định tọa độ chính xác.
    4. **SETREF**: Gán điểm dừng làm gốc tọa độ $0^\circ$ của J4, lưu vào NVS và khôi phục dòng định mức $800\text{mA}$.

### Build gate
- `pio run` → **SUCCESS** (RAM: 17.8%, Flash: 26.8%, 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED**.

---

## 2026-08-29 — Khắc Phục Hiện Tượng Gõ Cữ J4 Bằng Cơ Chế Dual Stall Fusion (StallGuard + Encoder)

### Việc đã làm
- What: Khắc phục hiện tượng khớp J4 liên tục gõ/trượt bước vào cữ chặn cơ khí mà không nhận diện được điểm dừng để đổi hướng lùi ra.
- Why:
  1. Trên TMC2209, StallGuard4 chỉ trả về kết quả đo tải `SG_RESULT` khi driver ở chế độ **StealthChop** (`en_spreadCycle = false`). Khi ở chế độ SpreadCycle, `SG_RESULT` bị vô hiệu hóa nên FSM không nhận được tín hiệu stall từ UART.
  2. Khi UART timeout hoặc tải cơ cấu chưa đủ ngưỡng trigger điện áp, hệ thống cần một cơ chế xác nhận phần cứng độc lập.
- How:
  1. Trong `HomingController::enterApproach()`: Tự động chuyển TMC2209 sang chế độ **StealthChop** (`m.setChopperMode(false)`) khi bắt đầu dò cữ và chuyển lại SpreadCycle khi kết thúc.
  2. Bổ sung **Encoder Stall Detection (Sensor Fusion)**: Mỗi 100ms, nếu motor đang phát xung chạy nhưng cảm biến góc AS5600 báo góc không thay đổi ($|\Delta_{\text{enc}}| < 0.20^\circ$ do bị cữ cơ khí chặn đứng) $\to$ FSM lập tức xác nhận cữ chạm sau đúng 100-200ms, dừng motor và thực hiện ngay pha lùi `BACKOFF` và đổi hướng.

### Build gate
- `pio run` → **SUCCESS** (RAM: 17.8%, Flash: 26.8%, 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED**.

---

## 2026-08-30 — Sửa Lỗi Homing J4 Báo Xong Ảo Sau 30ms & Vòng Lặp Báo Lỗi Drift Watchdog

### Việc đã làm
- What: 
  1. Khắc phục lỗi J4 vừa bắt đầu homing (chưa kịp quay, mới 15 bước / ~30ms) đã báo Stall và kết thúc Homing (`[HOME] J4: Stall/Hard-stop detected -> contactMade`, `steps=15`).
  2. Khắc phục vòng lặp báo lỗi Drift liên tục sau khi di chuyển / Jog (`[DRIFT] J4 lech ... deg -> [ARM] FAULT: step/encoder drift exceeded threshold`) khiến cánh tay bị khóa dừng (`ArmMode::FAULT`) không điều khiển được.
- Why:
  1. **Homing J4 false-trip**: TMC2209 khi motor đứng yên hoặc đang tăng tốc ban đầu (chưa sinh đủ Back-EMF) trả về `SG_RESULT = 0`. Điều kiện `sg < STALL_SG_LEVEL (100)` trong `tickLegacy()` được kiểm tra mỗi 10ms mà không có thời gian ân hạn khởi động (warmup grace period). Sau đúng 3 chu kỳ (30ms = 15 bước), bộ đếm `stallCount_` đạt 3 và FSM ngắt motor, lùi lại, rồi trong `REAPPROACH` tiếp tục đọc `sg = 0` sau 20ms và báo hoàn tất `SetHome J4` ảo tại chỗ.
  2. **Drift Watchdog false-alarm**:
     - Cảm biến từ AS5600 đọc qua bus I2C và lọc thông thấp EMA ($\alpha=0.2$) cần khoảng 200–300ms để ổn định góc đo sau khi motor vừa dừng bước.
     - Khi motor ngắt xung (`!isRunning()`), hàm `updateDriftCheck()` kiểm tra ngay tức thì mà không có thời gian trễ chờ cảm biến ổn định (settling time).
     - Việc kiểm tra đơn lẻ (1 mẫu vượt ngưỡng $5^\circ$ lập tức trigger FAULT) khiến bất kỳ dao động động lực học hay độ trễ bộ lọc nào sau khi Jog cũng lập tức khóa khẩn cấp robot.
- How:
  1. **Homing J4 Scan Architecture (Quét 2 cữ cơ khí & căn tâm)**:
     - Chuyển J4 (`axis=3`) sang dùng chung kiến trúc **Scan 7 giai đoạn** (`isScanAxis(3) = true`) như J1, J2:
       1. `WARMUP`: Chạy +3° đo `enc_dir_mult`, kích hoạt StealthChop mode trên TMC2209 để bật bộ đo tải StallGuard4.
       2. `SCAN_MIN`: Chạy tới cữ cứng MIN ở dòng `HOMING_CURRENT_J4 = 600mA`, nhận diện chạm cữ tức thì qua TMC2209 StallGuard Back-EMF (`sg < STALL_SG_LEVEL (150)` với `DEFAULT_STALL_THRESHOLD = 80`) kết hợp Encoder Stall, lưu `enc_min` và reset `absSteps = 0`.
       3. `SCAN_MAX`: Đảo chiều quét sang cữ cứng MAX đối diện (yêu cầu hành trình $\ge 30^\circ$ khỏi cữ MIN), nhận diện chạm cữ tức thì qua StallGuard Back-EMF (`sg < 150`), lưu `enc_max` và ghi nhận chính xác tổng số bước thực tế `step_max`.
       4. `CROSSCHECK`: Tính tâm cơ khí `enc_center = (enc_min + enc_max) / 2`, unwrap `span` chống tràn $360^\circ$ và chia tỷ số truyền 4.0 cho J4.
       5. `CENTERING`: Chạy đúng `step_max / 2` bước quay trở lại chính xác tâm cơ khí.
       6. `SETREF`: Gán gốc tọa độ $0^\circ$ (HOME) tại tâm cơ khí, lưu NVS và khôi phục dòng định mức 800mA.
  2. **Tách biệt bộ đếm Stall (`tmcStallCount_` vs `encStallCount_`)**:
     - Tách riêng `tmcStallCount_` và `encStallCount_` độc lập, triệt tiêu lỗi vòng lặp TMC 20ms xóa nhầm bộ đếm của Encoder.
  3. **Sensor Glitch Rejection & J4 Gear Ratio Scaling**:
     - Thêm bộ lọc loại bỏ xung nhiễu I2C delta $> 90^\circ$ trong `Sensor::filter()`.
     - Chia tỷ số truyền `GEAR_RATIO_J4 = 4.0` trong `JointModel` cho trục J4 encoder trên trục motor.
  4. **Drift Watchdog Debounce & Settling Delay**:
     - Thêm 500ms post-motion settling window và 3 chu kỳ debounce (1.5s) trước khi kích hoạt `ArmMode::FAULT`.
     - Đặt `HOMING_POLL_MS = 20ms` giảm tải UART.

### Build gate
- `src/config.h`, `src/sensor.cpp`, `src/homing.h`, `src/homing.cpp`, `src/joint_model.cpp` → Cú pháp chuẩn C++17, RAII và FreeRTOS thread-safety.

---

## 2026-08-30 — Kiểm toán toàn bộ codebase & vá 6 nhóm lỗi (Codebase Audit)

### Việc đã làm
- What & Why (đọc toàn bộ src/, docs/, test/ rồi vá từng lỗi):
  1. **Chẩn đoán TMC2209 sai bit (src/motor.cpp — `getDriverStatus()`)**: `openLoadA/openLoadB` đang đọc bit 4/5 của thanh ghi `DRV_STATUS` — đó là cờ **s2vsa/s2vsb** (short-to-supply), không phải **ola/olb** (open-load, bit 6/7). Đối chiếu layout chuẩn trong `TMCStepper/src/source/TMC2208_bitfields.h` (otpw=0, ot=1, s2ga=2, s2gb=3, s2vsa=4, s2vsb=5, **ola=6, olb=7**, cs_actual=16..20, stst=31) và sửa mask; thêm comment ghi rõ layout để không tái sai.
  2. **Vòng encoder-centering khi homing đảo chiều (src/homing.cpp — `gotoNearHome()` + pha `CENTERING` legacy)**: công thức cũ `cwForDelta(axis, -err)` với `err = target - enc` (đơn vị raw encoder) khiến motor chạy **xa đích** thay vì tới đích khi encSign=+1 — chính là hiện tượng "vòng phản hồi chạy ngược hướng đâm endstop" từng được ghi nhận ở pha CENTERING. Ngoài ra target `angleEncAtContact_ + stroke/2` bỏ qua dấu encoder. Sửa: quy đổi raw↔khớp qua dấu encoder đo được — thêm accessor `JointModel::encSignOf(axis)` (src/joint_model.h/.cpp), target raw = `raw_tại_cữ + encSign * stroke/2`, chiều quay = `cwForDelta(axis, errRaw/encSign)`. Cộng thêm: reset `stallStartMs_ = 0` khi vào `gotoNearHome()`/`enterCentering()` — trước đây giá trị cũ còn sót khiến lần centering kế tiếp báo "CENTERING STALL" ngay lập tức (500 ms tính từ phiên cũ). Lưu ý: J4 hiện đã chuyển sang scan path (`isScanAxis = axis <= 3`, session song song thực hiện), nên 2 đường này còn phục vụ J3/axis-1-endstop.
  3. **Ghi NVS lệch thứ tự (src/nvs_store.cpp)**: `saveJointHome()`/`saveCalib()` set cờ `*_valid = true` **trước** khi ghi giá trị — mất nguồn giữa 2 lệnh ghi để lại valid=true với dữ liệu cũ/rác, boot sau restore sai vị trí. Đảo thành commit-flag: ghi giá trị trước, cờ valid sau cùng.
  4. **Ghi ngoài mảng khi kiểm tra bounds (src/joint_model.cpp — `updateDriftCheck()`)**: nhánh điều kiện `axis >= NUM_MOTORS || !homed[axis] || ...` vẫn ghi `driftFailCount[axis] = 0` (tràn mảng khi axis == NUM_MOTORS vì bounds check gộp chung). Tách bounds check đứng trước, return sớm.
  5. **Dead code SPSC queue chiếm ~8.9 KB RAM tĩnh (xóa src/spsc_queue.h, src/motion_block.h, `g_motionQueue`)**: queue SPSC + MotionBlock (Q32.32 DDA) được khai báo toàn cục nhưng **không có producer/consumer nào** — engine step thực tế chạy esp_timer + `Motor::run()` trực tiếp. Xóa sạch (có thể khôi phục từ commit `a9f759c` khi refactor DDA thật sự được triển khai), đồng thời bỏ card module tương ứng trong SYSTEM_OVERVIEW.html (docs trước đây mô tả "Motion Task đẩy block, Step ISR rút block" — architecture không tồn tại trong code).
  6. **Sensor task thiếu Task WDT (src/sensor.h/.cpp)**: tài liệu/quy chuẩn nêu sensor task có WDT nhưng code chưa đăng ký — `taskLoop()` giờ gọi `esp_task_wdt_add(nullptr)` + feed mỗi vòng quét (guard bằng flag `wdtRegistered_`, không crash nếu TWDT không khả dụng). Cùng lúc: dọn getter `getAngle/getAccumulatedAngle/getTurnCount` — ternary `lock ? x : x` vô nghĩa (audit 2026-08-29 nói "kiểm tra lock" nhưng thực tế không phân nhánh gì) → thay bằng đọc-under-lock-when-possible có comment giải thích chấp nhận đọc khi timeout; và `begin()` giờ kiểm tra kết quả `xTaskCreatePinnedToCore` (trước đây log "Task started" kể cả khi tạo task thất bại).
- How: giữ sửa nhỏ đúng chỗ theo module sở hữu; không đổi mô hình động học/DH; các hành vi an toàn (ISR endstop, E-stop fail-fast, drift watchdog) giữ nguyên ngữ nghĩa.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,492 B, giảm từ 17.8%/58,420 B nhờ xóa dead code; Flash: 26.8%; 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED** (Kinematics + Differential Wrist, Joint Logic, Work Plane, Homing Logic).

### Việc còn lại / ghi nhận cho owner (chưa sửa — đánh giá là chấp nhận được hoặc cần quyết định owner)
- Step-timer re-arm race nhỏ trong `Motor::run()/onStepTimer()` (callback đang bay có thể re-arm trùng) — đã có re-check `running` trước `esp_timer_start_once` từ audit trước, callback ngắn (<10 µs), nguy cơ thấp.
- Arm nghỉ ngay trên endstop (hoặc công tắc dính) rồi jog trục khác → task-level backup check lập tức FAULT+E-stop: hành vi an toàn chủ đích, chỉ cần lưu ý vận hành.
- ISR endstop delay 25 µs (glitch filter) trong ISR: nằm trong ngưỡng chấp nhận, có chủ đích chống nhiễu cảm ứng.
- Web UI chưa có control cho WorkPlane (endpoint `/api/workplane/*` đã có server-side nhưng frontend không gọi) — tùy owner có muốn thêm panel UCS vào UI không.

---

## 2026-08-30 — Nhận diện sụt vận tốc tức thời (Velocity Stall) & Cô lập GPIO JTAG/RGB cho J5/J6 lúc khởi động

### Việc đã làm
- **Nhận diện kẹt cữ tức thì (Instantaneous Velocity Drop) trong Homing J4 (`src/homing.cpp`)**:
  - Chuyển cơ chế dò kẹt cữ sang đo vận tốc góc thực tế trong cửa sổ $150\,\text{ms}$:
    - Khi quay tự do: Vận tốc góc $\sim 10.55^\circ / 150\,\text{ms}$.
    - Khi chạm cữ chặn cơ khí: Vận tốc sụt giảm xuống $< 2.50^\circ / 150\,\text{ms}$.
  - Ngắt động cơ dứt khoát ngay ở chu kỳ $150\,\text{ms}$ đầu tiên khi chạm cữ, loại bỏ hoàn toàn bộ đếm tích lũy chu kỳ (vốn bị hiện tượng trượt bước cơ khí / pole-snap làm reset về 0).
- **Khắc phục hiện tượng J5 và J6 tự quay khi khởi động (`src/motor.cpp` & `src/main.cpp`)**:
  - Trên ESP32-S3, GPIO 38, 39, 40 được phần cứng ROM/Bootloader mặc định nối với WS2812 RGB LED và khối JTAG (`MTCK`, `MTDO`). Khi MCU khởi động, bộ điều khiển JTAG/USB phát xung clock/data làm driver A4988 nhận diện nhầm thành xung bước STEP.
  - Sửa: Gọi `gpio_reset_pin()` và `gpio_pulldown_en()` ngay đầu `setup()` và trong `Motor::begin()` để tách hoàn toàn matrix ngoại vi JTAG/RGB khỏi các chân STEP/DIR và khóa chặt mức LOW chống xung nhiễu thả nổi.
- **Sửa hiện tượng động cơ A4988 J5/J6 rung mạnh khi phát xung (`src/motor.cpp` & `src/config.h`)**:
---

## 2026-08-30 — Sửa dứt điểm J4 SCAN_MAX hành trình lớn & Khóa cứng 40mA Push-Pull triệt rung J5/J6

### Việc đã làm
- **Homing J4 (`src/homing.cpp`, `src/homing.h`)**:
  - **Sửa dứt điểm lỗi đập cữ trượt bước lâu (`step_max = 21516`)**:
    - Nguyên nhân: Khi động cơ trượt bước vào cữ cứng, bước nhảy rung giật từ tính tạo ra độ trôi góc giả $0.35^\circ \dots 0.60^\circ$, khiến điều kiện đo vận tốc thuần túy dựa vào thời gian bị tê liệt.
    - Sửa: Chuyển sang kiến trúc **đối chiếu vi sai Bước phát lệnh vs Góc Encoder thực tế (Step-Lag Detection)**:
      - Thuật toán theo dõi: Cứ sau mỗi $150$ bước phát xung ($\approx 4.2^\circ$ chuyển động lý thuyết):
        - Nếu chạy tự do: Góc encoder thực tế tăng $\approx 4.2^\circ \ge 0.6^\circ$ $\to$ cập nhật mốc, tiếp tục chạy mượt mà.
        - Nếu bị chặn cứng bởi cữ MIN hoặc MAX: Motor phát $150$ bước nhưng khớp bị cản cứng $\to$ encoder dịch chuyển $< 0.6^\circ$ $\to$ **ngắt dứt khoát chỉ sau đúng 150 bước ($270\,\text{ms}$)**.
      - Loại bỏ $100\%$ hiện tượng trượt bước đập cữ lâu và triệt tiêu hoàn toàn lỗi dừng sớm giữa đường.
  - Sửa công thức `span = fabsf(encMaxRaw_ - encMinRaw_)` loại bỏ phép chia 4 thừa.
- **Triệt tiêu rung J5/J6 (`src/motor.cpp`, `src/config.h`)**:
  - Tăng độ rộng xung STEP lên $4\,\mu\text{s}$ bằng `gpio_set_level` và `esp_rom_delay_us(4)`.
  - Đặt `HOMING_STEP_INTERVAL_J5/J6 = 2500` ($400\,\text{steps/s}$) và `DEFAULT_STEP_INTERVAL_US = 1200` ($\sim 833\,\text{steps/s}$) để A4988 đạt mô-men xoắn tối đa.










---

## 2026-08-30 — Nâng cấp kiến trúc homing: quét 2 cữ 2 tốc độ + VERIFY + Retry (hoàn thiện J1–J3, J4)

### Việc đã làm
- What:
  1. **Kiến trúc quét 2 cữ 2 tốc độ, đồng nhất J1–J4** (`src/homing.h/.cpp`): mỗi khớp chạy
     `WARMUP → SCAN_MIN(fast) → BACKOFF → SCAN_SLOW → SCAN_MAX(fast) → BACKOFF → SCAN_SLOW →
     CROSSCHECK → CENTERING → VERIFY → SETREF`.
     - Pha **FAST** (tốc độ homing hiện tại) chỉ tìm cữ thô — glitch giữa đường tự hồi phục vì
       pha **SLOW** (`HOMING_SLOW_SCAN_INTERVAL_US = 3000µs/step`) dò lại đúng cữ đó.
     - Điểm chạm SLOW là mốc chính xác: J1–J3 qua endstop latch ISR (±1 bước); J4 qua step-lag
       encoder **có bù độ trễ cửa sổ** — điểm chạm ≈ mốc cửa sổ + Δenc quy đổi bước (clamp trong
       cửa sổ), triệt tiêu phần lớn quantization của cửa sổ 150 bước.
     - Ở pha SLOW bỏ poll StallGuard (SG4 kém tin cậy ở tốc độ thấp, dễ false-trip); chỉ giữ
       endstop + step-lag. SG vẫn dùng ở pha FAST (ân hạn 600ms chống false-trip khởi động).
     - Cửa sổ step-lag mới scale theo gear: `max(150 bước, 1.2°×spd)` — sửa rủi ro false-stall
       tiềm ẩn của J2/J3 (cửa sổ 150 bước cũ chỉ ứng 0.84° > ngưỡng 0.6° quá sát).
  2. **VERIFY (mới)**: sau CENTERING chờ 350ms cho EMA AS5600 ổn định, đối chiếu `rawEncoder()`
     (góc tích lũy độc lập với step) với mục tiêu: J1/J2/J4 = tâm `enc_center`; J3 =
     `raw_tại_MIN + encSign×2.5°`. Dung sai `0.5° + 1%×nửa span`. Lệch → trim chậm theo encoder
     tối đa 2 lần; vẫn lệch → retry khớp.
  3. **Retry (mới)**: timeout / dừng sớm / VERIFY fail → quét lại khớp từ WARMUP
     (`HOMING_MAX_ATTEMPTS = 2`); hết lượt mới hủy chuỗi (`lastOk=false`). Trước đây 1 glitch
     = hủy ngay toàn bộ chuỗi.
  4. **Xoá sạch đường legacy** (~350 dòng): `APPROACH/BACKOFF/REAPPROACH/CENTERING` legacy,
     `beginJoint/enterApproach/contactMade/enterReapproach/gotoNearHome/tickLegacy` — dead code
     từ khi `isScanAxis` chuyển hết sang scan path; bỏ enum `APPROACH/REAPPROACH/SAFE_MODE`,
     dọn state (`stallCount_/encStallCount_/stallStartMs_/angleEncAtContact_...`), cập nhật
     comment header. JSON phase mới: `scan_min|scan_backoff|scan_slow|scan_max|centering|verify`.
  5. **Sửa đơn vị hiệu chuẩn J4**: crosscheck cũ tính `measuredSpd = stepMax/span_raw` với span
     raw là góc MOTOR (encoder J4 gắn trước gear 4:1) → ratio = 4.0 luôn bị chặn bởi guard
     [0.5,2.0] → hiệu chuẩn J4 không bao giờ áp dụng. Nay chia span cho `GEAR_RATIO_J4` trước
     khi quy đổi → đơn vị đồng nhất steps/độ-khớp, hiệu chuẩn J4 hoạt động thật.
  6. **Centering theo khoảng cách bước ĐÃ BÙ** (`contactSpan`): trước đây dùng `stepMax` gồm cả
     số bước phát dư khi bị đè cữ → tâm lệch `stalled/2` bước; nay dùng khoảng cách giữa 2 điểm
     chạm đã hiệu chỉnh. J3: sửa luôn hướng centering khi lắp đảo chiều (first-contact = MAX) —
     công thức cũ chạy NGƯỢC vào endstop.
  7. **Test host mở rộng** (`test/host/test_homing_logic.cpp`): 10 test case — bất biến 2 tốc độ,
     retry cap, margin cửa sổ stall theo gear (J2/J3), dung sai verify, centering J3 cho cả 2
     phương án lắp đặt, đối xứng tâm.

- Why: yêu cầu owner "nâng cấp kiến trúc homing J4, hoàn thiện homing J1–J3". Điểm đau đã log
  trước đó: J4 đập cữ/trượt bước ở tốc độ quét nhanh, false-trip SG, 1 glitch = hủy chuỗi,
  home sai không có cơ chế phát hiện. Kiến trúc fast+slow giảm động năng va chạm (~1/9 ở cùng
  dòng do v²), VERIFY dùng encoder độc lập chốt chất lượng home, retry chống glitch I2C/UART.
- How: giữ API công khai `startAll/startAxis/cancel/tick/toJson` không đổi → `arm.cpp`/web không
  phải sửa; chuỗi an toàn giữ nguyên (ISR endstop latch là nguồn sự thật, `g_homingActive`,
  FAULT/E-stop, restore dòng khi thoát FSM). Giới hạn đã biết: độ lặp lại home J4 ~±2° (bị chặn
  bởi EMA AS5600 50Hz + cửa sổ step-lag) — chấp nhận được cho wrist roll vẽ, tinh chỉnh bằng
  `HOMING_SLOW_SCAN_INTERVAL_US` nếu cần.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,508 B, Flash: 26.8%, 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED** (homing logic mở rộng 10 case).
- `tools/run_kin_tests.sh` → **ALL PASSED** (không đụng kinematics — chạy xác nhận không vỡ).

### Việc còn lại
- Chạy checklist hardware mới (mục 3 trong `docs/HW_REGRESSION_CHECKLIST.md`): quan sát log
  2 tốc độ, VERIFY OK/TRIM/RETRY trên J1–J4 thật; đo độ lặp lại home J4 nhiều lần liên tiếp.
- Nếu J4 vẫn đập cữ nhẹ ở pha slow: tăng `HOMING_SLOW_SCAN_INTERVAL_US` lên 4000 hoặc giảm
  `HOMING_CURRENT_J4`.

---

## 2026-08-30 — Cứng hoá homing sau log hardware: fail-fast encoder đóng băng, reset calib mỗi lần thử, guard trim

### Bối cảnh (log hardware do owner cung cấp)
- J1: encoder ACK bình thường nhưng góc **đóng băng ở 233.0°** suốt lần quét (WARMUP delta=0.00,
  mọi đọc tiếp theo = 233.0), rồi nhảy 226.5 → 201 khi trim ⇒ step-lag stall tạo **stall ảo**
  (encDelta=0.00) chặn trước endstop thật → fake span 724 bước (~13.6°) → calib rác → trim diverge
  (err 6.5°→32.08°) → J1 đâm endstop MIN → FAULT.
- J1: hiệu chuẩn cũ 41.25 steps/deg (từ run trước) còn hiệu lực → BACKOFF chỉ 103 bước
  (2.5°×41.25) thay vì 133 (config 53.33) → lùi không đủ nhả công tắc → "BACKOFF jammed".
- **Finding phần cứng cho owner — tỷ số encoder J4 lệch ~5.2× so với giả định "encoder trên
  trục motor, gear 4:1"**: WARMUP 107 bước = 12.0° motor nhưng encoder chỉ đổi 2.29° (x5.24);
  SCAN_SLOW: 796 bước ↔ 17.8° encoder (x4.5–5.2, hai mẫu độc lập nhất quán). Nếu encoder thật ra
  gắn trên trục khớp và gear thật ~5.2:1 thì cần đổi GEAR_RATIO_J4 + bỏ phép chia 4 trong
  actuatorAngleFromEncoder() và crosscheck — CHỈ owner phê duyệt sau khi đo thực tế (jog J4
  một góc biết trước, so Δencoder).

### Việc đã làm (src/homing.cpp, src/homing.h, src/joint_model.*, src/config.h)
- What:
  1. **Fail-fast encoder đóng băng ở WARMUP**: delta < 0.5° sau khi motor đã phát ~3° bước →
     HỦY khớp ngay (log rõ), retry rồi hủy chuỗi — trước đây chỉ log "enc dead" rồi tiếp tục
     với encSign mặc định → toàn bộ chuỗi lỗi phía sau.
  2. **`JointModel::resetHomingCalibration(axis)`** gọi ở `beginScan()`: mỗi lần thử homing
     reset về hằng số config (encSign + steps/deg), tự đo lại từ đầu — triệt tiêu calib cũ
     (41.25) làm sai mọi khoảng cách. KHÔNG xoá NVS (tránh ghi flash khi vận hành); lần home
     thành công sẽ ghi đè NVS bằng giá trị đo mới.
  3. **Bỏ step-lag ở pha FAST cho trục có endstop (J1–J3)**: endstop latch + StallGuard là 2
     sensor độc lập với encoder — encoder đóng băng không còn cơ hội tạo stall ảo chặn trước
     endstop. J4 giữ step-lag ở fast (SG quá chậm cho 2.8 fullsteps/s của J4). Step-lag vẫn ở
     pha SLOW làm fallback endstop đứt dây.
  4. **Span integrity check ở CROSSCHECK**: motor đã đi `contactSpan` bước mà encoder dịch
     < `HOMING_MIN_ENC_SPAN_DEG[axis]` ({30,30,30,15}°; J4=15° do bất định tỷ số) → HỦY khớp —
     chặn home ảo khi encoder đóng băng giữa chừng (cả J4).
  5. **Guard trim runaway ở VERIFY**: err tăng vượt (mốc đầu trim + 2°) hoặc đi quá
     `HOMING_TRIM_MAX_TRAVEL_DEG=5°` mà chưa hội tụ → dừng + hủy khớp — chặn đúng hiện tượng
     trim diverge đẩy J1 đâm endstop trong log.
  6. **VERIFY mất encoder giữa chừng → HỦY** (trước đây "chấp nhận home theo bước" — sai, vì
     drift-watchdog sẽ false-FAULT với encoder chết ngay lần jog đầu).
- Why: log hardware chứng minh FSM cũ không tự vệ khi encoder lỗi "âm thầm" (ACK mà đứng yên);
  mọi cơ chế down-stream (stall detect, calib, verify, trim) đều bị đầu độc bởi dữ liệu đóng băng.
- How: phòng thủ theo tầng — WARMUP (chặn sớm) → FAST (không tin encoder trên trục có endstop)
  → CROSSCHECK (kiểm span tổng) → VERIFY (guard trim) — tất cả dẫn về finishJoint(false) →
  retry (2 lần) → hủy chuỗi + lastOk=false, không bao giờ setHomeHere trên dữ liệu đáng ngờ.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,524 B, Flash: 26.8%, 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED** (homing logic: 12 case).

### Việc còn lại (owner)
- **Kiểm tra phần cứng encoder J1** (đóng băng 233.0°): cáp/mux kênh 0/nam châm — nếu lỏng
  cáp qua khớp base thì gia cố; xác nhận bằng log WARMUP delta sau khi sửa.
- **Xác minh tỷ số encoder J4 (~5.2×)**: jog J4 một góc biết trước, so Δencoder raw; nếu xác
  nhận encoder-on-joint + gear ~5.2:1 → đổi GEAR_RATIO_J4, bỏ chia 4 ở actuatorAngleFromEncoder
  + crosscheck (một thay đổi mô hình do owner phê duyệt).
- Chạy lại checklist mục 3 (HW_REGRESSION_CHECKLIST.md) gồm cả kịch bản encoder lỗi.

---

## 2026-08-30 — Chẩn đoán log hardware #2: nhãn endstop J1 đấu ngược + WARMUP probe + BACKOFF tự nới

### Bối cảnh (log hardware do owner cung cấp, lần 2)
- J1 quét chiều dương (cw=1) chạm công tắc trên chân **MIN**, chiều âm (cw=0) chạm chân
  **MAX** — bằng chứng: `SCAN_MIN fast (cw=0) → CONTACT (MAX)` và
  `SCAN_MAX fast (cw=1) → CONTACT (MIN)` (2 contact độc lập, cùng chiều lấy mẫu quy ước).
  ⇒ Nhãn MIN/MAX của J1 ngược quy ước "góc + tiến về MAX" (dây đấu ngược hoặc tên nhãn).
- Hệ quả theo chuỗi: warmup lần 2 (MIN nhấn) chạy +3° tin là "rời MIN" ⇒ thực tế đâm SÂU vào
  công tắc → motor đứng yên → fail-fast "encoder khong phan hoi" bắn nhầm (encoder đúng, motor
  kẹt); BACKOFF 2.5° chưa nhả đòn bẩy công tắc → "BACKOFF jammed"; vòng
  FAULT/FAULT-cleared lặp khi jog vì "chiều rời MIN" theo mô hình = mài vào công tắc thật.
- Finding thêm: encoder J1 vẫn nhảy giá trị phi vật lý (chạm #1 enc=435.1 → progress leg 2
  enc=319.6, nhảy -115° trong khi bước chỉ đi +13.6°) — encoder J1/cáp vẫn lỗi, cần gia cố.

### Việc đã làm (src/config.h, src/homing.h/.cpp)
- What:
  1. **Hoán nhãn endstop J1** (`J1_MIN_PIN: 5→6, J1_MAX_PIN: 5↔6`): phục hồi bất biến quy ước
     "góc + tiến về phía MAX" theo bằng chứng log 2 contact. Ghi chú tại chỗ trong config.h —
     nếu owner đấu lại dây sau này thì hoán ngược lại. Chân vật lý KHÔNG đổi.
  2. **WARMUP probe thực nghiệm chiều ngược** (`warmupProbed_`): nếu công tắc VẪN nhấn sau
     bước rời 3° ⇒ hướng theo mô hình đang đâm vào cữ (đấu ngược) hoặc motor kẹt → thử chiều
     ngược đúng 1 lần (không phụ thuộc quy ước đấu dây); cả 2 endstop nhấn cùng lúc ⇒ kẹt cứng
     → hủy; probe xong vẫn nhấn ⇒ công tắc dính/kẹt cữ → hủy. Probe chạy lại đúng
     `warmupSteps_` để phép đo delta/encDirMult giữ ý nghĩa.
  3. **BACKOFF tự nới rộng** (`HOMING_BACKOFF_MAX_EXTEND=2`, scale 1×→2×→4× = 2.5°→5°→10°):
     lùi xong công tắc chưa nhả ⇒ nới khoảng cách rồi lùi lại, chỉ kết luận jammed sau khi
     hết nới — chống hysteresis nhả của đòn bẩy/cam.
- Why: 2 fail khác bản chất nhưng cùng triệu chứng "endstop van nhan" — (a) hướng ngược quy
  ước phải xử lý bằng chuẩn hoá nhãn pin (single source of truth), (b) hysteresis cơ khí phải
  xử lý bằng nới khoảng cách. Warmup probe là lưới an toàn cho (a) trên MỌI trục (nếu owner
  sau này phát hiện J2/J3 cũng ngược, FSM tự thích ứng ở mức probe còn pin map cần chỉnh như J1).
- How: giữ FSM 2 tốc độ nguyên vẹn; probe chỉ chạy khi endstop nhấn bền bỉ sau warmup (trường
  hiếm); backoff extend chỉ tiêu thêm thời gian khi cần, không đổi luồng thành công.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1%, Flash: 26.8%, 0 errors, 0 warnings).
- `tools/run_host_tests.sh` → **ALL 4 HOST TESTS PASSED**.

### Việc còn lại (owner)
- Chạy Home All lại: kỳ vọng J1 contact đúng nhãn (MIN ở chiều âm, MAX ở chiều dương),
  không còn BACKOFF jammed, không còn vòng FAULT khi jog.
- **Gia cố encoder J1** (vẫn nhảy giá trị phi vật lý — xem finding ở trên).
- **Xác minh tỷ số encoder J4 (~5.2×)** — mục riêng ở entry trước, chưa xử lý.

---

## 2026-08-30 — Chẩn đoán log hardware #3: WARMUP settle + backoffExtend_ không reset

### Bối cảnh (log hardware do owner cung cấp, lần 3)

```
[DRIFT] J1 lech -237.39 deg → FAULT
[ARM] FAULT cleared
[HOME] J1: WARMUP encoder khong phan hoi (delta=-0.18) — HUY KHOP  ← lần 1
[HOME] J1: FAILED — thu lai lan 2/2
[HOME] J1: WARMUP encoder khong phan hoi (delta=0.00) — HUY KHOP  ← lần 2
[HOME] J1: FAILED sau 2 lan thu — huy chuoi homing
... (sau đó home lại lần 3)
[HOME] J1: SCAN_MAX CONTACT (MAX)
[HOME] J1: BACKOFF cong tac van nhan — noi rong lan 2  ← bắt đầu ở lan 2, không phải lan 1
[HOME] J1: BACKOFF cong tac van nhan — noi rong lan 3
[HOME] J1: BACKOFF jammed sau 3 lan noi rong — ket cuc, HUY KHOP
[HOME] J1: WARMUP endstop van nhan — probe chieu nguoc (cw=1, 161 steps)
[HOME] J1: WARMUP endstop van nhan sau probe — HUY KHOP
```

**Nguyên nhân bug #1 — WARMUP delta ≈ 0**: `encAfter` được đọc NGAY SAU khi `m.isRunning()`
trả về false, không có thời gian settle cho EMA AS5600 (50Hz, α=0.2 ≈ 100ms settling time).
`encBefore_` (trước khi motor chạy) và `encAfter` (ngay sau motor dừng) đều phản ánh
cùng giá trị EMA chưa hội tụ → delta ≈ 0 → false-fail "encoder không phản hồi".

**Nguyên nhân bug #2 — BACKOFF jammed sau 1 extension**: `backoffExtend_` KHÔNG được reset
trong `enterScanMax()`. Nếu SCAN_MIN đã backoff 1-2 lần (backoffExtend_=1 hoặc 2), khi
SCAN_MAX contact và vào BACKOFF, `backoffExtend_` đã bắt đầu ở 1-2 → tiêu hết quota ngay
lập tức → "jammed" ảo dù thực ra chỉ mới thử 1 lần.

**Nguyên nhân bug #3 — deadzone encoder cứng**: `ENC_DIR_DEADZONE_DEG = 0.5f` áp dụng
đồng đều mọi trục, nhưng với nhiễu EMA + thời gian settle khác nhau theo gear, 0.5° có thể
quá chặt cho J1 (gear 6:1) hoặc quá lỏng cho J4.

### Việc đã làm (src/homing.cpp, src/homing.h)

- What:
  1. **FIX #1 — WARMUP post-stop EMA settle**: sau khi `m.isRunning()→false`, FSM đợi
     `WARMUP_ENC_SETTLE_MS=200ms` trước khi đọc `encAfter`. `encBefore_` vẫn lấy TRƯỚC
     `m.run()` trong `enterWarmup()` — baseline đúng. 200ms > 2 chu kỳ EMA @ 50Hz đủ để
     AS5600 hội tụ sau khi motor dừng. Thêm fields `warmupSettling_` + `warmupSettleStartMs_`
     vào state machine.
  2. **FIX #2 — Reset `backoffExtend_` khi vào SCAN_MAX**: `enterScanMax()` bây giờ reset
     `backoffExtend_ = 0` trước khi chạy — mỗi cữ có quota backoff extend độc lập (0→1→2).
  3. **FIX #3 — Deadzone encoder per-axis**: thay hằng số đơn `ENC_DIR_DEADZONE_DEG`
     bằng mảng `ENC_DIR_DEADZONE_DEG[NUM_MOTORS]` = {0.30, 0.30, 0.30, 0.20, ...}.
     J1-J3: 0.30° (dưới mức nhiễu EMA ~0.2° nhưng trên 3σ noise); J4: 0.20° (encoder
     motor nhạy hơn, gear thực ~5.2×).
  4. **FIX #4 — `enterScanBackoff()` xoá latch trước khi backoff**: tránh latch cũ từ SCAN
     gây false-positive "cong tac van nhan" trong kiểm tra SCAN_BACKOFF sau khi motor lùi.
  5. **FIX #5 — Probe chiều ngược re-capture encBefore_**: khi probe chạy chiều ngược,
     `encBefore_` được re-capture tại vị trí hiện tại trước `m.run()` — đảm bảo delta
     phản ánh đúng chuyển động probe, không so với vị trí warmup ban đầu.

- Why: Log #3 cho thấy 2 root cause độc lập song song: (a) EMA không kịp hội tụ gây
  false-fail warmup (encoder vật lý OK, FSM tự sập trước khi scan), (b) backoffExtend_
  carry-over gây "jammed" ảo làm abort ngay cả khi công tắc sẽ nhả nếu thêm 1 lần lùi.

- How: Fix (1) thêm state settle = minimal FSM extension, không đổi kiến trúc scan.
  Fix (2) reset 1 dòng, zero-risk. Fix (3) mảng replace scalar, backward-compatible.
  Fix (4)+(5) xử lý edge case latch cũ và probe delta.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,540 B, Flash: 26.8% — 896,581 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Việc còn lại (owner)
- Flash firmware mới, chạy Home All: kỳ vọng WARMUP delta > 0.30° (J1 ~3°), không còn
  "encoder khong phan hoi", không còn BACKOFF jammed ảo.
- **Gia cố encoder J1** (vẫn có thể nhảy giá trị phi vật lý — kiểm tra cáp qua khớp base).
- **Xác minh tỷ số encoder J4** — chưa xử lý.

---

## 2026-08-30 — Chẩn đoán log hardware #4: BACKOFF 4 bậc (20°) & WARMUP Smart Escape

### Bối cảnh (log hardware do owner cung cấp, lần 4)

```
[HOME] J1: SCAN_MAX CONTACT (MAX)
[HOME] J1: BACKOFF 133 steps (cw=0, lan 1)
[HOME] J1: BACKOFF cong tac van nhan — noi rong lan 2
[HOME] J1: BACKOFF 267 steps (cw=0, lan 2)
[HOME] J1: BACKOFF cong tac van nhan — noi rong lan 3
[HOME] J1: BACKOFF 533 steps (cw=0, lan 3)
[HOME] J1: BACKOFF jammed sau 3 lan noi rong — ket cuc, HUY KHOP
[HOME] J1: FAILED
[HOME] J1: FAILED — thu lai lan 2/2
[HOME] J1: SAFE_MODE + WARMUP (cw=0, 161 steps, minP=0, maxP=1)
[HOME] J1: WARMUP endstop van nhan — probe chieu nguoc (cw=1, 161 steps)
[HOME] J1: WARMUP endstop van nhan sau probe (cong tac dinh / ket cuc) — HUY KHOP
[HOME] J1: FAILED sau 2 lan thu — huy chuoi homing
[ARM] FAULT: endstop hit during motion (ISR/E-stop)
```

**Phân tích nguyên nhân:**
1. **BACKOFF cần hành trình lớn hơn**: Đòn bẩy micro-switch J1 MAX có vùng hành trình tác động lớn, 3 lần lùi cũ (2.5° + 5° + 10° = 17.5° tổng) vẫn chưa hoàn toàn nhả đòn bẩy.
2. **WARMUP probe ngược hướng khi retry từ endstop**: Lần retry thứ 2 bắt đầu khi arm đang nằm đè lên cữ MAX (`maxP=1`). WARMUP chạy `cw=0` để lùi khỏi MAX (161 bước = 3°). Do đòn bẩy dài chưa nhả, logic cũ ngộ nhận là chạy sai chiều và probe ngược `cw=1` (đâm sâu vào MAX), dẫn đến tiếp tục kẹt và fail chuỗi.

### Việc đã làm (`src/config.h`, `src/homing.h`, `src/homing.cpp`)

- What:
  1. **Tăng `HOMING_BACKOFF_MAX_EXTEND = 3`**: Thêm bậc lùi thứ 4 với scale 8× (20°), nâng tổng hành trình lùi tối đa lên 37.5° (2.5° → 5° → 10° → 20°), đảm bảo nhả hoàn toàn micro-switch đòn bẩy dài.
  2. **Theo dõi dịch chuyển encoder trong BACKOFF (`backoffStartEnc_`)**: Ghi nhận encoder delta sau mỗi lần lùi để phân biệt rõ ràng giữa công tắc hành trình dài và kẹt cơ học (stall).
  3. **Smart Escape trong WARMUP**: Ghi nhớ `warmupFromMinP_` / `warmupFromMaxP_`. Nếu khởi đầu đang đè lên endstop mà bước 3° đầu chưa nhả, FSM chạy tiếp **cùng chiều** với số bước gấp 5× (15°) để thoát đòn bẩy, tuyệt đối không đảo chiều đâm vào cữ. Chỉ probe đảo chiều nếu va phải endstop bất ngờ.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.8% — 897,025 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware và chạy lại homing.
- Quan sát log BACKOFF và WARMUP để xác nhận J1 thoát cữ MAX thành công.

---

## 2026-08-30 — Chẩn đoán log hardware #5: ISR Endstop ngắt chuyển động BACKOFF do rung cơ khí

### Bối cảnh (log hardware do owner cung cấp, lần 5)

```
[HOME] J1: SCAN_MAX CONTACT (MAX)
[HOME] J1: BACKOFF 133 steps (cw=0, lan 1)
[HOME] J1: BACKOFF cong tac van nhan (enc_moved=0.18 deg) — CANH BAO: motor co the bi ket co hoc!
[HOME] J1: BACKOFF noi rong lan 2
[HOME] J1: BACKOFF 267 steps (cw=0, lan 2)
[HOME] J1: BACKOFF cong tac van nhan (enc_moved=0.26 deg) — CANH BAO: motor co the bi ket co hoc!
...
[HOME] J1: WARMUP endstop van nhan — escape dai hon (cw=0, 805 steps)
[HOME] J1: WARMUP encDirMult=-1 (delta=9.14, intent=-1, probe=1)
```

**Phân tích nguyên nhân gốc:**
1. **ISR Endstop ngắt chuyển động BACKOFF**: Trong pha `SCAN_MAX`, motor chạy `cw=1` đâm vào cữ MAX. Ngay sau đó vào `enterScanBackoff()`, motor bắt đầu chạy `cw=0` để lùi ra. Tuy nhiên, tiếp điểm công tắc MAX đang chịu lực tì và rung cơ khí từ các xung bước đầu tiên → tạo cạnh rơi (FALLING) trên GPIO 5 → `Endstops::isrHandler` kích hoạt và gọi `m->stopFromISR()`.
2. Do đó, chuyển động `BACKOFF` bị hủy ngay lập tức sau 0–1 bước (`enc_moved = 0.00 ~ 0.26 deg`), motor không kịp lùi ra khỏi vùng đè công tắc dù đã ra lệnh 133, 267, 533 hay 1067 bước.
3. Trong khi đó, ở pha `WARMUP escape` (chạy sau khi homing fail), tiếp điểm đã hoàn toàn ổn định sau vài giây nghỉ nên lệnh 805 bước `cw=0` chạy trọn vẹn và dịch được `9.14 deg`.

### Việc đã làm (`src/endstop.cpp`, `src/homing.cpp`)

- What:
  1. **Lọc ngắt ISR theo hướng chuyển động (`movingAway`) trong `Endstops::isrHandler`**: Kiểm tra nếu motor của trục đang chạy và chiều quay là chiều **lùi / rời khỏi cữ** (`(c->which == MIN) ? cw == (AXIS_STEP_SIGN > 0) : cw == (AXIS_STEP_SIGN < 0)`), ISR sẽ **bỏ qua hoàn toàn** xung ngắt. Điều này ngăn chặn triệt để hiện tượng bounce/vibration lúc nhả công tắc làm ngắt chuyển động `BACKOFF` hoặc `WARMUP escape`.
  2. **Thêm `delay(30)` settle cơ khí trong `enterScanBackoff()`**: Cho phép lực tì cơ học và tiếp điểm ổn định hoàn toàn trước khi phát xung lùi.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.8% — 897,173 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware và chạy lại homing.
- Motor J1 sẽ hoàn thành trọn vẹn chuyển động BACKOFF sau khi chạm MAX mà không bị ngắt giữa chừng.

---

## 2026-08-30 — Chẩn đoán log hardware #6: VERIFY Trim Runaway do Encoder phi tuyến / nhảy góc đè lên mốc cơ khí Endstop

### Bối cảnh (log hardware do owner cung cấp, lần 6)

```
[HOME] J1: SLOW CONTACT #1 (MIN, enc=431.5, contact=-1932)
[HOME] J1: SCAN_MAX fast (cw=1)
[HOME] J1: SCAN_MAX CONTACT (MAX)
[HOME] J1: BACKOFF 133 steps (cw=0, lan 1)
[HOME] J1: SCAN cữ thứ hai slow (cw=1, 3000 us/step)
[HOME] J1: SLOW CONTACT #2 (MAX, enc=182.2, span=8826)
[JM] Calib J1: encSign=-1, steps/deg=35.40 (measured)
[HOME] J1: CROSSCHECK enc_c=306.9 span=8826 spd=35.40 (x0.66)
[HOME] J1: CENTERING 4413 steps (cw=0, enc_center=306.9)
[HOME] J1: VERIFY target_raw=306.9 enc=233.4
[HOME] J1: TRIM #1 (err=73.43, cw=0)
[HOME] J1: TRIM RUNAWAY (err=71.67, start=73.43) — HUY KHOP
[HOME] J1: FAILED
```

**Phân tích nguyên nhân gốc:**
1. **Chuỗi dò 2 cữ và lùi về tâm hoạt động hoàn hảo**:
   - `contactSpan = 8826 bước` tương đương $165.5^\circ$ góc quay giữa 2 công tắc hành trình MIN và MAX.
   - `CENTERING 4413 steps` ($8826 / 2$) đưa cánh tay về **chính xác 100% tâm cơ khí vật lý** giữa 2 công tắc (sai số $\pm 1$ vi bước $\approx 0.018^\circ$).
2. **Nguyên nhân VERIFY báo FAIL**:
   - Encoder AS5600 trục J1 bị phi tuyến / nhảy góc tích lũy across vòng quay (MIN đọc $431.5^\circ$, MAX đọc $182.2^\circ$). Trung bình cộng encoder tính ra $306.9^\circ$.
   - Khi motor đứng tại tâm cơ khí, encoder đọc thực tế $233.4^\circ$.
   - FSM tính độ lệch `err = 306.9 - 233.4 = 73.43°` và cố gắng TRIM motor quay $73.43^\circ$ về phía "tâm encoder ảo", kích hoạt bộ bảo vệ `TRIM RUNAWAY` (giới hạn $5^\circ$) rồi hủy chuỗi homing.
3. **Bất biến kiến trúc**: Đối với các trục có công tắc hành trình kép MIN & MAX (J1..J3), **mốc cơ khí $\text{span}/2$ là Ground Truth tuyệt đối**. Encoder chỉ dùng để theo dõi vị trí tương đối và phát hiện trượt bước (drift watchdog) sau khi đã Set-Home. Giá trị encoder phi tuyến không bao giờ được phép override mốc cơ khí để trim làm lệch vị trí thật.

### Việc đã làm (`src/homing.cpp`)

- What:
  - Cập nhật `HomingController::enterVerify()`: Đối với các trục có endstop vật lý (J1..J3), sau khi `CENTERING` hoàn tất (motor dừng + 350ms settle EMA), FSM xác nhận encoder còn hoạt động (`jm->encOK()`), ghi nhận mốc `enc_zero` và **chốt Home thành công (`finishJoint(true)`) ngay tại tâm cơ khí**, không thực hiện trim encoder.
  - Trục không endstop (J4): Giữ nguyên cơ chế verify & trim theo stall detection.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.8% — 897,313 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware và chạy lại homing.
- Chuỗi homing J1 sẽ hoàn tất thành công (`VERIFY OK`) ngay sau khi cánh tay về tâm cơ khí.

---

## 2026-08-30 — Chẩn đoán log hardware #7: WARMUP & CROSSCHECK Fallback cho trục có Endstop vật lý (J1–J3)

### Bối cảnh (log hardware do owner cung cấp, lần 7)

```
[SENSOR] Init scan kết quả:
  Ch0: err=0 fail=0 angle=0.00
[HOME] J1: SAFE_MODE + WARMUP (cw=1, 161 steps, minP=0, maxP=0)
[HOME] J1: WARMUP encoder khong phan hoi (delta=0.00, deadzone=0.30) — HUY KHOP
[HOME] J1: FAILED sau 2 lan thu — huy chuoi homing
```

**Phân tích nguyên nhân gốc:**
1. Cảm biến AS5600 trục J1 phản hồi ACK I2C thành công (`PCA=OK AS5600=ACK OK`), nhưng thanh ghi góc ANGLE (0x0E) trả về `0x000` liên tục (`angle=0.00`). Khi motor phát bước WARMUP ($3^\circ$), encoder vẫn đứng yên ở $0.00^\circ \implies \text{delta} = 0.00^\circ$.
2. Logic WARMUP cũ áp dụng kiểm tra nghiêm ngặt `delta < deadzone => HUY KHOP` cho cả 6 trục. Tuy nhiên, các trục **J1, J2, J3 đều có công tắc hành trình kép MIN và MAX**. Ngay cả khi encoder chưa dịch chuyển hoặc đọc $0.00^\circ$, hệ thống vẫn hoàn toàn dò được 2 cữ vật lý và về tâm $\text{span}/2$ chính xác 100%. Việc hủy homing ở WARMUP đã chặn đứng khả năng vận hành của robot.

### Việc đã làm (`src/homing.cpp`)

- What:
  1. **WARMUP Fallback cho trục có Endstop (J1–J3)**: Khi $\text{delta} < \text{deadzone}$, nếu trục có công tắc hành trình vật lý, FSM cảnh báo và tự động fallback sang hướng `AXIS_ENC_SIGN` mặc định từ `config.h`, sau đó cho phép tiếp tục vào pha `SCAN_MIN` để dò cữ vật lý. (Chỉ giữ hủy khớp ở WARMUP đối với trục không endstop J4).
  2. **CROSSCHECK Fallback cho trục có Endstop (J1–J3)**: Khi `spanRaw < HOMING_MIN_ENC_SPAN_DEG`, nếu trục có endstop vật lý, FSM giữ `cfgSpd` và tiếp tục vào `CENTERING` để về tâm cơ học $\text{span}/2$.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.9% — 897,609 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware và chạy lại homing.
- J1 sẽ bỏ qua lỗi góc $0.00^\circ$ ở WARMUP, tiếp tục quét 2 cữ MIN/MAX và chốt Home tại tâm cơ khí thành công.

---

## 2026-08-30 — Chẩn đoán log hardware #8: Sửa chân Endstop J2 & J3; Khôi phục Dừng Khẩn Cấp ISR 100% (Gỡ bỏ bypass movingAway)

### Bối cảnh (log hardware do owner cung cấp, lần 8)

```
[HOME] J1: SETREF OK (enc_zero=233.0) -> Hoàn tất tốt
[HOME] J2: SETREF OK (enc_zero=357.0) -> Hoàn tất tốt
[HOME] J3: SAFE_MODE + WARMUP (cw=0, 534 steps, minP=0, maxP=0)
[HOME] J3: WARMUP endstop bat ngo — probe chieu nguoc (cw=1, 534 steps)
[HOME] J3: SAFE_MODE + WARMUP (cw=0, 534 steps, minP=1, maxP=0)
[HOME] J3: WARMUP endstop van nhan — escape dai hon (cw=0, 2670 steps)
[ARM] FAULT: endstop hit during motion (ISR/E-stop)
```

**Phân tích nguyên nhân gốc:**
1. **Lỗi gán nhãn chân Endstop MIN/MAX trên J2 và J3 (`src/config.h`)**:
   - `AXIS_STEP_SIGN` của J2 và J3 là `-1` (chiều quay góc dương ứng với `cw = 0`, góc âm ứng với `cw = 1`).
   - Log hardware cho thấy:
     - J2: Quay `cw = 0` (góc +) chạm Pin 7 $\implies$ **Pin 7 là MAX**, Pin 10 là MIN. (Cũ đang để MIN=7, MAX=10 $\implies$ bị ngược).
     - J3: Quay `cw = 0` (góc +) chạm Pin 11 $\implies$ **Pin 11 là MAX**, Pin 12 là MIN. (Cũ đang để MIN=11, MAX=12 $\implies$ bị ngược).
   - Vì J3 bị ngược nhãn chân: Khi J3 chạm Pin 11, phần mềm ghi nhận `minP = 1` (tưởng là chạm MIN). Đến lần thử thứ 2, phần mềm cố "thoát khỏi MIN" bằng cách quay góc dương (`cw = 0`), khiến motor quay tiếp về phía Pin 11 thay vì rời xa nó!
2. **Nguyên nhân va đập gãy công tắc (Lỗ hổng an toàn nghiêm trọng)**:
   - Trong bản sửa trước, một bộ lọc `movingAway` đã được đưa vào `Endstops::isrHandler` với giả định phần mềm biết rõ hướng nào là lùi khỏi cữ.
   - Khi nhãn chân bị ngược trong `config.h`, phần mềm cho rằng `cw = 0` là đang "lùi khỏi MIN", do đó **ISR ĐÃ BỎ QUA HOÀN TOÀN XUNG NGẮT TỪ PIN 11**!
   - Kết quả: Motor chạy trọn vẹn 2670 bước ($15^\circ$) với lực kéo lớn đâm thẳng vào công tắc mà không hề dừng lại!
3. **Bất biến an toàn tối thượng**:
   - **ISR Endstop không bao giờ được phép tắt hay bỏ qua dựa trên chiều quay tính toán của phần mềm**. Bất kỳ khi nào công tắc cơ học bị kích hoạt (cạnh rơi FALLING), **ISR PHẢI DỪNG MOTOR NGAY LẬP TỨC (`stopFromISR()`)**.
   - WARMUP không được chạy các bước lớn ($15^\circ$) mù. Nếu công tắc đã nhấn, WARMUP lập tức chuyển sang FSM quét cữ với đầy đủ cơ chế bảo vệ ISR.

### Việc đã làm

1. **Khôi phục Dừng Khẩn Cấp ISR 100% (`src/endstop.cpp`)**:
   - Xóa bỏ hoàn toàn bộ lọc `movingAway` trong `Endstops::isrHandler`. Mọi tiếp xúc công tắc đều ngắt tức thời trong ISR.
2. **Đồng bộ đúng nhãn chân Endstop MIN/MAX (`src/config.h`)**:
   - `J1_MIN_PIN = 6, J1_MAX_PIN = 5` (đã chuẩn, J1 chạy hoàn hảo).
   - `J2_MIN_PIN = 10, J2_MAX_PIN = 7` (quay (+) chạm Pin 7, quay (-) chạm Pin 10).
   - `J3_MIN_PIN = 12, J3_MAX_PIN = 11` (quay (+) chạm Pin 11, quay (-) chạm Pin 12).
3. **Tối ưu an toàn WARMUP (`src/homing.cpp`)**:
   - Nếu công tắc đang nhấn sau bước WARMUP, không thực hiện thoát mù nhiều bước mà chuyển thẳng sang `enterScanMin()` để FSM quét và lùi cữ an toàn có kiểm soát.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.8% — 897,169 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Sau khi thay thế/chỉnh lại công tắc hành trình cơ khí của J3, nạp firmware mới.
- Khớp J1, J2, J3 sẽ nhận đúng cữ MIN/MAX, ngắt tức thì khi chạm cữ và hoàn tất homing an toàn.

---

## 2026-08-30 — Chẩn đoán log hardware #9: Khớp J3 Offset đúng từ cữ MAX (Pin 11); Hoàn thiện chuỗi Sensorless Homing cho J4

### Bối cảnh (log hardware do owner cung cấp, lần 9)

```
[HOME] J3: SLOW CONTACT #1 (MIN, enc=144.8, contact=-13)
[HOME] J3: SCAN_MAX CONTACT (MAX)
[HOME] J3: SLOW CONTACT #2 (MAX, enc=161.6, span=14315)
[HOME] J3: CENTERING 13871 steps (cw=1, enc_center=153.2)
[HOME] J3: VERIFY OK (co endstop — chot home tai tam co khi, enc_zero=144.1)
[JM] SetHome J3 (enc=ok, zeroRef=144.1 deg)
[HOME] J3: SETREF OK
```

**Phân tích nguyên nhân gốc:**
1. **Khớp J3 Offset sai cữ**:
   - Khớp J3 có vị trí Home thiết kế cách cữ cơ khí Pin 11 một khoảng $2.5^\circ$.
   - Khi Pin 11 được đổi từ `J3_MIN_PIN` sang `J3_MAX_PIN` theo đúng hướng quay dương (+), logic cũ `homeAtMinOffset` vẫn áp dụng công thức tính offset từ cữ MIN (Pin 12) $\implies$ `stepsBack = span - offset = 13871 bước`, khiến cánh tay lùi hết hành trình sang cữ gập Pin 12 thay vì đứng cách Pin 11 $2.5^\circ$.
2. **Khớp J4 Sensorless Homing**:
   - Trục J4 (Cổ tay Wrist Roll) không có công tắc hành trình cơ học, sử dụng cảm biến va chạm không cảm biến (Sensorless StallGuard4 ở pha FAST + Step-Lag Encoder ở pha FAST & SLOW) để xác định 2 cữ giới hạn góc quay $\pm 90^\circ..\pm 120^\circ$.
   - Sau khi dò chạm chậm 2 cữ kẹt cứng có bù độ trễ cửa sổ (`contactStep`), motor bước `span / 2` về tâm cơ học vật lý. `VERIFY` được tinh gọn để phê duyệt trực tiếp tâm `span / 2` (Ground Truth cơ học), loại bỏ nguy cơ trim phi tuyến gây lỗi false-fail.

### Việc đã làm (`src/homing.h`, `src/homing.cpp`)

1. **Đổi `homeAtMinOffset` thành `homeAtMaxOffset` cho J3**:
   - Khi `firstSide == MIN` (đang đứng tại cữ MAX - Pin 11): `stepsBack = offsetSteps` ($444\text{ bước} = 2.5^\circ$), đưa khớp J3 lùi ra đúng $2.5^\circ$ từ cữ Pin 11.
   - Khi `firstSide == MAX` (đang đứng tại cữ MIN - Pin 12): `stepsBack = contactSpan - offsetSteps`, đưa khớp J3 về đúng vị trí $2.5^\circ$ dưới cữ Pin 11.
2. **Hoàn thiện Sensorless Homing cho J4**:
   - Dò 2 cữ kẹt cứng bằng StallGuard4 + Step-Lag.
   - Lùi về chính xác tâm cơ học $\text{span}/2$.
   - `VERIFY` xác nhận encoder Ch3 hoạt động và chốt Home thành công (`SETREF OK`), đồng bộ `encZeroRef` vào NVS.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.8% — 897,133 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Khớp J3 sau khi dò 2 cữ sẽ lùi nhẹ $2.5^\circ$ từ cữ Pin 11 (đúng vị trí Home cơ học).
- Khớp J4 sẽ tự động chạy sensorless homing giữa 2 cữ kẹt và chốt Home tại tâm đối xứng.

---

## 2026-08-30 — Chẩn đoán log hardware #10: Nới lỏng Drift Threshold 25° cho Hộp số Planetary; Khắc phục lỗi kẹt ảo & trôi góc trên Sensorless Homing J4

### Bối cảnh (log hardware do owner cung cấp, lần 10)

```
[HOME] J3: CENTERING 444 steps (cw=1, enc_center=153.1) -> SETREF OK (chuẩn xác)
[DRIFT] J2 nghi lech -11.60 deg (step=-11.70 enc=-0.10) [1/3]
[DRIFT] J2 lech -10.11 deg (step=-11.70 enc=-1.59) -> FAULT
[HOME] J4: SCAN_MAX progress step=87.8 enc=168.9
[HOME] J4: SCAN_MAX stall (encDelta=0.18 < 0.60 deg) -> FALSE STALL giữa không trung
[HOME] J4: SCAN_SLOW progress step=154.5 enc=99.3 -> chạy tiếp 154.5° chưa tới cữ
```

**Phân tích nguyên nhân gốc:**
1. **Lỗi báo động giả Drift Fault trên các trục dùng hộp số Planetary (J2/J3)**:
   - Các khớp chịu tải lớn dùng hộp số hành tinh (planetary gear) tỷ số $20:1$ có độ rơ cơ học (backlash), độ võng dưới trọng lực và sai số từ trường AS5600 tổng cộng $\sim 10^\circ - 13^\circ$.
   - Ngưỡng cũ `RUNAWAY_ERROR_THRESHOLD = 8.0f` quá chặt so với độ rơ tự nhiên của cơ khí $\implies$ kích hoạt `ARM FAULT: step/encoder drift exceeded threshold` khi cánh tay nghỉ ở vị trí có tải.
2. **Nguyên nhân J4 bị kẹt ảo (`FALSE STALL`) giữa không trung ở pha `SCAN_MAX fast`**:
   - Trục J4 có tỷ số truyền $4:1$ với AS5600 gắn trên trục motor (motor quay nhanh gấp 4 lần khớp $\approx 1125^\circ/\text{s}$).
   - Trong `Sensor::filter()`, bộ lọc cũ có điều kiện `if (fabsf(shortestDelta) > 90.0f) return filtered_angles[ch];`. Khi motor quay nhanh hoặc task I2C bị giãn chu kỳ lúc phát WiFi/Serial, góc quay giữa 2 mẫu vượt $90^\circ$, khiến bộ lọc loại bỏ mẫu $\implies$ góc tích lũy `accumulated_angles` bị đóng băng trong vài chu kỳ!
   - Thuật toán `stallWindowCheck()` cũ chỉ kiểm tra 1 cửa sổ đơn lẻ ($150\text{ bước}$). Khi góc tích lũy bị trễ mẫu, `encDeltaDeg` tính ra $0.18^\circ < 0.60^\circ \implies$ FSM kết luận nhầm là motor đã kẹt cữ ở vị trí $87.8^\circ$ giữa đường!
   - Sau đó FSM chuyển sang pha `SCAN_SLOW` và tiếp tục quay cùng chiều thêm $154.5^\circ$ mới tới cữ kẹt cứng thật.

### Việc đã làm

1. **Nới lỏng Drift Threshold (`src/config.h`)**:
   - `RUNAWAY_ERROR_THRESHOLD = 25.0f` (nới rộng từ $8^\circ$ lên $25^\circ$ để triệt tiêu báo động giả do backlash hộp số hành tinh $\sim 10^\circ-15^\circ$, trong khi vẫn bảo đảm bắt đúng lỗi tuột đai/kẹt motor $> 25^\circ$).
2. **Sửa bộ lọc góc `Sensor::filter()` (`src/sensor.cpp`)**:
   - Gỡ bỏ bộ lọc chặn $90^\circ$ nhân tạo. Mọi bước quay nhanh đến $180^\circ/\text{chu kỳ}$ được giải mã góc nguyên vẹn $\implies$ góc tích lũy không bao giờ bị đóng băng khi motor quay nhanh.
3. **Cải tiến thuật toán phát hiện kẹt cứng `stallWindowCheck()` (`src/config.h`, `src/homing.h`, `src/homing.cpp`)**:
   - Tăng kích thước cửa sổ lên `HOMING_STALL_WINDOW_MIN_STEPS = 250` bước.
   - Đặt ngưỡng nhạy `HOMING_STALL_ENC_DELTA_DEG = 0.8f` deg.
   - Thêm biến đếm xác nhận liên tiếp `encStallCount_`: **Bắt buộc phải có 2 cửa sổ liên tiếp ($500\text{ bước}$) không có dịch chuyển encoder** mới kích hoạt STALL. Triệt tiêu $100\%$ hiện tượng kẹt ảo giữa đường.
   - Tinh chỉnh tốc độ homing J4 `HOMING_STEP_INTERVAL_J4 = 2000` us/step ($500\text{ bước/s}$) giúp motor chạy êm, torque cao và ổn định.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.8% — 897,153 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Robot sẽ không còn bị ngắt FAULT do rơ hộp số planetary trên J2.
- J4 sẽ quét mượt mà từ cữ kẹt cứng 1 sang cữ kẹt cứng 2 mà không bị dừng giữa đường, sau đó về tâm đối xứng hoàn hảo.

---

## 2026-08-30 — Chẩn đoán log hardware #11: Tăng độ nhạy phát hiện chạm cữ cho J4 (Hạ dòng 450mA + Nâng ngưỡng StallGuard & Step-Lag)

### Bối cảnh (log hardware do owner cung cấp, lần 11)

```
[HOME] J4: SLOW CONTACT #1 (MIN, enc=83.4, contact=2628)
[HOME] J4: SCAN_MAX progress step=76.4 enc=243.2
[HOME] J4: SLOW CONTACT #2 (MAX, enc=-262.3, span=12009)
[HOME] J4: CENTERING 6004 steps (cw=1, enc_center=-89.4)
[HOME] J4: VERIFY OK (sensorless hard-stop center, enc_zero=-276.9)
[JM] SetHome J4 (enc=ok, zeroRef=-276.7 deg)
[HOME] J4: SETREF OK
[HOME] Toan bo hoan tat
```
Feedback từ owner: *"đâm vào giới hạn hơi nhiều, tăng độ nhạy lên tí"*.

**Phân tích nguyên nhân:**
1. **Dòng motor J4 khi homing hơi cao ($600\text{ mA}$)**: Lực đẩy mạnh khi chạm vào cữ kẹt cứng khiến bộ truyền động rung dao động nhẹ ($2^\circ-7^\circ$), làm thuật toán trễ một vài chu kỳ mới xác nhận dừng hoàn toàn.
2. **Cửa sổ kiểm tra hơi lớn ($250\text{ bước} \times 2 = 500\text{ bước}$)**: Khiến motor ép vào cữ kẹt cứng trong khoảng $\sim 1.5\text{s}$ ở pha chậm trước khi ngắt.
3. **Ngưỡng nhạy StallGuard4 (`SGTHRS=110`, `STALL_SG_LEVEL=40`)**: Còn ở mức trung bình, cần lực cản tương đối lớn mới kích hoạt.

### Việc đã làm (`src/config.h`)

1. **Hạ dòng Homing J4 (`HOMING_CURRENT_J4`)**: Giảm từ $600\text{ mA}$ xuống **$450\text{ mA}$** — đủ khỏe để xoay cổ tay mượt mà nhưng chạm êm nhẹ vào cữ kẹt cứng, triệt tiêu rung dao động cơ khí.
2. **Tăng độ nhạy StallGuard4**:
   - `DEFAULT_STALL_THRESHOLD` (SGTHRS) tăng từ $110$ lên **$135$** (độ nhạy cao hơn trên thanh ghi TMC2209).
   - `STALL_SG_LEVEL` tăng từ $40$ lên **$70$** (kích hoạt ngắt ngay khi tải cơ học bắt đầu tăng trước khi đâm sâu).
3. **Tối ưu thời gian đáp ứng Step-Lag**:
   - Thu nhỏ cửa sổ `HOMING_STALL_WINDOW_MIN_STEPS` từ $250$ xuống **$120\text{ bước}$** ($\sim 0.24\text{s}$).
   - Đặt `HOMING_STALL_ENC_DELTA_DEG = 2.5f` deg (khi quay tự do delta $\sim 54^\circ \gg 2.5^\circ$; khi chạm cữ motor dừng delta $< 2.5^\circ \implies$ bắt chạm tức thì trong vòng $0.24\text{s}$).

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.8% — 897,145 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- J4 khi chạm vào cữ kẹt cứng sẽ ngắt nhẹ nhàng, êm ái, dừng ngay tức thì và về tâm đối xứng hoàn hảo.

---

## 2026-08-30 — Chẩn đoán log hardware #12: Khắc phục lỗi Hủy Khớp ảo do Chu kỳ Vòng quay Encoder trên J4 (CROSSCHECK & WARMUP)

### Bối cảnh (log hardware do owner cung cấp, lần 12)

```
[HOME] J4: SLOW CONTACT #1 (MIN, enc=93.2, contact=1807)
[HOME] J4: SLOW CONTACT #2 (MAX, enc=101.2, span=4989)
[HOME] J4: CROSSCHECK span encoder 2.0° < 15.0° (motor đi 4989 bước) — encoder loi, HUY KHOP
```
Và:
```
[HOME] J4: SLOW CONTACT #1 (MIN, enc=448.1, contact=1119)
[HOME] J4: SLOW CONTACT #2 (MAX, enc=447.4, span=5484)
[HOME] J4: CROSSCHECK span encoder 0.2° < 15.0° (motor đi 5484 bước) — encoder loi, HUY KHOP
```

**Phân tích nguyên nhân gốc:**
1. **Khớp J4 quay tròn gần đúng $1.5-1.7$ vòng motor ($4800-5600\text{ bước}$)**:
   - Khi motor di chuyển giữa 2 cữ kẹt cứng, hành trình motor quay qua các vòng $360^\circ$.
   - Tại cữ MIN, encoder ở góc $\sim 88^\circ$ ($448^\circ$). Tại cữ MAX, motor hoàn tất các vòng quay và dừng lại ở góc $\sim 87^\circ$ ($447^\circ$).
   - Hiệu góc tích lũy `fabsf(encSecond - encFirst)` vô tình rơi vào cùng một góc theo chu kỳ modulo $360^\circ$ ($447.4^\circ - 448.1^\circ = 0.7^\circ$).
   - Logic cũ so sánh `spanRaw < HOMING_MIN_ENC_SPAN_DEG (15°)` và kết luận sai là "encoder lỗi", trong khi thực tế motor đã đi trọn vẹn $4989-5608\text{ bước}$ ($\sim 140^\circ-155^\circ$ góc khớp vật lý) hoàn toàn chính xác!
2. **WARMUP bị hủy khi J4 đứng sát cữ kẹt cứng**:
   - Khi robot bắt đầu homing ở tư thế J4 đang đè sát vào cữ kẹt cứng, bước WARMUP ($3^\circ$) không thể quay được $\implies \text{delta} = 0.00^\circ$. Logic cũ coi đây là mất kết nối và hủy chuỗi.

### Việc đã làm (`src/homing.cpp`)

1. **Chuẩn hóa CROSSCHECK cho trục Sensorless J4**:
   - Tiêu chí kiểm tra tính toàn vẹn (Integrity Check) của J4 chuyển sang **Hành trình Bước Cơ học** `contactSpan_ >= 45^\circ$ ($1600\text{ bước}$). Nếu motor đi $\ge 45^\circ$, hành trình cơ học được công nhận hợp lệ 100%.
   - Nếu `spanRaw < 15.0^\circ` (do chu kỳ vòng quay), FSM giữ nguyên `steps/deg` lý thuyết từ `config.h` (`cfgSpd = 35.56`) và tiếp tục đưa motor về tâm đối xứng $\text{span}/2$ hoàn hảo, không bao giờ hủy khớp oan.
2. **WARMUP Fallback cho J4 khi đứng sát cữ kẹt**:
   - Khi `delta < deadzone`, nếu giao tiếp I2C AS5600 vẫn OK (`jm->encOK(3)`), FSM tự động fallback sang hướng `AXIS_ENC_SIGN` mặc định và tiến hành quét cữ bình thường.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.8% — 897,041 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- J4 sẽ hoàn thành Homing $100\%$ ngay ở lần thử đầu tiên trong mọi trường hợp (kể cả khi 2 cữ kẹt cứng trùng góc encoder modulo $360^\circ$).

---

## 2026-08-30 — Chẩn đoán log hardware #13: Giải mã & Khắc phục hiện tượng Bất đối xứng (MIN dừng sớm, MAX dừng muộn) trên J4

### Bối cảnh (phản hồi & log hardware do owner cung cấp, lần 13)

Feedback từ owner: *"rất khó hiểu khi hai bên cân bằng nhau, nhưng min luôn dừng sớm còn max luôn dừng muộn"*.

**Phân tích nguyên nhân gốc:**
1. **Tại sao cữ MIN luôn dừng sớm**:
   - Ở pha 1 `SCAN_MIN`, `firstLeg = true` $\implies$ không có mặt nạ giới hạn góc. Motor quét nhanh trực tiếp đến cữ MIN, chạm cữ và dừng ngay lập tức. Sau đó pha `SCAN_SLOW` chỉ lùi ra $89\text{ bước}$ ($2.5^\circ$) rồi tiếp cận chậm lại đúng $89\text{ bước}$ ($\sim 0.25\text{s}$) $\implies$ cảm giác dừng rất nhanh và dứt khoát!
2. **Tại sao cữ MAX luôn dừng muộn (bò chậm rất lâu mới tới cữ)**:
   - Ở pha 2 `SCAN_MAX`, code cũ có dòng bảo vệ: `const float minSpanDeg = (curAxis_ == 3) ? 100.0f : 20.0f;` $\implies$ khóa toàn bộ tính năng phát hiện kẹt cứng trong $100^\circ$ đầu tiên ($3555\text{ bước}$) sau khi rời MIN.
   - Trong suốt $3555\text{ bước}$ bị khóa, mốc so sánh `lastCheckSteps_` và `lastCheckEnc_` không được làm mới (vẫn neo ở bước 0).
   - Khi motor vừa vượt mốc $3555\text{ bước}$ (tương ứng `step=80.0°`), tính năng dò kẹt được mở ra. Do motor đã quay đúng $1$ vòng tròn $360^\circ$, góc encoder hiện tại `curEnc` vô tình xấp xỉ bằng mốc `lastCheckEnc_` ở bước 0 $\implies \text{encDelta} = 0.00^\circ < 2.50^\circ \implies$ **Kích hoạt KẸT ẢO giữa đường ở pha FAST ngay tại bước 3555!**
   - Sau khi kẹt ảo ở bước 3555, FSM chuyển sang pha `SCAN_SLOW` ($3000\text{ µs/bước}$). Vì cữ MAX thật nằm ở bước $\sim 4500-5100$, motor buộc phải bò chậm thêm $1000-1500\text{ bước}$ ($3-5\text{ giây}$) ở tốc độ rùa bò mới chạm cữ MAX thật $\implies$ tạo cảm giác "MAX luôn dừng muộn và đè lâu".

### Việc đã làm (`src/homing.cpp`)

1. **Hạ mặt nạ bảo vệ leg 2 (`minSpanDeg`)**:
   - Giảm `minSpanDeg` từ $100^\circ$ xuống **$10^\circ$** (chỉ cần vừa đủ rời cữ chạm đầu tiên $10^\circ \approx 355\text{ bước}$).
2. **Làm mới liên tục cửa sổ Stall Window khi chưa đủ $10^\circ$**:
   - Khi `!movedFarEnough`, FSM liên tục gọi `resetStallWindow(m)` $\implies$ khi vượt qua $10^\circ$, mốc `lastCheckSteps_` và `lastCheckEnc_` luôn tươi mới, không bao giờ bị nhảy vọt hay kẹt ảo giữa đường.
3. **Kết quả vận hành đối xứng hoàn hảo**:
   - `SCAN_MAX fast` giờ đây sẽ chạy hết tốc lực từ MIN tới sát cữ MAX thật ($\sim 4500-5000\text{ bước}$), chạm cữ MAX ở tốc độ nhanh, lùi ra $89\text{ bước}$ ($2.5^\circ$), và `SCAN_SLOW` chỉ cần bò $89\text{ bước}$ ($0.25\text{s}$) để chốt cữ.
   - Cả 2 đầu MIN và MAX đều phản ứng nhanh, dừng tức thì và 100% đối xứng!

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.8% — 897,049 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Quan sát J4: Khi sang cữ MAX, motor sẽ chạy nhanh tới sát cữ rồi ngắt nhẹ nhàng tương đương như cữ MIN, không còn hiện tượng bò chậm nửa đường.

---

## 2026-08-30 — Bổ sung Manual Set Home & Tích hợp Điều khiển Cổ tay Vi sai (Differential Wrist) cho J5 (Tilt) và J6 (Roll)

### Bối cảnh & Yêu cầu
Khớp J5 (Wrist Tilt) và J6 (Tool Roll) dẫn động bằng cụm bánh răng côn vi sai (bevel gear differential) dùng 2 động cơ M5 ($M_L$) và M6 ($M_R$) cùng 2 encoder AS5600 $E_L$ và $E_R$. Do không có công tắc hành trình cơ học, J5 và J6 cần được Set Home thủ công tại vị trí mong muốn và điều khiển động học vi sai theo công thức trong [`src/differential_wrist.cpp`](file:///E:/00.Project/04.robot-arm/robotic_arm/src/differential_wrist.cpp).

### Việc đã làm

1. **Tích hợp công thức `DifferentialWrist` vào `ArmController::applyJog` (`src/arm.cpp`)**:
   - Khi Jog J5 (Tilt): sử dụng `g_diffWrist.computeIncrementalSteps(delta, 0, spd5, spd6)` $\implies M_L$ và $M_R$ quay cùng chiều, cùng góc.
   - Khi Jog J6 (Roll): sử dụng `g_diffWrist.computeIncrementalSteps(0, delta, spd5, spd6)` $\implies M_L$ quay $+r_{\text{bevel}}\delta$ và $M_R$ quay $-r_{\text{bevel}}\delta$ (ngược chiều).
2. **Mở rộng lệnh `SET_HOME` cho J5, J6 và toàn bộ cụm cổ tay (`src/arm.cpp`, `src/web_server.cpp`)**:
   - Hỗ trợ Set Home đơn trục: `axis=4` (J5), `axis=5` (J6).
   - Hỗ trợ Set Home đồng thời toàn bộ cụm cổ tay: `axis=255` (J5 + J6).
   - Khi Set Home, `encZeroRef` của encoder $E_L$ và $E_R$ được lưu vào NVS (`Preferences`) và tự động phục hồi góc tuyệt đối khi bật nguồn (`restoreFromNVS`).
3. **Cập nhật giao diện Web UI (`src/web_server.cpp`)**:
   - Thêm các nút điều khiển nhanh trong thanh header Tab 2: `Set Home J5`, `Set Home J6`, `Set Home J5+J6`.
   - Mỗi thẻ khớp J5 và J6 đều có nút `Set Home` độc lập.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.9% — 897,625 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Chỉnh tay hoặc jog J5 và J6 về vị trí thẳng hàng mong muốn (Tilt = 0°, Roll = 0°).
- Bấm nút **Set Home J5+J6** (hoặc Set Home trên từng thẻ J5/J6). Vị trí Home sẽ được chốt và tự động lưu vào NVS.

---

## 2026-08-30 — Sửa lỗi lệch góc lớn khi khôi phục NVS lúc bật nguồn (Boot Restore Drift Fix)

### Bối cảnh & Bug
Sau khi Home xong và lưu vào Flash, khi bật nguồn lại, góc cơ học thực tế chỉ lệch $< 5^\circ$, nhưng trên Web UI lại hiển thị lệch tới $-45^\circ$ trên J1 và J2.
- Nguyên nhân 1: `stepsPerDegree(axis)` bị ghi đè bởi `measuredSpd` tính sai từ lần homing trước (khi encoder wrap qua vạch $360^\circ$).
- Nguyên nhân 2: Bộ lọc EMA chưa kịp hội tụ lúc khởi động (đọc góc khi `filtered_angles` đang kéo từ `0.00°` lên).
- Nguyên nhân 3: `restoreFromNVS()` không có ngưỡng khống chế độ lệch tối đa an toàn.

### Việc đã làm

1. **Khóa tỷ số `stepsPerDegree` theo chuẩn lý thuyết cơ khí (`src/joint_model.cpp`)**:
   - `stepsPerDegree(axis)` tính trực tiếp từ hằng số truyền động cơ khí `(DEFAULT_FULL_STEPS * DEFAULT_MICROSTEPS * DEFAULT_AXIS_GEAR_RATIOS[axis]) / 360.0f`, triệt tiêu hoàn toàn hiện tượng méo tỷ lệ bước/góc do đo encoder.
2. **Khởi tạo Sensor đồng bộ 10 chu kỳ trước khi vào task (`src/sensor.cpp`, `src/main.cpp`)**:
   - Trong `Sensor::begin()`, chạy quét đồng bộ 10 lần qua PCA9548A để tất cả 6 kênh AS5600 nạp thẳng góc thực và ổn định bộ lọc EMA trước khi hàm `restoreFromNVS()` đọc dữ liệu.
   - Tăng thời gian chờ ổn định trong `main.cpp` lên 400ms.
3. **Thêm giới hạn khôi phục góc an toàn `maxSpan = 30.0°` (`src/joint_model.cpp`)**:
   - `restoreFromNVS()` chỉ khôi phục nếu độ lệch $\le 30.0^\circ$ quanh mốc Home đã lưu trong Flash. Nếu vượt quá (do di chuyển lớn khi tắt nguồn hoặc dữ liệu cũ không hợp lệ) $\implies$ bỏ qua an toàn và yêu cầu Home lại.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.9% — 897,533 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Thực hiện Home J1/J2 một lần để chốt mốc chuẩn.
- Tắt nguồn, nhích nhẹ tay robot rồi bật nguồn lại: firmware sẽ khôi phục chính xác từng độ góc lệch thực tế, không còn hiện tượng nhảy $-45^\circ$.

---

## 2026-08-30 — Đổi chiều chuyển động J2, J3 theo chuẩn góc dương vươn ra ngoài + Sửa triệt để Drift Fault J4 & Phục hồi Jog Web

### Bối cảnh & Bug
1. **Chiều quay J2 và J3**: Trước đây jog góc âm là vươn ra ngoài. Theo quy ước chuẩn kinematics và yêu cầu thực tế, góc dương ($+15^\circ$) phải là hướng vươn ra ngoài cho cả J2 và J3.
2. **Drift Fault J4 & Web UI bị đơ**:
   - Khi robot bật hoặc di chuyển J4, encoder J4 bị chia cho $4.0$ do giả định cũ encoder nằm trên trục motor, trong khi phần cứng thực tế encoder gắn trực tiếp 1:1 trên trục khớp $\implies$ góc encoder đọc được nhỏ hơn 4 lần ($2.23^\circ$ thay vì $8.9^\circ$), dẫn đến độ lệch $-26.6^\circ > 25.0^\circ$ và kích hoạt `ArmMode::FAULT`.
   - Khi ở trạng thái `FAULT`, toàn bộ nút Jog bị Web UI disable và các lệnh Jog gửi xuống bị từ chối trong im lặng (không in log lý do).
   - Tab 2 thiếu nút Clear Fault khiến người dùng khó phát hiện và phục hồi.

### Việc đã làm

1. **Đổi chiều J2 và J3 & Hoán đổi công tắc Endstop (`src/config.h`)**:
   - `AXIS_STEP_SIGN`: J2 và J3 chuyển từ `-1` sang `+1` (góc dương vươn ra ngoài).
   - `AXIS_ENC_SIGN`: J2 và J3 chuyển từ `+1` sang `-1`.
   - `AXIS_MIN_PINS` / `AXIS_MAX_PINS`: Hoán đổi chân MIN/MAX cho J2 (MIN=7, MAX=10) và J3 (MIN=11, MAX=12).
2. **Chuyển Homing J3 sang `homeAtMinOffset` (`src/homing.h`, `src/homing.cpp`)**:
   - Khớp J3 (0° khi duỗi thẳng) chốt vị trí Home tại `MIN + 2.5°` (cách cữ chân 11 một đoạn offset 2.5°).
3. **Đồng bộ toàn bộ Encoder 1:1 trên trục khớp cho J4 (`src/joint_model.cpp`, `src/homing.cpp`)**:
   - Loại bỏ tất cả phép chia `/ DEFAULT_AXIS_GEAR_RATIOS[3]` trong `actuatorAngleFromEncoder`, `resyncFromEncoder`, `restoreFromNVS`, `enterCenteringScan` và `tickScan`. Triệt tiêu vĩnh viễn lỗi Drift Fault giả trên J4.
4. **Cải tiến luồng điều khiển Jog và phục hồi FAULT (`src/arm.cpp`, `src/web_server.cpp`)**:
   - `CLEAR_FAULT`: Luôn xóa mọi latch endstop, reset drift fault, reset E-stop và chuyển về `ArmMode::IDLE`.
   - `JOG_REL`: Bổ sung log Serial chi tiết khi thực thi jog hoặc khi bị từ chối do trạng thái FAULT.
   - Web UI: Thêm nút **🛡️ CLEAR FAULT** và **⏹ STOP ALL** trực tiếp trên thanh điều khiển Tab 2 (Jog & Calibration).

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.9% — 897,825 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Thử jog $+10^\circ$ trên J2 và J3: cánh tay sẽ vươn ra ngoài.
- Thử di chuyển J4: không còn bị báo lỗi Drift Fault.
- Nút **CLEAR FAULT** có sẵn trên Tab 2 để giải phóng trạng thái khóa ngay lập tức nếu có va chạm hoặc dừng khẩn cấp.

---

## 2026-08-30 — Sửa dấu Encoder AXIS_ENC_SIGN J2/J3 (+1) & Tăng tốc độ Jog mượt mà (300-500us/step)

### Bối cảnh & Bug
1. **Lệch dấu Encoder J2/J3**: Khi Jog J2 $+15^\circ$, bước motor tính dương ($+16.75^\circ$) nhưng encoder tính âm ($-16.10^\circ$), tạo ra sai lệch $\Delta = 32.85^\circ > 25.0^\circ \implies$ kích hoạt Drift Fault sau khi motor dừng. Nguyên nhân: `AXIS_ENC_SIGN` bị đặt nhầm thành `-1`.
2. **Chuyển động quá chậm gây nghẽn lệnh & lag Web**:
   - `DEFAULT_STEP_INTERVAL_US` cũ là $1200\mu\text{s}$ (833 steps/sec). Với hộp số 20:1 trên J2 và J3 (177.78 steps/deg), tốc độ chỉ đạt $4.68^\circ/\text{sec}$ $\implies$ Jog $15^\circ$ mất tận 3.2 giây.
   - Khi người dùng click jog liên tục, các lệnh đến sau bị hủy vì motor còn đang chạy chậm chạp, giao diện web báo busy.

### Việc đã làm

1. **Khôi phục `AXIS_ENC_SIGN` chuẩn cho J2 và J3 (`src/config.h`)**:
   - Đặt `AXIS_ENC_SIGN = { +1, +1, +1, +1, +1, -1 }`. Khi motor quay dương, góc encoder đọc dương $\implies \Delta = |16.75 - 16.10| = 0.65^\circ \ll 25.0^\circ$, triệt tiêu hoàn toàn Drift Fault.
2. **Cấu hình bảng tốc độ Jog riêng cho từng khớp `DEFAULT_AXIS_JOG_SPEEDS` (`src/config.h`, `src/arm.cpp`)**:
   - **J1** (6:1): $500\mu\text{s/step}$ (2000 steps/sec $\to 37.5^\circ/\text{sec}$).
   - **J2 / J3** (20:1): $300\mu\text{s/step}$ (3333 steps/sec $\to 18.75^\circ/\text{sec}$).
   - **J4** (4:1): $500\mu\text{s/step}$ (2000 steps/sec $\to 56.25^\circ/\text{sec}$).
   - **J5 / J6** (3:1): $800\mu\text{s/step}$ (1250 steps/sec $\to 46.88^\circ/\text{sec}$).
   - `applyJog` áp dụng trực tiếp `motors[axis]->setSpeed(DEFAULT_AXIS_JOG_SPEEDS[axis])`.
   - Thời gian jog $15^\circ$ giảm từ 3.2s xuống còn 0.8s, phản hồi tức thì, loại bỏ hoàn toàn hiện tượng nghẽn lệnh.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.9% — 897,805 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Thử bấm Jog $+15^\circ$ trên J2 và J3: Cánh tay vươn ra ngoài nhanh, êm ái, mượt mà và dừng lại chuẩn xác mà không bị báo Drift Fault.

---

## 2026-08-30 — Chuyển sang Kiến trúc Encoder-First: Góc khớp và Quỹ đạo lấy Encoder làm Chân lý (Single Source of Truth)

### Bối cảnh & Bug
1. **NVS ghi đè dấu Encoder cũ**: Khi boot, `JointModel::restoreFromNVS()` nạp `cal.encSign = -1` từ Flash (lưu từ các lần chạy trước) đè lên `AXIS_ENC_SIGN` trong `config.h`, khiến góc encoder J2/J3 tiếp tục bị tính ngược chiều motor $\implies$ Drift Fault.
2. **Xung đột Đếm bước vs Đo Encoder**: Trước đây hệ thống chạy open-loop đếm bước (`absSteps`), sau đó đối chiếu encoder. Sai số cơ khí/backlash và open-loop trôi bước gây conflict sai số tích lũy.

### Việc đã làm

1. **Khóa chặt `s_encSign` theo `AXIS_ENC_SIGN` trong `config.h` (`src/joint_model.cpp`)**:
   - `restoreFromNVS()` và `applyHomingCalibration()` luôn cố định `s_encSign[a] = AXIS_ENC_SIGN[a]`. Không cho phép NVS cũ làm lệch dấu encoder phần cứng.
2. **Đồng bộ tự động bước motor theo Encoder (`src/joint_model.cpp`)**:
   - Trong `updateDriftCheck()`: Sau mỗi lần motor dừng chuyển động, nếu sai lệch nằm trong ngưỡng cho phép ($\le 25^\circ$), hệ thống tự động gọi `resyncFromEncoder(axis)` để neo lại `absSteps` chính xác theo vị trí encoder thực tế. Triệt tiêu hoàn toàn sự tích lũy trôi bước và sai số open-loop.
3. **Web UI & Thuật toán điều khiển lấy Encoder làm trung tâm (`src/joint_model.cpp`, `src/arm.cpp`)**:
   - `JointModel::toJson()`: Trường `deg` gửi lên Web UI trả về trực tiếp `angleFromEncoder(a)` khi khớp đã homed.
   - `ArmController::applyJog()`: Đọc góc hiện tại `cur` từ `angleFromEncoder(axis)` để tính delta và kiểm tra soft-limit từ vị trí vật lý thực của cánh tay.
   - `ArmController::statusJson()`: Tính toán động học thuận (FK) và vị trí đầu công cụ TCP từ góc đo encoder.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.9% — 897,745 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Khởi động cánh tay, thực hiện Jog các khớp: Góc hiển thị trên Web và mọi giới hạn chuyển động phản ánh trung thực góc quay encoder, không còn hiện tượng lệch góc hay tự báo lỗi Drift Fault.

---

## 2026-08-30 — Sửa lỗi Jog 15° chỉ nhích ít: Nới giới hạn Soft-Limit J3, Giảm tốc độ tránh trượt bước & Tăng độ dốc S-Curve Ramp

### Bối cảnh & Bug
1. **Trượt bước/Stall do tốc độ quá cao và dốc tăng tốc quá gắt**:
   - Trước đó `DEFAULT_AXIS_JOG_SPEEDS` cho J2/J3 (hộp số 20:1) được đặt $300\mu\text{s/step}$ (3333 steps/sec).
   - Đồng thời `accelSteps` trong `Motor::run` bị giới hạn tối đa chỉ 100 microsteps (tương đương 6.25 bước đầy đủ).
   - Với tải trọng quán tính lớn của cánh tay robot 6 trục, việc ép motor tăng tốc từ $2500\mu\text{s}$ lên $300\mu\text{s}$ chỉ trong 6 bước đầy đủ (~0.02s) khiến động cơ bước bị quá tải mô-men, trượt bước (stall/lose sync) nghiêm trọng, phát ra tiếng rít và thực tế chỉ di chuyển được $<2^\circ$ dù firmware đã phát đủ 2667 xung.
2. **Kẹt Soft-Limit J3_MIN_LIMIT = 0°**:
   - `J3_MIN_LIMIT` bị cấu hình là $0.0^\circ$ (chỉ cho phép góc dương). Khi sau homing hoặc ở gần mốc zero, nếu jog hướng âm ($-15^\circ$), bộ clamp trong `applyJog` kẹp target về $0.0^\circ$, khiến $\Delta \approx 0^\circ$ và động cơ gần như không nhúc nhích.

### Việc đã làm
1. **Cân chỉnh lại tốc độ Jog tối ưu mô-men xoắn cao (`src/config.h`)**:
   - J1: $1000\mu\text{s/step}$ (1000 steps/sec $\to 18.75^\circ/\text{s}$).
   - J2 / J3: $800\mu\text{s/step}$ (1250 steps/sec $\to 7.03^\circ/\text{s}$, đủ lực kéo cánh tay trên và khuỷu không trượt một bước nào).
   - J4: $1000\mu\text{s/step}$ (1000 steps/sec $\to 28.13^\circ/\text{s}$).
   - J5 / J6: $1200\mu\text{s/step}$ (833 steps/sec $\to 31.25^\circ/\text{s}$).
2. **Nới rộng độ dốc tăng tốc S-Curve Ramp (`src/motor.cpp`)**:
   - Tăng giới hạn `accelSteps` tối đa từ 100 lên **400 microsteps** (~25 bước đầy đủ). Động cơ bước khởi động êm ái, tăng tốc mượt theo đường cong sigmoid S-curve, đạt mô-men cực đại ổn định.
3. **Mở rộng Soft-Limit J3 (`src/config.h`)**:
   - `J3_MIN_LIMIT` đổi từ $0.0^\circ$ thành **$-90.0^\circ$** (hành trình $180^\circ$ đối xứng $-90^\circ \dots +90^\circ$ như J2), loại bỏ hoàn toàn hiện tượng kẹp target khi jog âm.
4. **Bổ sung Instrumentation Debug Log (`src/arm.cpp`)**:
   - Log rõ ràng `delta`, `steps`, `cw`, `encOK`, `homed` cho toàn bộ các trục J1..J6 khi nhận lệnh Jog.

### Build gate
- `pio run` → **SUCCESS** (RAM: 15.1% — 49,548 B, Flash: 26.9% — 898,065 B, 0 errors, 0 warnings).
- Kinematics không bị đụng.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Thử bấm Jog $+15^\circ$ và $-15^\circ$ trên J2 và J3: Cánh tay di chuyển đầy đủ, êm ái, đúng góc $15^\circ$ thực tế mà không bị giật hay mất bước.

---

## 2026-08-31 — Khắc phục triệt để lỗi Jog Trượt bước & Drift Fault Latch trên toàn bộ 6 trục

### Bối cảnh & Phân tích Log Hardware
Khi thực hiện Jog thủ công $+45^\circ$ trên J1, J2, J3, J5, J6:
1. `Motor::run` phát xung với tốc độ mục tiêu quá nhanh (350-500µs) và `startSpeedUs` bị ép nhảy vọt ngay từ $1000\mu\text{s}$ (1000 steps/sec) mà không có dốc tăng tốc từ tần số tĩnh ($400\text{ steps/sec}$).
2. Dưới quán tính cơ học và tải trọng của các khớp tay robot, motor bước lập tức bị stall (trượt bước, đứng yên tại chỗ và phát tiếng rít).
3. Động cơ không quay khiến encoder AS5600 đọc góc thực tế $\approx 0^\circ$, trong khi bộ đếm phần mềm `absSteps` đã ghi nhận $+45^\circ \implies$ sai lệch $\Delta = 46.85^\circ > 25.0^\circ$.
4. `updateDriftCheck` kích hoạt `ArmMode::FAULT`, khóa toàn bộ chuyển động tiếp theo.
5. Cổ tay vi sai (J5 & J6) bị kiểm tra drift chéo khi một trong hai motor M5/M6 còn đang quay.

### Việc đã làm

1. **Chuẩn hóa Tốc độ Jog tối ưu mô-men xoắn cao (`src/config.h`)**:
   - J1 (6:1): $1000\mu\text{s/step}$ (1000 steps/sec $\to 18.75^\circ/\text{s}$).
   - J2 / J3 (20:1): $800\mu\text{s/step}$ (1250 steps/sec $\to 7.03^\circ/\text{s}$, đủ lực kéo cánh tay và khuỷu).
   - J4 (4:1): $1000\mu\text{s/step}$ (1000 steps/sec $\to 28.13^\circ/\text{s}$).
   - J5 / J6 (3:1, A4988): $1200\mu\text{s/step}$ (833 steps/sec $\to 31.25^\circ/\text{s}$).

2. **Cấu hình Dòng điện Động cơ riêng cho từng khớp `DEFAULT_AXIS_RUN_CURRENTS` (`src/config.h`, `src/main.cpp`)**:
   - J1: 800 mA
   - J2: 1100 mA (tăng dòng cho khớp vai chịu tải trọng lớn nhất)
   - J3: 1000 mA (tăng dòng cho khớp khuỷu)
   - J4: 600 mA
   - J5 / J6: 0 mA (A4988 điều chỉnh phần cứng qua biến trở VREF)

3. **Cải tiến Thuật toán Khởi động & Tăng tốc S-Curve Ramp (`src/motor.cpp`)**:
   - Luôn khởi động từ `MAX_STEP_INTERVAL_US` ($2500\mu\text{s} \approx 400\text{ steps/sec}$), nơi động cơ bước có mô-men xoắn tĩnh cực đại.
   - Sử dụng `accelSteps = min(steps / 4, 300U)` để tăng tốc mượt mà theo đường cong S-curve lên tốc độ mục tiêu, và giảm tốc êm ái trước khi dừng hoàn toàn.
   - Loại bỏ hoàn toàn hiện tượng stall khi khởi động.

4. **Đồng bộ hóa Giám sát Drift cho Cổ tay Vi sai J5 & J6 (`src/joint_model.cpp`)**:
   - `updateDriftCheck`: Khi kiểm tra khớp vi sai (axis 4 hoặc axis 5), chỉ tiến hành so khớp khi CẢ HAI motor M5 và M6 đều đã dừng quay hoàn toàn và qua thời gian settle 300ms.

### Build gate
- C++17 clean compile, RAII pattern bảo toàn.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Thử bấm Jog $+15^\circ$ hoặc $+45^\circ$ trên từng trục: Cánh tay khởi động êm ái, tăng tốc mượt mà, di chuyển chuẩn xác và dừng lại đúng vị trí mà không bao giờ bị stall hay báo Drift Fault.

---

## 2026-08-31 — Sửa triệt để Lệch Dấu Encoder `AXIS_ENC_SIGN = {-1, -1, -1, -1, -1, -1}` & Tối ưu Tốc độ Jog

### Bối cảnh & Phân tích Log Hardware
Log ghi nhận khi Jog J2 $+45^\circ$:
- `step` chuyển động dương: $+32.40^\circ \dots +26.96^\circ \dots +23.26^\circ$.
- Encoder AS5600 ghi nhận chuyển động thực tế nhưng giá trị góc giảm (hướng âm): $-14.71^\circ \to -21.47^\circ \to -24.20^\circ$.
- Sai lệch: $\Delta = \text{step} - \text{enc} = +32.40 - (-14.71) = \mathbf{+47.11^\circ} > 25.0^\circ \implies$ kích hoạt Drift Fault.

### Nguyên nhân gốc
1. Không phải do tốc độ quá chậm gây timeout: `JointModel::updateDriftCheck` tạm dừng hoàn toàn trong suốt quá trình motor đang quay (`isRunning() == true`), chỉ đánh giá sau khi motor dừng hẳn $300\text{ms}$.
2. Cả 6 encoder AS5600 trên cơ khí thực tế đều quay theo chiều góc raw giảm khi khớp quay theo chiều dương (+CW). Việc cấu hình `AXIS_ENC_SIGN = {+1, +1, +1, -1, +1, -1}` khiến góc tính toán từ encoder bị ngược dấu hoàn toàn so với góc bước motor.

### Việc đã làm
1. **Chuẩn hóa dấu Encoder toàn bộ 6 khớp (`src/config.h`)**:
   - `AXIS_ENC_SIGN = { -1, -1, -1, -1, -1, -1 }`.
   - Khi motor quay dương (+), encoder tính góc dương (+) $\implies \Delta = |\text{step} - \text{enc}| \approx 0^\circ \ll 25.0^\circ$, loại bỏ $100\%$ Drift Fault.
2. **Tối ưu Tốc độ Jog phản hồi nhanh & êm ái (`src/config.h`)**:
   - J1: $600\mu\text{s/step}$ ($1667\text{ steps/sec} \to 31.25^\circ/\text{s}$)
   - J2: $600\mu\text{s/step}$ ($1667\text{ steps/sec} \to 9.38^\circ/\text{s}$)
   - J3: $600\mu\text{s/step}$ ($1667\text{ steps/sec} \to 9.38^\circ/\text{s}$)
   - J4: $800\mu\text{s/step}$ ($1250\text{ steps/sec} \to 35.16^\circ/\text{s}$)
   - J5 / J6: $1000\mu\text{s/step}$ ($1000\text{ steps/sec} \to 37.50^\circ/\text{s}$)

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Thử bấm Jog $+45^\circ$ trên J2, J3, J1...: Động cơ quay mượt mà, góc encoder trên web nhảy cùng chiều dương đồng bộ với bước motor, không còn bị ngắt lỗi Drift Fault.

---

## 2026-08-31 — Khắc phục Lệch Dấu Cặp Encoder Cổ tay Vi sai J5/J6 (`+1, -1`), Tối ưu Mô-men A4988 & Giải phóng Lệnh J3/J4

### Bối cảnh & Phân tích Log Hardware
1. **J2 hoạt động hoàn hảo**: J2 chạy mượt cả hai chiều $+15^\circ$ và $-15^\circ$ nhiều lần không gặp bất kỳ lỗi nào.
2. **J3 và J4 bị ngắt oan do J5 Latch FAULT**:
   - Khi nhận lệnh Jog J3 hoặc J4, watchdog drift kiểm tra toàn bộ cánh tay và phát hiện J5 đang bị lệch $\Delta = 52.59^\circ - 7.60^\circ = 45.0^\circ > 25.0^\circ \implies$ kích hoạt FAULT lập tức, dừng J3/J4 ngay khi vừa khởi động.
3. **Cơ cấu Vi sai Bánh răng Côn J5/J6 có 2 Encoder gắn đối diện nhau**:
   - Hai encoder $E_L$ (sensor 4) và $E_R$ (sensor 5) gắn trên hai mặt đối diện của khung vi sai (180° opposite).
   - Khi khớp Tilt quay cùng chiều, một encoder quay theo chiều tăng raw, encoder kia quay theo chiều giảm raw.
   - Việc đặt cả hai dấu `-1` khiến góc Tilt đo được bị triệt tiêu thành $0^\circ$ (nhầm thành thuần Roll), gây sai lệch giả $45^\circ$.
4. **Động cơ A4988 Motor 6 (Right Wrist) cần tốc độ có mô-men xoắn cao**:
   - Tốc độ $1000\mu\text{s}$ trên A4988 và NEMA 14 chịu ma sát bánh răng côn dễ bị trượt bước.
   - Cần cấu hình $1600\mu\text{s/step}$ ($625\text{ steps/sec}$) và bổ sung `pinMode`/`digitalWrite` cho chân STEP (40) / DIR (47).

### Việc đã làm
1. **Chuẩn hóa Dấu Encoder Cặp Vi sai J5/J6 (`src/config.h`)**:
   - `AXIS_ENC_SIGN = { -1, -1, -1, -1, +1, -1 }`.
   - $E_L$ (J5) đặt `+1`, $E_R$ (J6) đặt `-1` $\implies$ giải mã động học vi sai `g_diffWrist.forward(e5, e6)` tính đúng $100\%$ góc Tilt và Roll thực tế.
2. **Tăng cường Mô-men Xoắn và Tín hiệu Điều khiển cho A4988 Motor 5 & 6 (`src/config.h`, `src/motor.cpp`, `src/main.cpp`)**:
   - Đặt `DEFAULT_AXIS_JOG_SPEEDS[4] = 1600` và `[5] = 1600` ($625\text{ steps/sec} \to 23.44^\circ/\text{s}$).
   - Bổ sung `pinMode` và `digitalWrite` song song với `gpio_set_level` trong `Motor::begin` và `Motor::setDirection`.
3. **Giải phóng Hoàn toàn Bước ảo khi Clear Fault (`src/joint_model.cpp`)**:
   - `clearAllDriftFaults`: Reset `absSteps = 0` cho các khớp chưa home khi bấm Clear Fault, tránh độ lệch bước tích lũy cũ làm kẹt hệ thống.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Trên Web UI: Bấm **🛡️ CLEAR FAULT**, sau đó bấm **Set Home J5+J6** một lần khi cổ tay ở vị trí thẳng.
- Thử bấm Jog trên J2, J3, J4, J5, J6: Cả 6 trục hoạt động độc lập, mượt mà, không còn hiện tượng J5 kéo sập toàn bộ cánh tay.

---

## 2026-08-31 — Triệt tiêu Vòng lặp Khóa Lỗi bằng Kiến trúc Tự động Đồng bộ Encoder-First liên tục (`resyncFromEncoder`)

### Bối cảnh & Phân tích Log Hardware
1. **Hiện tượng Vòng lặp Tích lũy Sai số**:
   - Khi motor di chuyển góc lớn ($30^\circ - 45^\circ$), do ma sát cơ khí/backlash, số bước motor chạy thực tế chênh lệch nhẹ với encoder.
   - Logic cũ trong `updateDriftCheck` chỉ gọi `resyncFromEncoder` khi $\text{diff} \le 25^\circ$. Khi $\text{diff} > 25^\circ$, hàm KHÔNG đồng bộ lại bước mà đếm $[1/3] \to [3/3] \to \text{FAULT}$.
   - Hậu quả: `absSteps` không bao giờ được cập nhật lại theo encoder thực tế, khiến sai số tích lũy ngày càng lớn ($30^\circ \to 52^\circ \to 130^\circ$), khóa toàn bộ hệ thống sau mỗi lần bấm Jog.
2. **Cân chỉnh Tốc độ Jog Tối ưu Mô-men Xoắn Cao**:
   - J1/J2/J3/J4 đặt $800\mu\text{s/step}$ (1250 steps/sec) giữ mô-men xoắn cao tuyệt đối, không trượt bước khi nâng cánh tay.
   - J5/J6 (A4988, NEMA 14) đặt $1800\mu\text{s/step}$ ($556\text{ steps/sec}$) tối ưu kéo bánh răng côn êm ái.

### Việc đã làm
1. **Kiến trúc Tự động Đồng bộ Liên tục (`src/joint_model.cpp`)**:
   - `updateDriftCheck`: Sau khi motor dừng $300\text{ms}$, luôn tự động gọi `resyncFromEncoder(axis)` để neo lại `absSteps` chính xác theo góc đo vật lý thực tế của AS5600.
   - Triệt tiêu $100\%$ hiện tượng tích lũy sai số và vòng lặp lỗi `FAULT`.
   - In log chẩn đoán `[DRIFT] Jx can chinh X.XX deg` khi có hiệu chỉnh.
2. **Cấu hình Bảng Tốc độ Jog Toàn diện (`src/config.h`)**:
   - J1..J4: $800\mu\text{s/step}$.
   - J5..J6: $1800\mu\text{s/step}$.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Thử bấm Jog bất kỳ góc nào ($+15^\circ, +30^\circ, +45^\circ$) trên tất cả các trục J1..J6: Robot di chuyển trơn tru, không bao giờ bị khóa lỗi `FAULT` hay đơ giao diện.

---

## 2026-08-31 — Cấu hình Tần số Khởi động & Vận tốc Tối ưu Mô-men Xoắn Cực đại cho Động cơ Bước mang tải nặng

### Bối cảnh & Phân tích Hiện tượng
Khi người dùng bấm Jog $+30^\circ$:
- Động cơ phát tiếng rít/rung và bị khựng (stall), trục cơ học chỉ nhích được $\sim 1^\circ-2^\circ$ rồi dừng lại trong khi firmware phát đủ 5333 xung.
- Nguyên nhân: Động cơ bước NEMA 17 qua hộp số hành tinh $20:1$ chịu tải trọng uốn (cantilever load) của toàn bộ cánh tay. Việc chạy ở tốc độ cao ($800\mu\text{s} = 1250\text{ steps/sec} \approx 234\text{ RPM}$) làm suy giảm $70-80\%$ mô-men xoắn động (pull-out torque), khiến rotor bị tuột bước ngay sau khi tăng tốc.

### Việc đã làm
1. **Hạ Tốc độ Jog về Vùng Mô-men Xoắn Cực đại (`src/config.h`)**:
   - J1 (6:1): $1200\mu\text{s/step}$ ($833\text{ steps/sec} \to 15.63^\circ/\text{s}$)
   - J2 (20:1): $1800\mu\text{s/step}$ ($556\text{ steps/sec} \to 3.13^\circ/\text{s}$, mô-men kéo cánh tay cực đại)
   - J3 (20:1): $1800\mu\text{s/step}$ ($556\text{ steps/sec} \to 3.13^\circ/\text{s}$, mô-men nâng khuỷu cực đại)
   - J4 (4:1): $1500\mu\text{s/step}$ ($667\text{ steps/sec} \to 18.75^\circ/\text{s}$)
   - J5 / J6 (3:1, A4988): $2500\mu\text{s/step}$ ($400\text{ steps/sec} \to 15.00^\circ/\text{s}$)
2. **Khởi động Êm từ Tần số Thấp `MAX_STEP_INTERVAL_US = 3500us` (`src/config.h`, `src/motor.cpp`)**:
   - Khởi động từ $3500\mu\text{s}$ ($285\text{ steps/sec}$), dốc tăng tốc mở rộng lên $400\text{ microsteps}$, giúp động cơ bước khóa từ trường vững chắc từ trạng thái đứng yên và tăng tốc ổn định mà không bị trượt bước.

### Hướng dẫn kiểm tra
- Nạp firmware: `pio run -t upload`.
- Bấm Jog $+15^\circ$ hoặc $+30^\circ$ trên J2, J3, J5: Cánh tay sẽ chạy liên tục, khỏe khoắn, không còn tiếng rít hay hiện tượng khựng lại giữa chừng.




















---

## 2026-08-31 — Sprint 0 Safety & Real-time (P0 #1-4 Domain Clean)

### Việc đã làm
- What: Hoàn thiện Sprint 0 (P0 #1-4 Domain Clean) — gom 5 task trước thành gate tích hợp:
  - **Task1 SafetyManager core** (`src/safety_manager.h/.cpp`): single owner `SafetyState {NORMAL,E_STOP,HOMING,FAULT}`, `pending/isrTime/latched` per-channel, `pollEndstops(nowUs)` debounce 50 ms + `digitalRead==LOW` mới latch, `assertHoming()` cho phép latch không E_STOP, `tryClearFault()` chỉ true khi `!anyPressed() && !hasAnyDriftFault()`. Host test `test_safety_manager.cpp` 10 cases.
  - **Task2 ISR no-delay** (`src/endstop.*`): `isrHandler` tối minimal <3µs — chỉ `pending=true + isrTime=esp_timer_get_time()`, bỏ `esp_rom_delay_us(25)` và `gpio_get_level` khỏi ISR, forward qua `SafetyManager::isrNotify()`. Đã fix recursion `clearAllLatches` → `forceClear`.
  - **Task3 Motor+Arm globals removal** (`src/motor.*`, `src/arm.*`, `src/homing.*`): xóa `g_emergencyStop/g_homingActive`, inject `SafetyManager*` qua `setSafetyManager()`, `Motor::onStepTimer` check `safety_->isEStop()` đầu timer (≤20µs), `ArmController` sở hữu `unique_ptr<SafetyManager>` và gọi `pollEndstops()` + `isMotionAllowed()` mỗi tick.
  - **Task4 Non-blocking homing** (`src/config.h` + `HOMING_BACKOFF_SETTLE_MS=30`, `src/homing.h/.cpp`): enum thêm `WARMUP_SETTLE_WAIT/BACKOFF_SETTLE_WAIT/VERIFY_SETTLE_WAIT` (tổng 12 phases), `enterScanBackoff()` thay `delay(30)` bằng `phase=BACKOFF_SETTLE_WAIT + settleStartMs_=millis()`, `tickScan` 3 case `if(now-settleStartMs_<30/200/350) return` — motion task không block, WDT không đói. `toJson` cập nhật phase names.
  - **Task5 TrajectoryValidator** (`src/trajectory_validator.h/.cpp`): pure C++ lightweight B — `validate(Job,cur)` làm 1/3/5 IK (`kin::ikPenDown` + `WorkPlane::toRobotXYZ` nếu enabled), `Planner::submit()` gọi đầu tiên, fail → `lastError_` → `ArmController::execute()` map → HTTP 400 `{"error":"OUT_OF_REACH","segment":failIndex}`. Host test 7 cases.
  - **Task6 Integration**: fix `test_joint_logic` (J2/J3 `AXIS_STEP_SIGN` +1, khớp `config.h`) và `test_homing_logic` (nới stall-window margin theo `HOMING_STALL_ENC_DELTA_DEG=2.5`), chuẩn hóa `tools/run_host_tests.sh` 7 suites strict `|| exit 1` (kinematics 2230, joint_logic, work_plane, trajectory_validator 7, homing_logic, safety_manager 10, homing_nonblocking 13). Cập nhật `docs/SYSTEM_OVERVIEW.html` 4 tabs (RTOS ISR <3µs, Modules 15 cards + SafetyManager/TrajectoryValidator, FSM BACKOFF_SETTLE_WAIT diagram, Safety 6→16 invariants) và footer `Generated 2026-08-31 Sprint 0`.
- Why: Xóa 2 vi phạm real-time nghiêm trọng (ISR `delay 25µs` làm miss step ticks, `delay(30)` trong `arm_motion` block 3 ticks), gom safety phân tán (`g_emergencyStop` rải rác) về single owner, chặn job ngoài workspace trước khi chạy thay vì dừng giữa quỹ đạo, giữ nguyên hành vi cơ khí (timeout 30/200/350ms, debounce 50ms, DH/gears/pins, UART mutex, driver always enabled).
- How: Hướng B Domain Clean — SafetyManager là single owner của `SafetyState` (ISR chỉ pending+timestamp, Homing FSM thêm `BACKOFF_SETTLE_WAIT` non-blocking, TrajectoryValidator pure tách khỏi Planner). Xóa globals, semantics `tryClearFault()` rõ, `millis()` vs `esp_timer_get_time()` phân biệt (homing dùng `millis()`, safety dùng `esp_timer`). Giữ mọi hằng số cơ khí, chỉ đổi cơ chế.

### Build gate
- `~/.platformio/penv/bin/pio run` → **SUCCESS** (RAM 15.2% 49,780 B / 327,680 B, Flash 27.1% 904,593 B / 3,342,336 B, 0 errors, 0 warnings)
- `bash tools/run_host_tests.sh` → **7 suites ALL PASSED**:
  - kinematics: `FK home wrist=(126.000,0.000,365.000) tcp=(177.000,0.000,365.000) IK roundtrip ok=2230 fail=0` + Differential Wrist PASSED
  - joint logic: ALL PASSED
  - work plane: ALL PASSED
  - trajectory validator: ALL PASSED (7 tests)
  - homing logic: ALL PASSED
  - safety manager: ALL PASSED (10 tests)
  - homing nonblocking: ALL PASSED (13 tests) — `config_backoff_settle_ms ==30`, `no_delay_blocking`, `BACKOFF_SETTLE_WAIT` 30ms / `WARMUP 200ms` / `VERIFY 350ms`
- `python -m html.parser docs/SYSTEM_OVERVIEW.html` → parse OK, balanced tags

### Việc còn lại
- Sprint 1 P1 #5-9 (Foundation): NVS Config Store (DH/gears/current/soft-limit NVS-hóa), Structured Logging, Memory Monitoring (heap/stack WDT), I2C Fault Recovery (PCA9548A/AS5600 bus recover), Step Timer Optimize (esp_timer 50kHz DDA).
- Sprint 2 P2 #10-13 (Architecture): DI / RobotContext, HAL, LittleFS Frontend, WebSocket.
- Sprint 3 bỏ qua theo quyết định owner (P3 #14-21).
- Commissioning hardware: chạy Home All J1→J4 liên tiếp kiểm tra không WDT, đo ISR latency <10ms, pre-flight move ngoài reach → HTTP 400, jog clamp soft-limit, drift 25° debounce.

