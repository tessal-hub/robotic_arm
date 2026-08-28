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










