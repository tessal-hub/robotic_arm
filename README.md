# NEMA-6AXIS-ARM-CONTROLLER

Firmware điều khiển cánh tay robot 6 trục (NEMA) trên **ESP32-S3 DevKitC-1** — PlatformIO + Arduino framework + FreeRTOS, C++17.

> **Mục tiêu**: Web app nhúng (STA/AP fallback) → Homing fusion (endstop + StallGuard) → Joint jog (FK) → IK vẽ line/circle trên giấy.

---

## 🎯 Tính năng chính

| Module | Mô tả |
|--------|-------|
| **Homing J1–J4** | FSM tuần tự J1→J2→J3→J4. Fusion: endstop latch **HOẶC** StallGuard (SG_RESULT < 100 trong 3 poll liên tiếp). J1/J2 home ở giữa hành trình (centering), J3/J4 tại min-stop + backoff 2°. |
| **Joint Control** | Jog tương đối 6 trục (bước 0.5°/1°/5°/15°), clamp theo soft limit, encoder AS5600 feedback real-time. |
| **Cartesian Move** | MOVE_CART (POINT/LINE/CIRCLE) — IK closed-form pen-down (bút thẳng đứng θ4=θ6=0). Segment sync 6 trục: trục chủ đạo đặt interval, các trục còn lại scale nghịch đảo để cùng kết thúc. |
| **Draw on Paper** | DRAW_LINE / DRAW_CIRCLE: LIFT (+5mm) → TRAVEL → DROP (z giấy) → DRAW (~1mm/segment) → FINISH_LIFT. Preview canvas top-view (mm). |
| **NVS Persistence** | WiFi credentials + home offset từng khớp (raw góc AS5600). Boot: restore vị trí từ encoder → **không cần home lại** nếu encoder còn nguyên. |
| **Web UI** | Single-page PROGMEM (dark theme), polling 300ms `/api/status`. Tabs: Dashboard · Joints · Homing · WiFi · Cartesian · Draw. E-STOP fixed bottom-right. ARIA + focus-visible + loading states + toast. |
| **Safety** | ISR endstop (50ms debounce) → dừng motor ngay + latch. FAULT latch + CLEAR_FAULT. Gate `allPositioningHomed()` cho Cartesian. Drift watchdog 5°. UART TMC mutex timeout ngắn. |

---

## 🛠 Phần cứng

| Thiết bị | Số lượng | Giao tiếp | Lưu ý |
|----------|----------|-----------|-------|
| **ESP32-S3 DevKitC-1** | 1 | — | Core 0: Sensor task (200Hz), Core 1: Motion task (100Hz) + WebServer |
| **TMC2209** | 4 (J1–J4) | UART1 (RX15/TX16) @115200 | Chung bus, địa chỉ 0b00–0b11 (jumper MS1/MS2). **Không DIR pin** — đảo chiều qua `shaft()` register. |
| **A4988** | 2 (J5, J6) | STEP+DIR | J5: STEP=38, DIR=39. J6: STEP=40, DIR=47. Vref cứng (trimpot). |
| **AS5600** | 6 | I2C qua PCA9548A (0x70) | SDA=8, SCL=9 @800kHz. Mux 2 kênh: J1–J3 / J4–J6. |
| **Endstop** | 6 (J1–J3 MIN/MAX) | GPIO | Active LOW, pull-up nội bộ. ISR FALLING. J1: 5/6, J2: 7/10, J3: 11/12. |
| **Stepper NEMA** | 6 | — | Tỷ số truyền: J1=6:1, J2=20:1, J3=20:1, J4=4:1, J5=3:1, J6=3:1. 1.8°/200 bước, 1/16 microstep. |

> ⚠️ **GPIO cấm**: 0, 3, 4, 19, 20, 26–32, 45, 46 (strapping/flash/USB). Xem `config.h` bảng đầy đủ.

---

## 📐 Hình học & Kinematics

- **Single Source of Truth**: `docs/ARM_GEOMETRY.md` — KHÔNG đổi hằng số DH nếu thiếu phép đo thực nghiệm.
- **Modified DH (Craig convention)**: `T_i = Rx(αᵢ₋₁) · Tx(aᵢ₋₁) · Rz(θᵢ) · Tz(dᵢ)`
- **Bảng DH chính xác** (đã đối chiếu vật lý):

| i | aᵢ₋₁ (mm) | αᵢ₋₁ (°) | dᵢ (mm) | θᵢ offset (°) | Mô tả |
|---|-----------|----------|---------|---------------|-------|
| 1 | 0 | 0 | **139** | 0 | Base yaw |
| 2 | 0 | −90 | 0 | **−90** | Shoulder pitch |
| 3 | 138 | 0 | 0 | 0 | Elbow pitch |
| 4 | 88 | −90 | **126** (16+110) | 0 | Wrist pan |
| 5 | 0 | +90 | 0 | 0 | Wrist tilt |
| 6 | 0 | −90 | 0 | 0 | Wrist roll (pen 20mm) |

- **FK**: Nhân ma trận 6 khớp + `Tz(20)` tool. Home: wrist center = (126, 0, 365), TCP = (146, 0, 365).
- **IK pen-down** (closed-form): Wrist center `C = TCP + (0,0,20)`, base yaw `t1 = atan2(Cy,Cx)`, mặt phẳng 2 khâu (A2=138, L_fore=153.69mm, δ=atan2(126,88)=55.06°), chọn nhánh J3 ∈ [0,90] có |J3| nhỏ nhất, wrist: θ4=0, θ5=−q23 (bút thẳng), θ6=0.
- **Test host**: 3280/3280 roundtrip IK→FK OK, sai số ≤ 0.5mm, từ chối đúng điểm ngoài workspace.

---

## 🏗 Cấu trúc repo

```
platformio.ini          env esp32-s3-devkitc-1, lib TMCStepper, C++17
src/config.h            MỌI hằng số: pin, gear, DH, limit, motion, network
src/main.cpp            Wiring khởi tạo + loop (chỉ handleClient)
src/motor.*             Stepper engine (esp_timer, S-curve, absSteps atomic)
src/sensor.*            AS5600×6 qua PCA9548A (task core0 @200Hz)
src/endstop.*           ISR endstop + latch an toàn
src/joint_model.*       deg<->step, home/NVS restore, drift watchdog
src/homing.*            FSM homing J1->J4 (endstop+stallguard fusion)
src/planner.*           Cartesian POINT/LINE/CIRCLE, segment sync 6 trục
src/kinematics.*        FK ma trận Modified-DH + IK closed-form pen-down
src/nvs_store.*         Preferences "arm-cfg": wifi creds + joint homes
src/wifi_manager.*      STA->AP fallback, provision()
src/web_server.*        PROGMEM SPA + REST API
src/arm.*               Arbiter: command queue + motion task core1 @100Hz
src/rtos_guard.h        RAII mutex guard
test/kinematics/        Host unit test (g++)
tools/run_kin_tests.sh  Test runner
docs/ARM_GEOMETRY.md    SOURCE OF TRUTH hình học — đọc đầu tiên khi đụng động học
docs/IMPLEMENTATION_LOG.md  Nhật ký thay đổi (append-only)
docs/SYSTEM_OVERVIEW.html   Bản đồ hệ thống sống (mở bằng browser)
AGENTS.md               Hợp đồng bảo trì cho AI agent
PRODUCT.md              Ngữ cảnh sản phẩm (register, users, brand, anti-refs)
DESIGN.md               Design system (colors, typo, elevation, components)
.impeccable/design.json Sidecar cho live panel
```

---

## 🚀 Build & Flash

```bash
# Build firmware
pio run

# Monitor serial (115200)
pio device monitor

# Flash
pio run -t upload
```

**Gate bắt buộc**: `pio run` → SUCCESS (RAM ~15%, Flash ~25%).

---

## 🧪 Test Kinematics (Host)

```bash
# Chạy test g++ trực tiếp (KHÔNG dùng pio test -e native — PIO Core 6.1.19 lỗi "Nothing to build")
tools/run_kin_tests.sh
# hoặc thủ công:
mkdir -p /tmp/opencode && \
g++ -std=gnu++17 -Wall -Wextra -I src src/kinematics.cpp test/kinematics/test_kinematics.cpp -o /tmp/opencode/kinematics_test && \
/tmp/opencode/kinematics_test
```

Kết quả kỳ vọng: `ALL KINEMATICS TESTS PASSED` (3280/3280 OK).

---

## 🌐 Web App & REST API

Sau khi nạp firmware, ESP32 sẽ:
1. Thử kết nối WiFi từ NVS (STA, timeout 6s)
2. Thất bại → AP mode: **SSID `6AXIS-CONTROLLER` / Pass `12345678`**
3. mDNS: `http://robot-arm.local` (hoặc IP in serial monitor)

### Trang chính
Mở browser → `/` → Single-page app (dark theme, 6 tabs).

### REST API (port 80)

| Method | Endpoint | Tham số | Mô tả |
|--------|----------|---------|-------|
| GET | `/` | — | Trang web chính |
| GET | `/api/status` | — | JSON tổng hợp (mode, joints, pose, wifi, homing, endstops, motors) |
| POST | `/api/jog` | `axis` (0–5), `deg` (≠0, ≤45°) | JOG_REL. Hoặc `fault_clear=1` → CLEAR_FAULT |
| GET | `/api/stop` | — | STOP_ALL — luôn chấp nhận (wait 50ms) |
| POST | `/api/move` | `x,y,z`, `feed` (default 30) | MOVE_CART (POINT). z ∈ [−20, 450]. Yêu cầu J1–J4 homed. |
| POST | `/api/draw` | `shape=line` → `x1,y1,x2,y2`<br>`shape=circle` → `cx,cy,r` (5–250)<br>`z`, `feed` | DRAW_LINE / DRAW_CIRCLE. Yêu cầu J1–J4 homed. |
| GET | `/api/home/all` | — | HOME_ALL chuỗi J1→J4 |
| GET | `/api/home/axis` | `axis` (0–3) | HOME_AXIS 1 khớp |
| GET | `/api/sethome` | `axis` (0–5) | SET_HOME tại chỗ + persist NVS |
| GET | `/api/clearcalib` | `axis` (0–5) | Xoá calib NVS (confirm trên UI) |
| POST | `/api/wifi` | `ssid` (≤32), `pass` (≤64) | Lưu NVS → delay 1s → ESP.restart() |

**Mã lỗi**: 400 bad args · 409 busy · 503 queue full · 500 not ready.

---

## 🔧 Commissioning Checklist (khi có hardware)

1. [ ] Jumper MS1/MS2: J1=00, J2=01, J3=10, J4=11 — log `[TMC2209 OK] Ver 0x21` ×4
2. [ ] Đo Vref A4988 J5/J6, ghi dòng coil vào `IMPLEMENTATION_LOG.md`
3. [ ] Jog mù từng trục — flip `AXIS_STEP_SIGN`/`AXIS_ENC_SIGN` nếu ngược
4. [ ] Tune `STALL_SG_LEVEL` per-joint (xem SG_RESULT qua `/api/status`)
5. [ ] Home All → jog → Move Cartesian → Draw circle thử
6. [ ] **Thí nghiệm phân biệt mô hình**: Home all → jog J3 +30° → đo displacement bút thực vs firmware pose. ≤2mm = Model A đúng. Ghi kết quả.
7. [ ] Xác nhận offset θ4/θ5/θ6 (đo giống θ2) — ảnh hưởng trực tiếp độ chính xác nét vẽ
8. [ ] Xác nhận encoder gắn 1:1 trên trục ra khớp (để restore NVS đúng)

---

## 📚 Tài liệu sống (bắt buộc đọc trước khi sửa)

| File | Mục đích |
|------|----------|
| `docs/SYSTEM_OVERVIEW.html` | Bản đồ hệ thống: hardware, RTOS tasks, 13 modules, FSM, kinematics, API, safety. Mở bằng browser. |
| `docs/IMPLEMENTATION_LOG.md` | Timeline bất biến. Append entry mới sau MỖI thay đổi (build/test pass). |
| `docs/ARM_GEOMETRY.md` | Single source of truth hình học cơ khí, DH table, FK/IK derivation. |
| `AGENTS.md` | Hợp đồng bảo trì AI agent: gate, quy tắc bất biến, workflow, cấu trúc repo. |
| `PRODUCT.md` | Ngữ cảnh sản phẩm: register=product, users, brand personality, anti-references. |
| `DESIGN.md` | Design system: tokens, typography, elevation, components, do's/don'ts. |

> **Quy tắc**: Sau khi sửa code thành công → **CẬP NHẬT CẢ `SYSTEM_OVERVIEW.html` VÀ `IMPLEMENTATION_LOG.md`** trong cùng một lần làm việc (xem AGENTS.md §3).

---

## 🧠 Tri thức kỹ thuật (tóm gọn từ `skill/`)

- **ISR**: Ngắn, chỉ set flag/latch + dừng motor. Không blocking (cấm UART/I2C/delay/mutex). Biến chia sẻ: `std::atomic`/`volatile`.
- **FreeRTOS**: Priority: ISR > esp_timer > Sensor task (prio 4, core 0) > Motion task (prio 3, core 1) > Loop/web (core 1). Queue là kênh lệnh DUY NHẤT. Mutex = `xSemaphoreCreateMutex()` (priority inheritance).
- **C++**: RAII (`RtosLockGuard`), `std::atomic` (relaxed cho counter, release/acquire cho publish), `unique_ptr`, const-correct, `float` với f-suffix (`cosf`, `sqrtf`), `-Wall` sạch.
- **Bus I2C/UART**: Timeout mọi giao dịch. Lỗi liên tiếp > ngưỡng → đánh dấu `sensor_error`. Bus treo → recovery 9 xung SCL + STOP giả + re-init.

---

## 📝 License

Internal project — không công khai.

---

## 📞 Liên hệ / Maintainer

- Owner: **Owner**
- AI Agent contract: `AGENTS.md`
- Implementation log: `docs/IMPLEMENTATION_LOG.md`