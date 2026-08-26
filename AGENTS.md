# AGENTS.md — Hướng dẫn AI Agent làm việc với repo này

> File này là hợp đồng bảo trì cho MỌI AI agent (opencode, Claude, Copilot...) làm việc trong repo.
> Đọc hết file này TRƯỚC khi sửa bất kỳ thứ gì.

## 1. Dự án là gì

Firmware **NEMA-6AXIS-ARM-CONTROLLER** — điều khiển cánh tay robot 6 trục (bút vẽ đồng trục J6)
trên ESP32-S3 DevKitC-1, PlatformIO + Arduino framework + FreeRTOS, C++17.

- J1–J4: TMC2209 chung UART1 (addr 0b00–0b11, đảo chiều qua `shaft()`, KHÔNG có chân DIR)
- J5–J6: A4988 STEP+DIR
- Encoder AS5600 ×6 qua mux PCA9548A (I2C)
- Web app nhúng: STA (creds NVS) → AP fallback, REST API + trang single-page PROGMEM

Bản đồ hệ thống đầy đủ (hardware, RTOS tasks, modules, FSM, kinematics, API):
**`docs/SYSTEM_OVERVIEW.html`** — mở bằng browser, đọc tab liên quan trước khi đụng code.

## 2. Lệnh build & test (gate bắt buộc)

```bash
pio run                    # build firmware — PHẢI SUCCESS trước khi tuyên bố xong
tools/run_kin_tests.sh     # host test kinematics — PHẢI ALL PASSED nếu đụng kinematics.*
```

- KHÔNG dùng `pio test -e native`: PIO Core 6.1.19 lỗi "Nothing to build" với env này —
  luôn chạy test kinematics qua script g++ trực tiếp.
- Monitor serial: `pio device monitor` @115200.

## 3. Giao thức bảo trì tài liệu — BẮT BUỘC cho MỖI thay đổi

Hai file dưới đây là **tài liệu sống**. Sau khi sửa code thành công (build/test pass), agent
PHẢI cập nhật cả hai trong cùng một lần làm việc:

### 3a. `docs/SYSTEM_OVERVIEW.html` — bản đồ hệ thống sống

Cập nhật mọi phần bị thay đổi bởi chỉnh sửa của bạn:

| Bạn đã sửa... | Thì cập nhật trong HTML... |
|---|---|
| `config.h` (pin, hằng số, limit, DH) | Tab "Phần cứng" / số liệu trong các tab liên quan |
| Task/RTOS/priority/period | Tab "RTOS & Tasks" |
| Module bất kỳ (API mới, behavior mới) | Card module tương ứng ở tab "Modules" |
| FSM homing/planner/arm mode | Sơ đồ tab "FSM & Luồng dữ liệu" |
| `kinematics.*` / ARM_GEOMETRY | Tab "Kinematics" |
| Endpoint web / validation | Bảng API tab "REST API" |
| Invariant an toàn / checklist commissioning | Tab "An toàn & TODO" |
| Module mới | Thêm card mới; module xoá → bỏ card |

Quy tắc:
- Giữ file **tự chứa hoàn toàn** (không CDN, không ảnh ngoài) — phải mở được offline.
- Ngôn ngữ: tiếng Việt, thuật ngữ kỹ thuật giữ tiếng Anh.
- Cập nhật ngày "Generated" ở footer khi có thay đổi nội dung.
- Chỉ ghi NHỮNG THỨ CODE THẬT LÀM — không viết tính năng tương lai như hiện hữu.

### 3b. `docs/IMPLEMENTATION_LOG.md` — nhật ký timeline

Append entry mới ở CUỐI file (không sửa/xoá entry cũ):

```markdown
---

## YYYY-MM-DD — <tiêu đề ngắn gọn>

### <Việc đã làm>
- What: thay đổi gì, ở file nào
- Why: lý do / yêu cầu của owner / bug nào
- How (nếu thiết kế có lựa chọn): phương án đã chọn và vì sao

### Build gate
- `pio run` → SUCCESS/FAIL + RAM/Flash nếu đáng chú ý
- `tools/run_kin_tests.sh` → kết quả (chỉ khi đụng kinematics)

### Việc còn lại (nếu có)
```

Nguyên tắc: nhật ký là lịch sử bất biến — sai cái gì sửa forward bằng entry mới, không rewrite lịch sử.
Nếu thay đổi chỉ là docs/tài liệu nhưng chạm quyết định thiết kế → vẫn log entry ngắn.

## 4. Quy tắc bất biến (vi phạm = lỗi nghiêm trọng)

1. **`docs/ARM_GEOMETRY.md` là single source of truth hình học.** KHÔNG đổi hằng số DH
   (`DH_D1_MM`, `DH_A2_MM`, `DH_D4_MM`, `DH_D6_TOOL_MM`, theta offsets) nếu thiếu phép đo
   thực nghiệm do owner cung cấp. Nếu đổi (có bằng chứng): phải đồng bộ cả 3 nơi —
   `config.h`, `kinematics.cpp`, test host — rồi log vào IMPLEMENTATION_LOG.
2. **KHÔNG tái nhập** `atan2(110,88)=51.34°` hay d4=110 — mô hình đúng là d4=126,
   δ=atan2(126,88)=55.06°. Chi tiết: IMPLEMENTATION_LOG mục "Kiểm chứng mô hình động học".
   Chỉ owner mới được phê duyệt đổi mô hình (qua thí nghiệm jog J3 +30°).
3. **WiFi credentials CHỈ qua NVS** (`wifi_manager.provision()` → trang `/api/wifi`).
   Không secrets.h, không hardcode SSID/pass thật vào source.
4. **Web handler không đụng hardware trực tiếp** — chỉ enqueue `ArmCommand` qua
   `ArmController::submit()`. Ngoại lệ đã duyệt: clearcalib (xoá data), wifi save (restart).
5. **Mọi giao dịch UART TMC2209 phải qua `g_uartMutex`** với timeout ngắn; fail → bỏ lệnh, không chờ vô hạn.
6. **Giữ nguyên chuỗi an toàn**: ISR endstop abort + latch 50 ms debounce, FAULT latch +
   CLEAR_FAULT, gate `allPositioningHomed()` cho Cartesian, clamp jog theo soft limit,
   STOP_ALL luôn được chấp nhận, planner stop khi điểm ngoài vùng với.
7. **Driver luôn enabled** — không thêm logic EN pin.
8. `kin::forward()`/`ikPenDown()` phải thuần C++ không phụ thuộc Arduino (host-testable).

## 5. Tri thức kỹ thuật cô đọng (rút từ `skill/`)

Tinh gọn từ `skill/embedded-systems`, `skill/embedded-agent-skills` (ESP32) và `skill/cpp-pro`
— các nguyên tắc áp dụng cho firmware này. Chi tiết đầy đủ nằm trong thư mục tương ứng.

### ISR & thời gian thực
- **ISR phải ngắn**: chỉ set flag/latch + dừng motor, xử lý nặng đẩy xuống task
  (`Endstops::isrHandler` là mẫu chuẩn của repo). Không blocking trong ISR
  (cấm UART/I2C/delay/mutex-take).
- Biến chia sẻ ISR↔task: `std::atomic` hoặc `volatile`; nếu cần signal từ ISR thì dùng API `*FromISR`.
- Task tuần hoàn dùng `vTaskDelayUntil()` (không trôi chu kỳ), không dùng `vTaskDelay()`.
- Task dài hạn đăng ký Task WDT (sensor task đã làm); nghi tràn stack thì check
  `uxTaskGetStackHighWaterMark()`.
- Mutex bus (UART/I2C) luôn take với timeout ngắn — fail thì bỏ lệnh. Chỉ `portMAX_DELAY`
  chấp nhận được lúc setup một luồng.

### ESP32-S3 GPIO (lý do đứng sau bảng cấm trong config.h)
- Strapping pins KHÔNG dùng: GPIO0 (boot mode), GPIO3 (JTAG source), GPIO45 (VDD_SPI),
  GPIO46 (boot/log ROM).
- Flash/PSRAM octal chiếm **GPIO26–32** — tuyệt đối không assign (crash ngay).
- USB OTG chiếm GPIO19/20.
- Thêm cảm biến analog sau này: **chỉ dùng ADC1 khi WiFi đang bật** (ADC2 xung đột phần cứng
  với RF calibration của WiFi — không fix bằng software được). Trên S3: ADC1 = GPIO1–10,
  ADC2 = GPIO11–20.
- Nút/endstop input luôn cần pull-up (nội bộ hoặc ngoài) — floating input đọc unreliable.

### FreeRTOS
- Thứ tự ưu tiên hiện tại (giữ nguyên trừ khi có số đo): ISR > esp_timer > sensor task
  (prio 4, core 0) > motion task (prio 3, core 1) > loop/web (core 1).
- Queue là kênh lệnh DUY NHẤT qua ranh giới task (`ArmCommand` queue) — không thêm biến
  global chưa bảo vệ.
- Tài nguyên chia sẻ dùng `xSemaphoreCreateMutex()` (có priority inheritance chống đảo ưu tiên),
  không dùng binary semaphore làm khóa.
- Hạn chế heap sau setup: object chính static allocation. `String` JSON trong `statusJson()`
  chấp nhận được vì polling thấp (~3 Hz) — đừng nhân bản pattern đó vào đường chạy 100 Hz.

### C++ (chuẩn cpp-pro)
- RAII cho mọi resource — `RtosLockGuard` (rtos_guard.h) là chuẩn của repo, không take/give tay
  khi return sớm.
- `std::atomic` với memory order đúng: `relaxed` cho counter/flag đơn; `release/acquire` khi
  publish trạng thái (tham khảo `Motor::running`, `absSteps`).
- Ownership đơn dùng `unique_ptr` (như `Motor::driver`); không thêm raw `new`/`delete`.
- Const-correct; `static_cast` thay C-style cast; `-Wall` sạch trước khi nói "xong".
- ESP32-S3 có FPU: dùng `float` thoải mái trong motion/kinematics (codebase dùng f-suffix:
  `cosf/sqrtf/fabsf`); tránh `double` trong hot path.

### Giao tiếp bus (I2C/UART)
- Mọi giao dịch đều có timeout; lỗi đọc liên tiếp quá ngưỡng ⇒ đánh dấu lỗi
  (`sensor_error`), không retry vô hạn.
- Bus treo ⇒ recovery chủ động: 9 xung SCL + STOP giả rồi re-init (`Sensor::recoverI2CBus()`
  là mẫu).
- Đọc register device: write địa chỉ reg với repeated-start
  (`Wire.endTransmission(false)`) rồi read — xem `Sensor::readRaw()`.

## 6. Quy trình làm việc chuẩn

1. **Đọc trước khi sửa**: AGENTS.md (file này) → tab liên quan trong SYSTEM_OVERVIEW.html →
   IMPLEMENTATION_LOG.md các entry gần nhất (bối cảnh quyết định cũ) → code.
2. **Sửa nhỏ, đúng chỗ**: đặt logic ở module sở hữu nó; không thêm layer/interface chỉ có 1 implementation.
3. **Gate**: `pio run` (+ kin tests nếu đụng kinematics) — pass rồi mới nói "xong".
4. **Cập nhật tài liệu**: theo giao thức mục 3 (cả 2 file).
5. **Không tự commit** trừ khi owner yêu cầu rõ ràng.

## 7. Cấu trúc repo

```
platformio.ini          env esp32-s3-devkitc-1, lib TMCStepper, C++17
src/config.h            MỌI hằng số: pin, gear, DH, limit, motion, network
src/main.cpp            wiring khởi tạo + loop (chỉ handleClient)
src/motor.*             stepper engine (esp_timer, S-curve, absSteps atomic)
src/sensor.*            AS5600×6 qua PCA9548A (task core0 @200Hz)
src/endstop.*           ISR endstop + latch an toàn
src/joint_model.*       deg<->step, home/NVS restore, drift watchdog
src/homing.*            FSM homing J1->J4 (endstop+stallguard fusion)
src/planner.*           Cartesian POINT/LINE/CIRCLE, segment sync 6 trục
src/kinematics.*        FK ma trận Modified-DH + IK closed-form pen-down
src/nvs_store.*         Preferences "arm-cfg": wifi creds + joint homes
src/wifi_manager.*      STA->AP fallback, provision()
src/web_server.*        PROGMEM SPA + REST API
src/arm.*               arbiter: command queue + motion task core1 @100Hz
src/rtos_guard.h        RAII mutex guard
test/kinematics/        host unit test (g++)
tools/run_kin_tests.sh  test runner
docs/ARM_GEOMETRY.md    SOURCE OF TRUTH hình học — đọc đầu tiên khi đụng động học
docs/IMPLEMENTATION_LOG.md  nhật ký thay đổi (append-only)
docs/SYSTEM_OVERVIEW.html   bản đồ hệ thống sống (mở bằng browser)
skill/                  thư viện skill tham khảo cho agent (embedded/cpp/electronics)
```
