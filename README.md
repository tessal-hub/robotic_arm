# Bộ điều khiển cánh tay robot 6 trục

Firmware cho **ESP32-S3 DevKitC-1**, điều khiển sáu khớp J1–J6 và một servo **MG90 làm tay gắp**. Robot được vận hành bằng Web UI do ESP32 phục vụ trực tiếp, không cần cài ứng dụng riêng.

Dự án dùng **PlatformIO · Arduino · FreeRTOS · C++17**. Có mô phỏng 3D trên Web và Digital Twin Python chạy offline.

## Bắt đầu từ đâu?

| Bạn cần… | Xem phần… |
|---|---|
| Build, nạp firmware và kết nối robot | [Chạy lần đầu](#chạy-lần-đầu) |
| Jog, Home, Teach hoặc vẽ | [Vận hành trên Web](#vận-hành-trên-web) |
| Đấu nối driver, encoder và tay gắp | [Phần cứng và GPIO](#phần-cứng-và-gpio) |
| Chạy mô phỏng hoặc kiểm thử | [Mô phỏng và kiểm thử](#mô-phỏng-và-kiểm-thử) |
| Tìm module, API hoặc hình học chi tiết | [Tài liệu và mã nguồn](#tài-liệu-và-mã-nguồn) |

> Build và host tests đã pass ở lượt cập nhật servo ngày 22/09/2026. Servo MG90 chưa được flash/kiểm chứng trên robot thật; giao diện mới chưa được xác nhận render. Kết quả phần mềm không thay thế kiểm tra phần cứng.

## Chạy lần đầu

### 1. Build firmware

Cài PlatformIO Core hoặc extension PlatformIO trong VS Code. Mở terminal tại thư mục chứa `platformio.ini`, rồi chạy:

```sh
pio run
```

Trên Windows, nếu `pio` chưa có trong PATH, dùng PowerShell:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
```

Cấu hình board và thư viện nằm trong [platformio.ini](platformio.ini). Firmware hiện dùng partition cho flash 8 MB.

### 2. Nạp và xem log

Kiểm tra dây, nguồn, khoảng trống chuyển động và đỡ cánh tay trước khi nạp. Driver có thể có lực giữ sau khi khởi tạo.

```sh
pio run -t upload
pio device monitor
```

Serial monitor dùng **115200 baud**. Nếu máy có nhiều cổng, chỉ định `--upload-port COMx` khi nạp và `--port COMx` khi monitor.

### 3. Mở Web UI

Firmware thử kết nối WiFi đã lưu trong NVS; nếu chưa có hoặc kết nối thất bại, nó phát Access Point:

| Mục | Mặc định |
|---|---|
| Tên WiFi AP | `6AXIS-CONTROLLER` |
| Mật khẩu AP | `12345678` |
| Địa chỉ Web | IP in trong log serial; hoặc `http://robot-arm.local` nếu mDNS hoạt động |

Kết nối máy tính/điện thoại vào cùng mạng với robot rồi mở địa chỉ trên. Nhập WiFi của bạn qua phần WiFi trong Web UI; thông tin được lưu vào NVS và ESP32 khởi động lại. Không đưa SSID/mật khẩu mạng riêng vào source.

## Vận hành trên Web

| Thao tác | Cách dùng và lưu ý |
|---|---|
| **Jog** | Di chuyển từng khớp theo bước góc. Khi kiểm tra lần đầu, bắt đầu bằng bước nhỏ và quan sát đúng chiều. |
| **Home J1–J4** | Homing tự động: J1–J3 dùng endstop, J4 dùng StallGuard kết hợp encoder. |
| **Set Home J5/J6** | Đặt mốc tại vị trí hiện tại cho hai khớp cổ tay độc lập; thao tác này thay đổi tham chiếu vị trí. |
| **Release / Enable J1–J4** | Nhả/bật lại mô-men bốn TMC2209, giữ dữ liệu home/NVS. Release không nhả động cơ A4988 J5/J6. |
| **Teach A/B/C** | Đưa arm tới từng pose → Save A/B/C → Play A–B–C. Save cần đủ sáu khớp đã home, encoder khỏe và robot đứng yên. Đổi mốc home hoặc dấu encoder thì phải Teach lại. |
| **Cartesian / Quick Draw** | Di chuyển TCP hoặc vẽ Line, Circle, Square, HELLO theo cấu hình bên dưới. |
| **Gripper MG90** | Trong tab Jog, nhập góc rồi bấm **Set angle**. Chỉ nhận lệnh khi arm đứng yên, không Release/FAULT và PWM sẵn sàng. |
| **STOP ALL** | Hủy chuyển động, xóa lệnh chờ và ngắt xung servo tay gắp. |
| **Clear Fault** | Acknowledge lỗi khi robot đứng yên và endstop đã nhả; không bỏ qua nguyên nhân phần cứng. |

### Vẽ bằng bút

- Mặt phẳng Draw cố định tại **base Z = 20 mm**. Quick Draw giữ các ô **Start X / Start Y** để chọn điểm đầu.
- Cartesian/Draw điều khiển **J1–J5**; **J6 giữ nguyên**. Cần các khớp liên quan đã home và encoder J5 sẵn sàng.
- WorkPlane vẫn dùng được cho **Cartesian POINT**. Draw bị từ chối khi WorkPlane đang bật.
- Thử đường đi khi bút chưa chạm giấy trước khi vẽ thật.

Hình học bút nằm trong [ARM_GEOMETRY.md](docs/ARM_GEOMETRY.md). Thêm servo tay gắp không tự thay đổi TCP hoặc mô hình IK của bút.

### Tay gắp MG90

| Kết nối/cấu hình | Giá trị hiện tại |
|---|---|
| Chân signal | **GPIO13** |
| PWM | LEDC channel 0 · 50 Hz · 14-bit |
| Dải góc lệnh mặc định | 0–180° → xung 1000–2000 µs |
| Khi boot | Không phát xung servo |
| Nguồn | Nguồn servo riêng phù hợp model, nối chung GND với ESP32; không cấp nguồn servo từ GPIO/3V3 |

Các hằng số `GRIPPER_*` nằm trong [src/config.h](src/config.h). Thử không tải quanh góc lệnh 90° trước, rồi hiệu chỉnh dải xung/góc theo cơ cấu thật; dải lệnh không bảo đảm bằng hành trình cơ khí.

**STOP ALL, Release và FAULT ngắt xung servo, có thể làm mất lực giữ.** Web chỉ hiển thị góc đã ra lệnh, không có phản hồi góc thật hoặc xác nhận servo đã tới đích. Teach A/B/C hiện chỉ lưu J1–J6, chưa lưu tay gắp.

## Phần cứng và GPIO

Bảng dưới đối chiếu với [src/config.h](src/config.h). J1–J4 đảo chiều bằng UART `shaft()`, không dùng chân DIR; J5 và J6 là hai khớp độc lập.

| Khớp | Driver | STEP | DIR / địa chỉ UART | Tỷ số truyền |
|---|---|---:|---|---:|
| J1 — đế | TMC2209 | 1 | UART `0b00` | 6:1 |
| J2 — vai | TMC2209 | 2 | UART `0b01` | 20:1 |
| J3 — khuỷu | TMC2209 | 41 | UART `0b10` | 20:1 |
| J4 — cổ tay | TMC2209 | 42 | UART `0b11` | 4:1 |
| J5 — gập cổ tay | A4988 | 38 | GPIO39 | 3:1 |
| J6 — xoay công cụ | A4988 | 40 | GPIO47 | 1:1 |

| Kết nối khác | GPIO / cấu hình |
|---|---|
| UART1 chung J1–J4 | RX **15**, TX **16** · 115200 baud |
| I2C encoder | SDA **8**, SCL **9** · cấu hình hiện tại **40 kHz** |
| PCA9548A → AS5600 ×6 | Mux `0x70`; kênh 0–5 tương ứng J1–J6, mỗi sensor `0x36` |
| Endstop J1 | MIN **6**, MAX **5** |
| Endstop J2 | MIN **7**, MAX **10** |
| Endstop J3 | MIN **11**, MAX **12** |
| Servo MG90 | Signal **13** |
| Dự phòng theo pinout dự án | **14, 17, 18, 48** |

Endstop dùng `INPUT_PULLUP`, active LOW. Dự án không dùng chân EN; Release J1–J4 thực hiện qua UART của TMC2209. Xem các chân bị loại trừ và quy tắc đấu nối trong [config.h](src/config.h) và [bản đồ hệ thống](docs/SYSTEM_OVERVIEW.html) trước khi đổi GPIO.

Trước khi chạy robot thật: xác nhận UART từng driver, dòng/Vref, chiều Jog, encoder và endstop theo [checklist phần cứng](docs/HW_REGRESSION_CHECKLIST.md).

## Mô phỏng và kiểm thử

### Digital Twin offline

Cần Python có Tkinter:

```sh
python tools/digital_twin.py
python tools/digital_twin.py --self-test
```

Twin giúp xem pose và đường vẽ mà không điều khiển phần cứng. Web UI cũng có phần mô phỏng/preview 3D.

### Host tests

Cần `g++` và Bash; trên Windows có thể dùng Git Bash với `g++` trong PATH:

```sh
bash tools/run_host_tests.sh
```

Suite kiểm tra kinematics, calibration/joint logic, WorkPlane, trajectory validation, homing, safety, input validation và regression firmware. `tools/run_kin_tests.sh` hiện chuyển tiếp sang suite này.

Không dùng `pio test -e native` cho repo này; dùng script host ở trên. Khi sửa firmware, chạy thêm `pio run`.

### Công cụ kiểm tra driver riêng

- [A4988 J5/J6](tools/a4988_dual_test/README.md).
- [TMC2209 địa chỉ 0](tools/uart_loopback/README.md): thư mục vẫn mang tên `uart_loopback`, nhưng chương trình hiện tại là test driver TMC2209.

Đọc README của từng công cụ trước khi nạp; nạp lại firmware chính sau khi test.

## Tài liệu và mã nguồn

| Tài liệu | Nội dung |
|---|---|
| [SYSTEM_OVERVIEW.html](docs/SYSTEM_OVERVIEW.html) | Bản đồ hệ thống: mở bằng browser để xem hardware, RTOS, modules, FSM và REST API. |
| [ARM_GEOMETRY.md](docs/ARM_GEOMETRY.md) | Nguồn chuẩn hình học, kích thước và quy ước động học. |
| [HW_REGRESSION_CHECKLIST.md](docs/HW_REGRESSION_CHECKLIST.md) | Các bước xác nhận trên robot thật. |
| [IMPLEMENTATION_LOG.md](docs/IMPLEMENTATION_LOG.md) | Lịch sử thay đổi và kết quả kiểm chứng từng lượt. |
| [AGENTS.md](AGENTS.md) | Quy định bảo trì code, build/test và cập nhật tài liệu. |

Luồng điều khiển chính:

```text
Web UI → REST API → ArmCommand queue → motion task → motor / planner / servo
AS5600 → sensor task → JointModel → homing, vị trí và telemetry
Endstop → ISR dừng trục → SafetyManager debounce và latch lỗi
```

Sensor task đặt chu kỳ **20 ms** trên core 0; motion task **10 ms** trên core 1. Motor phát STEP bằng `esp_timer`. Drift watchdog hiện đã tắt theo cấu hình hành vi của dự án; encoder vẫn phục vụ homing, vị trí, resync và Teach.

```text
src/config.h          Pin, tỷ số truyền, giới hạn, timing, servo, mạng
src/arm.*             Command queue, điều phối motion, Teach, servo
src/motor.*           STEP, hướng quay, driver TMC2209/A4988
src/sensor.*          AS5600 qua PCA9548A
src/joint_model.*     Góc khớp, calibration, home và resync
src/homing.*          FSM homing
src/planner.*         Thực thi chuyển động Cartesian và đường vẽ
src/kinematics.*      FK / IK thuần C++
src/web_server.*      Web UI nhúng và REST API
src/nvs_store.*       Dữ liệu WiFi, home, calibration, Teach
src/safety_manager.*  Trạng thái an toàn và fault latch
src/endstop.*         GPIO và ISR endstop
test/                 Host tests và mock phần cứng
tools/                Test runner, Digital Twin, test driver
docs/                 Tài liệu chi tiết và lịch sử
```

## Giấy phép

Dự án phục vụ nghiên cứu và phát triển nội bộ. Bảo lưu mọi quyền.
