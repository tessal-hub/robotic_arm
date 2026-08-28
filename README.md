# NEMA-6AXIS Robotic Arm Controller

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32--S3-orange.svg)](https://platformio.org/)
[![Framework](https://img.shields.io/badge/Framework-Arduino%20%2B%20FreeRTOS-blue.svg)](https://docs.espressif.com/projects/arduino-esp32/)
[![Language](https://img.shields.io/badge/Language-C%2B%2B17-green.svg)](https://isocpp.org/)
[![Kinematics](https://img.shields.io/badge/Kinematics-Craig%20Modified%20DH-brightgreen.svg)](docs/ARM_GEOMETRY.md)
[![License](https://img.shields.io/badge/License-Proprietary-red.svg)](#license)

Firmware cấp production và bộ công cụ digital twin 3D cho cánh tay robot 6 bậc tự do (6-DOF) dùng bút vẽ đồng trục, chạy trên vi điều khiển **ESP32-S3 DevKitC-1** dual-core.

---

## 📑 Mục lục

- Tổng quan & kiến trúc
- Tính năng chính
- Thông số phần cứng & pinout đầy đủ
- Động học & hệ tọa độ
- Hướng dẫn bắt đầu nhanh
- Trình mô phỏng Digital Twin
- Web interface & REST API
- Cấu trúc repository
- Quy trình commissioning phần cứng
- Tài liệu tham chiếu kỹ thuật

---

## 🔭 Tổng quan & kiến trúc

Repository này chứa firmware thời gian thực, web studio nhúng và mô phỏng Python 3D để điều khiển chính xác từng khớp, chạy quỹ đạo Cartesian và vẽ trên mặt phẳng nghiêng.

```
                  ┌─────────────────────────────────────────────────────────┐
                  │                 ESP32-S3 Dual-Core SoC                  │
                  ├────────────────────────────┬────────────────────────────┤
                  │     Core 0 (Sensors)       │      Core 1 (Motion)       │
                  │                            │                            │
                  │  ┌──────────────────────┐  │  ┌──────────────────────┐  │
                  │  │ SensorScanTask       │  │  │ ArmMotionTask        │  │
                  │  │ - 200 Hz Rate        │  │  │ - 100 Hz Rate        │  │
                  │  │ - PCA9548A I2C Mux   │  │  │ - FSM & Arbiter      │  │
                  │  │ - 6x AS5600 Encoders │  │  │ - Trajectory Planner │  │
                  │  │ - Real-time Tracking │  │  │ - Closed-Form IK     │  │
                  │  └──────────┬───────────┘  │  └──────────┬───────────┘  │
                  └─────────────┼────────────────────────────┼──────────────┘
                                │                            │ Lock-Free SPSC
                                │ Differential Angle         ▼ (Align 64B)
                                │ Pipeline       ┌──────────────────────┐
                                └───────────────►│ Step Timer ISR       │
                                                 │ - 50 kHz Hardware    │
                                                 │ - Q32.32 DDA Engine  │
                                                 │ - Fail-Fast E-Stop   │
                                                 └──────────┬───────────┘
                                                            ▼
                                                [ 6x Stepper Motors ]
```

### Pipeline thực thi chính
1. **Core 0 — Sensor Task (200 Hz)**: Poll 6× encoder từ AS5600 qua mux PCA9548A, cập nhật góc tích lũy và chạy pipeline bảo vệ drift theo thời gian thực.
2. **Core 1 — Motion Task (100 Hz) & Web Server**: Xử lý queue lệnh (`JOG`, `MOVE_CART`, `DRAW_LINE`, `DRAW_CIRCLE`), giải động học Craig Modified DH, chiếu WorkPlane Gram-Schmidt và tạo motion block.
3. **Hardware Timer ISR (50 kHz)**: SPSC ring buffer lock-free + phát xung DDA fixed-point Q32.32, phản hồi E-Stop phần cứng dưới 20 µs.

---

## ✨ Tính năng chính

- **Động học Craig Modified DH**:
  - Forward Kinematics (FK) bằng chuỗi ma trận biến đổi 4×4.
  - Inverse Kinematics (IK) closed-form cho tư thế bút thẳng đứng ($\theta_4 = 0, \theta_6 = 0$).
  - Bảo vệ biên workspace, vùng chết bên trong và singularity.
- **Hiệu chuẩn WorkPlane 3 điểm**:
  - Tạo hệ tọa độ trực chuẩn $(\vec{u}, \vec{v}, \vec{n})$ từ 3 điểm thực bằng Gram-Schmidt.
  - Vẽ quỹ đạo 2D (line, circle, custom shape) trên mặt bàn nghiêng, giá vẽ ($35^\circ$) hoặc tường đứng mà không cần sửa code điều khiển.
- **Engine stepper 50 kHz tính quyết định**:
  - Ring buffer lock-free căn hàng cacheline 64B giữa task FreeRTOS và timer ISR.
  - Tích phân Q32.32 fixed-point, không dùng floating-point trong ISR.
  - Snap bước mục tiêu để triệt tiêu sai số rời rạc tích lũy.
- **An toàn closed-loop hai pipeline**:
  - **Pipeline A (online 200 Hz)**: E-Stop ngay khi sai số tức thời $|\Delta\theta| > 3.0^\circ$.
  - **Pipeline B (sau dừng 150 ms)**: Lọc trung bình 4 mẫu để phân biệt kẹt motor thật với nhiễu sensor/backlash.
- **Web studio nhúng**:
  - SPA tự chứa 100% trong PROGMEM ESP32-S3 (không phụ thuộc CDN ngoài).
  - Canvas preview 3D thời gian thực tính bằng JavaScript kinematics ở client.
  - Tối ưu viewport mobile (`pointer: coarse`, safe area) và tương phản đạt WCAG AA.
- **Hiệu chuẩn lưu bền NVS**:
  - Lưu thông tin WiFi, dấu hướng homing đã đo, tỉ lệ steps-per-degree và zero encoder tuyệt đối trong partition `arm-cfg`.

---

## 🔌 Thông số phần cứng & pinout đầy đủ

### 1) Bảng pinout chính (ESP32-S3 DevKitC-1 N8)

| GPIO Pin | Chức năng / Net | Linh kiện kết nối | Loại tín hiệu / Logic | Ghi chú phần cứng |
| :---: | :--- | :--- | :--- | :--- |
| **GPIO 1** | `STEP_J1` | Motor 1 (J1 Base Yaw) TMC2209 | Digital Output (3.3V Pulse) | Xung DDA từ timer 50 kHz |
| **GPIO 2** | `STEP_J2` | Motor 2 (J2 Shoulder) TMC2209 | Digital Output (3.3V Pulse) | Xung DDA từ timer 50 kHz |
| **GPIO 41** | `STEP_J3` | Motor 3 (J3 Elbow) TMC2209 | Digital Output (3.3V Pulse) | Xung DDA từ timer 50 kHz |
| **GPIO 42** | `STEP_J4` | Motor 4 (J4 Wrist Pan) TMC2209 | Digital Output (3.3V Pulse) | Xung DDA từ timer 50 kHz |
| **GPIO 38** | `STEP_J5` | Motor 5 (J5 Wrist Tilt - Left) A4988 | Digital Output (3.3V Pulse) | Xung DDA từ timer 50 kHz |
| **GPIO 39** | `DIR_J5` | Motor 5 (J5 Wrist Tilt - Left) A4988 | Digital Output (3.3V Level) | Điều khiển hướng CW/CCW |
| **GPIO 40** | `STEP_J6` | Motor 6 (J6 Tool Roll - Right) A4988 | Digital Output (3.3V Pulse) | Xung DDA từ timer 50 kHz |
| **GPIO 47** | `DIR_J6` | Motor 6 (J6 Tool Roll - Right) A4988 | Digital Output (3.3V Level) | Điều khiển hướng CW/CCW |
| **GPIO 15** | `UART1_RX` | TMC2209 J1–J4 Shared Bus | Serial Input (115200 Baud) | Nối trực tiếp bus PDN single-wire |
| **GPIO 16** | `UART1_TX` | TMC2209 J1–J4 Shared Bus | Serial Output (115200 Baud) | Nối qua điện trở nối tiếp $1\,\text{k}\Omega$ tới bus PDN |
| **GPIO 8** | `I2C_SDA` | PCA9548A 8-Ch I2C Mux | Open-Drain Bidirectional | $800\,\text{kHz}$ Fast-Mode+, pull-up $2.2\,\text{k}\Omega$ lên 3.3V |
| **GPIO 9** | `I2C_SCL` | PCA9548A 8-Ch I2C Mux | Open-Drain Output Clock | $800\,\text{kHz}$ Fast-Mode+, pull-up $2.2\,\text{k}\Omega$ lên 3.3V |
| **GPIO 5** | `LIMIT_J1_MIN` | Công tắc hành trình J1 Min | Digital Input (`INPUT_PULLUP`) | Active LOW, ISR debounce 50 ms |
| **GPIO 6** | `LIMIT_J1_MAX` | Công tắc hành trình J1 Max | Digital Input (`INPUT_PULLUP`) | Active LOW, ISR debounce 50 ms |
| **GPIO 7** | `LIMIT_J2_MIN` | Công tắc hành trình J2 Min | Digital Input (`INPUT_PULLUP`) | Active LOW, ISR debounce 50 ms |
| **GPIO 10** | `LIMIT_J2_MAX` | Công tắc hành trình J2 Max | Digital Input (`INPUT_PULLUP`) | Active LOW, ISR debounce 50 ms |
| **GPIO 11** | `LIMIT_J3_MIN` | Công tắc hành trình J3 Min | Digital Input (`INPUT_PULLUP`) | Active LOW, ISR debounce 50 ms |
| **GPIO 12** | `LIMIT_J3_MAX` | Công tắc hành trình J3 Max | Digital Input (`INPUT_PULLUP`) | Active LOW, ISR debounce 50 ms |
| **GPIO 19, 20** | `USB_D-`, `USB_D+` | Native USB OTG Port | USB Differential Data | Flash/debug/CDC serial monitor |
| **GPIO 13, 14, 17, 18, 48** | `SPARE_GPIO` | Chưa dùng / Header mở rộng | Tri-state / Floating | Dành cho phụ kiện tương lai (gripper, laser...) |

---

### 2) Motor, driver và thông số chấp hành

| Trục khớp | Vai trò động học | Khung motor | Driver IC | Chế độ điều khiển | Địa chỉ / chân cứng | Tỉ số truyền | Steps / Degree |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **J1** | Base Yaw | NEMA 17 | TMC2209 | UART1 + StealthChop / SpreadCycle | Addr `0b00` (MS1=GND, MS2=GND) | **6:1** | 53.33 |
| **J2** | Shoulder Pitch | NEMA 17 | TMC2209 | UART1 + StealthChop / SpreadCycle | Addr `0b01` (MS1=VIO, MS2=GND) | **20:1** | 177.78 |
| **J3** | Elbow Pitch | NEMA 17 | TMC2209 | UART1 + StealthChop / SpreadCycle | Addr `0b10` (MS1=GND, MS2=VIO) | **20:1** | 177.78 |
| **J4** | Wrist Pan | NEMA 17 | TMC2209 | UART1 + StealthChop / SpreadCycle | Addr `0b11` (MS1=VIO, MS2=VIO) | **4:1** | 35.56 |
| **J5** | Wrist Tilt (Carrier) | NEMA 14 | A4988 | STEP + DIR (1/16 Microstepping) | STEP: GPIO 38, DIR: GPIO 39 | **3:1** | 26.67 |
| **J6** | Tool Roll (Spider) | NEMA 14 | A4988 | STEP + DIR (1/16 Microstepping) | STEP: GPIO 40, DIR: GPIO 47 | **3:1** | 26.67 |

> [!NOTE]
> - **TMC2209 Single-Wire UART**: J1–J4 dùng chung một UART trên `GPIO 15` (RX) và `GPIO 16` (TX). Đảo chiều được xử lý qua thanh ghi `shaft()` nên không cần 4 dây DIR riêng.
> - **Driver Enable Line**: Tất cả chân `EN` của driver nối thẳng GND (luôn bật). Firmware không thả cuộn motor trong runtime để tránh back-drive do trọng lực.

---

### 3) Topology bus I2C encoder từ tính và mux

```
                  ┌────────────────────────────────────────────────────────┐
                  │                 ESP32-S3 (I2C Master)                  │
                  │         GPIO 8 (SDA) ────┬──── GPIO 9 (SCL)            │
                  └──────────────────────────┼─────────────────────────────┘
                                             │ 800 kHz Fast-Mode+ (2.2k Pullups)
                                             ▼
                  ┌────────────────────────────────────────────────────────┐
                  │          PCA9548A 8-Channel I2C Multiplexer            │
                  │                 (I2C Address: 0x70)                    │
                  ├────────┬────────┬────────┬────────┬────────┬───────────┤
                  │  Ch 0  │  Ch 1  │  Ch 2  │  Ch 3  │  Ch 4  │   Ch 5    │
                  └───┬────┴───┬────┴───┬────┴───┬────┴───┬────┴─────┬─────┘
                      │        │        │        │        │          │
                      ▼        ▼        ▼        ▼        ▼          ▼
                    AS5600   AS5600   AS5600   AS5600   AS5600     AS5600
                    (0x36)   (0x36)   (0x36)   (0x36)   (0x36)     (0x36)
                    Joint 1  Joint 2  Joint 3  Joint 4  Joint 5    Joint 6
                   (Base Yaw)(Shoulder)(Elbow) (Wrist) (Tilt/Left)(Roll/Right)
```

| Kênh mux | Cảm biến đích | Trục khớp giám sát | Khe hở nam châm | Tần số sensor task |
| :---: | :--- | :--- | :--- | :--- |
| **Channel 0** | AS5600 12-bit (`0x36`) | Joint 1 (Base Yaw) | $1.0\text{ mm} - 2.0\text{ mm}$ diametric | 200 Hz (Core 0) |
| **Channel 1** | AS5600 12-bit (`0x36`) | Joint 2 (Shoulder Pitch) | $1.0\text{ mm} - 2.0\text{ mm}$ diametric | 200 Hz (Core 0) |
| **Channel 2** | AS5600 12-bit (`0x36`) | Joint 3 (Elbow Pitch) | $1.0\text{ mm} - 2.0\text{ mm}$ diametric | 200 Hz (Core 0) |
| **Channel 3** | AS5600 12-bit (`0x36`) | Joint 4 (Wrist Pan) | $1.0\text{ mm} - 2.0\text{ mm}$ diametric | 200 Hz (Core 0) |
| **Channel 4** | AS5600 12-bit (`0x36`) | Joint 5 (Wrist Tilt / Left Side Gear) | $1.0\text{ mm} - 2.0\text{ mm}$ diametric | 200 Hz (Core 0) |
| **Channel 5** | AS5600 12-bit (`0x36`) | Joint 6 (Tool Roll / Right Side Gear) | $1.0\text{ mm} - 2.0\text{ mm}$ diametric | 200 Hz (Core 0) |

---

### 4) Endstop và kiến trúc homing

| Trục khớp | Endstop | GPIO | Loại microswitch | Giao tiếp / điện |
| :---: | :--- | :---: | :--- | :--- |
| **J1** | `J1_MIN` | **GPIO 5** | Roller Lever Microswitch | NO xuống GND, pull-up nội, Active LOW |
| **J1** | `J1_MAX` | **GPIO 6** | Roller Lever Microswitch | NO xuống GND, pull-up nội, Active LOW |
| **J2** | `J2_MIN` | **GPIO 7** | Roller Lever Microswitch | NO xuống GND, pull-up nội, Active LOW |
| **J2** | `J2_MAX` | **GPIO 10** | Roller Lever Microswitch | NO xuống GND, pull-up nội, Active LOW |
| **J3** | `J3_MIN` | **GPIO 11** | Roller Lever Microswitch | NO xuống GND, pull-up nội, Active LOW |
| **J3** | `J3_MAX` | **GPIO 12** | Roller Lever Microswitch | NO xuống GND, pull-up nội, Active LOW |
| **J4** | Sensorless StallGuard | — | Tích hợp trong TMC2209 | Đọc tải SG_RESULT qua UART (`STALL_SG_LEVEL = 100`) |
| **J5 / J6** | Differential Zero đồng trục | — | AS5600 từ tính | Tách góc 2-DOF từ 2 encoder $E_L$ và $E_R$ |

---

### 5) Invariant an toàn & chân cấm ESP32-S3

> [!CAUTION]
> **Các chân cấm / dành riêng trên ESP32-S3**:
> - **Strapping Pins (không nối ngoại vi)**: `GPIO 0` (boot mode), `GPIO 3` (JTAG source), `GPIO 45` (VDD_SPI), `GPIO 46` (tắt ROM log).
> - **Flash & PSRAM nội bộ**: `GPIO 26` đến `GPIO 37` gắn trực tiếp với flash QD 8MB. Cắm tín hiệu ngoài vào các chân này có thể khóa bus silicon ngay lập tức.
> - **Xung đột bootloader**: `GPIO 4` có thể gây bootloop trên ESP32-S3 nên bị loại bỏ hoàn toàn.
> - **Native USB OTG**: `GPIO 19` (D-) và `GPIO 20` (D+) dành cho debug, nạp firmware và serial telemetry.

---

## 📐 Động học & hệ tọa độ

Hình học robot dùng quy ước **Craig Modified Denavit-Hartenberg (MDH)**:
$$^{i-1}T_i = R_x(\alpha_{i-1}) \cdot T_x(a_{i-1}) \cdot R_z(\theta_i) \cdot T_z(d_i)$$

```
  Base (J1) ──[D1=139]──► Shoulder (J2) ──[A2=138]──► Elbow (J3)
                                                        │
                                                 [A3=88, D4=126]
                                                        ▼
  Pen TCP ◄──[D_tool=20]── Tool Roll (J6) ◄── Wrist Center (J4/J5)
```

### Bảng tham số Craig Modified DH

| Frame $i$ | Khớp | Link Length $a_{i-1}$ | Link Twist $\alpha_{i-1}$ | Link Offset $d_i$ | Theta Offset $\theta_{\text{offset}}$ | Dải hoạt động |
| :---: | :--- | :---: | :---: | :---: | :---: | :---: |
| **1** | J1 (Base Yaw) | $0\,\text{mm}$ | $0^\circ$ | **$139\,\text{mm}$** | $0^\circ$ | $[-90^\circ, +90^\circ]$ |
| **2** | J2 (Shoulder) | $0\,\text{mm}$ | $-90^\circ$ | $0\,\text{mm}$ | **$-90^\circ$** | $[-90^\circ, +90^\circ]$ |
| **3** | J3 (Elbow) | **$138\,\text{mm}$** | $0^\circ$ | $0\,\text{mm}$ | $0^\circ$ | $[0^\circ, +90^\circ]$ |
| **4** | J4 (Wrist Pan) | **$88\,\text{mm}$** | $-90^\circ$ | **$126\,\text{mm}$** | $0^\circ$ | $[-180^\circ, +180^\circ]$ |
| **5** | J5 (Wrist Tilt) | $0\,\text{mm}$ | $+90^\circ$ | $0\,\text{mm}$ | $0^\circ$ | $[-120^\circ, +120^\circ]$ |
| **6** | J6 (Tool Roll) | $0\,\text{mm}$ | $-90^\circ$ | **$31\,\text{mm}$** | $0^\circ$ | $[-360^\circ, +360^\circ]$ |
| **Tool**| Bút vẽ | $0\,\text{mm}$ | $0^\circ$ | **$20\,\text{mm}$** | $0^\circ$ | Trục dụng cụ cố định |

- **Pose home $(0^\circ, 0^\circ, 0^\circ, 0^\circ, 0^\circ, 0^\circ)$**:
  - Wrist Center (J5): $(X = 126.0\,\text{mm}, Y = 0.0\,\text{mm}, Z = 365.0\,\text{mm})$
  - Gốc J6: $(X = 157.0\,\text{mm}, Y = 0.0\,\text{mm}, Z = 365.0\,\text{mm})$
  - Đầu bút TCP: $(X = 177.0\,\text{mm}, Y = 0.0\,\text{mm}, Z = 365.0\,\text{mm})$
- **Chiều dài tool hiệu dụng (J5 $\to$ TCP)**: $D_{\text{tool\_eff}} = 31\,\text{mm} + 20\,\text{mm} = \mathbf{51\,\text{mm}}$
- **Tầm với phẳng tối đa (đến J5)**: $R_{\text{max}} = A_2 + \sqrt{A_3^2 + D_4^2} = 138.0 + 153.69 = 291.69\,\text{mm}$
- **Vùng chết trong**: $R_{\text{min}} = |A_2 - \sqrt{A_3^2 + D_4^2}| = |138.0 - 153.69| = 15.69\,\text{mm}$

---

## 🚀 Hướng dẫn bắt đầu nhanh

### Yêu cầu
- [PlatformIO Core (CLI)](https://docs.platformio.org/page/core/index.html) hoặc [PlatformIO IDE cho VSCode](https://platformio.org/install/ide?install=vscode)
- Python 3.10+ (cho host test và Digital Twin)
- Git

### 1) Clone & build firmware
```bash
# Clone repository
git clone https://github.com/<your-username>/robotic_arm.git
cd robotic_arm

# Biên dịch firmware
pio run
```

### 2) Nạp vào ESP32-S3
Kết nối ESP32-S3 DevKitC-1 qua cổng USB OTG / UART:
```bash
# Upload firmware
pio run -t upload

# Mở serial monitor (115200 baud)
pio device monitor
```

### 3) Chạy bộ test kinematics trên host
Dùng để xác minh engine động học với hàng nghìn tọa độ:
```bash
# Chạy unit test bằng host g++
bash tools/run_kin_tests.sh
```

---

## 💻 Trình mô phỏng Digital Twin

Dự án có sẵn trình mô phỏng desktop độc lập tại [`digital_clone.py`](digital_clone.py), triển khai bằng Matplotlib và NumPy.

```bash
# Chạy giao diện kỹ thuật 3D/2D tương tác
py -3 digital_clone.py

# Chạy audit tự động kinematics, WorkPlane và trajectory
py -3 digital_clone.py --test
```

### Tính năng mô phỏng
- **Bố cục 3 viewport kỹ thuật**: 3D Perspective (xoay/zoom), 2D Side Elevation ($X-Z$), 2D Top Plan ($X-Y$).
- **Click-to-Move**: Click trực tiếp lên viewport 2D Top hoặc Side để gửi lệnh di chuyển Cartesian.
- **WorkPlane Calibrator**: Hiển thị bề mặt nghiêng 3D và chiếu đường Line/Circle/Spiral/Star lên hệ đã hiệu chuẩn.
- **WiFi Bridge thời gian thực**:
  - `[📡 Sync ESP32]`: Poll `/api/status` từ robot thật và phản chiếu pose ngay trên mô phỏng.
  - `[⚡ Send to Robot]`: Gửi vị trí mục tiêu trực tiếp tới `/api/move` trên controller.
- **Velocity & Stepper Rate Profiler**: Kiểm tra vận tốc góc và tần số xung step có nằm trong giới hạn timer 50 kHz hay không.

---

## 🌐 Web interface & REST API

Khi khởi động, controller sẽ kết nối WiFi đã provisioned (STA mode). Nếu thất bại, hệ thống tự chuyển sang AP mode:
- **Fallback AP SSID**: `6AXIS-CONTROLLER`
- **Fallback AP Password**: `12345678`
- **mDNS Hostname**: `http://robot-arm.local` (hoặc IP in trên Serial)

### REST API (Port 80)

| Method | Endpoint | Payload / Query | Mô tả |
| :--- | :--- | :--- | :--- |
| `GET` | `/` | — | Trang web SPA nhúng |
| `GET` | `/api/status` | — | Snapshot telemetry JSON đầy đủ (mode, joints, pose, homing, safety) |
| `POST`| `/api/jog` | `{"axis": 0..5, "deg": float}` | Jog góc tương đối một khớp |
| `POST`| `/api/move` | `{"x": float, "y": float, "z": float, "feed": float}` | Di chuyển điểm Cartesian tuyến tính |
| `POST`| `/api/draw` | `{"shape": "line"\|"circle", ...}` | Chạy quỹ đạo vẽ line/circle phối hợp đa trục |
| `GET` | `/api/home/all`| — | Homing tự động tuần tự (J1 $\to$ J4) |
| `GET` | `/api/home/axis`| `?axis=0..3` | Homing một trục chỉ định |
| `GET` | `/api/sethome` | `?axis=0..5` | Đặt zero tại vị trí hiện tại và lưu NVS |
| `GET` | `/api/clearcalib`|`?axis=0..5` | Xóa zero đã lưu khỏi NVS |
| `POST`| `/api/workplane/calib`|`{"p1": [...], "p2": [...], "p3": [...]}` | Hiệu chuẩn WorkPlane nghiêng từ 3 điểm |
| `POST`| `/api/workplane/toggle`|`{"enable": true\|false}` | Bật/tắt biến đổi hệ tọa độ WorkPlane |
| `GET` | `/api/stop` | — | **Emergency Stop**: dừng toàn bộ xung motor ngay lập tức |
| `POST`| `/api/wifi` | `{"ssid": "...", "pass": "..."}` | Lưu WiFi credentials và restart controller |

---

## 📁 Cấu trúc repository

```
.
├── platformio.ini              # Cấu hình build PlatformIO & môi trường ESP32-S3
├── digital_clone.py            # Trình mô phỏng desktop 3D/2D độ trung thực cao
├── src/
│   ├── config.h                # Single source of truth cho pin, hình học DH và limits
│   ├── main.cpp                # Khởi tạo hệ thống, task FreeRTOS và vòng lặp web
│   ├── arm.h / .cpp            # Motion arbiter, command queue và task core 1
│   ├── motor.h / .cpp          # Bộ phát xung DDA 50kHz, UART TMC2209 và SPSC queue
│   ├── kinematics.h / .cpp     # FK ma trận Craig Modified DH và IK closed-form
│   ├── work_plane.h / .cpp     # Engine biến đổi tọa độ Gram-Schmidt từ 3 điểm
│   ├── planner.h / .cpp        # Đồng bộ segment đa trục (Line, Circle, Pen lift)
│   ├── sensor.h / .cpp         # AS5600 ×6 qua PCA9548A I2C mux (task 200Hz Core 0)
│   ├── homing.h / .cpp         # FSM homing nhiều pha tuần tự (J1–J4)
│   ├── joint_model.h / .cpp    # Đổi degree-step, lưu NVS và soft limits
│   ├── endstop.h / .cpp        # ISR + debounce phần cứng cho endstop
│   ├── web_server.h / .cpp     # SPA PROGMEM nhúng và REST API handlers
│   ├── spsc_queue.h            # SPSC ring buffer lock-free căn hàng cacheline 64-byte
│   ├── motion_block.h          # Định nghĩa segment chuyển động fixed-point Q32.32
│   ├── nvs_store.h / .cpp      # Wrapper Preferences ESP-IDF cho lưu trữ bền
│   ├── wifi_manager.h / .cpp   # Supervisor mạng STA / AP fallback tự động
│   └── rtos_guard.h            # RAII mutex guard cho FreeRTOS
├── test/
│   └── kinematics/             # Unit test động học phân tích (host g++)
├── tools/
│   └── run_kin_tests.sh        # Script chạy test trên host
└── docs/
    ├── ARM_GEOMETRY.md         # Đặc tả toán học DH matrices và động học
    ├── SYSTEM_OVERVIEW.html    # Bản đồ hệ thống tương tác (hardware, tasks, FSM, safety)
    └── IMPLEMENTATION_LOG.md   # Nhật ký kỹ thuật dạng append-only theo thời gian
```

---

## 🛠 Quy trình commissioning phần cứng

Khi bring-up cánh tay robot thật lần đầu:

1. **Địa chỉ driver**: Cấu hình jumper MS1/MS2 của TMC2209 (J1=`0b00`, J2=`0b01`, J3=`0b10`, J4=`0b11`). Kiểm tra log serial có `[TMC2209 OK] Ver 0x21` cho cả 4 driver.
2. **Chỉnh dòng**: Đo/chỉnh Vref trên module A4988 J5/J6 theo dòng coil định mức của motor.
3. **Kiểm tra jog open-loop**: Jog từng trục bước nhỏ ($+1.0^\circ$), xác nhận đúng chiều vật lý; nếu ngược thì đảo `AXIS_STEP_SIGN` trong [`src/config.h`](src/config.h).
4. **Zero encoder**: Xác nhận nam châm AS5600 đồng tâm đúng (air gap 1–2 mm).
5. **Hiệu chuẩn homing**: Chạy `/api/home/all`, kiểm tra J1/J2 về center và J3 backoff đúng.
6. **Test mặt phẳng nghiêng**: Hiệu chuẩn WorkPlane bằng 3 điểm góc của bề mặt vẽ, sau đó chạy `/api/draw` để xác nhận bám mặt chính xác.

---

## 📚 Tài liệu & tham chiếu kỹ thuật

- **[System Architecture & Interactive Map](docs/SYSTEM_OVERVIEW.html)**: Bản đồ đầy đủ task FreeRTOS, wiring phần cứng và safety interlock.
- **[Mathematical Kinematics Specification](docs/ARM_GEOMETRY.md)**: Suy luận FK, phương trình IK closed-form và hệ tọa độ.
- **[Engineering Implementation Log](docs/IMPLEMENTATION_LOG.md)**: Nhật ký append-only cho quyết định thiết kế, bug fix và kết quả test hardware.
- **[Agent Maintenance Contract](AGENTS.md)**: Quy tắc và bất biến dành cho agent khi bảo trì repo.

---

## 📄 License

Repository này phục vụ nghiên cứu và phát triển nội bộ. Mọi quyền được bảo lưu.
