# Kế Hoạch Triển Khai: Tối Đa Hóa Năng Lực ESP32-S3 Cho Cánh Tay Robot 6 Trục

- **Tài liệu thiết kế gốc**: `docs/superpowers/specs/2026-08-28-esp32-s3-power-maximization-design.md`
- **Mã dự án**: NEMA-6AXIS-ARM-CONTROLLER (ESP32-S3 DevKitC-1)
- **Ngày tạo**: 2026-08-28
- **Trạng thái**: SẴN SÀNG THỰC THI (READY FOR EXECUTION)

---

## 1. MỤC TIÊU & CHIẾN LƯỢC TRIỂN KHAI

Kế hoạch này phân rã toàn bộ đặc tả thiết kế kỹ thuật thành các nhiệm vụ nguyên tử (atomic tasks), có thể kiểm thử độc lập (testable), đảm bảo không làm gián đoạn các tính năng đang hoạt động và tuân thủ tuyệt đối quy ước bảo trì trong `AGENTS.md`.

---

## 2. DANH SÁCH NHIỆM VỤ THEO GIAI ĐOẠN

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                             TIẾN ĐỘ 4 GIAI ĐOẠN                             │
├─────────────────────────────────────────────────────────────────────────────┤
│ Giai đoạn 1: Master Step Engine 50kHz, SPSC Buffer Epoch & Fail-Fast ISR    │
│ Giai đoạn 2: Chuỗi An Toàn 2 Pipeline & Phân Loại 5 Cấp Độ + I2C Detach     │
│ Giai đoạn 3: S-Curve 7 Pha, 2-Pass Look-Ahead & Bù Gia Tốc Jacobian         │
│ Giai đoạn 4: Work Plane UCS 3 Điểm & WebSocket Binary Telemetry Studio 50Hz │
│ Giai đoạn 5: Đo Kiểm Chu Kỳ ISR, Test Kinematics & Đồng Bộ Tài Liệu Sống    │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

### 🔹 GIAI ĐOẠN 1: Master Step Engine 50kHz, SPSC Ring-Buffer Epoch & Direct Fail-Fast ISR

#### Task 1.1: Xây dựng cấu trúc SPSC Lock-Free Ring-Buffer với Epoch Invalidation
- **Tập tin**: Tạo mới `src/spsc_queue.h`
- **Nội dung**:
  - Triển khai template `SPSCQueue<T, Capacity>` với `Capacity = 64`.
  - Căn lề cache-line `alignas(64)` cho các biến atomic: `m_head` (chỉ Producer ghi), `m_flushEpoch` (chỉ Producer ghi), `m_tail` (chỉ Consumer ghi).
  - Triển khai phương thức `invalidate()` dùng `m_flushEpoch.fetch_add(1, release)`.
  - Triển khai phương thức `pop()` trong ISR tự động kiểm tra `m_localEpoch != m_flushEpoch` để tự nhảy `m_tail` bằng `m_head` (bảo toàn Single-Writer).
- **Tiêu chí nghiệm thu**: Unit test lock-free xác nhận không có race condition khi producer invalidate đúng lúc consumer đang pop.

#### Task 1.2: Định nghĩa Cấu trúc Motion Block Bậc 2 Fixed-Point Q32.32
- **Tập tin**: Tạo mới `src/motion_block.h`
- **Nội dung**:
  - Định nghĩa `struct MotionBlock` với các trường: `totalTicks`, `targetAbsSteps[6]`, `ddaStepFraction[6]` (Q32.32), `ddaStepAccel[6]`, `ddaStepJerk[6]`, `dirMask`, `isLastSegment`.
  - Hỗ trợ hàm khởi tạo và tính toán `ddaStepFraction` ban đầu đảm bảo tính liên tục vận tốc $C^1$ giữa các block liền kề.
- **Tiêu chí nghiệm thu**: Struct có kích thước cố định, căn lề bộ nhớ 64-bit, tương thích hoàn toàn với IRAM.

#### Task 1.3: Nâng cấp Step Generator lên GPTimer 50kHz & State-Machine Pulse trong IRAM
- **Tập tin**: Chỉnh sửa `src/motor.h`, `src/motor.cpp`, `src/config.h`
- **Nội dung**:
  - Thay thế ngắt `esp_timer` bằng Hardware GPTimer chạy ở tần số cơ sở $f_{\text{base}} = 50\,\text{kHz}$ ($T = 20\,\mu\text{s}$).
  - Triển khai State-Machine 2-Tick (Tick 1: HIGH qua `GPIO.out_w1ts`, Tick 2: LOW qua `GPIO.out_w1tc`).
  - Tích hợp cờ toàn cục `std::atomic<bool> g_emergencyStop{false}` kiểm tra ngay đầu ISR ($\le 20\,\mu\text{s}$ fail-fast).
  - Tích hợp cơ chế **Snap-to-Target**: tại `tick == totalTicks`, gán chính xác `absSteps[i] = targetAbsSteps[i]` và reset phần dư `accumulator[i] = 0`.
  - Đánh dấu toàn bộ hàm ISR và hàm liên quan thuộc tính `IRAM_ATTR`.
- **Tiêu chí nghiệm thu**: Xung bước 6 trục phát song song chính xác ở tần số tối đa 25kHz, không có jitter, ngắt xung tức thì khi `g_emergencyStop = true`.

---

### 🔹 GIAI ĐOẠN 2: Chuỗi An Toàn 2 Pipeline & Phân Loại 5 Cấp Độ + I2C Detach Recovery

#### Task 2.1: Triển khai Pipeline A (Online Compensation Loop @ 200Hz)
- **Tập tin**: Chỉnh sửa `src/sensor.cpp`, `src/joint_model.cpp`, `src/joint_model.h`
- **Nội dung**:
  - Trong `sensorTask` (Core 0, Priority 5):
    - Đọc 6 encoder AS5600 qua PCA9548A với timeout $300\,\mu\text{s}$ mỗi trục.
    - Tính toán độ lệch tức thời $|\Delta\theta_{\text{instant}}| = |\theta_{\text{step}} - \theta_{\text{enc}}|$.
    - Nếu $|\Delta\theta_{\text{instant}}| \in (1.5^\circ, 3.0^\circ] \implies$ Gửi vector bù vi sai $\Delta\vec{q}$ sang `MotionTask` để cộng vào block kế tiếp.
    - Nếu $|\Delta\theta_{\text{instant}}| > 3.0^\circ \implies$ Kích hoạt ngay `g_emergencyStop.store(true, release)`.
  - Gọi `esp_task_wdt_reset()` mỗi chu kỳ 5ms để duy trì an toàn Watchdog.
- **Tiêu chí nghiệm thu**: Phản ứng ngắt xung $\le 20\,\mu\text{s}$ khi có trượt tải đột ngột $> 3.0^\circ$.

#### Task 2.2: Triển khai Pipeline B (Post-E-Stop Settle & 5-Tier Classification)
- **Tập tin**: Chỉnh sửa `src/arm.cpp`, `src/arm.h`, `src/nvs_store.h`, `src/nvs_store.cpp`
- **Nội dung**:
  - Khi `g_emergencyStop == true`, `MotionTask` (Core 1) kích hoạt `invalidate()` trên SPSC queue và chuyển trạng thái sang `FAULT_LATCHED`.
  - Chờ $t_{\text{settle}} = 150\,\text{ms}$ cho dao động cơ khí tắt hẳn, sau đó lấy trung bình 4 mẫu liên tiếp $\bar{\theta}_{\text{settle}}$.
  - Kiểm tra biến thiên góc thô encoder trước hộp số $\Delta\theta_{\text{raw}}$:
    - **Cấp 1** ($|\Delta\theta_{\text{settle}}| \le 3.0^\circ$): Nhiễu rung tức thời $\to$ Tự động resync `absSteps = encDeg`, sẵn sàng clear fault.
    - **Cấp 3** ($3.0^\circ < |\Delta\theta_{\text{settle}}| \le 6.0^\circ$): Trượt tải vừa $\to$ Tự động resync `absSteps = encDeg`, yêu cầu lệnh Clear Fault từ Web.
    - **Cấp 4** ($|\Delta\theta_{\text{settle}}| > 6.0^\circ$ và $\Delta\theta_{\text{raw}} \le 90^\circ$): Trượt hộp số cycloidal $\to$ CẤM auto-resync, khóa `HARD_FAULT_GEARBOX_SLIP`, tăng `wear_counter` trong NVS, yêu cầu chạy lại Homing.
    - **Cấp 5** ($\Delta\theta_{\text{raw}} > 90^\circ$): Nhảy góc thô/tuột dây đai $\to$ CẤM auto-resync, khóa `HARD_FAULT_UNWRAP_LOST`, yêu cầu kiểm tra vật lý và Home lại.
- **Tiêu chí nghiệm thu**: Phân loại chính xác 5 cấp lỗi, bảo toàn mốc tọa độ và không bao giờ tự động resync khi có dấu hiệu trượt hộp số hoặc unwrap sai vòng.

#### Task 2.3: Triển khai Bit-Bang I2C Detach Recovery
- **Tập tin**: Chỉnh sửa `src/sensor.cpp`
- **Nội dung**:
  - Viết lại hàm `recoverI2CBus()`:
    1. Gọi `i2c_driver_delete(I2C_NUM_0)` để detach chân GPIO khỏi I2C controller matrix.
    2. Chuyển SDA/SCL sang GPIO Open-Drain Output với Pull-up nội bộ.
    3. Bit-bang 9 xung clock SCL và phát tín hiệu STOP thủ công.
    4. Cài đặt lại `i2c_driver_install()` và kiểm tra kết nối PCA9548A.
- **Tiêu chí nghiệm thu**: Tự động giải phóng bus trong $< 2\,\text{ms}$ khi cố tình kéo chân SDA xuống GND.

---

### 🔹 GIAI ĐOẠN 3: S-Curve 7 Pha, 2-Pass Look-Ahead & Bù Gia Tốc Jacobian

#### Task 3.1: Bộ Lọc Gộp Vi Đoạn Hình Học (Micro-Segment Coalescing)
- **Tập tin**: Chỉnh sửa `src/planner.cpp`, `src/planner.h`
- **Nội dung**:
  - Triển khai hàm `coalesceSegments()` kiểm tra 2 điều kiện kép:
    1. Góc lệch tiếp tuyến: $\Delta\theta < 2.0^\circ$.
    2. Sai số độ võng vuông góc: $d_{\text{perp}} \le 0.05\,\text{mm}$.
  - Gộp các vi đoạn thỏa mãn thành 1 đoạn thẳng duy nhất trước khi đưa vào bộ đệm Look-Ahead.
- **Tiêu chí nghiệm thu**: Giảm $40-60\%$ số lượng block khi đọc G-code mật độ cao mà sai lệch nét vẽ không quá $0.05\,\text{mm}$.

#### Task 3.2: Bộ Đệm 2-Pass Look-Ahead Planner & Gia Tốc Động $a_{\text{effective}}$
- **Tập tin**: Chỉnh sửa `src/planner.cpp`, `src/kinematics.cpp`, `src/kinematics.h`
- **Nội dung**:
  - Triển khai hàm tính $a_{\text{effective}}(\vec{q}, \vec{u}_{\text{dir}})$ dựa trên ma trận nghịch đảo Jacobian $J^{-1}(\vec{q})$ và giới hạn gia tốc góc của từng trục.
  - Triển khai thuật toán 2-Pass Look-Ahead cho bộ đệm 32 đoạn:
    - **Forward Pass**: Tính toán $v_{\text{junction}}$ cho phép theo gia tốc từ điểm trước.
    - **Backward Pass**: Quét ngược từ block cuối về đầu để ép giảm entry-speed, đảm bảo đủ quãng đường phanh trước góc cua gắt.
- **Tiêu chí nghiệm thu**: Cánh tay bo góc mượt mà không bị dừng giật, tự động giảm tốc khi vào cua gắt hoặc khi tay vươn xa nằm ngang.

#### Task 3.3: Bộ Sinh S-Curve 7 Pha & Nạp Batch Đa Khối (Batch Refill)
- **Tập tin**: Chỉnh sửa `src/planner.cpp`, `src/arm.cpp`
- **Nội dung**:
  - Xây dựng thuật toán phân tích 7 pha Jerk-limited cho từng đoạn chuyển động.
  - Chuyển đổi các pha thành các `MotionBlock` bậc 2 (Q32.32 fixed point).
  - Trong `motionTask` (Core 1, 100Hz): Thực hiện vòng lặp nạp batch lấp đầy SPSC Queue (tối đa 16 blocks/lần gọi).
- **Tiêu chí nghiệm thu**: Chuyển động tăng/giảm tốc êm ái tuyệt đối, không có rung lắc cơ khí ở J2/J3, SPSC Queue không bao giờ bị cạn (underrun).

---

### 🔹 GIAI ĐOẠN 4: Work Plane UCS 3 Điểm & WebSocket Binary Telemetry Studio 50Hz

#### Task 4.1: Triển khai Lớp Mặt Phẳng Vẽ Tùy Biến (WorkPlane)
- **Tập tin**: Tạo mới `src/work_plane.h`, `src/work_plane.cpp`
- **Nội dung**:
  - Xây dựng lớp `WorkPlane` hỗ trợ phương thức `setThreePointCalibration(P1, P2, P3)`.
  - Tích hợp bộ kiểm tra suy biến hình học: Từ chối nếu khoảng cách $< 20\,\text{mm}$ hoặc góc mở vector $\sin\phi < 0.1736$ ($\phi < 10^\circ$).
  - Triển khai hàm chuyển đổi tọa độ 2D $(u, v, w) \to (x, y, z)_{\text{robot}}$.
- **Tiêu chí nghiệm thu**: Chuyển đổi tọa độ chính xác cho mọi mặt phẳng nghiêng, báo lỗi rõ ràng khi 3 điểm thẳng hàng.

#### Task 4.2: Tích hợp Work Plane vào Motion Planner & Closed-Form IK
- **Tập tin**: Chỉnh sửa `src/arm.cpp`, `src/planner.cpp`
- **Nội dung**:
  - Bổ sung lệnh `ArmCommand::SET_WORK_PLANE` và `ArmCommand::CLEAR_WORK_PLANE`.
  - Khi chế độ Work Plane kích hoạt, mọi lệnh `DRAW_LINE`, `DRAW_CIRCLE`, `MOVE_CART` tự động chiếu qua ma trận mặt phẳng trước khi giải IK.
- **Tiêu chí nghiệm thu**: Robot vẽ đường tròn/đường thẳng hoàn hảo trên bảng nghiêng bất kỳ với đầu bút luôn vuông góc với mặt phẳng.

#### Task 4.3: WebSocket Binary Stream Telemetry Server @ 50Hz
- **Tập tin**: Chỉnh sửa `src/web_server.cpp`, `src/web_server.h`
- **Nội dung**:
  - Tích hợp WebSocket Server nhị phân trên Core 0 @ 50Hz.
  - Đóng gói packed struct 64 bytes (`ArmTelemetryPacket`) gửi đều đặn mỗi 20ms:
    `[absSteps: 24B | encDeg: 24B | tcpXYZ: 12B | flags: 4B]`.
  - Hỗ trợ nhận gói nhị phân điều khiển Jog/Move từ Web với độ trễ $< 8\,\text{ms}$.
  - Bảo vệ Flash Write Isolation: Chặn 100% việc ghi LittleFS/NVS khi `isMoving() == true`.
- **Tiêu chí nghiệm thu**: Giao diện Web hiển thị telemetry mượt mà 50fps, độ trễ phản hồi tức thì, không làm ảnh hưởng tới ngắt bước Core 1.

#### Task 4.4: Nâng Cấp Web Studio: Canvas 3D Work Plane & Gamepad Jog
- **Tập tin**: Chỉnh sửa `src/web_server.cpp` (INDEX_HTML PROGMEM)
- **Nội dung**:
  - Thêm giao diện hiệu chuẩn 3 điểm (Point 1, Point 2, Point 3 Jog & Touch) trên tab Draw/Plan.
  - Hiển thị trực quan mặt phẳng làm việc nghiêng (Work Plane Grid) trong Canvas 3D Digital Clone.
  - Kết nối WebSocket nhị phân mượt mà với thanh trượt và hỗ trợ Gamepad API.
- **Tiêu chí nghiệm thu**: Người dùng có thể chạm 3 điểm trên phôi nghiêng, xem trực quan mặt phẳng 3D và ra lệnh vẽ trực tiếp từ trình duyệt.

---

### 🔹 GIAI ĐOẠN 5: Đo Kiểm Chu Kỳ ISR, Test Kinematics & Đồng Bộ Tài Liệu Sống

#### Task 5.1: Đo Kiểm Chu Kỳ CPU Thực Tế của Step ISR
- **Nội dung**:
  - Chèn macro `esp_cpu_get_cycle_count()` trước và sau khối xử lý Step ISR.
  - Đo thời gian thực thi xấu nhất (worst-case execution cycles) khi cả 6 trục cùng phát xung và tích phân Q32.32.
  - Ghi nhận số liệu chính xác vào tài liệu hệ thống.
- **Tiêu chí nghiệm thu**: Thời gian thực thi ISR $\le 1.2\,\mu\text{s}$ ($< 6\%$ CPU Core 1 @ 50kHz).

#### Task 5.2: Chạy Bộ Kiểm Thử Tự Động Kinematics & Work Plane
- **Lệnh thực thi**: `tools/run_kin_tests.sh` hoặc g++ unit tests.
- **Nội dung**: Xác nhận 100% test cases Forward Kinematics, Closed-Form IK, Work Plane Transformation và S-curve pass sạch sẽ.
- **Tiêu chí nghiệm thu**: ALL TESTS PASSED.

#### Task 5.3: Đồng Bộ Hóa Toàn Diện Tài Liệu Sống (AGENTS.md Contract)
- **Tập tin**: Cập nhật `docs/SYSTEM_OVERVIEW.html` và append `docs/IMPLEMENTATION_LOG.md`.
- **Nội dung**: Cập nhật sơ đồ Dual-Core, SPSC Buffer, Master GPTimer 50kHz, 2 Pipeline Safety, Work Plane UCS và API WebSocket.
- **Tiêu chí nghiệm thu**: Mở file HTML xem offline hiển thị đầy đủ, không có CDN ngoài; nhật ký ghi lại đầy đủ các quyết định thiết kế.

---

Kế hoạch triển khai đã sẵn sàng. Bạn có thể duyệt kế hoạch này để chúng ta bắt đầu thực thi từng task một cách tuần tự và an toàn!
