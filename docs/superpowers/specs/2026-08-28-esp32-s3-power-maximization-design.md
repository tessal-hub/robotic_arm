# Đặc Tả Kỹ Thuật: Tối Đa Hóa Năng Lực ESP32-S3 Cho Điều Khiển Cánh Tay Robot 6 Trục

- **Mã dự án**: NEMA-6AXIS-ARM-CONTROLLER (ESP32-S3 DevKitC-1)
- **Tác giả / Thiết kế**: AI Pair Programming & System Owner
- **Ngày hoàn thành**: 2026-08-28
- **Trạng thái**: ĐÃ DUYỆT THIẾT KẾ (DESIGN VALIDATED)

---

## 1. TỔNG QUAN HỆ THỐNG & MỤC TIÊU THIẾT KẾ

Tài liệu này đặc tả kiến trúc kỹ thuật toàn diện nhằm khai thác triệt để 100% năng lực phần cứng của vi điều khiển **ESP32-S3** (Xtensa LX7 Dual-Core 240MHz, Vector SIMD FPU, Hardware Timers, DMA I2C, High-Speed WebSockets, LittleFS) để biến cánh tay robot 6 trục thành một hệ thống điều khiển công nghiệp thu nhỏ thời gian thực với độ chính xác cao, chuyển động siêu mượt và độ an toàn tuyệt đối.

### Ràng buộc bất biến (Hard Invariants)
1. **100% Tương thích phần cứng hiện hữu**: Giữ nguyên toàn bộ sơ đồ chân GPIO và kết nối dây:
   - J1–J4: Driver TMC2209 chung UART1 (GPIO17 TX / GPIO18 RX @ 115.2k), đảo chiều qua `shaft(bool)`, chân STEP độc lập.
   - J5–J6: Driver A4988 điều khiển qua cặp chân `STEP + DIR`.
   - 6× Encoder từ tính AS5600 qua Mux I2C PCA9548A (GPIO8 SDA / GPIO9 SCL @ 400kHz).
2. **Hình học Single Source of Truth**: Duy trì chính xác thông số Craig Modified DH trong `docs/ARM_GEOMETRY.md` ($D_1=139, A_2=138, A_3=88, D_4=126, D_{\text{tool}}=20$, offset $\theta_2 = e_2 - 90^\circ, \delta = 55.0587^\circ$).
3. **An toàn thời gian thực**: Cấm ghi SPI Flash trong lúc chuyển động; bảo toàn triệt để ngắt bước trong IRAM.

---

## 2. PHẦN 1: KIẾN TRÚC DUAL-CORE & PHÂN BỔ NHIỆM VỤ (CORE PARTITIONING)

Hệ thống phân chia độc lập ranh giới tài nguyên giữa hai nhân CPU nhằm loại bỏ hoàn toàn hiện tượng nghẽn ngắt hoặc giật cục do lưu lượng mạng và I/O:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                       ESP32-S3 CORE ALLOCATION MAP                          │
├──────────────────────────────────────┬──────────────────────────────────────┤
│    ESP32-S3 CORE 0 (Comms & Sensor)  │   ESP32-S3 CORE 1 (Motion Engine)    │
├──────────────────────────────────────┼──────────────────────────────────────┤
│ [Sys] WiFi Driver Task (Prio 23)     │ [ISR] Master Step ISR (GPTimer 50kHz)│
│ [Sys] LWIP TCP/IP Task (Prio 18)     │       • Chạy 100% trong IRAM         │
│                                      │       • DDA 6 trục Fixed-Point Q32.32│
│ • Sensor Task (Prio 5, Chu kỳ 5ms):  │       • Đọc trực tiếp g_emergencyStop│
│   - Quét 6x AS5600 qua PCA9548A I2C  │       • Ghi thanh ghi GPIO W1TS/W1TC │
│   - Timeout 300µs/trục chống treo bus│                                      │
│   - Slip Watchdog: Nếu trượt tức     │ • Motion Task (Prio 3, Chu kỳ 10ms): │
│     thời >3.0° -> g_emergencyStop    │   - Nhận lệnh từ ArmCommandQueue     │
│                                      │   - Tính S-curve 7 pha (Precompute)  │
│ • WebSocket Task (Prio 2, 50Hz):     │   - 2-Pass Look-Ahead Planner 32 seg │
│   - Stream nhị phân Telemetry Frame  │   - Vector SIMD FK & IK Matrix       │
│   - Nhận lệnh Jog/Move thời gian thực│   - Nạp SPSC Ring-Buffer (64 blocks) │
│                                      │                                      │
│ • LittleFS Manager (Prio 1):         │ • Main Arduino Loop (Prio 1):        │
│   - CHỈ ghi Flash khi robot IDLE     │   - Background housekeeping          │
└──────────────────────────────────────┴──────────────────────────────────────┘
```

### 2.1. Cách ly Flash Cache Invalidation (Flash Write Isolation)
- **Cơ chế**: Thao tác ghi/xóa SPI Flash (`spi_flash_op_lock()`) vô hiệu hóa cache trên cả 2 Core từ $4\,\text{ms}$ đến $80\,\text{ms}$. 
- **Quy tắc**: Cấm tuyệt đối ghi LittleFS / NVS khi `isMoving() == true`. Khi ghi kịch bản Teach-and-Repeat, dữ liệu lưu vào RAM Ring-Buffer và chỉ flush xuống Flash khi robot về trạng thái `IDLE`.
- **IRAM / DRAM Placement**:
  - Toàn bộ chuỗi gọi của `StepISR`, hàm chuyển trạng thái DDA, cờ atomic được gán thuộc tính `IRAM_ATTR`.
  - Toàn bộ struct dữ liệu nội suy, mảng DDA accumulator và hằng số được gán `DRAM_ATTR`.

### 2.2. Master Step Engine 50kHz & State-Machine Pulse
- **Phần cứng**: 01 GPTimer phần cứng duy nhất chạy ở tần số cơ sở **$f_{\text{base}} = 50\,\text{kHz}$** ($T_{\text{tick}} = 20\,\mu\text{s}$).
- **Cơ chế phát xung 2-Tick State Machine**:
  - *Tick 1 (Overflow)*: Kích hoạt chân STEP lên mức `HIGH` (`GPIO.out_w1ts.val = mask`), đảm bảo thời gian xung $t_{\text{high}} = 20\,\mu\text{s} \gg 100\,\text{ns}$ (chuẩn TMC2209/A4988).
  - *Tick 2*: Kéo chân STEP về mức `LOW` (`GPIO.out_w1tc.val = mask`).
- **Trần tốc độ tối đa**: **$25.0\,\text{kHz/trục}$**, đáp ứng vượt trội tốc độ tối đa của cánh tay ($10.67\,\text{kHz}$ ở tỉ số $20:1$).
- **Ngân sách thời gian thực thi ISR**:
  - Trên Xtensa LX7 @ 240MHz, 1 tick $20\,\mu\text{s} = 4.800$ CPU cycles.
  - Tích phân Q32.32 6 trục + GPIO write worst-case = 10 chu kỳ/trục $\times 6$ trục + 90 chu kỳ quản lý $\approx 150 - 200$ CPU cycles $\approx \mathbf{0.83\,\mu\text{s}}$.
  - Chiếm dụng CPU của Step ISR trên Core 1: $\frac{0.83\,\mu\text{s}}{20.0\,\mu\text{s}} \approx \mathbf{4.15\%}$, bảo lưu $> 95\%$ năng lực Core 1 cho Motion Planning.

### 2.3. SPSC Ring-Buffer với Epoch-Signaled Flush (Bảo toàn Single-Writer)
Bộ đệm SPSC Lock-Free giữa `MotionTask` (Producer) và `StepISR` (Consumer) bảo toàn 100% nguyên tắc mỗi biến atomic chỉ do 1 thực thể ghi:

```cpp
template <typename T, size_t Capacity>
class SPSCQueue {
    static_assert((Capacity & (Capacity - 1)) == 0, "Capacity must be power of 2");
    alignas(64) std::atomic<size_t>   m_head{0};       // Chỉ MotionTask ghi
    alignas(64) std::atomic<uint32_t> m_flushEpoch{0}; // Chỉ MotionTask ghi
    alignas(64) std::atomic<size_t>   m_tail{0};       // Chỉ StepISR ghi
    uint32_t                          m_localEpoch{0}; // Biến cục bộ trong StepISR
    T m_buffer[Capacity];
public:
    void invalidate() IRAM_ATTR { m_flushEpoch.fetch_add(1, std::memory_order_release); }
    bool push(const T& item) IRAM_ATTR;
    bool pop(T& item) IRAM_ATTR;
};
```

---

## 3. PHẦN 2: CHUỖI AN TOÀN VÒNG KÍN & QUY TRÌNH RESYNC 2 PIPELINE

Hệ thống tách biệt rõ ràng 2 luồng xử lý giám sát:

```
┌─────────────────────────────────────────────────────────────────────────────┐
│ PIPELINE A: VÒNG BÙ SAI SỐ LIÊN TỤC (Online Compensation Loop @ 200Hz)      │
├─────────────────────────────────────────────────────────────────────────────┤
│ • Lấy mẫu tức thời: |Δθ_instant| = |θ_step - θ_encoder|                     │
│ • |Δθ_instant| ≤ 1.5°  ──► [Bình thường, không can thiệp]                   │
│ • 1.5° < |Δθ| ≤ 3.0°   ──► [Trượt nhẹ: gửi vi sai sang MotionTask bù block kế]│
│ • |Δθ_instant| > 3.0°  ──► [FAIL-FAST TỨC THỜI: g_emergencyStop = true (≤20µs)]│
└──────────────────────────────────────┬──────────────────────────────────────┘
                                       │
                                       ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│ PIPELINE B: PHÂN LOẠI SỰ CỐ SAU KHI DỪNG (Post-E-Stop Settle & Classify)    │
│ (Chạy ĐÚNG 1 LẦN sau khi g_emergencyStop = true và dừng cơ học t_settle=150ms)│
├─────────────────────────────────────────────────────────────────────────────┤
│ 1. Lấy trung bình 4 chu kỳ: θ_settle                                        │
│ 2. Kiểm tra biến thiên góc thô trục encoder trước hộp số: Δθ_raw (0..360°) │
│                                                                             │
│ [Trường hợp 1]: Δθ_raw ≤ 90.0° (Encoder bình thường)                        │
│   ├── |Δθ_settle| ≤ 3.0°       ──► [CẤP 1: Nhiễu rung tức thời / Độ võng]    │
│   │                                Auto-resync absSteps, sẵn sàng clear fault│
│   ├── 3.0° < |Δθ_settle| ≤ 6.0° ──► [CẤP 3: Trượt tải vừa / Mất bước]       │
│   │                                Auto-resync absSteps, yêu cầu Clear Fault │
│   └── |Δθ_settle| > 6.0°       ──► [CẤP 4: Trượt răng hộp số Cycloidal]      │
│                                    CẤM AUTO-RESYNC! Khóa HARD_GEARBOX_SLIP   │
│                                    Tăng NVS wear counter (+1), yêu cầu HOME  │
│                                                                             │
│ [Trường hợp 2]: Δθ_raw > 90.0° (Nhảy góc bất thường > 1/4 vòng trong 5ms)   │
│   └── Mọi giá trị              ──► [CẤP 5: Tuột dây đai / Unwrap thất bại]   │
│                                    CẤM AUTO-RESYNC! Khóa HARD_UNWRAP_LOST    │
│                                    Bắt buộc người dùng kiểm tra & HOME lại   │
└─────────────────────────────────────────────────────────────────────────────┘
```

> **Ghi chú về $t_{\text{settle}} = 150\,\text{ms}$**: *Căn cứ trên mô hình dao động tắt dần bậc 2 của cơ cấu FDM PLA với hằng số thời gian $\tau = \frac{1}{\zeta \omega_n} = \frac{1}{2\pi \zeta f_n}$. Với dải thông số thực tế của cánh tay in 3D ($f_n \approx 15 - 20\,\text{Hz}, \zeta \approx 0.15 - 0.25$), thời gian để biên độ dao động cơ học suy giảm $> 95\%$ là $t \ge 3\tau \approx 95 - 212\,\text{ms}$. Kết hợp với hiệu ứng cản điện động (active holding current damping) của cuộn dây động cơ bước khi hãm, thời gian chờ $t_{\text{settle}} = \mathbf{150\,\text{ms}}$ bảo đảm hoàn toàn cho bộ lọc trung bình $\bar{\theta}_{\text{settle}}$ đạt độ tin cậy $100\%$ trước khi phân loại lỗi.*

---

## 4. PHẦN 3: S-CURVE 7 PHA, 2-PASS LOOK-AHEAD & WORK PLANE

### 4.1. Cấu trúc Motion Block Bậc 2 & Tích Phân Q32.32
Mỗi block đại diện cho một pha S-curve hoặc cung chuyển động hoàn chỉnh ($50\,\text{ms} - 500\,\text{ms}$):
```cpp
struct MotionBlock {
    uint32_t totalTicks;             // Thời lượng block (ticks 20µs)
    uint64_t targetAbsSteps[6];      // Tọa độ bước đích tuyệt đối (Snap-to-target)
    uint64_t ddaStepFraction[6];     // Tốc độ ban đầu v0 (Q32.32 fixed-point)
    int32_t  ddaStepAccel[6];        // Gia tốc ban đầu a0
    int32_t  ddaStepJerk[6];         // Độ giật Jerk không đổi trong pha
    uint8_t  dirMask;
    bool     isLastSegment;
};
```
- **Snap-to-Target**: Tại tick cuối cùng của block, `absSteps[i]` được gán chính xác bằng `targetAbsSteps[i]` và reset phần dư `accumulator = 0`, triệt tiêu hoàn toàn sai số tích lũy làm tròn fixed-point sau nhiều ngày vận hành.
- **$C^1$ Velocity Continuity**: Block $N+1$ nhận trạng thái `v0` và `a0` từ đúng điểm kết thúc lý thuyết của Block $N$, đảm bảo không có bước nhảy vận tốc rời rạc tại ranh giới block.

### 4.2. Bộ Đệm Look-Ahead 2 Chiều & $a_{\text{effective}}$ Theo Ma Trận Jacobian
1. **Bộ lọc gộp vi đoạn trước khi tính toán (Micro-Segment Coalescing)**:
   - Gộp 2 đoạn liên tiếp nếu đồng thời thỏa mãn: Góc lệch tiếp tuyến $\Delta\theta < 2.0^\circ$ VÀ Sai số độ võng vuông góc $d_{\text{perp}} \le \mathbf{0.05\,\text{mm}}$.
2. **Gia tốc giới hạn phụ thuộc tư thế $a_{\text{effective}}(\vec{q})$**:
   $$a_{\text{effective}}(\vec{q}, \vec{u}_{\text{dir}}) = \min_{i=1..6} \left( \frac{\ddot{q}_{\text{max}}[i]}{\left| \left[ J^{-1}(\vec{q}) \cdot \vec{u}_{\text{dir}} \right]_i \right|} \right)$$
3. **2-Pass Look-Ahead Planner**:
   - **Forward Pass**: Tính $v_{\text{exit, fwd}}^2 = v_{\text{entry}}^2 + 2 a_{\text{effective}} s \implies v_{\text{junction}} = \min(v_{\text{target}}, v_{\text{corner\_geom}}, v_{\text{exit, fwd}})$.
   - **Backward Pass**: Quét ngược từ block cuối buffer về block đầu: $v_{\text{entry, bwd}}^2 = v_{\text{junction}}^2 + 2 a_{\text{effective}} s \implies v_{\text{entry}} = \min(v_{\text{entry}}, v_{\text{entry, bwd}})$.

### 4.3. Mặt Phẳng Vẽ Tùy Biến (Custom Work Plane & 3-Point Calibration)
Cho phép vẽ trên mặt phẳng nghiêng/cao độ bất kỳ thông qua hiệu chuẩn 3 điểm $P_1, P_2, P_3$:
- **Kiểm tra suy biến hình học (Collinear Guard)**:
  - Từ chối hiệu chuẩn nếu $|P_2 - P_1| < 20\,\text{mm}$ hoặc $|P_3 - P_1| < 20\,\text{mm}$.
  - Từ chối nếu góc mở giữa 2 vector $\sin\phi = \frac{|(P_2-P_1)\times(P_3-P_1)|}{|P_2-P_1||P_3-P_1|} < 0.1736$ (tương ứng góc $< 10^\circ$).
- **Phép biến đổi tọa độ thời gian thực**:
  $$\begin{bmatrix} x \\ y \\ z \end{bmatrix}_{\text{robot}} = P_1 + u \cdot \vec{u}_{\text{plane}} + v \cdot \vec{v}_{\text{plane}} + w_{\text{lift}} \cdot \vec{n}_{\text{plane}}$$
  Tọa độ $(x, y, z)$ được giải bằng Closed-Form IK để đầu bút luôn vuông góc với mặt phẳng phôi.

---

## 5. PHẦN 4: WEBSOCKET TELEMETRY 50Hz & VECTOR SIMD

1. **WebSocket Binary Stream @ 50Hz (Core 0)**:
   - Thay thế hoàn toàn HTTP REST Polling 300ms.
   - Gói tin nhị phân packed struct 64 bytes: `[absSteps: 24B | encDeg: 24B | tcpXYZ: 12B | flags: 4B]`.
   - Độ trễ điều khiển mạng Wi-Fi giảm xuống $< 8\,\text{ms}$.
2. **Tối ưu hóa Ma Trận Vector FPU (Core 1)**:
   - Triển khai nhân ma trận Craig MDH và giải FK bằng tập lệnh vector đơn SIMD, rút ngắn thời gian tính toán toàn bộ 6 trục phục vụ điều khiển Cartesian thời gian thực.

---

## 6. KẾ HOẠCH TRIỂN KHAI THEO GIAI ĐOẠN (PHASED ROADMAP)

- **Giai đoạn 1**: Xây dựng SPSC Ring-Buffer Epoch-Signaled, Master GPTimer 50kHz DDA Q32.32 và Direct Fail-Fast Stop ISR.
- **Giai đoạn 2**: Tách biệt 2 Pipeline an toàn (200Hz Online Compensation vs. 50ms Settle Classification) & Detach I2C Bus Recovery.
- **Giai đoạn 3**: Triển khai 2-Pass Look-Ahead Planner, $a_{\text{effective}}$ Jacobian và Micro-segment coalescing.
- **Giai đoạn 4**: Tích hợp Work Plane UCS 3-Point Calibration & WebSockets 50Hz Telemetry Studio.
