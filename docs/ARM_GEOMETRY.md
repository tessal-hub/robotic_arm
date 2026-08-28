# Arm Geometry Reference — 6-DOF NEMA Robotic Arm

**Đây là tài liệu gốc duy nhất (single source of truth) về cấu trúc hình học cơ khí của cánh tay.**
Mọi lần tính DH table, viết FK/IK, hay giải thích lại cấu trúc — luôn tham chiếu file này trước,
không suy diễn lại từ đầu. Nội dung đã được xác nhận bằng đo đạc vật lý thực tế và khớp
chính xác với bản vẽ tay gốc (xem `docs/sketches/` nếu có lưu ảnh gốc).

---

## 1. Sơ đồ cơ khí gốc (từ bản vẽ tay)

```
                              <--16-->
                                 |
                    ,----[ J4 ]--+------[ J5 ] ─────── [ J6 ] ─────── (Pen TCP)
                    |            |          \    31mm     \    20mm     \
                    | 88mm       '--110mm----'             '-(D_TOOL)---'
                    |
                  [ J3 ]---- (elbow, quay ngang)
                    |
                    | 138mm
                    |
                  [ J2 ]---- (shoulder, quay ngang)
                    |
                    | 139mm
                    |
                  [ J1 ]==== (base, quay dọc/vertical axis)
                    |
                 ===GND===
```

- **J1 → J2**: 139mm — chiều cao đế lên vai. J1 là khớp **base yaw**, trục xoay **thẳng đứng**.
- **J2 → J3**: 138mm — cánh tay trên (upper arm). J2 là khớp **shoulder pitch**, trục xoay **nằm ngang**.
- **J3 → điểm gập**: 88mm — J3 là khớp **elbow pitch**, trục xoay **song song J2**.
- **Điểm gập → J5 (Wrist Tilt)**: 16mm + 110mm = **126mm** nối tiếp trên trục cẳng tay.
- **J5 → J6 (Tool Roll)**: **31mm** dọc theo trục công cụ.
- **J6 → Pen Tip (TCP)**: **20mm** gắn đồng trục với J6.
- **Tổng chiều dài khâu công cụ hiệu dụng (J5 → Pen TCP)**: $31\text{mm} + 20\text{mm} = \mathbf{51\text{mm}}$.

## 2. Quan hệ giao nhau giữa các trục khớp

| Cặp khớp | Quan hệ |
|---|---|
| J1, J2, J3 | Các trục xoay **giao nhau** theo chuỗi liên tiếp (cấu trúc RRR kinh điển) |
| J3 → J4 | Trục **vuông góc** với J3, có khoảng lệch vuông góc chung (common perpendicular) = $a_3 = 88\text{mm}$. |
| J4, J5 | Trục xoay cắt nhau tại tâm cổ tay J5 ($d_4 = 126\text{mm}$). |
| J5 → J6 | Trục $Z_6$ (Tool Roll) vuông góc với trục nghiêng $Z_5$, đặt cách J5 một khoảng $d_6 = 31\text{mm}$. |
| J6 → TCP | Bút gắn đồng trục với $Z_6$, dài $D_{\text{tool}} = 20\text{mm}$. |

**Hệ quả đối với IK Pen-Down (Bút chỉ thẳng đứng $\theta_4 = 0, \theta_6 = 0$):**
Vì bút luôn hướng thẳng đứng xuống dưới (song song trục $-Z$), đoạn $31\text{mm}$ (J5 $\to$ J6) và $20\text{mm}$ (J6 $\to$ TCP) nằm thẳng hàng dọc, tạo thành cánh tay đòn thẳng đứng dài **$51\text{mm}$**.
IK được giải theo 2 bước:
1. Tính tọa độ tâm trục nghiêng **J5** ($Z_{\text{J5}} = Z_{\text{target}} + 51.0\text{mm}, X_{\text{J5}} = X_{\text{target}}, Y_{\text{J5}} = Y_{\text{target}}$).
2. Giải hình học phẳng 2 khâu cho J1-J2-J3 theo định lý cosin, đặt $\theta_5 = -(t_2 + t_3 + \delta)$.

## 3. Bảng tham số DH (Modified DH — Craig convention)

Công thức biến đổi: `T_i = Rx(alpha_{i-1}) · Tx(a_{i-1}) · Rz(theta_i) · Tz(d_i)`

| Khớp *i* | a₍ᵢ₋₁₎ (mm) | α₍ᵢ₋₁₎ (độ) | dᵢ (mm) | θᵢ | Mô tả |
|---|---|---|---|---|---|
| 1 | 0 | 0 | **139** | θ1 (biến) | Base yaw |
| 2 | 0 | -90 | 0 | θ2 (biến) | Shoulder pitch |
| 3 | **138** | 0 | 0 | θ3 (biến) | Elbow pitch |
| 4 | **88** | -90 | **126** (=16+110) | θ4 (biến) | Forearm offset → Wrist pan |
| 5 | 0 | +90 | 0 | θ5 (biến) | Wrist tilt |
| 6 | 0 | -90 | **31** | θ6 (biến) | Wrist roll (J5 → J6) |
| TCP | — | — | **D_TOOL = 20** | — | Bút, gắn đồng trục với J6 |

## 4. Offset góc: Encoder Zero ↔ DH Theta

Công thức chuyển đổi: `theta_DH(i) = theta_encoder(i) + OFFSET(i)`

| Khớp | Offset (độ) | Trạng thái xác nhận |
|---|---|---|
| θ1 | 0 | ✅ Trục Z đế |
| θ2 | **-90** | ✅ Đã xác nhận vật lý (thẳng đứng tại home) |
| θ3 | 0 | ✅ Đã xác nhận (thẳng hàng chuỗi 139-138-88) |
| θ4 | 0 | ✅ Giữ mặt phẳng khi vẽ |
| θ5 | 0 | ✅ Đồng trục cổ tay |
| θ6 | 0 | ✅ Xoay tròn ngòi bút |

## 5. Xác nhận đối chiếu vật lý tại Home (0, 0, 0, 0, 0, 0)

Tại vị trí Home (mọi encoder = 0°), mô hình FK dự đoán:
- **Tâm trục J5**: $(X = 126.0\text{mm}, Y = 0.0\text{mm}, Z = 365.0\text{mm})$ ($126 = 16+110; 365 = 139+138+88$)
- **Tâm trục J6**: $(X = 157.0\text{mm}, Y = 0.0\text{mm}, Z = 365.0\text{mm})$ ($157 = 126 + 31$)
- **Đầu nhọn bút (TCP)**: $(X = 177.0\text{mm}, Y = 0.0\text{mm}, Z = 365.0\text{mm})$ ($177 = 126 + 31 + 20$)

## 6. Tầm với & vùng làm việc (Workspace)

| Thông số | Giá trị |
|---|---|
| Khoảng cách trục J3 $\to$ tâm J5 | 153.69mm ($=\sqrt{88^2 + 126^2}$) |
| Góc lệch cẳng tay $\delta$ | 55.06° ($=\text{atan2}(126, 88)$) |
| Tầm với xa nhất đến tâm J5 | 291.7mm ($= 138.0 + 153.69$) |
| Tầm với gần nhất (inner deadzone) | 15.7mm ($=|153.69 - 138.0|$) |
| Chiều dài khâu công cụ hiệu dụng (J5 $\to$ TCP) | **51.0mm** ($= 31\text{mm} + 20\text{mm}$) |
| Độ cao tối đa tâm J5 | 430.7mm |

## 7. Kỳ dị động học đã phát hiện (Singularity)

Khi hướng công cụ (tool z-axis) **chỉ thẳng đứng** (song song trục J1) $\to$ trục J1 và J6 trùng phương.
Sử dụng **Closed-form Analytic IK** (`kin::ikPenDown()` trong `src/kinematics.cpp`) giải giải tích trực tiếp góc vai-khuỷu (J2-J3) với $Z_{\text{wrist}} = Z_{\text{target}} + 51.0\text{mm}$ và cố định J4=0, J6=0, triệt tiêu hoàn toàn trôi nghiệm.

## 8. Cơ cấu Vi sai Bánh răng Côn Cổ tay J5–J6 (2-DOF Bevel Gear Differential Wrist)

Cụm cổ tay J5 (Tilt / Nghiêng) và J6 (Roll / Xoay bút) sử dụng cơ cấu **2-DOF Coupled Differential Gimbal / Bevel Gear Differential**:

```
                       ┌─────────────────────────┐
                       │  Động cơ M5 (Left)      │ ──► Encoder E_L (Side Gear 1)
                       │  Động cơ M6 (Right)     │ ──► Encoder E_R (Side Gear 2)
                       └───────────┬─────────────┘
                                   │
                     ┌─────────────┴─────────────┐
                     │ Bánh răng côn bên trái    │ (Side Gear Left, góc θ_L)
                     │ Bánh răng côn bên phải    │ (Side Gear Right, góc θ_R)
                     └─────────────┬─────────────┘
                                   │ Ăn khớp vuông góc
                     ┌─────────────▼─────────────┐
                     │ Bánh răng côn đầu ra      │ (Output / Spider Gear, r_bevel = 1.0)
                     │ Khung mang (Carrier)      │
                     └─────────────┬─────────────┘
                                   │
              ┌────────────────────┴────────────────────┐
              ▼                                         ▼
   Góc nghiêng J5 (Tilt):                    Góc xoay J6 (Roll):
   θ_J5 = (θ_L + θ_R) / 2                    θ_J6 = (θ_L − θ_R) / (2 · r_bevel)
   (Khi θ_L = θ_R → Thuần Tilt)              (Khi θ_L = −θ_R → Thuần Roll)
```

### 8.1. Công thức Động học Vi sai (Forward & Inverse)

- **Thuận (Từ góc 2 Encoder $E_L, E_R$ hoặc 2 Động cơ $M_L, M_R \to$ Góc Khớp $J_5, J_6$)**:
  $$\begin{aligned}
  \theta_{J5} &= \frac{\theta_L + \theta_R}{2} \\
  \theta_{J6} &= \frac{\theta_L - \theta_R}{2 \cdot r_{\text{bevel}}} \quad (\text{với } r_{\text{bevel}} = 1.0)
  \end{aligned}$$

- **Nghịch (Từ mục tiêu góc Khớp $J_5, J_6 \to$ Góc trục Động cơ $M_L, M_R$)**:
  $$\begin{aligned}
  \theta_L &= \theta_{J5} + r_{\text{bevel}} \cdot \theta_{J6} \\
  \theta_R &= \theta_{J5} - r_{\text{bevel}} \cdot \theta_{J6}
  \end{aligned}$$

- **Quy đổi Xung bước (Incremental Steps)**:
  $$\begin{aligned}
  \Delta \text{Steps}_5 &= \text{round}\left( (\Delta\theta_{J5} + \Delta\theta_{J6}) \cdot \text{stepsPerDeg}_5 \right) \\
  \Delta \text{Steps}_6 &= \text{round}\left( (\Delta\theta_{J5} - \Delta\theta_{J6}) \cdot \text{stepsPerDeg}_6 \right)
  \end{aligned}$$

### 8.2. Đặc tính vận hành thực tế
1. **Chia sẻ tải trọng**: Cả hai động cơ NEMA 14 (J5 và J6) đều đặt cố định trên thân cẳng tay (không cần slip-ring hay dây xoắn), cùng đồng thời chia đôi mô-men xoắn khi nghiêng hoặc xoay bút.
2. **Không trôi điểm gốc (Encoder Alignment)**: Hai encoder AS5600 đọc góc tuyệt đối trực tiếp của 2 bánh răng côn bên ($E_L, E_R$). Khi bật nguồn hoặc hiệu chuẩn Home, firmware khôi phục tức thời trạng thái $(\theta_{J5}, \theta_{J6})$ thông qua lớp `DifferentialWrist` trong `src/differential_wrist.h`.

---

## 9. File tham chiếu liên quan trong repo

| File | Vai trò |
|---|---|
| `src/differential_wrist.h` / `.cpp` | Thư viện giải mã động học vi sai 2-DOF bánh răng côn (C++17 thuần, host-testable) |
| `src/joint_model.h` / `.cpp` | Tích hợp giải mã $E_L, E_R \to J_5, J_6$ và $M_L, M_R \to J_5, J_6$ |
| `src/arm.cpp` | Điều khiển Jog độc lập (Jog Tilt $\to M_L, M_R$ cùng chiều; Jog Roll $\to M_L, M_R$ ngược chiều) |
| `src/planner.cpp` | Đồng bộ bước vi sai cho các segment vẽ quỹ đạo |
| `digital_clone.py` | Trình mô phỏng 3D/2D tích hợp module vi sai `DifferentialWrist` |
| `test/kinematics/test_kinematics.cpp` | Host unit test tự động cho toàn bộ chuyển đổi vi sai |
| `src/kinematics.h` / `src/kinematics.cpp` | Bản port C++/float cho ESP32-S3 firmware |


