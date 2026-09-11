# Arm Geometry Reference — 6-DOF NEMA Robotic Arm

**Đây là tài liệu gốc duy nhất (single source of truth) về cấu trúc hình học cơ khí của cánh tay.**
Mọi lần tính DH table, viết FK/IK, hay giải thích lại cấu trúc — luôn tham chiếu file này trước,
không suy diễn lại từ đầu. Nội dung đã được xác nhận bằng đo đạc vật lý thực tế và khớp
chính xác với bản vẽ tay gốc (xem `docs/sketches/` nếu có lưu ảnh gốc).

---

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
                    ,----[ J4 ]---------[ J5 ] ─────── [ J6 ] ─────── (Pen TCP)
                    |              125mm       \    45mm     \    30mm     \
                    | 88mm                                  '-(D_TOOL)---'
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
- **J4 → J5**: **125mm**.
- **J5 → J6**: **45mm**.
- **J6 → Pen Tip (TCP)**: **30mm** gắn đồng trục với J6.
- **Tổng chiều dài khâu công cụ hiệu dụng (J5 → Pen TCP)**: $45\text{mm} + 30\text{mm} = \mathbf{75\text{mm}}$.

## 2. Quan hệ giao nhau giữa các trục khớp

| Cặp khớp | Quan hệ |
|---|---|
| J1, J2, J3 | Các trục xoay **giao nhau** theo chuỗi liên tiếp (cấu trúc RRR kinh điển) |
| J3 → J4 | Trục **vuông góc** với J3, có khoảng lệch vuông góc chung (common perpendicular) = $a_3 = 88\text{mm}$. |
| J4, J5 | Chuỗi khớp quay nối tiếp, khoảng cách J4 → J5 là $d_4 = 125\text{mm}$. |
| J5 → J6 | Hai khớp quay nối tiếp độc lập; J5 → J6 là $d_6 = 45\text{mm}$. |
| J6 → TCP | Bút gắn đồng trục với $Z_6$, dài $D_{\text{tool}} = 30\text{mm}$. |

**Hệ quả đối với IK Pen-Down (Bút chỉ thẳng đứng $\theta_4 = 0, \theta_6 = 0$):**
Vì bút luôn hướng thẳng đứng xuống dưới (song song trục $-Z$), đoạn $45\text{mm}$ (J5 $\to$ J6) và $30\text{mm}$ (J6 $\to$ TCP) nằm thẳng hàng dọc, tạo thành cánh tay đòn thẳng đứng dài **$75\text{mm}$**.
IK được giải theo 2 bước:
1. Tính tọa độ tâm **J5** ($Z_{\text{J5}} = Z_{\text{target}} + 75.0\text{mm}, X_{\text{J5}} = X_{\text{target}}, Y_{\text{J5}} = Y_{\text{target}}$).
2. Giải hình học phẳng 2 khâu cho J1-J2-J3 theo định lý cosin, đặt $\theta_5 = -(t_2 + t_3 + \delta)$.

## 3. Bảng tham số DH (Modified DH — Craig convention)

Công thức biến đổi: `T_i = Rx(alpha_{i-1}) · Tx(a_{i-1}) · Rz(theta_i) · Tz(d_i)`

| Khớp *i* | a₍ᵢ₋₁₎ (mm) | α₍ᵢ₋₁₎ (độ) | dᵢ (mm) | θᵢ | Mô tả |
|---|---|---|---|---|---|
| 1 | 0 | 0 | **139** | θ1 (biến) | Base yaw |
| 2 | 0 | -90 | 0 | θ2 (biến) | Shoulder pitch |
| 3 | **138** | 0 | 0 | θ3 (biến) | Elbow pitch |
| 4 | **88** | -90 | **125** | θ4 (biến) | Forearm offset → J5 |
| 5 | 0 | +90 | 0 | θ5 (biến) | Independent revolute joint |
| 6 | 0 | -90 | **45** | θ6 (biến) | Independent revolute joint (J5 → J6) |
| TCP | — | — | **D_TOOL = 30** | — | Bút, gắn đồng trục với J6 |

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
- **Tâm trục J5**: $(X = 125.0\text{mm}, Y = 0.0\text{mm}, Z = 365.0\text{mm})$ ($365 = 139+138+88$)
- **Tâm trục J6**: $(X = 170.0\text{mm}, Y = 0.0\text{mm}, Z = 365.0\text{mm})$ ($170 = 125 + 45$)
- **Đầu nhọn bút (TCP)**: $(X = 200.0\text{mm}, Y = 0.0\text{mm}, Z = 365.0\text{mm})$ ($200 = 125 + 45 + 30$)

## 6. Tầm với & vùng làm việc (Workspace)

| Thông số | Giá trị |
|---|---|
| Khoảng cách trục J3 $\to$ tâm J5 | 152.87mm ($=\sqrt{88^2 + 125^2}$) |
| Góc lệch cẳng tay $\delta$ | 54.85° ($=\text{atan2}(125, 88)$) |
| Tầm với xa nhất đến tâm J5 | 290.87mm ($= 138.0 + 152.87$) |
| Tầm với gần nhất (inner deadzone) | 14.87mm ($=|152.87 - 138.0|$) |
| Chiều dài khâu công cụ hiệu dụng (J5 $\to$ TCP) | **75.0mm** ($= 45\text{mm} + 30\text{mm}$) |
| Độ cao tối đa tâm J5 | 429.87mm |

## 7. Kỳ dị động học đã phát hiện (Singularity)

Khi hướng công cụ (tool z-axis) **chỉ thẳng đứng** (song song trục J1) $\to$ trục J1 và J6 trùng phương.
Sử dụng **Closed-form Analytic IK** (`kin::ikPenDown()` trong `src/kinematics.cpp`) giải giải tích trực tiếp góc vai-khuỷu (J2-J3) với $Z_{\text{wrist}} = Z_{\text{target}} + 75.0\text{mm}$ và cố định J4=0, J6=0, triệt tiêu hoàn toàn trôi nghiệm.

## 8. Truyền động độc lập J5–J6

J5 và J6 là hai khớp quay nối tiếp, không còn cơ cấu pan–tilt vi sai. Mỗi A4988 điều khiển trực tiếp một khớp và mỗi AS5600 phản hồi đúng khớp tương ứng:

- J5: tỷ số truyền **3:1**, khoảng cách J4 → J5 **125mm**.
- J6: tỷ số truyền **1:1**, khoảng cách J5 → J6 **45mm**.
- Không có limit switch hay StallGuard; J5/J6 tiếp tục dùng **Set Home thủ công**.
- Jog đổi trực tiếp góc khớp sang bước của cùng trục. Cartesian/Draw chỉ điều khiển J1–J5; J6 là tool roll nên giữ nguyên và không cần Home/encoder để vẽ.

---

## 9. File tham chiếu liên quan trong repo

| File | Vai trò |
|---|---|
| `src/joint_model.h` / `.cpp` | Quy đổi độc lập step/encoder cho từng khớp J5 và J6 |
| `src/arm.cpp` | Jog trực tiếp đúng motor của J5 hoặc J6 |
| `src/planner.cpp` | Đồng bộ bước J1–J5 cho Cartesian/Draw; không điều khiển J6 |
| `test/kinematics/test_kinematics.cpp` | Host unit test FK/IK với kích thước mới |
| `src/kinematics.h` / `src/kinematics.cpp` | Bản port C++/float cho ESP32-S3 firmware |


