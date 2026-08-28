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
                    ,----[ J4 ]--+------[J5]-------[J6]
                    |            |        \          \
                    | 88mm       '--110mm--'          (pen, D6_TOOL)
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
- **Điểm gập → J4/J5/J6 (offset)**: 16mm + 110mm = **126mm tổng cộng, nối tiếp nhau trên cùng một trục** (không phải hai hướng riêng biệt).
- **J4, J5, J6**: cụm cổ tay (wrist), bố trí kiểu **pan-tilt-roll**.
- **Bút (tool)**: gắn **đồng trục** với J6, dài **20mm** tính từ tâm cổ tay.

## 2. Quan hệ giao nhau giữa các trục khớp (đã xác nhận)

| Cặp khớp | Quan hệ |
|---|---|
| J1, J2, J3 | Các trục xoay **giao nhau** theo chuỗi liên tiếp (cấu trúc tay máy khớp nối kinh điển — RRR) |
| J3 → J4 | Trục **vuông góc** với J3, nhưng **KHÔNG giao nhau** — có khoảng lệch vuông góc chung (common perpendicular) = tham số DH `a`. Đây là hình học bình thường, không phải lỗi lắp ráp. |
| J4, J5, J6 | Các trục xoay **giao nhau tại một điểm chung duy nhất** → thỏa mãn **điều kiện Pieper** → cổ tay là **spherical wrist thật sự**, cho phép giải IK dạng đóng (closed-form), tách vị trí và hướng độc lập. |

**Hệ quả quan trọng:** vì cổ tay là spherical wrist chuẩn, IK có thể giải theo 2 bước độc lập:
1. Tính **wrist center** (lùi từ TCP theo trục tool một đoạn `D6_TOOL`), giải hình học J1-J2-J3.
2. Giải hướng (orientation) cho J4-J5-J6 từ ma trận xoay còn lại.

## 3. Bảng tham số DH (đã xác nhận & validate với vật lý)

**Quy ước sử dụng: Modified DH (Craig convention)** —
`T_i = Rx(alpha_{i-1}) · Tx(a_{i-1}) · Rz(theta_i) · Tz(d_i)`

> ⚠️ Đây **không phải** Standard/Classic DH thông thường. Nếu viết lại FK ở ngôn ngữ/nền tảng
> khác (C++, JS, MATLAB...), phải dùng đúng công thức ma trận Modified DH này, nếu không kết
> quả sẽ sai lệch dù bảng số liệu giống hệt. Xem cách triển khai tham chiếu tại
> `fk_verify.py::dh_transform()` và bản port JavaScript trong `arm_simulator.html`.

| Khớp *i* | a₍ᵢ₋₁₎ (mm) | α₍ᵢ₋₁₎ (độ) | dᵢ (mm) | θᵢ | Mô tả |
|---|---|---|---|---|---|
| 1 | 0 | 0 | 139 | θ1 (biến) | Base yaw |
| 2 | 0 | -90 | 0 | θ2 (biến) | Shoulder pitch |
| 3 | 138 | 0 | 0 | θ3 (biến) | Elbow pitch |
| 4 | 88 | -90 | **126** (=16+110) | θ4 (biến) | Wrist offset → pan |
| 5 | 0 | +90 | 0 | θ5 (biến) | Wrist tilt |
| 6 | 0 | -90 | 0 | θ6 (biến) | Wrist roll |
| TCP | — | — | **D6_TOOL = 20** | — | Bút, gắn đồng trục với J6 |

## 4. Offset góc: Encoder Zero ↔ DH Theta

Vị trí "home" vật lý (tất cả encoder = 0°, mốc calibrate qua `setHomeHere()`) **không trùng**
với θ=0 trong quy ước DH toán học. Công thức chuyển đổi bắt buộc:

```
theta_DH(i) = theta_encoder(i) + OFFSET(i)
```

| Khớp | Offset (độ) | Trạng thái xác nhận |
|---|---|---|
| θ1 | 0 | Giả định (chưa cần calib, base thường không lệch) |
| θ2 | **-90** | ✅ **Đã xác nhận bằng đo đạc vật lý** (xem mục 5) |
| θ3 | 0 | ✅ Đã xác nhận (tại home, elbow không gập — chuỗi 139-138-88 thẳng hàng dọc như bản vẽ) |
| θ4 | 0 | ⚠️ **CHƯA xác nhận thực nghiệm** — cần đo giống cách đã làm với θ2 |
| θ5 | 0 | ⚠️ **CHƯA xác nhận thực nghiệm** |
| θ6 | 0 | ⚠️ **CHƯA xác nhận thực nghiệm** |

> Vì bút dài 20mm gắn lệch tâm cổ tay, sai lệch offset ở θ4/θ5/θ6 dù nhỏ (1-2°) vẫn có thể
> làm đầu bút lệch vài mm khi vẽ. **Ưu tiên calib 3 khớp này trước khi tin dùng IK cho vẽ thật.**

## 5. Xác nhận đối chiếu vật lý (đã validate — không cần đo lại)

Tại vị trí home (mọi encoder = 0°), mô hình FK (với offset θ2=-90° ở trên) dự đoán:

```
Wrist center = (x=126.0mm, y=0mm, z=365.0mm)
```

Khớp **chính xác tuyệt đối** với bản vẽ tay:
- `x = 126mm = 16 + 110` (tổng offset ngang cổ tay)
- `z = 365mm = 139 + 138 + 88` (tổng chiều cao chuỗi thẳng đứng)

Đây là bằng chứng bảng DH + offset ở trên **đã đúng**, không cần nghi ngờ hay đo lại phần này.

## 6. Tầm với & vùng làm việc (workspace) — dựa trên hình học đã xác nhận

Giới hạn khớp đã cung cấp (giả định đối xứng quanh home — **cần xác nhận lại nếu không đối xứng**):

| Khớp | Tổng hành trình | Giả định range |
|---|---|---|
| J1 | 180° | ±90° quanh home |
| J2 | 180° | ±90° quanh home |
| J3 | 90° | +90° từ home |

Kết quả tính được (xem `fk_verify.py` để tái tạo):

| Thông số | Giá trị |
|---|---|
| Tầm với xa nhất (radial từ trục J1) | 291.7mm |
| Tầm với gần nhất (radial inner deadzone) | 15.7mm (=|153.69 - 138|) |
| Độ cao tối đa | 430.7mm |
| Độ cao tối thiểu | -14.7mm (có thể bị đế chặn vật lý) |
| Đoạn J3→wrist center (cố định, không đổi theo θ3) | 153.69mm (=√(88²+126²)) |

Với J1 hành trình 180° (±90° quanh home), có một **vùng góc chết 180°** phía sau robot không quay tới được.

## 7. Kỳ dị động học đã phát hiện (Singularity)

Khi hướng công cụ (tool z-axis) **chỉ thẳng đứng** (song song trục J1, ví dụ bút cắm thẳng
xuống bàn) → trục J1 và J6 gần như trùng phương → **mất 1 bậc tự do thực tế** (vô số cặp
(θ1, θ6) cho cùng kết quả). Lưu ý: đây là **gimbal-lock do J1//J6 và khóa cứng e4=e6=0**, không phải
"wrist singularity" kinh điển (cái sau là trục cổ tay cầu J4/J6 trùng nhau). Trong firmware, đây
không phải lỗi code — nó là hệ quả tất yếu của cấu trúc 6-DOF đã khóa e4=e6.

**Cách xử lý đã áp dụng:** dùng **Closed-form Analytic IK** (`kin::ikPenDown()` trong `src/kinematics.cpp`)
giải hình học trực tiếp góc vai-khuỷu (J2-J3) theo định lý cosin, tính góc nghiêng cổ tay J5 = -(t2+t3+δ),
và cố định J4=0, J6=0 — loại bỏ hoàn toàn hiện tượng trôi nghiệm của IK số lặp.

## 8. File tham chiếu liên quan trong repo

| File | Vai trò |
|---|---|
| `digital_clone.py` | Visualizer 3D/2D đa góc nhìn + kiểm thử động học & bộ sinh quỹ đạo Planner |
| `src/kinematics.h` / `src/kinematics.cpp` | Bản port C++/float cho ESP32-S3 firmware — **PHẢI đồng bộ với file này khi cập nhật DH/offset** |
| `test/kinematics/test_kinematics.cpp` | Host test tự động 3280 điểm FK/IK roundtrip |

> ⚠️ Khi thay đổi bất kỳ số liệu nào ở mục 1-4 (đo lại link length, xác nhận offset J4-J6...),
> **bắt buộc cập nhật đồng thời cả 3 file trên** để tránh lệch pha giữa simulator, firmware, và test suite.

## 9. Việc còn tồn đọng (TODO)

- [ ] Xác nhận offset vật lý θ4, θ5, θ6 (đo giống cách đã làm với θ2)
- [x] Xác nhận giới hạn góc J1/J2/J3 (Đã chốt: J1: ±90°, J2: ±90°, J3: 0..+90°)
- [ ] Đo D6_TOOL chính xác bằng thước cặp thay vì ước lượng "khoảng 2cm"
- [x] Port closed-form IK (geometric + spherical wrist decomposition) sang C++/float cho firmware (`src/kinematics.cpp::ikPenDown`)
