# Hardware Regression Checklist

Chạy trên phần cứng thật sau thay đổi an toàn / homing / endstop (2026-08-29 audit fixes).

## Chuẩn bị

- [ ] Flash firmware mới (`pio run -t upload`)
- [ ] Serial monitor @115200
- [ ] Xác nhận J2 `AXIS_ENC_SIGN` trong `config.h` khớp chiều encoder thực tế

## 1. Endstop → FAULT

- [ ] HOME J1–J4 thành công
- [ ] Jog J1 **vào** MIN endstop → motor dừng, web `mode=fault`, phải bấm Clear Fault
- [ ] Clear Fault **bị từ chối** khi endstop vẫn nhấn
- [ ] Nhả endstop → Clear Fault → `mode=idle`, jog lại được
- [ ] Lặp với J2 MAX (trục hay nhiễu EMI trước đây)

## 2. Multi-axis E-stop

- [ ] Cartesian LINE ngắn, trong lúc chạy chạm endstop J1
- [ ] **Tất cả** trục dừng (không chỉ J1), FAULT latch

## 3. Homing regression

- [ ] Home All J1→J4 sau thay đổi FSM
- [ ] J3 scan path (MIN+MAX) — home đúng vị trí offset
- [ ] Warmup không lao vào endstop (J2/J3 negative step sign)
- [ ] Home từ cả MIN và MAX (symmetrical scan)
- [ ] **2 tốc độ**: log thấy mỗi cữ chạm 2 lần — `SCAN_MIN fast` → `BACKOFF` → `SCAN SLOW` (3000µs/step); điểm chạm SLOW mới là mốc chính xác
- [ ] **VERIFY**: mỗi khớp log `VERIFY OK (err=..., tol=...)`; nếu steps/deg lệch thì thấy `TRIM #1/#2` rồi OK — không được FAIL thường xuyên
- [ ] **J4 (không endstop)**: chạm cữ bằng `SG stall` (fast) và `SLOW stall (encDelta=..., cur=..., contact=...)` (slow); motor KHÔNG đập cữ mạnh ở pha slow; home lặp lại 3 lần liên tiếp → sai lệch bước giữa các lần ≤ ~±2° (giới hạn EMA 50Hz + cửa sổ stall)
- [ ] **Retry**: thử gây fail (chặn khớp giữa đường quét) → log `FAILED — thu lai lan 2/2`, chuỗi KHÔNG hủy ngay; fail đủ 2 lần mới `huy chuoi homing` + `lastOk=false`
- [ ] Timeout mỗi phase hoạt động (60s scan / 30s phase) → retry, không treo FSM
- [ ] **Encoder J1 đóng băng (đã gặp thật)**: ngắt/lỏng encoder J1 rồi Home All → J1 phải FAIL với log `WARMUP encoder khong phan hoi` (hoặc `span encoder ... loi`), KHÔNG home ảo, KHÔNG trim chạy loạn, KHÔNG đâm endstop; sau 2 lần thử → `huy chuoi homing`
- [ ] **Endstop J1 nhãn đúng quy ước (đã hoán pin 5↔6)**: Home All → J1 contact chiều ÂM = `CONTACT (MIN)`, chiều DƯƠNG = `CONTACT (MAX)`; jog dương tiến về MAX không FAULT
- [ ] **Warmup probe**: đặt J1 đứng ngay trên endstop rồi Home → log `probe chieu nguoc`, quét tiếp bình thường (không kẹt), không "encoder khong phan hoi" ảo
- [ ] **BACKOFF tự nới**: nếu gặp `cong tac van nhan — noi rong` → lùi 5°/10° nhả được công tắc, quét tiếp; không còn `BACKOFF jammed` thường xuyên
- [ ] Sau khi gia cố encoder J1 (nhảy giá trị phi vật lý: 435.1 → 319.6 khi bước chỉ đi +13.6°): Home All pass, log `WARMUP encDirMult=±1`, `VERIFY OK`
- [ ] **J4 ratio (cần owner xác minh)**: jog J4 một góc biết trước → so Δencoder raw với Δgóc; nếu encoder KHÔNG phải trên trục motor 1:1 (log gợi ý ~5.2× lệch) → quyết định đổi `GEAR_RATIO_J4` + bỏ chia 4 trong `actuatorAngleFromEncoder`/crosscheck

## 4. Planner pen-up travel

- [ ] `/api/move` tới điểm mới — bút **không** hạ xuống Z vẽ
- [ ] `/api/draw` LINE — bút hạ, vẽ, nâng xong mới `planner.active=false`

## 5. Drift watchdog đã tắt

- Drift watchdog đã vô hiệu hóa theo yêu cầu owner; không dùng lệch step/encoder làm tiêu chí phải phát sinh FAULT. Endstop/STOP vẫn phải pass các mục riêng.

## 6. UART direction fail (TMC)

- [ ] Ngắt UART tạm (nếu test được) — jog không chạy sai chiều im lặng

## Ghi chú

Đánh dấu từng mục sau khi pass. Nếu fail, ghi serial log + mô tả vào `docs/IMPLEMENTATION_LOG.md` (entry mới).

## 7. Web command latency

- [ ] Sau khi flash, để robot IDLE; Jog J1 `+0.5°` 10 lần và ghi `Command latency` trên Dashboard. PASS: không lần nào vượt 20 ms; ghi median/max.
- [ ] Với cùng 10 lần Jog, đo thời gian click → motor bắt đầu bằng video slow-motion hoặc logic analyzer STEP. Nếu `commandLatencyUs ≤20 ms` nhưng tổng thời gian lớn, lỗi nằm ngoài queue/motion task (WiFi/browser hoặc motor ramp), không giảm `MOTION_TASK_PERIOD_MS`.
- [ ] Chạy LINE 120 mm và ghi `commandLatencyUs`. Nếu cao hơn Jog rõ rệt, đo thời gian synchronous trajectory preflight trước khi bỏ validation; không di chuyển validation ra khỏi trust boundary.
- [ ] Sau mỗi POST, UI chuyển BUSY không còn chờ tới poll 300 ms tiếp theo và không lóe IDLE giả.

## 8. J5 tại đường vẽ

- [ ] WorkPlane OFF. Đặt J5 đúng zero cơ khí hướng bút đã xác nhận, Set Home J5, rồi Jog `+5°`, `-5°` ba chu kỳ. PASS: `deg`, `encDeg` và chiều quay vật lý cùng dấu, trở lại gần 0°; không đổi `AXIS_ENC_SIGN`/DH offset nếu thiếu ba số đo này.
- [ ] Dry-run LINE với bút cách giấy ≥10 mm. Tại một waypoint ghi `/api/status`: `planner.targetJ5Deg`, `joints[4].deg`, `joints[4].encDeg`. PASS: target/step/encoder hội tụ cùng góc sau segment; không có drift lặp về ±90°.
- [ ] Nếu lệch, ghi thêm raw encoder trước/sau và chiều STEP thực tế; dừng commissioning, không hiệu chỉnh offset để che command/mapping fault.

## 9. WRITE HELLO

- [ ] WorkPlane OFF; chọn profile chính, Start X/Y mặc định và width 120 mm. Tháo bút hoặc nâng giấy cách TCP ≥10 mm trước dry-run đầu tiên.
- [ ] Bấm `WRITE HELLO`; UI chạy đủ `stroke 1/15` → `15/15`, không 409/503/FAULT. Quan sát robot nâng bút giữa các nét và không di chuyển J6.
- [ ] Trong một dry-run khác, bấm ABORT ở khoảng stroke 5. PASS: motor dừng và không nét 6+ nào được enqueue/chạy.
- [ ] Gắn bút, xác nhận mặt giấy đúng Z profile, chạy một LINE 120 mm trước. Chỉ chạy HELLO khi line thẳng, J5 ổn định và pen lift rời giấy hoàn toàn.
- [ ] Viết HELLO ba lần. PASS cuối: đủ năm chữ nhận diện được, không FAULT/out-of-reach, không runaway J5, không bỏ nét; ghi ảnh, Start X/Y/Z/width, command latency median/max và sai lệch endpoint đo được vào IMPLEMENTATION_LOG.


## 10. Teach points và các bản sửa 2026-09-15

- [ ] Sau khi nâng cấp, Save lại A/B/C: bản ghi cũ không có mốc home sẽ bị từ chối. Home đủ sáu khớp, encoder khỏe; thử các pose gần nhau với bút tháo và không gian trống.
- [ ] Play A→B→C: quan sát tăng/giảm tốc êm, không mất bước; kiểm tra góc encoder tại từng pose. Thời gian thực có thêm ramp và overhead UART/timer, không coi 3 giây là deadline chính xác.
- [ ] STOP giữa A→B: mọi motor dừng; C không tự chạy sau đó. Clear Fault trong lúc đang chuyển động không được đổi bộ đếm bước.
- [ ] Save A, đổi mốc Set Home một khớp tại vị trí khác: A không còn hợp lệ. Reboot và thử lại: A vẫn bị từ chối; chỉ Save lại mới chấp nhận hệ tọa độ mới.
- [ ] Save các điểm ở cùng hệ tọa độ, reboot: các slot được phục hồi; thử Play trong không gian trống.
- [ ] Clear và reboot: các slot đã xóa không xuất hiện lại. Khi có lỗi NVS, UI hiển thị lỗi và phản ánh đúng từng slot còn hợp lệ; không coi HTTP 200 enqueue là bằng chứng đã ghi flash thành công.
- [ ] Khởi động thiếu một encoder: khớp đó không được restore home từ góc mặc định 0°. Kiểm tra lại sau khi kết nối encoder ổn định.
- [ ] Với robot đứng yên, ngắt UART rồi yêu cầu Jog TMC đổi chiều: không phát STEP sai chiều. Khôi phục UART và xác nhận chiều thực tế trước khi chạy pose dài.
