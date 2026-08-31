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

## 5. Drift FAULT

- [ ] (Nếu có điều kiện) lệch step/encoder > 5° → FAULT, motion bị chặn

## 6. UART direction fail (TMC)

- [ ] Ngắt UART tạm (nếu test được) — jog không chạy sai chiều im lặng

## Ghi chú

Đánh dấu từng mục sau khi pass. Nếu fail, ghi serial log + mô tả vào `docs/IMPLEMENTATION_LOG.md` (entry mới).
