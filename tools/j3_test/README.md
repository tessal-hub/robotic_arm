# Chương trình Test riêng động cơ J3 (Elbow Pitch)

Chương trình độc lập này dùng để kiểm tra riêng động cơ **Joint 3 (J3)** mà **không có bất kỳ tầng logic nào** (không kinematics, không endstops, không soft limits, không WiFi/Web, không FreeRTOS sensor/homing task).

## 1. Cấu hình phần cứng giữ nguyên từ config chính

- **STEP Pin:** `GPIO 41`
- **UART1 RX:** `GPIO 16`
- **UART1 TX:** `GPIO 15`
- **TMC2209 Address:** `0b10` (addr 2, jumper MS1=GND, MS2=VIO)
- **Baudrate:** `115200`
- **R_SENSE:** `0.11`
- **Dòng mặc định:** `1700 mA` (có thể chỉnh qua lệnh `curr`)
- **Đảo chiều:** Thuần túy qua UART lệnh `shaft(0)` hoặc `shaft(1)` (không dùng chân DIR)

---

## 2. Lệnh Nạp Firmware Test

Từ terminal (hoặc VS Code PlatformIO):

```powershell
# Biên dịch và nạp vào ESP32-S3
python -m platformio run -d tools/j3_test -t upload

# Mở Serial Monitor xem log và gõ lệnh
python -m platformio device monitor -d tools/j3_test
```

*(Sau khi test xong, nạp lại firmware chính từ thư mục gốc: `python -m platformio run -t upload`)*

---

## 3. Danh sách lệnh Serial (115200 baud)

| Lệnh | Ý nghĩa |
|---|---|
| `+` hoặc `f` | Quay tiến 1600 bước |
| `-` hoặc `r` | Quay lùi 1600 bước |
| `+ 3200` / `f 10000` | Quay tiến đúng số bước chỉ định |
| `- 3200` / `r 10000` | Quay lùi đúng số bước chỉ định |
| `c+` hoặc `cont+` | Quay tiến LIÊN TỤC |
| `c-` hoặc `cont-` | Quay lùi LIÊN TỤC |
| `s` hoặc `x` hoặc Space | DỪNG NGAY LẬP TỨC |
| `speed <us>` | Đổi tốc độ (chu kỳ xung, mặc định 1000 us) |
| `curr <mA>` | Cài dòng điện qua UART (vd: `curr 1500`, `curr 1700`) |
| `shaft 0` / `shaft 1` | Đổi trực tiếp thanh ghi shaft qua UART |
| `status` / `info` | Đọc toàn bộ thanh ghi TMC2209 (version, IFCNT, DRV_STATUS) |
| `scan` | Quét thử cả 4 địa chỉ UART (0..3) tìm driver |
| `pulse <n>` | Phát xung trực tiếp trên chân 41 để đo điện áp/xung |
| `help` hoặc `?` | In hướng dẫn |
