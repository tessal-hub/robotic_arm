# Test một TMC2209 — địa chỉ UART 0

Thư mục giữ tên uart_loopback, nhưng chương trình đã thay bằng test driver thật.
Tắt nguồn trước khi đổi dây. Tháo cầu nối loopback; chỉ nối một TMC2209:

```text
ESP32 GPIO16 TX ── 1kΩ ──┬── TMC PDN_UART
ESP32 GPIO15 RX ─────────┘
ESP32 GND ────────────────── TMC GND
```

MS1=LOW, MS2=LOW (địa chỉ 0). Cấp VM và VIO đúng cho module, GND chung.
Chân/baud lấy từ src/config.h: hiện Serial1, RX15/TX16, 115200.
Driver được bật dòng giữ; test giữ các STEP cấu hình ở LOW, không phát bước.

```powershell
pio run -d tools/uart_loopback -t upload
pio device monitor -d tools/uart_loopback
```

Init theo standalone cũ: toff=4, pdn_disable, I_scale_analog=false,
mstep_reg_select, 700 mA, microstep=16, SpreadCycle, ihold=8, iholddelay=10.
Không init WiFi/sensor hoặc ghi NVS. Giao dịch UART dùng mutex timeout 50 ms.

- `[TMC WRITE]`: IFCNT trước/sau một lần ghi lại pdn_disable(true), delta kỳ vọng 1 (modulo 256), chỉ có ý nghĩa khi cả hai readError=0.
- `[TMC READ]`: version mỗi giây; PASS khi version=0x21 và readError=0. readError gộp timeout/CRC, không xác định riêng nguyên nhân.

Sau test, nạp lại firmware chính từ thư mục gốc:
`pio run -e esp32-s3-devkitc-1 -t upload`.
