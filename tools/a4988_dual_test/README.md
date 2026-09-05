# A4988 individual-module test

Firmware tối giản cho ESP32-S3 để cô lập hai module A4988 khỏi firmware robot.

- J5: `STEP=GPIO38`, `DIR=GPIO39`
- J6: `STEP=GPIO40`, `DIR=GPIO47`
- Mỗi lệnh phát 200 xung ở 250 steps/s cho đúng một module.

## Nạp và dùng

Từ root repo:

```powershell
& 'C:\Users\laptop\.platformio\penv\Scripts\pio.exe' run -d tools/a4988_dual_test -t upload
& 'C:\Users\laptop\.platformio\penv\Scripts\pio.exe' device monitor -d tools/a4988_dual_test
```

Mở monitor 115200 baud, gửi `5+`, `5-`, `6+`, `6-` hoặc `h`.

## Đấu nối tối thiểu

Module phải có `VMOT` và GND nguồn motor, GND chung với ESP32, `VDD` logic, và `RESET` nối `SLEEP` rồi kéo HIGH. Nếu có chân `ENABLE`, kéo LOW. Tín hiệu ESP32 3.3 V chỉ đáng tin cậy khi VDD logic của A4988 cũng là 3.3 V (hoặc module có level shifter); không cấp VDD logic 5 V rồi giả định 3.3 V là mức HIGH hợp lệ.

Firmware này thay firmware đang chạy trên board. Nạp lại project root sau khi test xong.
