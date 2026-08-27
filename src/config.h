#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==============================================================================
// 1. SYSTEM & FIRMWARE METADATA
// ==============================================================================
#define FW_NAME                                 "NEMA-6AXIS-ARM-CONTROLLER"
#define FW_VERSION                              "2.5.0-PRO"
#define FW_BUILD_DATE                           __DATE__ " " __TIME__

// ==============================================================================
// 2. HARDWARE PINOUT (ESP32-S3 DevKitC-1) — BẢNG CHỐT 2026-08-26
//    Loại trừ: GPIO4 (bootloop), GPIO0/3/45/46 (strapping), GPIO19/20 (USB-OTG),
//              GPIO26-32 (flash nội bộ). Dự phòng trống: GPIO13, 14, 17, 18, 48.
//    KHÔNG có chân EN — mọi driver luôn ở trạng thái enabled.
// ==============================================================================
// Cụm 1: Joint 1 -> Joint 4 (TMC2209, chung Serial1: GPIO 15 RX / GPIO 16 TX)
//         Đảo chiều qua UART (shaft register), KHÔNG dùng chân DIR.
constexpr int8_t RX_PIN_1                       = 15;
constexpr int8_t TX_PIN_1                       = 16;
#define SERIAL_PORT_1                           Serial1

// Cụm 2: Joint 5, 6 dùng A4988 (STEP+DIR, không UART)

constexpr uint32_t TMC_UART_BAUD                = 115200;
constexpr float R_SENSE                         = 0.11f;

// Địa chỉ UART trên bus chung (do jumper MS1/MS2 trên board driver)
constexpr uint8_t UART_ADDR_J1                  = 0b00;
constexpr uint8_t UART_ADDR_J2                  = 0b01;
constexpr uint8_t UART_ADDR_J3                  = 0b10;
constexpr uint8_t UART_ADDR_J4                  = 0b11;

// Sentinel "không có chân" (TMC2209 đảo chiều bằng UART shaft)
constexpr uint8_t PIN_UNSET                     = 255;

// Hardware STEP / DIR Pins for 6 Stepper Motors
constexpr uint8_t STEP_PIN_0                    = 1;   // Motor 0 (Joint 1 - Base Yaw)       TMC2209 addr 0b00
constexpr uint8_t STEP_PIN_1                    = 2;   // Motor 1 (Joint 2 - Shoulder Pitch) TMC2209 addr 0b01
constexpr uint8_t STEP_PIN_2                    = 41;  // Motor 2 (Joint 3 - Elbow Pitch)    TMC2209 addr 0b10
constexpr uint8_t STEP_PIN_3                    = 42;  // Motor 3 (Joint 4 - Wrist Pan)      TMC2209 addr 0b11

constexpr uint8_t STEP_PIN_4                    = 38;  // Motor 4 (Joint 5 - Wrist Tilt) - A4988
constexpr uint8_t DIR_PIN_4                     = 39;

constexpr uint8_t STEP_PIN_5                    = 40;  // Motor 5 (Joint 6 - Flange Roll) - A4988
constexpr uint8_t DIR_PIN_5                     = 47;

constexpr uint8_t NUM_MOTORS                    = 6;
constexpr uint8_t MAX_WAYPOINTS                 = 32;

// Chiều quay logic: +1 nếu step CW ứng với góc khớp tăng dương (hiệu chỉnh lúc lắp).
// J2/J3/J4 quay ngược chiều motor (gear âm trước đây) -> bây giờ gear bỏ dấu, dời dấu vào đây.
constexpr int8_t AXIS_STEP_SIGN[NUM_MOTORS]     = { +1, -1, -1, -1, +1, +1 };
// Chiều đo của encoder AS5600 so với góc khớp dương (hiệu chỉnh lúc lắp).
constexpr int8_t AXIS_ENC_SIGN[NUM_MOTORS]      = { +1, +1, +1, +1, +1, +1 };

// ==============================================================================
// 3. I2C SENSOR BUS (PCA9548A Multiplexer + AS5600 Magnetic Encoders)
// ==============================================================================
constexpr int8_t SDA_PIN                        = 8;
constexpr int8_t SCL_PIN                        = 9;
constexpr uint32_t I2C_FREQUENCY                = 800000; // 800kHz Fast-Mode Plus

constexpr uint8_t PCA_ADDR                      = 0x70;
constexpr uint8_t AS5600_ADDR                   = 0x36;
constexpr uint8_t NUM_SENSORS                   = 6;

// Sensor Health & Diagnostics Constants
constexpr uint8_t AS5600_AGC_MIN_HEALTHY        = 30;
constexpr uint8_t AS5600_AGC_MAX_HEALTHY        = 225;

// Sensor Task configuration (Core 0, 200Hz)
constexpr uint32_t SENSOR_TASK_PERIOD_MS        = 5;
constexpr uint32_t SENSOR_TASK_STACK_SIZE       = 4096;
constexpr UBaseType_t SENSOR_TASK_PRIORITY      = 4;
constexpr BaseType_t SENSOR_TASK_CORE           = 0;
constexpr uint32_t SENSOR_I2C_MUTEX_TIMEOUT_MS  = 10;
constexpr uint32_t WDT_TIMEOUT_SEC              = 5;

// ==============================================================================
// 4. ENDSTOPS / LIMIT SWITCHES (J1, J2, J3)
// ==============================================================================
constexpr uint8_t ENDSTOP_ACTIVE_STATE          = LOW;    // Pull-up active low

constexpr int8_t J1_MIN_PIN                     = 5;
constexpr int8_t J1_MAX_PIN                     = 6;
constexpr int8_t J2_MIN_PIN                     = 7;
constexpr int8_t J2_MAX_PIN                     = 10;
constexpr int8_t J3_MIN_PIN                     = 11;
constexpr int8_t J3_MAX_PIN                     = 12;

constexpr int AXIS_MIN_PINS[NUM_MOTORS] = { J1_MIN_PIN, J2_MIN_PIN, J3_MIN_PIN, -1, -1, -1 };
constexpr int AXIS_MAX_PINS[NUM_MOTORS] = { J1_MAX_PIN, J2_MAX_PIN, J3_MAX_PIN, -1, -1, -1 };

// ==============================================================================
// 5. MOTION & CLOSED-LOOP CONTROL DEFAULTS
// ==============================================================================
// Tỉ số truyền độc lập cho từng khớp J1..J6 (Gear Ratios):
constexpr float GEAR_RATIO_J1                   = 6.0f;   // Joint 1 (Base Yaw) - 6:1
constexpr float GEAR_RATIO_J2                   = 20.0f;  // Joint 2 (Shoulder Pitch) - 20:1
constexpr float GEAR_RATIO_J3                   = 20.0f;  // Joint 3 (Elbow Pitch) - 20:1
constexpr float GEAR_RATIO_J4                   = 4.0f;   // Joint 4 (Wrist Roll) - 4:1
constexpr float GEAR_RATIO_J5                   = 3.0f;   // Joint 5 (Wrist Pitch) - 3:1
constexpr float GEAR_RATIO_J6                   = 3.0f;   // Joint 6 (Flange Roll) - 3:1

constexpr float DEFAULT_AXIS_GEAR_RATIOS[NUM_MOTORS] = {
    GEAR_RATIO_J1,
    GEAR_RATIO_J2,
    GEAR_RATIO_J3,
    GEAR_RATIO_J4,
    GEAR_RATIO_J5,
    GEAR_RATIO_J6
};

constexpr float DEFAULT_GEAR_RATIO              = GEAR_RATIO_J1;
constexpr uint16_t DEFAULT_FULL_STEPS           = 200;    // 1.8 degree stepper (200 steps/rev)
constexpr uint16_t DEFAULT_MICROSTEPS           = 16;     // 1/16 microstepping
constexpr uint16_t DEFAULT_NORMAL_CURRENT       = 800;    // Normal running current (mA)
constexpr uint16_t DEFAULT_HOMING_CURRENT       = 350;    // Ultra-low current for Homing (mA)

constexpr uint16_t HOMING_CURRENT_J1            = 800;    // Base Yaw (mA) — cần traverse full range, ISR endstop bảo vệ chạm
constexpr uint16_t HOMING_CURRENT_J2            = 1000;    // Shoulder Pitch (mA)
constexpr uint16_t HOMING_CURRENT_J3            = 1000;    // Elbow Pitch (mA)
constexpr uint16_t HOMING_CURRENT_J4            = 300;    // Wrist Roll (mA)
constexpr uint16_t HOMING_CURRENT_J5            = 0;      // A4988 - VREF cứng
constexpr uint16_t HOMING_CURRENT_J6            = 0;      // A4988 - VREF cứng

constexpr uint16_t DEFAULT_AXIS_HOMING_CURRENTS[NUM_MOTORS] = {
    HOMING_CURRENT_J1,
    HOMING_CURRENT_J2,
    HOMING_CURRENT_J3,
    HOMING_CURRENT_J4,
    HOMING_CURRENT_J5,
    HOMING_CURRENT_J6
};

constexpr uint32_t HOMING_STEP_INTERVAL_J1      = 1800;   // us/step
constexpr uint32_t HOMING_STEP_INTERVAL_J2      = 1500;
constexpr uint32_t HOMING_STEP_INTERVAL_J3      = 1500;
constexpr uint32_t HOMING_STEP_INTERVAL_J4      = 1800;
constexpr uint32_t HOMING_STEP_INTERVAL_J5      = 1800;
constexpr uint32_t HOMING_STEP_INTERVAL_J6      = 1800;

constexpr uint32_t DEFAULT_AXIS_HOMING_SPEEDS[NUM_MOTORS] = {
    HOMING_STEP_INTERVAL_J1,
    HOMING_STEP_INTERVAL_J2,
    HOMING_STEP_INTERVAL_J3,
    HOMING_STEP_INTERVAL_J4,
    HOMING_STEP_INTERVAL_J5,
    HOMING_STEP_INTERVAL_J6
};

constexpr uint8_t DEFAULT_HOLD_SCALE            = 30;     // TMC2209 ihold scale
constexpr uint16_t DEFAULT_STALL_THRESHOLD      = 0;      // StallGuard threshold (0 = disabled)

// Homing fusion: endstop + StallGuard + encoder + step counting
constexpr float HOME_BACKOFF_DEG                = 2.0f;   // Lùi ra sau khi chạm endstop/stall
constexpr uint16_t STALL_SG_LEVEL               = 100;    // SG_RESULT < mức này => nghi stall (tune thực tế)
constexpr uint8_t STALL_CONSECUTIVE_POLLS       = 3;      // Poll liên tiếp xác nhận stall
constexpr uint32_t HOMING_JOINT_TIMEOUT_MS      = 30000;  // Timeout tối đa cho 1 khớp
constexpr uint32_t HOMING_POLL_MS               = 5;      // Chu kỳ giám sát endstop/stall trong FSM

// Speed & Acceleration Timing (microseconds per step pulse)
constexpr uint32_t DEFAULT_STEP_INTERVAL_US     = 400;    // Target step interval -> 2500 steps/sec
constexpr uint32_t HOMING_STEP_INTERVAL_US      = 1600;   // Fallback homing speed
constexpr uint32_t MIN_STEP_INTERVAL_US         = 120;    // Max speed limit -> ~8333 steps/sec
constexpr uint32_t MAX_STEP_INTERVAL_US         = 2500;   // Starting speed interval
constexpr uint32_t DEFAULT_ACCEL_RATE           = 12;

// Schmitt-Trigger Deadband for closed-loop holding
constexpr float DEFAULT_DEADBAND_ENTER          = 0.3f;   // Enter holding window (degrees)
constexpr float DEFAULT_DEADBAND_EXIT           = 0.8f;   // Exit holding window (degrees)
constexpr float DEFAULT_ANGLE_TOLERANCE         = 0.5f;   // Tolerance (degrees)
constexpr float RUNAWAY_ERROR_THRESHOLD         = 5.0f;   // Runaway threshold (degrees)

// Drawing (Cartesian trajectory, pen = tool coaxial J6)
constexpr float PEN_LIFT_MM                     = 5.0f;   // Độ nâng bút giữa các nét (Z-raise)
constexpr float DRAW_FEED_MM_S                  = 20.0f;  // Feed mặc định khi vẽ
constexpr float DRAW_SEGMENT_MM                 = 1.0f;   // Bước rời rạc hóa quỹ đạo
constexpr uint8_t PLANNER_QUEUE_DEPTH           = 24;     // Số segment tối đa trong hàng đợi

// Motion Control Task configuration (Core 1, 100Hz)
constexpr uint32_t MOTION_TASK_PERIOD_MS        = 10;
constexpr uint32_t MOTION_TASK_STACK_SIZE       = 5120;
constexpr UBaseType_t MOTION_TASK_PRIORITY      = 3;
constexpr BaseType_t MOTION_TASK_CORE           = 1;

// ==============================================================================
// 6. DH PARAMETERS & ROBOT GEOMETRY (docs/ARM_GEOMETRY.md — SINGLE SOURCE OF TRUTH)
// ==============================================================================
constexpr float DH_D1_MM                        = 139.0f; // J1 -> J2: Chiều cao đế lên vai (Base height)
constexpr float DH_A2_MM                        = 138.0f; // J2 -> J3: Cánh tay trên (Upper arm length)
constexpr float DH_A3_MM                        = 88.0f;  // J3 -> Điểm gập (Elbow longitudinal offset)
constexpr float DH_D4_MM                        = 126.0f; // Điểm gập -> Cụm cổ tay (16mm + 110mm = 126mm wrist offset)
constexpr float DH_D6_TOOL_MM                   = 20.0f;  // Bút (tool), gắn đồng trục với J6 (20mm từ tâm cổ tay)

// Angle Offsets: theta_DH = theta_encoder + OFFSET
constexpr float DH_THETA1_OFFSET_DEG            = 0.0f;
constexpr float DH_THETA2_OFFSET_DEG            = -90.0f; // Khớp vai lệch -90 độ khi encoder = 0
constexpr float DH_THETA3_OFFSET_DEG            = 0.0f;   // Khớp khuỷu thẳng đứng khi encoder = 0
constexpr float DH_THETA4_OFFSET_DEG            = 0.0f;
constexpr float DH_THETA5_OFFSET_DEG            = 0.0f;
constexpr float DH_THETA6_OFFSET_DEG            = 0.0f;

// Joint Soft Angle Limits (Degrees relative to Calibrated Home)
constexpr float J1_MIN_LIMIT                    = -90.0f; // 270 deg total stroke
constexpr float J1_MAX_LIMIT                    = +90.0f;
constexpr float J2_MIN_LIMIT                    = -90.0f;  // 180 deg total stroke
constexpr float J2_MAX_LIMIT                    = +90.0f;
constexpr float J3_MIN_LIMIT                    = 0.0f;   // Khuỷu 1 chiều: không gập âm (0 = duỗi thẳng)
constexpr float J3_MAX_LIMIT                    = +90.0f;
constexpr float J4_MIN_LIMIT                    = -180.0f;
constexpr float J4_MAX_LIMIT                    = +180.0f;
constexpr float J5_MIN_LIMIT                    = -120.0f;
constexpr float J5_MAX_LIMIT                    = +120.0f;
constexpr float J6_MIN_LIMIT                    = -360.0f;
constexpr float J6_MAX_LIMIT                    = +360.0f;

constexpr float DEFAULT_AXIS_LIMIT_MIN[NUM_MOTORS] = {
    J1_MIN_LIMIT, J2_MIN_LIMIT, J3_MIN_LIMIT, J4_MIN_LIMIT, J5_MIN_LIMIT, J6_MIN_LIMIT
};

constexpr float DEFAULT_AXIS_LIMIT_MAX[NUM_MOTORS] = {
    J1_MAX_LIMIT, J2_MAX_LIMIT, J3_MAX_LIMIT, J4_MAX_LIMIT, J5_MAX_LIMIT, J6_MAX_LIMIT
};

constexpr float DEFAULT_AXIS_CALIB_RANGE[NUM_MOTORS] = {
    (J1_MAX_LIMIT - J1_MIN_LIMIT),
    (J2_MAX_LIMIT - J2_MIN_LIMIT),
    (J3_MAX_LIMIT - J3_MIN_LIMIT),
    (J4_MAX_LIMIT - J4_MIN_LIMIT),
    (J5_MAX_LIMIT - J5_MIN_LIMIT),
    (J6_MAX_LIMIT - J6_MIN_LIMIT)
};

// ==============================================================================
// 7. NETWORKING & SYSTEM CONFIGURATION
// ==============================================================================
#define DEFAULT_AP_SSID                         "6AXIS-CONTROLLER"
#define DEFAULT_AP_PASS                         "12345678"
#define DEFAULT_MDNS_HOST                       "robot-arm"
constexpr uint16_t WEB_SERVER_PORT              = 80;
constexpr uint32_t WIFI_STA_BOOT_TIMEOUT_MS     = 6000;

#endif // CONFIG_H