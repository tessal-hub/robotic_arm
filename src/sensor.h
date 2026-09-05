#ifndef SENSOR_H
#define SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <array>
#include <atomic>
#include "config.h"

// AS5600 Register Map
constexpr uint8_t AS5600_ZMCO_REG    = 0x00;
constexpr uint8_t AS5600_ZPOS_REG    = 0x01;
constexpr uint8_t AS5600_MPOS_REG    = 0x03;
constexpr uint8_t AS5600_MANG_REG    = 0x05;
constexpr uint8_t AS5600_CONF_REG    = 0x07;   // 2 byte: 0x07 = high, 0x08 = low
constexpr uint8_t AS5600_STATUS_REG  = 0x0B;
constexpr uint8_t AS5600_RAW_REG     = 0x0C;
constexpr uint8_t AS5600_ANGLE_REG   = 0x0E;   // Hardware-filtered angle
constexpr uint8_t AS5600_AGC_REG     = 0x1A;
constexpr uint8_t AS5600_MAG_REG     = 0x1B;

// Hardware Filter & Power Configuration
constexpr uint8_t AS5600_CONF_PM     = 0b00;   // Power mode: NOM (Normal)
constexpr uint8_t AS5600_CONF_HYST   = 0b10;   // Hysteresis: 2LSB
constexpr uint8_t AS5600_CONF_OUTS   = 0b00;   // Output stage: analog
constexpr uint8_t AS5600_CONF_PWMF   = 0b00;   // PWM freq: off
constexpr uint8_t AS5600_CONF_SF     = 0b00;   // Slow filter: 16x
constexpr uint8_t AS5600_CONF_FTH    = 0b000;  // Fast filter threshold: slow filter only

// Cấu trúc thông tin chẩn đoán phần cứng AS5600
struct AS5600Diag {
    uint8_t status{0};        // Reg 0x0B: STATUS
    bool magnetDetected{false};   // Bit 5: MD
    bool magnetTooLow{false};     // Bit 4: ML
    bool magnetTooHigh{false};    // Bit 3: MH
    bool magnetOptimal{false};    // AGC và khoảng cách tối ưu
    uint8_t agc{0};           // Reg 0x1A: AGC (0..255)
    uint16_t magnitude{0};    // Reg 0x1B, 0x1C: CORDIC magnitude (0..4095)
    uint16_t rawAngle{0};     // Reg 0x0C, 0x0D: Raw angle (0..4095)
    uint16_t angleReg{0};     // Reg 0x0E, 0x0F: Hardware filtered angle (0..4095)
    uint16_t zpos{0};         // Reg 0x01, 0x02: Zero position
    uint16_t mpos{0};         // Reg 0x03, 0x04: Max position
    uint16_t mang{0};         // Reg 0x05, 0x06: Max angle
    uint8_t zmco{0};          // Reg 0x00: Burn count (0 = chua burn)
    bool readSuccess{false};  // Đọc thành công
};

class Sensor {
private:
    static constexpr float ALPHA = 0.2f;
    std::array<float, NUM_SENSORS> filtered_angles{};
    std::array<float, NUM_SENSORS> last_raw_angles{};
    std::array<float, NUM_SENSORS> accumulated_angles{};
    std::array<int32_t, NUM_SENSORS> turn_counts{};
    std::array<bool, NUM_SENSORS> initialized{};
    std::array<std::atomic<bool>, NUM_SENSORS> sensor_error{};
    std::array<uint32_t, NUM_SENSORS> read_fail_counts{};

    // Lock-free fallback snapshots for readers that cannot obtain dataMutex
    // within their bounded timeout. uint32_t atomics are lock-free on ESP32-S3.
    std::array<std::atomic<uint32_t>, NUM_SENSORS> published_angles{};
    std::array<std::atomic<uint32_t>, NUM_SENSORS> published_accumulated{};
    std::array<std::atomic<int32_t>, NUM_SENSORS> published_turn_counts{};

    SemaphoreHandle_t dataMutex{nullptr};        // Bảo vệ mảng góc khi task ghi / main đọc
    SemaphoreHandle_t i2cMutex{nullptr};         // Bảo vệ I2C bus khi đọc chẩn đoán từ task khác
    TaskHandle_t taskHandle{nullptr};
    volatile bool taskRunning{false};
    bool wdtRegistered_{false};                  // Task WDT đã đăng ký thành công trong taskLoop

    bool setPCAChannel(uint8_t channel);
    void disableAllPCAChannels();
    uint16_t readRaw();
    float filter(uint8_t ch, uint16_t raw);
    void publishSample(uint8_t ch);
    [[nodiscard]] static uint32_t floatToBits(float value);
    [[nodiscard]] static float bitsToFloat(uint32_t bits);
    void scanOnce();                    // Quét toàn bộ NUM_SENSORS 1 lần
    void configureAS5600();             // Ghi CONF register
    void recoverI2CBus();               // Khôi phục I2C bus khi bị treo

    static void taskEntry(void* param);
    void taskLoop();

public:
    Sensor();
    ~Sensor();

    // Disable copy semantics
    Sensor(const Sensor&) = delete;
    Sensor& operator=(const Sensor&) = delete;

    void begin(uint8_t coreID = SENSOR_TASK_CORE,
               uint8_t priority = SENSOR_TASK_PRIORITY,
               uint32_t period_ms = SENSOR_TASK_PERIOD_MS);

    [[nodiscard]] float getAngle(uint8_t ch = 0);
    [[nodiscard]] float getAccumulatedAngle(uint8_t ch = 0);
    [[nodiscard]] int32_t getTurnCount(uint8_t ch = 0);
    void resetAccumulatedAngle(uint8_t ch = 0);

    [[nodiscard]] bool isSensorOK(uint8_t ch = 0);
    [[nodiscard]] AS5600Diag getDiagnostics(uint8_t ch = 0);
};

#endif // SENSOR_H
