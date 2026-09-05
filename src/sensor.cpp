#include "sensor.h"
#include "rtos_guard.h"
#include <esp_task_wdt.h>

#include <cstring>

Sensor::Sensor() {
    filtered_angles.fill(0.0f);
    last_raw_angles.fill(0.0f);
    accumulated_angles.fill(0.0f);
    turn_counts.fill(0);
    initialized.fill(false);
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        sensor_error[i].store(false, std::memory_order_relaxed);
    }
    read_fail_counts.fill(0);
    for (uint8_t i = 0; i < NUM_SENSORS; ++i) {
        published_angles[i].store(floatToBits(0.0f), std::memory_order_relaxed);
        published_accumulated[i].store(floatToBits(0.0f), std::memory_order_relaxed);
        published_turn_counts[i].store(0, std::memory_order_relaxed);
    }
}

uint32_t Sensor::floatToBits(float value) {
    uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "float must be 32-bit");
    memcpy(&bits, &value, sizeof(bits));
    return bits;
}

float Sensor::bitsToFloat(uint32_t bits) {
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

void Sensor::publishSample(uint8_t ch) {
    published_angles[ch].store(floatToBits(filtered_angles[ch]), std::memory_order_relaxed);
    published_accumulated[ch].store(floatToBits(accumulated_angles[ch]), std::memory_order_relaxed);
    published_turn_counts[ch].store(turn_counts[ch], std::memory_order_relaxed);
}

Sensor::~Sensor() {
    if (taskHandle != nullptr) {
        taskRunning = false;
        vTaskDelete(taskHandle);
        taskHandle = nullptr;
    }
    if (dataMutex != nullptr) {
        vSemaphoreDelete(dataMutex);
        dataMutex = nullptr;
    }
    if (i2cMutex != nullptr) {
        vSemaphoreDelete(i2cMutex);
        i2cMutex = nullptr;
    }
}

bool Sensor::setPCAChannel(uint8_t channel) {
    if (channel >= 8) return false;
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(static_cast<uint8_t>(1 << channel));
    return (Wire.endTransmission() == 0);
}

void Sensor::disableAllPCAChannels() {
    Wire.beginTransmission(PCA_ADDR);
    Wire.write(0x00);
    Wire.endTransmission();
}

void Sensor::recoverI2CBus() {
    Wire.end();
    pinMode(SCL_PIN, OUTPUT);
    pinMode(SDA_PIN, INPUT_PULLUP);

    for (int i = 0; i < 9; i++) {
        digitalWrite(SCL_PIN, HIGH);
        delayMicroseconds(5);
        digitalWrite(SCL_PIN, LOW);
        delayMicroseconds(5);
    }

    pinMode(SDA_PIN, OUTPUT);
    digitalWrite(SDA_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SDA_PIN, HIGH);
    delayMicroseconds(5);

    pinMode(SDA_PIN, INPUT_PULLUP);
    pinMode(SCL_PIN, INPUT_PULLUP);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQUENCY);
    Wire.setTimeOut(50);

    delay(10);
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        setPCAChannel(i);
        configureAS5600();
        delay(1);
    }
}

void Sensor::configureAS5600() {
    uint16_t conf = 0;
    conf |= (static_cast<uint16_t>(AS5600_CONF_PM)   & 0x03) << 0;
    conf |= (static_cast<uint16_t>(AS5600_CONF_HYST) & 0x03) << 2;
    conf |= (static_cast<uint16_t>(AS5600_CONF_OUTS) & 0x03) << 4;
    conf |= (static_cast<uint16_t>(AS5600_CONF_PWMF) & 0x03) << 6;
    conf |= (static_cast<uint16_t>(AS5600_CONF_SF)   & 0x03) << 8;
    conf |= (static_cast<uint16_t>(AS5600_CONF_FTH)  & 0x07) << 10;

    const uint8_t highByte = static_cast<uint8_t>((conf >> 8) & 0xFF);
    const uint8_t lowByte  = static_cast<uint8_t>(conf & 0xFF);

    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_CONF_REG);
    Wire.write(highByte);
    Wire.write(lowByte);
    Wire.endTransmission();
}

uint16_t Sensor::readRaw() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_ANGLE_REG);
    if (Wire.endTransmission(false) != 0) {
        return 0xFFFF;
    }

    if (Wire.requestFrom(static_cast<uint8_t>(AS5600_ADDR), static_cast<uint8_t>(2)) == 2) {
        const uint8_t h = Wire.read();
        const uint8_t l = Wire.read();
        return (static_cast<uint16_t>(h & 0x0F) << 8) | l;
    }
    return 0xFFFF;
}

float Sensor::filter(uint8_t ch, uint16_t raw) {
    const float new_angle = (static_cast<float>(raw) / 4096.0f) * 360.0f;

    if (!initialized[ch]) {
        filtered_angles[ch] = new_angle;
        last_raw_angles[ch] = new_angle;
        accumulated_angles[ch] = new_angle;
        turn_counts[ch] = 0;
        initialized[ch] = true;
        return filtered_angles[ch];
    }

    // Delta with wrap-around compensation
    const float rawDelta = new_angle - last_raw_angles[ch];
    float shortestDelta = rawDelta;
    if (shortestDelta > 180.0f) shortestDelta -= 360.0f;
    if (shortestDelta < -180.0f) shortestDelta += 360.0f;

    if (rawDelta > 180.0f) {
        turn_counts[ch]--;
    } else if (rawDelta < -180.0f) {
        turn_counts[ch]++;
    }
    last_raw_angles[ch] = new_angle;
    accumulated_angles[ch] += shortestDelta;

    // Exponential Smoothing low-pass filter
    float delta = new_angle - filtered_angles[ch];
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;

    filtered_angles[ch] += delta * ALPHA;

    if (filtered_angles[ch] < 0.0f) filtered_angles[ch] += 360.0f;
    if (filtered_angles[ch] >= 360.0f) filtered_angles[ch] -= 360.0f;

    return filtered_angles[ch];
}

void Sensor::scanOnce() {
    auto i2cLock = makeTimedLock(i2cMutex, SENSOR_I2C_MUTEX_TIMEOUT_MS);
    if (!i2cLock) return;

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        if (!setPCAChannel(i)) {
            read_fail_counts[i]++;
            if (read_fail_counts[i] > 20) sensor_error[i].store(true, std::memory_order_relaxed);
            continue;
        }
        const uint16_t raw = readRaw();

        if (raw > 4095) {
            read_fail_counts[i]++;
            if (read_fail_counts[i] > 20) {
                sensor_error[i].store(true, std::memory_order_relaxed);
            }
            continue;
        }

        read_fail_counts[i] = 0;
        sensor_error[i].store(false, std::memory_order_relaxed);

        auto dataLock = makeTimedLock(dataMutex, SENSOR_I2C_MUTEX_TIMEOUT_MS);
        if (dataLock) {
            filter(i, raw);
            publishSample(i);
        }
    }
}

void Sensor::taskEntry(void* param) {
    auto* self = static_cast<Sensor*>(param);
    self->taskLoop();
}

void Sensor::taskLoop() {
    // Đăng ký Task WDT: task quét I2C chạy vô hạn trên Core 0, phải feed watchdog.
    if (esp_task_wdt_add(nullptr) == ESP_OK) {
        wdtRegistered_ = true;
    }

    const TickType_t period = pdMS_TO_TICKS(
        (SENSOR_TASK_PERIOD_MS > 0) ? SENSOR_TASK_PERIOD_MS : 2
    );
    TickType_t lastWake = xTaskGetTickCount();
    uint32_t lastRecoveryMs = millis();

    while (taskRunning) {
        scanOnce();
        if (wdtRegistered_) esp_task_wdt_reset();

        bool allInError = true;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            if (!sensor_error[i].load(std::memory_order_relaxed)) {
                allInError = false;
                break;
            }
        }

        if (allInError) {
            const uint32_t nowMs = millis();
            if (nowMs - lastRecoveryMs >= 5000) {
                lastRecoveryMs = nowMs;
                auto i2cLock = makeTimedLock(i2cMutex, 100);
                if (i2cLock) {
                    recoverI2CBus();
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            lastWake = xTaskGetTickCount();
        } else {
            const TickType_t now = xTaskGetTickCount();
            if ((now - lastWake) > period * 2) {
                lastWake = now;
            }
            vTaskDelayUntil(&lastWake, period);
        }
    }

    vTaskDelete(nullptr);
}

void Sensor::begin(uint8_t coreID, uint8_t priority, uint32_t period_ms) {
    Serial.printf("[SENSOR] begin: SDA=%d SCL=%d freq=%lu Hz timeout=50ms\n",
                  SDA_PIN, SCL_PIN, (unsigned long)I2C_FREQUENCY);

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQUENCY);
    Wire.setTimeOut(50);

    dataMutex = xSemaphoreCreateMutex();
    i2cMutex  = xSemaphoreCreateMutex();

    if (dataMutex == nullptr || i2cMutex == nullptr) {
        Serial.println("[FATAL] Sensor: Failed to create mutexes! Check heap.");
        return;
    }

    // Thử kết nối PCA9548A trực tiếp
    Wire.beginTransmission(PCA_ADDR);
    const uint8_t pcaErr = Wire.endTransmission();
    Serial.printf("[SENSOR] PCA9548A @0x%02X: %s\n",
                  PCA_ADDR, pcaErr == 0 ? "ACK OK" : "NACK/FAIL");

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        const bool pcaOk = setPCAChannel(i);
        // Thử kết nối AS5600 trực tiếp
        Wire.beginTransmission(AS5600_ADDR);
        const uint8_t asErr = Wire.endTransmission();
        Serial.printf("[SENSOR] Ch%d: PCA=%s AS5600=%s\n",
                      i, pcaOk ? "OK" : "FAIL",
                      asErr == 0 ? "ACK OK" : "NACK/FAIL");
        if (pcaOk && asErr == 0) configureAS5600();
        delay(2);
    }

    // Initial synchronous scan: quét lặp 10 lần đồng bộ để tất cả AS5600 qua PCA9548A ổn định góc
    for (uint8_t k = 0; k < 10; ++k) {
        scanOnce();
        delay(5);
    }

    Serial.println("[SENSOR] Init scan kết quả:");
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        Serial.printf("  Ch%d: err=%d fail=%u angle=%.2f\n",
                      i, (int)sensor_error[i].load(std::memory_order_relaxed), (unsigned)read_fail_counts[i],
                      filtered_angles[i]);
    }

    taskRunning = true;
    const BaseType_t taskOk = xTaskCreatePinnedToCore(
        taskEntry,
        "SensorScanTask",
        SENSOR_TASK_STACK_SIZE,
        this,
        priority,
        &taskHandle,
        coreID
    );
    if (taskOk != pdPASS) {
        taskRunning = false;
        taskHandle = nullptr;
        Serial.println("[SENSOR] LOI: khong tao duoc SensorScanTask!");
        return;
    }
    Serial.printf("[SENSOR] Task started on Core %d prio %d\n", coreID, priority);
}

float Sensor::getAngle(uint8_t ch) {
    if (ch >= NUM_SENSORS) return -1.0f;

    // Ưu tiên đọc dưới lock; nếu timeout 5ms vẫn trả giá trị hiện tại
    // (đọc float 32-bit aligned trên ESP32 không bị rách, đừng block caller).
    auto lock = makeTimedLock(dataMutex, 5);
    if (lock) return filtered_angles[ch];
    return bitsToFloat(published_angles[ch].load(std::memory_order_relaxed));
}

float Sensor::getAccumulatedAngle(uint8_t ch) {
    if (ch >= NUM_SENSORS) return 0.0f;

    auto lock = makeTimedLock(dataMutex, 5);
    if (lock) return accumulated_angles[ch];
    return bitsToFloat(published_accumulated[ch].load(std::memory_order_relaxed));
}

int32_t Sensor::getTurnCount(uint8_t ch) {
    if (ch >= NUM_SENSORS) return 0;

    auto lock = makeTimedLock(dataMutex, 5);
    if (lock) return turn_counts[ch];
    return published_turn_counts[ch].load(std::memory_order_relaxed);
}

void Sensor::resetAccumulatedAngle(uint8_t ch) {
    if (ch >= NUM_SENSORS) return;
    auto lock = makeTimedLock(dataMutex, 10);
    if (lock) {
        accumulated_angles[ch] = filtered_angles[ch];
        turn_counts[ch] = 0;
        publishSample(ch);
    }
}

bool Sensor::isSensorOK(uint8_t ch) {
    if (ch >= NUM_SENSORS) return false;
    return !sensor_error[ch].load(std::memory_order_relaxed);
}

AS5600Diag Sensor::getDiagnostics(uint8_t ch) {
    AS5600Diag diag{};
    diag.readSuccess = false;

    if (ch >= NUM_SENSORS || i2cMutex == nullptr) return diag;

    auto lock = makeTimedLock(i2cMutex, 50);
    if (!lock) return diag;

    if (!setPCAChannel(ch)) {
        return diag;
    }

    // 1. Read STATUS (0x0B)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_STATUS_REG);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<uint8_t>(AS5600_ADDR), static_cast<uint8_t>(1)) == 1) {
        diag.status = Wire.read();
        diag.magnetDetected = (diag.status & 0x20) != 0;
        diag.magnetTooLow   = (diag.status & 0x10) != 0;
        diag.magnetTooHigh  = (diag.status & 0x08) != 0;
        diag.readSuccess = true;
    }

    // 2. Read AGC (0x1A)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_AGC_REG);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<uint8_t>(AS5600_ADDR), static_cast<uint8_t>(1)) == 1) {
        diag.agc = Wire.read();
    }

    // 3. Read MAGNITUDE (0x1B, 0x1C)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_MAG_REG);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<uint8_t>(AS5600_ADDR), static_cast<uint8_t>(2)) == 2) {
        const uint8_t h = Wire.read();
        const uint8_t l = Wire.read();
        diag.magnitude = (static_cast<uint16_t>(h & 0x0F) << 8) | l;
    }

    // 4. Read RAW ANGLE (0x0C, 0x0D)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_RAW_REG);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<uint8_t>(AS5600_ADDR), static_cast<uint8_t>(2)) == 2) {
        const uint8_t h = Wire.read();
        const uint8_t l = Wire.read();
        diag.rawAngle = (static_cast<uint16_t>(h & 0x0F) << 8) | l;
    }

    // 5. Read ANGLE (0x0E, 0x0F)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_ANGLE_REG);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<uint8_t>(AS5600_ADDR), static_cast<uint8_t>(2)) == 2) {
        const uint8_t h = Wire.read();
        const uint8_t l = Wire.read();
        diag.angleReg = (static_cast<uint16_t>(h & 0x0F) << 8) | l;
    }

    // 6. Read ZPOS (0x01, 0x02)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_ZPOS_REG);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<uint8_t>(AS5600_ADDR), static_cast<uint8_t>(2)) == 2) {
        const uint8_t h = Wire.read();
        const uint8_t l = Wire.read();
        diag.zpos = (static_cast<uint16_t>(h & 0x0F) << 8) | l;
    }

    // 7. Read MPOS (0x03, 0x04)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_MPOS_REG);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<uint8_t>(AS5600_ADDR), static_cast<uint8_t>(2)) == 2) {
        const uint8_t h = Wire.read();
        const uint8_t l = Wire.read();
        diag.mpos = (static_cast<uint16_t>(h & 0x0F) << 8) | l;
    }

    // 8. Read MANG (0x05, 0x06)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_MANG_REG);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<uint8_t>(AS5600_ADDR), static_cast<uint8_t>(2)) == 2) {
        const uint8_t h = Wire.read();
        const uint8_t l = Wire.read();
        diag.mang = (static_cast<uint16_t>(h & 0x0F) << 8) | l;
    }

    // 9. Read ZMCO (0x00)
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_ZMCO_REG);
    if (Wire.endTransmission(false) == 0 && Wire.requestFrom(static_cast<uint8_t>(AS5600_ADDR), static_cast<uint8_t>(1)) == 1) {
        diag.zmco = Wire.read() & 0x03;
    }

    diag.magnetOptimal = diag.magnetDetected && !diag.magnetTooLow && !diag.magnetTooHigh &&
                         (diag.agc >= AS5600_AGC_MIN_HEALTHY && diag.agc <= AS5600_AGC_MAX_HEALTHY);

    return diag;
}
