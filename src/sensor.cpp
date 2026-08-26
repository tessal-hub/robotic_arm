#include "sensor.h"
#include "rtos_guard.h"
#include <esp_task_wdt.h>

Sensor::Sensor() {
    filtered_angles.fill(0.0f);
    last_raw_angles.fill(0.0f);
    accumulated_angles.fill(0.0f);
    turn_counts.fill(0);
    initialized.fill(false);
    sensor_error.fill(false);
    read_fail_counts.fill(0);
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

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQUENCY);
    Wire.setTimeOut(3);
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
    if (Wire.endTransmission(false) != 0) return 0xFFFF;

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
    float rawDelta = new_angle - last_raw_angles[ch];
    if (rawDelta > 180.0f) {
        rawDelta -= 360.0f;
        turn_counts[ch]--;
    } else if (rawDelta < -180.0f) {
        rawDelta += 360.0f;
        turn_counts[ch]++;
    }
    last_raw_angles[ch] = new_angle;
    accumulated_angles[ch] += rawDelta;

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
            if (read_fail_counts[i] > 5) sensor_error[i] = true;
            continue;
        }
        const uint16_t raw = readRaw();

        if (raw > 4095) {
            read_fail_counts[i]++;
            if (read_fail_counts[i] > 5) {
                sensor_error[i] = true;
            }
            continue;
        }

        read_fail_counts[i] = 0;
        sensor_error[i] = false;

        auto dataLock = makeTimedLock(dataMutex, SENSOR_I2C_MUTEX_TIMEOUT_MS);
        if (dataLock) {
            filter(i, raw);
        }
    }
}

void Sensor::taskEntry(void* param) {
    auto* self = static_cast<Sensor*>(param);
    self->taskLoop();
}

void Sensor::taskLoop() {
    const TickType_t period = pdMS_TO_TICKS(
        (SENSOR_TASK_PERIOD_MS > 0) ? SENSOR_TASK_PERIOD_MS : 2
    );
    TickType_t lastWake = xTaskGetTickCount();

    // Register Task Watchdog Timer
    esp_task_wdt_add(nullptr);

    while (taskRunning) {
        esp_task_wdt_reset();
        scanOnce();

        bool allInError = true;
        for (uint8_t i = 0; i < NUM_SENSORS; i++) {
            if (!sensor_error[i]) {
                allInError = false;
                break;
            }
        }

        if (allInError) {
            auto i2cLock = makeTimedLock(i2cMutex, 100);
            if (i2cLock) {
                recoverI2CBus();
            }
            vTaskDelay(pdMS_TO_TICKS(50));
            lastWake = xTaskGetTickCount();
        } else {
            vTaskDelayUntil(&lastWake, period);
        }
    }

    esp_task_wdt_delete(nullptr);
    vTaskDelete(nullptr);
}

void Sensor::begin(uint8_t coreID, uint8_t priority, uint32_t period_ms) {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_FREQUENCY);
    Wire.setTimeOut(3);

    dataMutex = xSemaphoreCreateMutex();
    i2cMutex  = xSemaphoreCreateMutex();

    if (dataMutex == nullptr || i2cMutex == nullptr) {
        Serial.println("[FATAL] Sensor: Failed to create mutexes! Check heap.");
        return;
    }

    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
        setPCAChannel(i);
        configureAS5600();
        delay(2);
    }

    // Initial synchronous scan
    scanOnce();

    taskRunning = true;
    xTaskCreatePinnedToCore(
        taskEntry,
        "SensorScanTask",
        SENSOR_TASK_STACK_SIZE,
        this,
        priority,
        &taskHandle,
        coreID
    );
}

float Sensor::getAngle(uint8_t ch) {
    if (ch >= NUM_SENSORS) return -1.0f;

    auto lock = makeTimedLock(dataMutex, 5);
    return filtered_angles[ch];
}

float Sensor::getAccumulatedAngle(uint8_t ch) {
    if (ch >= NUM_SENSORS) return 0.0f;

    auto lock = makeTimedLock(dataMutex, 5);
    return accumulated_angles[ch];
}

int32_t Sensor::getTurnCount(uint8_t ch) {
    if (ch >= NUM_SENSORS) return 0;

    auto lock = makeTimedLock(dataMutex, 5);
    return turn_counts[ch];
}

void Sensor::resetAccumulatedAngle(uint8_t ch) {
    if (ch >= NUM_SENSORS) return;
    auto lock = makeTimedLock(dataMutex, 10);
    if (lock) {
        accumulated_angles[ch] = filtered_angles[ch];
        turn_counts[ch] = 0;
    }
}

bool Sensor::isSensorOK(uint8_t ch) {
    if (ch >= NUM_SENSORS) return false;
    return !sensor_error[ch];
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