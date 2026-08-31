#include "motor.h"
#include "driver/gpio.h"
#include "rtos_guard.h"
#ifdef ARDUINO
#include "safety_manager.h"
#endif

Motor::Motor(HardwareSerial* serial, float rSense, uint8_t uartAddress,
             uint8_t stepPinNum, uint8_t dirPinNum, const char* motorLabel)
    : serialPort(serial),
      uartMutex(nullptr),
      driver(serial ? new TMC2209Stepper(serial, rSense, uartAddress) : nullptr),
      isTMC(serial != nullptr), // Xác định loại driver
      stepPin(stepPinNum),
      dirPin(dirPinNum),
      address(uartAddress),
      label(motorLabel),
      running(false),
      dirCW(true),
      lastShaftDir(-1),
      targetSpeedUs(DEFAULT_STEP_INTERVAL_US),
      currentSpeedUs(MAX_STEP_INTERVAL_US),
      startSpeedUs(MAX_STEP_INTERVAL_US),
      targetSteps(0),
      stepsRemaining(0),
      stepCounter(0),
      accelSteps(60),
      decelSteps(60),
      continuousMode(false),
      coordinatedMode(false),
      preparedRun(false),
      currentMa(DEFAULT_NORMAL_CURRENT),
      spreadCycleMode(true),
      microstepsVal(DEFAULT_MICROSTEPS),
      holdScale(DEFAULT_HOLD_SCALE),
      enabled(true),
      uartOk(false),
      driverVersion(0),
      stepTimer(nullptr)
{
    isHighPin = (stepPinNum >= 32);
    if (isHighPin) {
        stepPinMaskLow = 0;
        stepPinMaskHigh = (1UL << (stepPinNum - 32));
    } else {
        stepPinMaskLow = (1UL << stepPinNum);
        stepPinMaskHigh = 0;
    }
}

Motor::~Motor() {
    if (stepTimer != nullptr) {
        esp_timer_stop(stepTimer);
        esp_timer_delete(stepTimer);
        stepTimer = nullptr;
    }
}

inline uint32_t Motor::calculateSCurveInterval(uint32_t currentStep, uint32_t totalSteps,
                                               uint32_t startInterval, uint32_t targetInterval) {
    if (totalSteps == 0 || currentStep >= totalSteps) return targetInterval;
    if (startInterval == targetInterval) return targetInterval;

    const float x = static_cast<float>(currentStep) / static_cast<float>(totalSteps);
    const float s = x * x * (3.0f - 2.0f * x);

    const float vStart = 1000000.0f / static_cast<float>(startInterval);
    const float vTarget = 1000000.0f / static_cast<float>(targetInterval);
    const float vCurrent = vStart + s * (vTarget - vStart);

    if (vCurrent <= 1.0f) return MAX_STEP_INTERVAL_US;
    uint32_t interval = static_cast<uint32_t>(1000000.0f / vCurrent);
    if (interval < MIN_STEP_INTERVAL_US) interval = MIN_STEP_INTERVAL_US;
    if (interval > MAX_STEP_INTERVAL_US) interval = MAX_STEP_INTERVAL_US;
    return interval;
}

void IRAM_ATTR Motor::onStepTimer(void* arg) {
    Motor* self = static_cast<Motor*>(arg);
    if (!self->running.load(std::memory_order_acquire)) return;

    // Fail-fast emergency stop check (Instantaneous abort <= 20us) — SafetyManager single owner
#ifdef ARDUINO
    if (self->safety_ && self->safety_->isEStop()) {
        if (self->isHighPin) GPIO.out1_w1tc.val = self->stepPinMaskHigh;
        else GPIO.out_w1tc = self->stepPinMaskLow;
        self->running.store(false, std::memory_order_release);
        return;
    }
#endif

    gpio_set_level(static_cast<gpio_num_t>(self->stepPin), 1);
    esp_rom_delay_us(4); // 4us clean pulse width cho A4988 và TMC
    gpio_set_level(static_cast<gpio_num_t>(self->stepPin), 0);

    uint32_t nextInterval = self->targetSpeedUs.load(std::memory_order_relaxed);
    const bool isCont = self->continuousMode.load(std::memory_order_relaxed);
    uint32_t remaining = self->stepsRemaining.load(std::memory_order_relaxed);

    if (!isCont && remaining > 0 && remaining != 0xFFFFFFFF) {
        const uint32_t counter = self->stepCounter.fetch_add(1, std::memory_order_relaxed) + 1;
        remaining = self->stepsRemaining.fetch_sub(1, std::memory_order_relaxed) - 1;
        self->absSteps.fetch_add(self->dirCW.load(std::memory_order_relaxed) ? 1 : -1,
                                 std::memory_order_relaxed);

        const bool coordinated = self->coordinatedMode.load(std::memory_order_relaxed);
        if (!coordinated && counter < self->accelSteps) {
            nextInterval = calculateSCurveInterval(counter, self->accelSteps, self->startSpeedUs, self->targetSpeedUs.load(std::memory_order_relaxed));
        } else if (!coordinated && remaining <= self->decelSteps) {
            nextInterval = calculateSCurveInterval(remaining, self->decelSteps, self->startSpeedUs, self->targetSpeedUs.load(std::memory_order_relaxed));
        }

        self->currentSpeedUs.store(nextInterval, std::memory_order_relaxed);

        if (remaining == 0) {
            self->running.store(false, std::memory_order_release);
            return;
        }
    } else if (isCont) {
        const uint32_t counter = self->stepCounter.fetch_add(1, std::memory_order_relaxed) + 1;
        self->absSteps.fetch_add(self->dirCW.load(std::memory_order_relaxed) ? 1 : -1,
                                 std::memory_order_relaxed);
        if (counter < self->accelSteps) {
            nextInterval = calculateSCurveInterval(counter, self->accelSteps, self->startSpeedUs, self->targetSpeedUs.load(std::memory_order_relaxed));
        }
        self->currentSpeedUs.store(nextInterval, std::memory_order_relaxed);
    }

    if (nextInterval < MIN_STEP_INTERVAL_US) nextInterval = MIN_STEP_INTERVAL_US;
    if (!self->coordinatedMode.load(std::memory_order_relaxed) &&
        nextInterval > MAX_STEP_INTERVAL_US) {
        nextInterval = MAX_STEP_INTERVAL_US;
    }

    bool estop = false;
#ifdef ARDUINO
    estop = (self->safety_ && self->safety_->isEStop());
#endif
    if (!self->running.load(std::memory_order_acquire) || estop) {
        self->running.store(false, std::memory_order_release);
        return;
    }

    esp_timer_start_once(self->stepTimer, nextInterval);
}

bool Motor::takeUart(uint32_t timeoutMs) {
    if (!isTMC) return true;
    // A TMC transaction without the shared mutex can corrupt another driver's
    // datagram. Missing startup synchronization therefore fails closed.
    if (uartMutex == nullptr || *uartMutex == nullptr) return false;
    if (timeoutMs == portMAX_DELAY) return xSemaphoreTake(*uartMutex, portMAX_DELAY) == pdTRUE;
    return xSemaphoreTake(*uartMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

void Motor::giveUart() {
    if (isTMC && uartMutex != nullptr && *uartMutex != nullptr) xSemaphoreGive(*uartMutex);
}

void Motor::flushUartRx() {
    if (isTMC && serialPort != nullptr) {
        while (serialPort->available()) serialPort->read();
    }
}

bool Motor::testUART() {
    if (!isTMC) return true; // A4988 ko có UART
    if (!takeUart(50)) return uartOk;
    flushUartRx();
    driverVersion = driver->version();
    uartOk = (driverVersion == 0x21);
    giveUart();
    return uartOk;
}

TMC2209Diag Motor::getDriverStatus() {
    TMC2209Diag diag = {};
    if (!isTMC) {
        diag.uartOk = true;
        return diag;
    }

    if (!takeUart(50)) return diag;

    flushUartRx();
    driverVersion = driver->version();
    uartOk = (driverVersion == 0x21);
    diag.uartOk = uartOk;
    diag.driverVersion = driverVersion;

    if (uartOk) {
        flushUartRx();
        const uint32_t drvStatus = driver->DRV_STATUS();
        // Bit layout DRV_STATUS (TMC2209 datasheet / TMCStepper TMC2208_bitfields.h):
        // otpw=0, ot=1, s2ga=2, s2gb=3, s2vsa=4, s2vsb=5, ola=6, olb=7, t120..t157=8..11,
        // cs_actual=16..20, stealth=30, stst=31.
        diag.overTemp        = (drvStatus & (1UL << 1)) != 0;
        diag.overTempWarning = (drvStatus & (1UL << 0)) != 0;
        diag.shortToGndA     = (drvStatus & (1UL << 2)) != 0;
        diag.shortToGndB     = (drvStatus & (1UL << 3)) != 0;
        diag.openLoadA       = (drvStatus & (1UL << 6)) != 0;
        diag.openLoadB       = (drvStatus & (1UL << 7)) != 0;
        diag.standStill      = (drvStatus & (1UL << 31)) != 0;
        diag.csActual        = static_cast<uint8_t>((drvStatus >> 16) & 0x1F);

        flushUartRx();
        diag.sgResult        = static_cast<uint16_t>(driver->SG_RESULT());
    }

    giveUart();
    return diag;
}

void Motor::begin(uint16_t initialCurrentMa, uint16_t initialMicrosteps,
                   bool initialSpreadCycle, uint8_t initialHoldScale, uint8_t iholddelay) {
    pinMode(stepPin, OUTPUT);
    digitalWrite(stepPin, LOW);
    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << stepPin);
    cfg.mode = GPIO_MODE_OUTPUT;
    cfg.pull_up_en = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    cfg.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&cfg);
    gpio_set_level(static_cast<gpio_num_t>(stepPin), 0);
    gpio_set_drive_capability(static_cast<gpio_num_t>(stepPin), GPIO_DRIVE_CAP_3);

    if (dirPin != 255) {
        pinMode(dirPin, OUTPUT);
        digitalWrite(dirPin, LOW);
        gpio_config_t dirCfg = {};
        dirCfg.pin_bit_mask = (1ULL << dirPin);
        dirCfg.mode = GPIO_MODE_OUTPUT;
        dirCfg.pull_up_en = GPIO_PULLUP_DISABLE;
        dirCfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        dirCfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&dirCfg);
        gpio_set_level(static_cast<gpio_num_t>(dirPin), 0);
        gpio_set_drive_capability(static_cast<gpio_num_t>(dirPin), GPIO_DRIVE_CAP_3);
    }

    if (stepTimer != nullptr) {
        esp_timer_stop(stepTimer);
        esp_timer_delete(stepTimer);
        stepTimer = nullptr;
    }

    const esp_timer_create_args_t timerArgs = {
        .callback = &Motor::onStepTimer,
        .arg = this,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "motor_step",
        .skip_unhandled_events = true
    };
    esp_timer_create(&timerArgs, &stepTimer);

    if (!isTMC) {
        enabled.store(true, std::memory_order_relaxed);
        Serial.printf("  >> [A4988 OK] %s (STEP Pin %d, DIR Pin %d)\n", label, stepPin, dirPin);
        return; // A4988 xong
    }

    if (!takeUart(portMAX_DELAY)) return;
    flushUartRx();

    driver->begin();
    driver->toff(4);
    enabled.store(true, std::memory_order_relaxed);
    driver->pdn_disable(true);
    driver->I_scale_analog(false);
    driver->mstep_reg_select(true);

    currentMa = initialCurrentMa;
    driver->rms_current(currentMa);

    microstepsVal = initialMicrosteps;
    driver->microsteps(microstepsVal);

    spreadCycleMode = initialSpreadCycle;
    driver->en_spreadCycle(spreadCycleMode);
    if (spreadCycleMode) {
        driver->pwm_autoscale(false);
        driver->pwm_autograd(false);
    } else {
        driver->pwm_autoscale(true);
        driver->pwm_autograd(true);
    }

    holdScale = initialHoldScale;
    driver->ihold(holdScale);
    driver->iholddelay(iholddelay);

    driver->TCOOLTHRS(0xFFFFF); // Kích hoạt StallGuard4 ở mọi vận tốc
    driver->SGTHRS(DEFAULT_STALL_THRESHOLD);

    driver->shaft(false);
    lastShaftDir = -1;

    giveUart();

    testUART();
    if (uartOk) {
        Serial.printf("  >> [TMC2209 OK] %s (Addr %u, STEP Pin %d): UART tot! Ver: 0x%02X\n", label, address, stepPin, driverVersion);
    } else {
        Serial.printf("  >> [TMC2209 LOI] %s (Addr %u, STEP Pin %d): KHONG UART!\n", label, address, stepPin);
    }
}

bool Motor::setDirection(bool cw) {
    if (dirPin != 255) { // A4988 hoặc TMC có chân DIR vật lý
        dirCW.store(cw, std::memory_order_relaxed);
        digitalWrite(dirPin, cw ? HIGH : LOW);
        gpio_set_level(static_cast<gpio_num_t>(dirPin), cw ? 1 : 0);
        return true;
    }

    if (isTMC && dirPin == 255) { // TMC dùng UART đảo chiều
        const int8_t requestedDir = cw ? 1 : 0;
        if (lastShaftDir != requestedDir) {
            if (!takeUart(100)) {
                Serial.printf("[MOTOR] Canh bao: %s take UART timeout, khong doi duoc chieu!\n", label);
                return false;
            }
            flushUartRx();
            driver->shaft(cw);
            lastShaftDir = requestedDir;
            dirCW.store(cw, std::memory_order_relaxed);
            giveUart();
        } else {
            dirCW.store(cw, std::memory_order_relaxed);
        }
    }
    return true;
}

void Motor::run(bool cw, uint32_t steps) {
    if (steps == 0) {
        stop();
        return;
    }
    if (stepTimer != nullptr) esp_timer_stop(stepTimer);
    running.store(false, std::memory_order_relaxed);
    preparedRun.store(false, std::memory_order_relaxed);
    coordinatedMode.store(false, std::memory_order_relaxed);

    if (!enabled.load(std::memory_order_relaxed)) enable(true);
    if (!setDirection(cw)) {
        Serial.printf("[MOTOR] %s bo qua run: khong doi duoc chieu UART\n", label);
        return;
    }

    continuousMode.store(false, std::memory_order_relaxed);
    targetSteps = steps;
    stepsRemaining.store(steps, std::memory_order_relaxed);
    stepCounter.store(0, std::memory_order_relaxed);

    const uint32_t targetInterval = targetSpeedUs.load(std::memory_order_relaxed);
    if (steps < 40) {
        // Chuyển động siêu ngắn: khởi động êm ở tốc độ vừa phải
        accelSteps = steps / 2;
        decelSteps = steps - accelSteps;
        startSpeedUs = (targetInterval > 2500U) ? targetInterval : 2500U;
        currentSpeedUs.store(startSpeedUs, std::memory_order_relaxed);
    } else {
        accelSteps = steps / 4;
        if (accelSteps > 400) accelSteps = 400;
        decelSteps = accelSteps;
        // Bắt đầu từ MAX_STEP_INTERVAL_US (3500us ~ 285 steps/sec) đảm bảo mô-men xoắn khởi động cực đại
        startSpeedUs = (targetInterval > MAX_STEP_INTERVAL_US) ? targetInterval : MAX_STEP_INTERVAL_US;
        currentSpeedUs.store(startSpeedUs, std::memory_order_relaxed);
    }

    running.store(true, std::memory_order_release);
    if (stepTimer != nullptr) esp_timer_start_once(stepTimer, currentSpeedUs.load(std::memory_order_relaxed));
}

void Motor::runContinuous(bool cw) {
    if (stepTimer != nullptr) esp_timer_stop(stepTimer);
    running.store(false, std::memory_order_relaxed);
    preparedRun.store(false, std::memory_order_relaxed);
    coordinatedMode.store(false, std::memory_order_relaxed);

    if (!enabled.load(std::memory_order_relaxed)) enable(true);
    if (!setDirection(cw)) {
        Serial.printf("[MOTOR] %s bo qua runContinuous: khong doi duoc chieu UART\n", label);
        return;
    }

    continuousMode.store(true, std::memory_order_relaxed);
    targetSteps = 0xFFFFFFFF;
    stepsRemaining.store(0xFFFFFFFF, std::memory_order_relaxed);
    stepCounter.store(0, std::memory_order_relaxed);
    accelSteps = 120;
    decelSteps = 0;

    startSpeedUs = MAX_STEP_INTERVAL_US; // Bắt đầu từ 400 steps/sec
    currentSpeedUs.store(startSpeedUs, std::memory_order_relaxed);

    running.store(true, std::memory_order_release);
    if (stepTimer != nullptr) esp_timer_start_once(stepTimer, currentSpeedUs.load(std::memory_order_relaxed));
}

bool Motor::prepareCoordinatedRun(bool cw, uint32_t steps, uint32_t intervalUs) {
    if (steps == 0 || stepTimer == nullptr) return false;

    esp_timer_stop(stepTimer);
    running.store(false, std::memory_order_release);
    preparedRun.store(false, std::memory_order_relaxed);

    if (!enabled.load(std::memory_order_relaxed)) enable(true);
    if (!enabled.load(std::memory_order_relaxed) || !setDirection(cw)) {
        Serial.printf("[MOTOR] %s khong prepare duoc coordinated run\n", label);
        return false;
    }

    if (intervalUs < MIN_STEP_INTERVAL_US) intervalUs = MIN_STEP_INTERVAL_US;
    continuousMode.store(false, std::memory_order_relaxed);
    coordinatedMode.store(true, std::memory_order_relaxed);
    targetSteps.store(steps, std::memory_order_relaxed);
    stepsRemaining.store(steps, std::memory_order_relaxed);
    stepCounter.store(0, std::memory_order_relaxed);
    accelSteps = 0;
    decelSteps = 0;
    startSpeedUs = intervalUs;
    targetSpeedUs.store(intervalUs, std::memory_order_relaxed);
    currentSpeedUs.store(intervalUs, std::memory_order_relaxed);
    preparedRun.store(true, std::memory_order_release);
    return true;
}

bool Motor::startPreparedRun() {
    if (!preparedRun.exchange(false, std::memory_order_acq_rel) ||
        stepTimer == nullptr || stepsRemaining.load(std::memory_order_relaxed) == 0) {
        return false;
    }

    running.store(true, std::memory_order_release);
    const esp_err_t err = esp_timer_start_once(
        stepTimer, currentSpeedUs.load(std::memory_order_relaxed));
    if (err != ESP_OK) {
        running.store(false, std::memory_order_release);
        stepsRemaining.store(0, std::memory_order_relaxed);
        targetSteps.store(0, std::memory_order_relaxed);
        coordinatedMode.store(false, std::memory_order_relaxed);
        return false;
    }
    return true;
}

void Motor::stop() {
    if (stepTimer != nullptr) esp_timer_stop(stepTimer);
    running.store(false, std::memory_order_release);
    preparedRun.store(false, std::memory_order_release);
    continuousMode.store(false, std::memory_order_relaxed);
    coordinatedMode.store(false, std::memory_order_relaxed);
    stepsRemaining.store(0, std::memory_order_relaxed);
    targetSteps = 0;
    gpio_set_level(static_cast<gpio_num_t>(stepPin), 0);
}

void IRAM_ATTR Motor::stopFromISR() {
    running.store(false, std::memory_order_release);
    preparedRun.store(false, std::memory_order_release);
    continuousMode.store(false, std::memory_order_relaxed);
    coordinatedMode.store(false, std::memory_order_relaxed);
    stepsRemaining.store(0, std::memory_order_relaxed);
    targetSteps = 0;
    if (isHighPin) GPIO.out1_w1tc.val = stepPinMaskHigh;
    else GPIO.out_w1tc = stepPinMaskLow;
}

void Motor::enable(bool en) {
    enabled.store(en, std::memory_order_relaxed);
    if (!en) stop();
    if (isTMC) {
        if (!takeUart(20)) return;
        flushUartRx();
        driver->toff(en ? 4 : 0);
        giveUart();
    }
}

void Motor::setSpeed(uint32_t intervalUs) {
    if (intervalUs < MIN_STEP_INTERVAL_US) intervalUs = MIN_STEP_INTERVAL_US;
    if (intervalUs > MAX_STEP_INTERVAL_US) intervalUs = MAX_STEP_INTERVAL_US;
    targetSpeedUs.store(intervalUs, std::memory_order_relaxed);
    if (!running.load(std::memory_order_relaxed)) {
        currentSpeedUs.store(intervalUs, std::memory_order_relaxed);
    }
}

void Motor::setCurrent(uint16_t mA) {
    currentMa = mA;
    if (!isTMC || !takeUart(20)) return;
    flushUartRx();
    driver->rms_current(currentMa);
    giveUart();
}

void Motor::setHold(uint8_t scale) {
    holdScale = scale;
    if (!isTMC || !takeUart(20)) return;
    flushUartRx();
    driver->ihold(holdScale);
    giveUart();
}

void Motor::setChopperMode(bool spreadCycle) {
    spreadCycleMode = spreadCycle;
    if (!isTMC || !takeUart(20)) return;
    flushUartRx();
    driver->en_spreadCycle(spreadCycleMode);
    driver->pwm_autoscale(!spreadCycleMode);
    driver->pwm_autograd(!spreadCycleMode);
    giveUart();
}

void Motor::setMicrosteps(uint16_t ms) {
    microstepsVal = ms;
    if (!isTMC || !takeUart(20)) return;
    flushUartRx();
    driver->microsteps(ms);
    giveUart();
}

void Motor::setSGThreshold(uint8_t sgthrs) {
    if (!isTMC || !takeUart(20)) return;
    flushUartRx();
    driver->SGTHRS(sgthrs);
    giveUart();
}

uint16_t Motor::getSGResult() {
    if (!isTMC || !uartOk || !takeUart(10)) return 1023;
    flushUartRx();
    const uint16_t sg = static_cast<uint16_t>(driver->SG_RESULT());
    giveUart();
    return sg;
}

void Motor::update() {
}

String Motor::toJson() const {
    char buf[288];
    snprintf(buf, sizeof(buf),
             "{\"running\":%s,\"dir\":\"%s\",\"stepsRemaining\":%u,\"targetSteps\":%u,"
             "\"stepIntervalUs\":%u,\"currentSpeedUs\":%u,\"currentMa\":%u,\"holdScale\":%u,"
             "\"spreadCycle\":%s,\"microsteps\":%u,\"uartOk\":%s,\"version\":%u,\"isTMC\":%s,"
             "\"absSteps\":%lld}",
             running.load(std::memory_order_relaxed) ? "true" : "false",
             dirCW.load(std::memory_order_relaxed) ? "cw" : "ccw",
             stepsRemaining.load(std::memory_order_relaxed),
             targetSteps.load(std::memory_order_relaxed),
             targetSpeedUs.load(std::memory_order_relaxed),
             currentSpeedUs.load(std::memory_order_relaxed),
             currentMa,
             holdScale,
             spreadCycleMode ? "true" : "false",
             microstepsVal,
             uartOk ? "true" : "false",
             driverVersion,
             isTMC ? "true" : "false",
             absSteps.load(std::memory_order_relaxed));
    return String(buf);
}
