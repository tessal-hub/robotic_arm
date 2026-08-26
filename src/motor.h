#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <TMCStepper.h>
#include <esp_timer.h>
#include <atomic>
#include <memory>
#include "config.h"

struct TMC2209Diag {
    bool uartOk{false};
    uint8_t driverVersion{0};
    bool overTemp{false};
    bool overTempWarning{false};
    bool shortToGndA{false};
    bool shortToGndB{false};
    bool openLoadA{false};
    bool openLoadB{false};
    bool standStill{false};
    uint8_t csActual{0};
    uint16_t sgResult{0};
};

class Motor {
private:
    HardwareSerial* serialPort;
    SemaphoreHandle_t* uartMutex;
    std::unique_ptr<TMC2209Stepper> driver;
    bool isTMC; // Phân biệt TMC2209 và A4988

    uint8_t stepPin;
    uint8_t dirPin;
    uint8_t address;
    const char* label;

    std::atomic<bool> running{false};
    std::atomic<bool> dirCW{true};
    int8_t lastShaftDir{-1};
    std::atomic<uint32_t> targetSpeedUs{DEFAULT_STEP_INTERVAL_US};
    std::atomic<uint32_t> currentSpeedUs{MAX_STEP_INTERVAL_US};
    uint32_t startSpeedUs{MAX_STEP_INTERVAL_US};
    uint32_t targetSteps{0};
    std::atomic<uint32_t> stepsRemaining{0};
    std::atomic<uint32_t> stepCounter{0};
    std::atomic<int64_t> absSteps{0};   // Vị trí tuyệt đối (dấu theo dirCW), cập nhật trong step timer
    uint32_t accelSteps{60};
    uint32_t decelSteps{60};
    std::atomic<bool> continuousMode{false};

    uint16_t currentMa{DEFAULT_NORMAL_CURRENT};
    bool spreadCycleMode{true};
    uint16_t microstepsVal{DEFAULT_MICROSTEPS};
    uint8_t holdScale{DEFAULT_HOLD_SCALE};
    std::atomic<bool> enabled{true};

    bool uartOk{false};
    uint8_t driverVersion{0};

    uint32_t stepPinMaskLow{0};
    uint32_t stepPinMaskHigh{0};
    bool isHighPin{false};

    esp_timer_handle_t stepTimer{nullptr};
    static void IRAM_ATTR onStepTimer(void* arg);

    bool takeUart(uint32_t timeoutMs);
    void giveUart();
    void flushUartRx();

    static inline uint32_t calculateSCurveInterval(uint32_t currentStep, uint32_t totalSteps,
                                                   uint32_t startInterval, uint32_t targetInterval);

public:
    Motor(HardwareSerial* serial, float rSense, uint8_t uartAddress,
          uint8_t stepPinNum, uint8_t dirPinNum = 255, const char* motorLabel = "Motor");
    ~Motor();

    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    void setUartMutex(SemaphoreHandle_t* mutex) noexcept { uartMutex = mutex; }

    void begin(uint16_t initialCurrentMa = DEFAULT_NORMAL_CURRENT,
               uint16_t initialMicrosteps = DEFAULT_MICROSTEPS,
               bool initialSpreadCycle = true,
               uint8_t initialHoldScale = DEFAULT_HOLD_SCALE,
               uint8_t iholddelay = 10);

    void setDirection(bool cw);
    void run(bool cw, uint32_t steps);
    void runContinuous(bool cw);
    void stop();
    void stopFromISR();
    void enable(bool en = true);

    void setSpeed(uint32_t intervalUs);
    void setCurrent(uint16_t mA);
    void setHold(uint8_t scale);
    void setChopperMode(bool spreadCycle);
    void setMicrosteps(uint16_t ms);
    void setSGThreshold(uint8_t sgthrs);
    uint16_t getSGResult();

    void update();
    bool testUART();
    [[nodiscard]] bool isUartOK() const noexcept { return uartOk; }
    [[nodiscard]] bool isTmc() const noexcept { return isTMC; }
    [[nodiscard]] uint8_t getDriverVersion() const noexcept { return driverVersion; }
    TMC2209Diag getDriverStatus();
    TMC2209Stepper* getDriver() noexcept { return driver.get(); }

    [[nodiscard]] bool isRunning() const noexcept { return running.load(std::memory_order_relaxed); }
    [[nodiscard]] bool isEnabled() const noexcept { return enabled.load(std::memory_order_relaxed); }
    [[nodiscard]] bool getDirCW() const noexcept { return dirCW.load(std::memory_order_relaxed); }
    [[nodiscard]] uint32_t getStepsRemaining() const noexcept { return stepsRemaining.load(std::memory_order_relaxed); }
    [[nodiscard]] uint32_t getTargetSteps() const noexcept { return targetSteps; }
    [[nodiscard]] uint32_t getStepCounter() const noexcept { return stepCounter.load(std::memory_order_relaxed); }
    void resetStepCounter() noexcept { stepCounter.store(0, std::memory_order_relaxed); }
    // Vị trí tuyệt đối tính bằng microstep. Chỉ set khi motor KHÔNG chạy (homing/calib).
    [[nodiscard]] int64_t getAbsoluteSteps() const noexcept { return absSteps.load(std::memory_order_relaxed); }
    void setAbsoluteSteps(int64_t v) noexcept { absSteps.store(v, std::memory_order_relaxed); }
    [[nodiscard]] uint32_t getStepInterval() const noexcept { return targetSpeedUs.load(std::memory_order_relaxed); }
    [[nodiscard]] uint32_t getCurrentInterval() const noexcept { return currentSpeedUs.load(std::memory_order_relaxed); }
    [[nodiscard]] uint16_t getCurrent() const noexcept { return currentMa; }
    [[nodiscard]] uint8_t getHoldScale() const noexcept { return holdScale; }
    [[nodiscard]] bool getSpreadCycle() const noexcept { return spreadCycleMode; }
    [[nodiscard]] uint16_t getMicrosteps() const noexcept { return microstepsVal; }
    [[nodiscard]] uint8_t getAddress() const noexcept { return address; }
    [[nodiscard]] uint8_t getStepPin() const noexcept { return stepPin; }
    [[nodiscard]] uint8_t getDirPin() const noexcept { return dirPin; }
    [[nodiscard]] const char* getLabel() const noexcept { return label; }

    String toJson() const;
};

#endif