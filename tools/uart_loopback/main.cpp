#include <Arduino.h>
#include <TMCStepper.h>
#include "../../src/config.h"
#include "../../src/rtos_guard.h"

static TMC2209Stepper driver(&SERIAL_PORT_1, R_SENSE, 0);
static SemaphoreHandle_t g_uartMutex = nullptr;

void setup() {
    Serial.begin(115200);
    delay(1500);
    // Hold every configured STEP low; this diagnostic never starts motion.
    const uint8_t steps[] = {STEP_PIN_0, STEP_PIN_1, STEP_PIN_2, STEP_PIN_3, STEP_PIN_4, STEP_PIN_5};
    for (uint8_t pin : steps) {
        digitalWrite(pin, LOW);
        pinMode(pin, OUTPUT);
    }
    SERIAL_PORT_1.begin(TMC_UART_BAUD, SERIAL_8N1, RX_PIN_1, TX_PIN_1);
    Serial.printf("\n[TMC TEST] addr=0 TX=%d RX=%d baud=%lu; NO STEP\n",
                  TX_PIN_1, RX_PIN_1, static_cast<unsigned long>(TMC_UART_BAUD));
    g_uartMutex = xSemaphoreCreateMutex();
    auto lock = makeTimedLock(g_uartMutex, 50);
    if (!lock) { Serial.println("[TMC TEST] mutex FAIL; init skipped"); return; }
    driver.begin();
    driver.toff(4);
    driver.pdn_disable(true);
    driver.I_scale_analog(false);
    driver.mstep_reg_select(true);
    driver.rms_current(700);
    driver.microsteps(16);
    driver.en_spreadCycle(true);
    driver.pwm_autoscale(false);
    driver.pwm_autograd(false);
    driver.ihold(8);
    driver.iholddelay(10);
    const uint8_t before = driver.IFCNT();
    const bool beforeError = driver.CRCerror;
    driver.pdn_disable(true); // Repeat same setting: one write, no motion.
    const uint8_t after = driver.IFCNT();
    const bool afterError = driver.CRCerror;
    Serial.printf("[TMC WRITE] IFCNT=%u->%u delta=%u expected=1 readError=%u/%u (delta valid only if both errors=0)\n",
                  before, after, static_cast<uint8_t>(after - before), beforeError, afterError);
}

void loop() {
    delay(1000);
    auto lock = makeTimedLock(g_uartMutex, 50);
    if (!lock) { Serial.println("[TMC TEST] mutex FAIL; read skipped"); return; }
    const uint8_t version = driver.version();
    const bool readError = driver.CRCerror;
    Serial.printf("[TMC READ] addr=0 version=0x%02X expected=0x21 readError=%u (timeout/CRC) %s\n",
                  version, readError, !readError && version == 0x21 ? "PASS" : "FAIL");
}
