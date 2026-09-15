#pragma once
#include <Arduino.h>
class TMC2209Stepper {
public:
    bool CRCerror = false, direction = false, readOK = true, writeOK = true;
    int reads = 0, failReadAt = 0;
    uint8_t off = 4;
    TMC2209Stepper(HardwareSerial*, float, uint8_t) {}
    bool shaft() { CRCerror = !readOK || ++reads == failReadAt; return CRCerror ? false : direction; }
    void shaft(bool b) { if (writeOK) direction = b; }
    uint8_t version() { return readOK ? 0x21 : 0; }
    void begin() {}
    void toff(uint8_t v) { if (writeOK) off = v; }
    uint8_t toff() { return off; }
    void pdn_disable(bool) {}
    void I_scale_analog(bool) {}
    void mstep_reg_select(bool) {}
    void rms_current(uint16_t) {}
    void microsteps(uint16_t) {}
    void en_spreadCycle(bool) {}
    void pwm_autoscale(bool) {}
    void pwm_autograd(bool) {}
    void ihold(uint8_t) {}
    void iholddelay(uint8_t) {}
    void TCOOLTHRS(uint32_t) {}
    void SGTHRS(uint8_t) {}
    uint16_t SG_RESULT() { return 200; }
    uint32_t DRV_STATUS() { return 0; }
};
