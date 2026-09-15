#pragma once
#include <Arduino.h>
struct FakeWire {
    bool ok = true;
    int byteIndex = 0;
    uint16_t raw = 1024;
    void begin(int, int) {}
    void end() {}
    void setClock(uint32_t) {}
    void setTimeOut(uint32_t) {}
    void beginTransmission(uint8_t) {}
    void write(uint8_t) {}
    uint8_t endTransmission(bool = true) { return ok ? 0 : 1; }
    uint8_t requestFrom(uint8_t, uint8_t) { byteIndex = 0; return ok ? 2 : 0; }
    uint8_t read() { return byteIndex++ == 0 ? raw >> 8 : raw & 255; }
};
inline FakeWire Wire;
