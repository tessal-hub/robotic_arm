#pragma once
// Hardware boundaries for tests of the production firmware modules.
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <functional>
#define IRAM_ATTR
#define LOW 0
#define HIGH 1
#define INPUT_PULLUP 2
#define OUTPUT 3
#define FALLING 4
constexpr float TWO_PI = 6.2831853071795864769f;
using BaseType_t = int;
using UBaseType_t = unsigned;
using TickType_t = uint32_t;
using TaskHandle_t = void*;
using SemaphoreHandle_t = void*;
constexpr int pdTRUE = 1, pdFALSE = 0, pdPASS = 1;
constexpr uint32_t portMAX_DELAY = UINT32_MAX;
inline uint32_t pdMS_TO_TICKS(uint32_t ms) { return ms; }
inline uint32_t fakeNow = 1000;
inline bool fakeTaskOK = true;
inline bool fakeMutexOK = true;
inline SemaphoreHandle_t fakeBlockedMutex = nullptr;
inline uint32_t millis() { return fakeNow; }
inline uint32_t micros() { return fakeNow * 1000; }
inline void delay(uint32_t ms) { fakeNow += ms; }
inline void delayMicroseconds(uint32_t) {}
inline void esp_rom_delay_us(uint32_t) {}
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return HIGH; }
inline int digitalPinToInterrupt(int pin) { return pin; }
inline void attachInterruptArg(int, void (*)(void*), void*, int) {}
inline int xSemaphoreTake(SemaphoreHandle_t m, uint32_t) { return fakeMutexOK && m != fakeBlockedMutex; }
inline void xSemaphoreGive(SemaphoreHandle_t) {}
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return fakeMutexOK ? reinterpret_cast<void*>(1) : nullptr; }
inline void vSemaphoreDelete(SemaphoreHandle_t) {}
inline int xTaskCreatePinnedToCore(void (*)(void*), const char*, uint32_t, void*, uint8_t, TaskHandle_t* t, uint8_t) {
    *t = fakeTaskOK ? reinterpret_cast<void*>(1) : nullptr;
    return fakeTaskOK ? pdPASS : pdFALSE;
}
inline void vTaskDelete(TaskHandle_t) {}
inline TickType_t xTaskGetTickCount() { return fakeNow; }
inline void vTaskDelayUntil(TickType_t*, TickType_t) {}
inline void vTaskDelay(TickType_t) {}
struct FakeQueue { size_t itemSize; std::vector<std::vector<uint8_t>> items; };
using QueueHandle_t = FakeQueue*;
inline QueueHandle_t xQueueCreate(unsigned, size_t size) { return new FakeQueue{size, {}}; }
inline unsigned uxQueueMessagesWaiting(QueueHandle_t q) { return q->items.size(); }
inline int xQueueSend(QueueHandle_t q, const void* p, uint32_t) {
    auto b = static_cast<const uint8_t*>(p); q->items.emplace_back(b, b + q->itemSize); return pdTRUE;
}
inline int xQueueReceive(QueueHandle_t q, void* p, uint32_t) {
    if (q->items.empty()) return pdFALSE;
    memcpy(p, q->items.front().data(), q->itemSize); q->items.erase(q->items.begin()); return pdTRUE;
}
inline void xQueueReset(QueueHandle_t q) { q->items.clear(); }
class String : public std::string {
public:
    using std::string::string;
    String(const std::string& s) : std::string(s) {}
    template<class T, typename = std::enable_if_t<std::is_arithmetic_v<T>>>
    String(T n) : std::string(std::to_string(n)) {}
    bool isEmpty() const { return empty(); }
};
struct HardwareSerial {
    int available() { return 0; }
    int read() { return -1; }
    void println(const char*) {}
    void print(char) {}
    template<class... T> void printf(const char*, T...) {}
};
inline HardwareSerial Serial, Serial1;
