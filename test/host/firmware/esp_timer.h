#pragma once
#include <Arduino.h>
#include <driver/gpio.h>
using esp_err_t = int;
constexpr int ESP_OK = 0, ESP_TIMER_TASK = 0;
struct FakeTimer { void (*callback)(void*); void* arg; uint64_t interval{0}; bool active{false}; };
using esp_timer_handle_t = FakeTimer*;
struct esp_timer_create_args_t {
    void (*callback)(void*); void* arg; int dispatch_method; const char* name; bool skip_unhandled_events;
};
inline bool fakeTimerOK = true;
inline int esp_timer_create(const esp_timer_create_args_t* a, esp_timer_handle_t* t) {
    *t = new FakeTimer{a->callback, a->arg}; return ESP_OK;
}
inline int esp_timer_start_once(FakeTimer* t, uint64_t interval) {
    if (!fakeTimerOK) return -1;
    t->interval = interval; t->active = true; return ESP_OK;
}
inline void esp_timer_stop(FakeTimer* t) { t->active = false; }
inline void esp_timer_delete(FakeTimer* t) { delete t; }
inline int64_t esp_timer_get_time() { return micros(); }
