#pragma once
#include <Arduino.h>
#include <esp_timer.h>
inline int esp_task_wdt_add(void*) { return 0; }
inline void esp_task_wdt_reset() {}
inline void esp_task_wdt_delete(void*) {}
