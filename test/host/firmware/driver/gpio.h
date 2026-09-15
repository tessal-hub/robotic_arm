#pragma once
#include <Arduino.h>
using gpio_num_t = int;
constexpr int GPIO_MODE_OUTPUT = 0, GPIO_PULLUP_DISABLE = 0, GPIO_PULLDOWN_DISABLE = 0;
constexpr int GPIO_INTR_DISABLE = 0, GPIO_DRIVE_CAP_3 = 3;
struct gpio_config_t { uint64_t pin_bit_mask; int mode, pull_up_en, pull_down_en, intr_type; };
inline void gpio_reset_pin(int) {}
inline void gpio_set_level(int, int) {}
inline int gpio_get_level(int) { return HIGH; }
inline void gpio_config(gpio_config_t*) {}
inline void gpio_set_drive_capability(int, int) {}
struct FakeGPIO { struct { uint32_t val; } out1_w1tc; uint32_t out_w1tc; };
inline FakeGPIO GPIO;
