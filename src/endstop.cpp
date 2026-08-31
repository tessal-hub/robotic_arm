#include "endstop.h"
#include "motor.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

namespace {
constexpr int64_t ENDSTOP_DEBOUNCE_US = 50000; // 50 ms
constexpr uint32_t GLITCH_FILTER_DELAY_US = 25; // 25 us verify delay for EMI noise rejection
}

Endstops::Endstops() {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        minCh[a].pin = AXIS_MIN_PINS[a];
        maxCh[a].pin = AXIS_MAX_PINS[a];
        owner[a] = nullptr;
        for (uint8_t w = 0; w < 2; ++w) {
            ctx[a][w] = IsrCtx{this, a, static_cast<EndstopWhich>(w)};
        }
    }
}

void Endstops::begin(Motor** motors) {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        owner[a] = (motors != nullptr) ? motors[a] : nullptr;
        for (const auto w : {EndstopWhich::MIN, EndstopWhich::MAX}) {
            Channel& c = ch(a, w);
            __atomic_store_n(&c.latched, false, __ATOMIC_RELEASE);
            c.lastEdgeUs = 0;
            if (!hasPin(a, w)) continue;
            pinMode(c.pin, INPUT_PULLUP);
        }
    }
    // Attach sau khi mọi pinMode đã xong để tránh ISR bắn trong lúc setup.
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        for (const auto w : {EndstopWhich::MIN, EndstopWhich::MAX}) {
            if (hasPin(a, w)) installPin(a, w);
        }
    }
    clearAllLatches(); // Xoá mọi latch cũ — endstop pressed at boot không phải fault
    initialized.store(true, std::memory_order_release);
}

void Endstops::installPin(uint8_t axis, EndstopWhich w) {
    const Channel& c = ch(axis, w);
    attachInterruptArg(digitalPinToInterrupt(c.pin), &Endstops::isrHandler,
                       &ctx[axis][static_cast<uint8_t>(w)], FALLING);
}

void IRAM_ATTR Endstops::isrHandler(void* arg) {
    auto* c = static_cast<IsrCtx*>(arg);
    if (c == nullptr || c->self == nullptr) return;

    Channel& chan = c->self->ch(c->axis, c->which);

    // 1. Kiểm tra nhanh: nếu chân đã về HIGH (3.3V) -> xung nhiễu sub-microsecond, bỏ qua
    if (gpio_get_level(static_cast<gpio_num_t>(chan.pin)) != ENDSTOP_ACTIVE_STATE) {
        return;
    }

    // 2. Debounce window
    const int64_t now = esp_timer_get_time();
    if (now - chan.lastEdgeUs < ENDSTOP_DEBOUNCE_US) return;

    // 3. Glitch filter: chờ 25us và kiểm tra lại mức logic để lọc sạch nhiễu cảm ứng từ dây stepper
    esp_rom_delay_us(GLITCH_FILTER_DELAY_US);
    if (gpio_get_level(static_cast<gpio_num_t>(chan.pin)) != ENDSTOP_ACTIVE_STATE) {
        return; // Nhiễu gai ngắn (< 25us), không phải tiếp xúc cơ học thật
    }

    chan.lastEdgeUs = now;
    __atomic_store_n(&chan.latched, true, __ATOMIC_RELEASE);

    // AN TOÀN TUYỆT ĐỐI: BẤT KỲ KHI NÀO ENDSTOP BỊ NHẤN (FALLING EDGE) -> DỪNG MOTOR NGAY TRONG ISR
    if (g_homingActive.load(std::memory_order_acquire)) {
        Motor* m = c->self->owner[c->axis];
        if (m != nullptr) m->stopFromISR();
    } else {
        // Ngoài homing: dừng mọi trục + fail-fast E-stop cho step timer
        g_emergencyStop.store(true, std::memory_order_release);
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
            Motor* mot = c->self->owner[i];
            if (mot != nullptr) mot->stopFromISR();
        }
    }
}

bool Endstops::hasPin(uint8_t axis, EndstopWhich w) const noexcept {
    if (axis >= NUM_MOTORS) return false;
    return ch(axis, w).pin >= 0;
}

bool Endstops::isPressed(uint8_t axis, EndstopWhich w) const noexcept {
    if (!hasPin(axis, w)) return false;
    return digitalRead(ch(axis, w).pin) == ENDSTOP_ACTIVE_STATE;
}

bool Endstops::isLatched(uint8_t axis, EndstopWhich w) const noexcept {
    if (!hasPin(axis, w)) return false;
    return __atomic_load_n(&ch(axis, w).latched, __ATOMIC_ACQUIRE);
}

bool Endstops::consumeLatch(uint8_t axis, EndstopWhich w) noexcept {
    if (!hasPin(axis, w)) return false;
    Channel& c = ch(axis, w);
    return __atomic_exchange_n(&c.latched, false, __ATOMIC_ACQ_REL);
}

void Endstops::clearLatch(uint8_t axis, EndstopWhich w) noexcept {
    if (!hasPin(axis, w)) return;
    __atomic_store_n(&ch(axis, w).latched, false, __ATOMIC_RELEASE);
}

void Endstops::clearAllLatches() noexcept {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        if (minCh[a].pin >= 0) __atomic_store_n(&minCh[a].latched, false, __ATOMIC_RELEASE);
        if (maxCh[a].pin >= 0) __atomic_store_n(&maxCh[a].latched, false, __ATOMIC_RELEASE);
    }
}

bool Endstops::anyLatched() const noexcept {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        if (minCh[a].pin >= 0 && __atomic_load_n(&minCh[a].latched, __ATOMIC_ACQUIRE)) return true;
        if (maxCh[a].pin >= 0 && __atomic_load_n(&maxCh[a].latched, __ATOMIC_ACQUIRE)) return true;
    }
    return false;
}

String Endstops::toJson() const {
    String j = "[";
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        if (a > 0) j += ",";
        j += "{\"min\":";
        if (!hasPin(a, EndstopWhich::MIN)) {
            j += "null";
        } else {
            j += "{\"pressed\":";
            j += isPressed(a, EndstopWhich::MIN) ? "true" : "false";
            j += ",\"latched\":";
            j += __atomic_load_n(&minCh[a].latched, __ATOMIC_ACQUIRE) ? "true" : "false";
            j += "}";
        }
        j += ",\"max\":";
        if (!hasPin(a, EndstopWhich::MAX)) {
            j += "null";
        } else {
            j += "{\"pressed\":";
            j += isPressed(a, EndstopWhich::MAX) ? "true" : "false";
            j += ",\"latched\":";
            j += __atomic_load_n(&maxCh[a].latched, __ATOMIC_ACQUIRE) ? "true" : "false";
            j += "}";
        }
        j += "}";
    }
    j += "]";
    return j;
}
