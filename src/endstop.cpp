#include "endstop.h"
#include "motor.h"

namespace {
constexpr int64_t ENDSTOP_DEBOUNCE_US = 50000; // 50 ms
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
            c.latched = false;
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
    const int64_t now = esp_timer_get_time();
    if (now - chan.lastEdgeUs < ENDSTOP_DEBOUNCE_US) return; // bounce
    chan.lastEdgeUs = now;
    chan.latched = true;

    Motor* m = c->self->owner[c->axis];
    if (m != nullptr) m->stopFromISR();
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
    return ch(axis, w).latched;
}

bool Endstops::consumeLatch(uint8_t axis, EndstopWhich w) noexcept {
    if (!hasPin(axis, w)) return false;
    Channel& c = ch(axis, w);
    return __atomic_exchange_n(&c.latched, false, __ATOMIC_ACQ_REL);
}

void Endstops::clearLatch(uint8_t axis, EndstopWhich w) noexcept {
    if (!hasPin(axis, w)) return;
    ch(axis, w).latched = false;
}

bool Endstops::anyLatched() const noexcept {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        if ((minCh[a].pin >= 0 && minCh[a].latched) ||
            (maxCh[a].pin >= 0 && maxCh[a].latched)) return true;
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
            j += minCh[a].latched ? "true" : "false";
            j += "}";
        }
        j += ",\"max\":";
        if (!hasPin(a, EndstopWhich::MAX)) {
            j += "null";
        } else {
            j += "{\"pressed\":";
            j += isPressed(a, EndstopWhich::MAX) ? "true" : "false";
            j += ",\"latched\":";
            j += maxCh[a].latched ? "true" : "false";
            j += "}";
        }
        j += "}";
    }
    j += "]";
    return j;
}
