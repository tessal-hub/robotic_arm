#include "endstop.h"
class Motor;
#ifdef ARDUINO
#include "safety_manager.h"
#include <esp_timer.h>
#else
class SafetyManager;
#endif

Endstops::Endstops() {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        minCh[a].pin = AXIS_MIN_PINS[a];
        maxCh[a].pin = AXIS_MAX_PINS[a];
        minCh[a].latched.store(false, std::memory_order_relaxed);
        maxCh[a].latched.store(false, std::memory_order_relaxed);
        owner[a] = nullptr;
        for (uint8_t w = 0; w < 2; ++w) {
            ctx[a][w].axis = a;
            ctx[a][w].which = static_cast<EndstopWhich>(w);
            ctx[a][w].self = this;
            ctx[a][w].safety = nullptr;
            ctx[a][w].pending.store(false, std::memory_order_relaxed);
            ctx[a][w].isrTime.store(0, std::memory_order_relaxed);
        }
    }
}

void Endstops::begin(Motor** motors) {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        owner[a] = (motors != nullptr) ? motors[a] : nullptr;
        for (const auto w : {EndstopWhich::MIN, EndstopWhich::MAX}) {
            Channel& c = ch(a, w);
            c.latched.store(false, std::memory_order_release);
            IsrCtx& ic = ctx[a][static_cast<uint8_t>(w)];
            ic.pending.store(false, std::memory_order_relaxed);
            ic.isrTime.store(0, std::memory_order_relaxed);
            if (!hasPin(a, w)) continue;
#ifdef ARDUINO
            pinMode(c.pin, INPUT_PULLUP);
#endif
        }
    }
    // Propagate safety_ to ctx if already injected before begin
    if (safety_ != nullptr) {
        for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
            for (uint8_t w = 0; w < 2; ++w) {
                ctx[a][w].safety = safety_;
            }
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

void Endstops::setSafetyManager(SafetyManager* sm) noexcept {
    safety_ = sm;
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        for (uint8_t w = 0; w < 2; ++w) {
            ctx[a][w].safety = sm;
        }
    }
}

void Endstops::installPin(uint8_t axis, EndstopWhich w) {
#ifdef ARDUINO
    const Channel& c = ch(axis, w);
    attachInterruptArg(digitalPinToInterrupt(c.pin), &Endstops::isrHandler,
                       &ctx[axis][static_cast<uint8_t>(w)], FALLING);
#else
    (void)axis; (void)w;
#endif
}

void IRAM_ATTR Endstops::isrHandler(void* arg) {
    auto* c = static_cast<IsrCtx*>(arg);
    if (c == nullptr) return;
    // Minimal ISR: only pending + timestamp (<3µs). No delay, no gpio_get_level, no debounce.
    // Intentional double-store: IsrCtx pending/isrTime + SafetyManager pending for fallback
    // when safety_ is null (e.g., early boot before injection). SafetyManager is primary.
    c->pending.store(true, std::memory_order_relaxed);
#ifdef ARDUINO
    int64_t now = esp_timer_get_time();
#else
    int64_t now = 0;
#endif
    c->isrTime.store(now, std::memory_order_relaxed);
#ifdef ARDUINO
    SafetyManager* sm = c->safety;
    if (sm == nullptr && c->self != nullptr) sm = c->self->safety_;
    if (sm != nullptr) {
        sm->isrNotify(c->axis, c->which, now);
    }
#endif
}

bool Endstops::hasPin(uint8_t axis, EndstopWhich w) const noexcept {
    if (axis >= NUM_MOTORS) return false;
    return ch(axis, w).pin >= 0;
}

bool Endstops::isPressed(uint8_t axis, EndstopWhich w) const noexcept {
    if (!hasPin(axis, w)) return false;
#ifdef ARDUINO
    return digitalRead(ch(axis, w).pin) == ENDSTOP_ACTIVE_STATE;
#else
    return false;
#endif
}

bool Endstops::isLatched(uint8_t axis, EndstopWhich w) const noexcept {
    if (!hasPin(axis, w)) return false;
#ifdef ARDUINO
    if (safety_ != nullptr) {
        return safety_->isLatched(axis, w);
    }
#endif
    return ch(axis, w).latched.load(std::memory_order_acquire);
}

bool Endstops::consumeLatch(uint8_t axis, EndstopWhich w) noexcept {
    if (!hasPin(axis, w)) return false;
    uint8_t wi = static_cast<uint8_t>(w);
#ifdef ARDUINO
    if (safety_ != nullptr) {
        bool was = safety_->consumeLatched(axis, w);
        // Ensure local IsrCtx and mirror latched are also cleared — avoids delegation leak
        safety_->clearPending(axis, w);
        ctx[axis][wi].pending.store(false, std::memory_order_relaxed);
        ctx[axis][wi].isrTime.store(0, std::memory_order_relaxed);
        ch(axis, w).latched.store(false, std::memory_order_release);
        return was;
    }
#endif
    ctx[axis][wi].pending.store(false, std::memory_order_relaxed);
    ctx[axis][wi].isrTime.store(0, std::memory_order_relaxed);
    Channel& c = ch(axis, w);
    return c.latched.exchange(false, std::memory_order_acq_rel);
}

void Endstops::clearLatch(uint8_t axis, EndstopWhich w) noexcept {
    if (!hasPin(axis, w)) return;
    uint8_t wi = static_cast<uint8_t>(w);
#ifdef ARDUINO
    if (safety_ != nullptr) {
        safety_->clearLatched(axis, w);
        safety_->clearPending(axis, w);
    }
#endif
    ch(axis, w).latched.store(false, std::memory_order_release);
    ctx[axis][wi].pending.store(false, std::memory_order_relaxed);
    ctx[axis][wi].isrTime.store(0, std::memory_order_relaxed);
}

void Endstops::clearAllLatches() noexcept {
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        if (minCh[a].pin >= 0) minCh[a].latched.store(false, std::memory_order_release);
        if (maxCh[a].pin >= 0) maxCh[a].latched.store(false, std::memory_order_release);
        for (uint8_t w = 0; w < 2; ++w) {
            ctx[a][w].pending.store(false, std::memory_order_relaxed);
            ctx[a][w].isrTime.store(0, std::memory_order_relaxed);
        }
    }
#ifdef ARDUINO
    if (safety_ != nullptr) {
        // Force clear SafetyManager latched/pending without pressed/drift check (boot sync)
        safety_->forceClear();
    }
#endif
}

bool Endstops::anyLatched() const noexcept {
#ifdef ARDUINO
    if (safety_ != nullptr) return safety_->anyLatched();
#endif
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        if (minCh[a].pin >= 0 && minCh[a].latched.load(std::memory_order_acquire)) return true;
        if (maxCh[a].pin >= 0 && maxCh[a].latched.load(std::memory_order_acquire)) return true;
    }
    return false;
}

bool Endstops::isrPending(uint8_t axis, EndstopWhich w) const noexcept {
    if (axis >= NUM_MOTORS) return false;
    return ctx[axis][static_cast<uint8_t>(w)].pending.load(std::memory_order_relaxed);
}

int64_t Endstops::isrTimeUs(uint8_t axis, EndstopWhich w) const noexcept {
    if (axis >= NUM_MOTORS) return 0;
    return ctx[axis][static_cast<uint8_t>(w)].isrTime.load(std::memory_order_relaxed);
}

String Endstops::toJson() const {
#ifdef ARDUINO
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
            j += isLatched(a, EndstopWhich::MIN) ? "true" : "false";
            j += "}";
        }
        j += ",\"max\":";
        if (!hasPin(a, EndstopWhich::MAX)) {
            j += "null";
        } else {
            j += "{\"pressed\":";
            j += isPressed(a, EndstopWhich::MAX) ? "true" : "false";
            j += ",\"latched\":";
            j += isLatched(a, EndstopWhich::MAX) ? "true" : "false";
            j += "}";
        }
        j += "}";
    }
    j += "]";
    return j;
#else
    return String("[]");
#endif
}
