#ifndef NVS_STORE_H
#define NVS_STORE_H

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

/**
 * Lưu trữ bền vững (NVS partition của flash):
 *  - WiFi credentials: cấp phát qua trang provisioning, không cần compile lại.
 *  - Home point từng khớp: góc RAW single-turn của encoder lúc đặt home.
 *    Khi bật nguồn, JointModel đối chiếu raw hiện tại vs raw đã lưu để khôi phục.
 */
class NvsStore {
public:
    NvsStore() = default;

    NvsStore(const NvsStore&) = delete;
    NvsStore& operator=(const NvsStore&) = delete;

    void begin();

    // --- WiFi ---
    [[nodiscard]] bool loadWifiCreds(String& ssid, String& pass) const;
    bool saveWifiCreds(const String& ssid, const String& pass);
    // --- Joint home points ---
    struct JointHome {
        bool valid{false};
        float rawDeg{0.0f};   // góc encoder [0,360) tại thời điểm joint = 0°
    };
    [[nodiscard]] JointHome loadJointHome(uint8_t axis) const;
    bool saveJointHome(uint8_t axis, float rawDeg);
    void clearJointHome(uint8_t axis);
    // --- Joint calibration parameters ---
    struct CalibData {
        bool valid{false};
        float encSign{1.0f};
        float stepsPerDeg{0.0f};
    };
    [[nodiscard]] CalibData loadCalib(uint8_t axis) const;
    bool saveCalib(uint8_t axis, float encSign, float stepsPerDeg);
    void clearCalib(uint8_t axis);

    // --- Teach trajectory persistence (NVS blob) ---
    bool saveTeachTrajectory(const void* data, size_t size);
    size_t loadTeachTrajectory(void* data, size_t maxSize) const;
    bool clearTeachTrajectory();

private:
    mutable Preferences prefs_;
    bool ok_{false};
};

#endif // NVS_STORE_H
