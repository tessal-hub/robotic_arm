#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include "nvs_store.h"

/**
 * WiFi STA -> AP fallback, credentials lưu NVS.
 * - Có creds trong NVS: thử STA trong WIFI_STA_BOOT_TIMEOUT_MS.
 * - Thất bại / chưa có creds: bật AP "6AXIS-CONTROLLER" (config.h).
 * - provision(): web gọi khi user submit form -> lưu NVS -> caller chịu trách nhiệm restart.
 */
class WifiManager {
public:
    WifiManager() = default;

    WifiManager(const WifiManager&) = delete;
    WifiManager& operator=(const WifiManager&) = delete;

    void begin(NvsStore* nvs);

    // Lưu creds mới vào NVS. Trả false nếu tham số rác. Sau lời gọi này nên restart.
    [[nodiscard]] bool provision(const String& ssid, const String& pass);
    [[nodiscard]] bool hasStoredCreds() const { return storedSsid_.length() > 0; }

    [[nodiscard]] bool isSTA() const noexcept { return mode_ == Mode::STA; }
    [[nodiscard]] bool isAP() const noexcept { return mode_ == Mode::AP; }
    [[nodiscard]] IPAddress localIP() const;
    [[nodiscard]] String ipString() const;
    [[nodiscard]] int32_t rssi() const;

    String toJson() const;

private:
    enum class Mode : uint8_t { OFF = 0, STA, AP };
    Mode mode_{Mode::OFF};
    NvsStore* nvs{nullptr};
    String storedSsid_;
};

#endif // WIFI_MANAGER_H
