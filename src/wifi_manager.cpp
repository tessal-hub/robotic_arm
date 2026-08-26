#include "wifi_manager.h"
#include "config.h"

void WifiManager::begin(NvsStore* nvsStore) {
    nvs = nvsStore;
    String pass;
    const bool haveCreds = (nvs != nullptr) && nvs->loadWifiCreds(storedSsid_, pass);

    if (haveCreds) {
        Serial.printf("[WIFI] Thu STA voi creds NVS: %s\n", storedSsid_.c_str());
        WiFi.mode(WIFI_STA);
        WiFi.begin(storedSsid_.c_str(), pass.c_str());
        const uint32_t t0 = millis();
        while (WiFi.status() != WL_CONNECTED &&
               millis() - t0 < WIFI_STA_BOOT_TIMEOUT_MS) {
            delay(200);
            Serial.print('.');
        }
        Serial.println();
        if (WiFi.status() == WL_CONNECTED) {
            mode_ = Mode::STA;
            Serial.printf("[WIFI] STA OK, IP: %s (RSSI %d)\n",
                          WiFi.localIP().toString().c_str(), WiFi.RSSI());
            return;
        }
        Serial.println("[WIFI] STA that bai -> fallback AP");
        WiFi.disconnect(false, true);
    } else {
        Serial.println("[WIFI] Chua co creds trong NVS -> dung che do AP");
    }

    WiFi.mode(WIFI_AP);
    WiFi.softAP(DEFAULT_AP_SSID, DEFAULT_AP_PASS);
    mode_ = Mode::AP;
    Serial.printf("[WIFI] AP \"%s\" IP: %s\n",
                  DEFAULT_AP_SSID, WiFi.softAPIP().toString().c_str());
}

bool WifiManager::provision(const String& ssid, const String& pass) {
    if (nvs == nullptr || ssid.isEmpty()) return false;
    if (!nvs->saveWifiCreds(ssid, pass)) return false;
    storedSsid_ = ssid;
    Serial.printf("[WIFI] Da luu creds moi: %s (se restart)\n", ssid.c_str());
    return true;
}

IPAddress WifiManager::localIP() const {
    return (mode_ == Mode::STA) ? WiFi.localIP() : WiFi.softAPIP();
}

String WifiManager::ipString() const { return localIP().toString(); }

int32_t WifiManager::rssi() const {
    return (mode_ == Mode::STA && WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
}

String WifiManager::toJson() const {
    String j = "{\"mode\":\"";
    switch (mode_) {
        case Mode::STA: j += "sta"; break;
        case Mode::AP:  j += "ap";  break;
        default:        j += "off"; break;
    }
    j += "\",\"ip\":\"" + ipString() + "\",";
    if (isAP()) {
        j += "\"ssid\":\"" + String(DEFAULT_AP_SSID) + "\",\"rssi\":0}";
    } else {
        j += "\"ssid\":\"" + String(WiFi.SSID()) + "\",\"rssi\":" + String(rssi()) + "}";
    }
    return j;
}
