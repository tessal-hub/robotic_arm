#include "nvs_store.h"

#include <cmath>

namespace {
constexpr const char* NS = "arm-cfg";
constexpr const char* WIFI_VALID_KEY = "wf_valid";
constexpr const char* WIFI_SSID_KEY = "wf_ssid";
constexpr const char* WIFI_PASS_KEY = "wf_pass";

bool putStringVerified(Preferences& prefs, const char* key, const String& value) {
    const size_t written = prefs.putString(key, value);
    if (!value.isEmpty() && written != value.length()) return false;

    // Preferences::putString() returns strlen(value), therefore an empty string
    // returns 0 even after a successful commit. Verify the stored key/value.
    return prefs.isKey(key) && prefs.getString(key, "") == value;
}
}

void NvsStore::begin() {
    ok_ = prefs_.begin(NS, false);
    if (!ok_) Serial.printf("[NVS] LOI: khong mo duoc namespace %s\n", NS);
}

bool NvsStore::loadWifiCreds(String& ssid, String& pass) const {
    if (!ok_) return false;

    // Legacy records have no wf_valid key. Accept them until the next save,
    // while a present false flag always means an interrupted/incomplete update.
    if (prefs_.isKey(WIFI_VALID_KEY) && !prefs_.getBool(WIFI_VALID_KEY, false)) {
        ssid = "";
        pass = "";
        return false;
    }
    ssid = prefs_.getString(WIFI_SSID_KEY, "");
    pass = prefs_.getString(WIFI_PASS_KEY, "");
    return ssid.length() > 0;
}

bool NvsStore::saveWifiCreds(const String& ssid, const String& pass) {
    if (!ok_ || ssid.isEmpty()) return false;
    if (prefs_.putBool(WIFI_VALID_KEY, false) != 1) return false;
    if (!putStringVerified(prefs_, WIFI_SSID_KEY, ssid)) return false;
    if (!putStringVerified(prefs_, WIFI_PASS_KEY, pass)) return false;
    return prefs_.putBool(WIFI_VALID_KEY, true) == 1;
}

void NvsStore::clearWifiCreds() {
    if (ok_) {
        if (prefs_.putBool(WIFI_VALID_KEY, false) != 1) return;
        prefs_.remove(WIFI_SSID_KEY);
        prefs_.remove(WIFI_PASS_KEY);
    }
}

NvsStore::JointHome NvsStore::loadJointHome(uint8_t axis) const {
    JointHome h;
    if (!ok_ || axis >= NUM_MOTORS) return h;
    char key[12];
    snprintf(key, sizeof(key), "j%u_valid", axis);
    h.valid = prefs_.getBool(key, false);
    if (h.valid) {
        snprintf(key, sizeof(key), "j%u_raw", axis);
        h.rawDeg = prefs_.getFloat(key, 0.0f);
        if (!std::isfinite(h.rawDeg) || h.rawDeg < 0.0f || h.rawDeg >= 360.0f) h.valid = false;
    }
    return h;
}

bool NvsStore::saveJointHome(uint8_t axis, float rawDeg) {
    if (!ok_ || axis >= NUM_MOTORS || !std::isfinite(rawDeg) || rawDeg < 0.0f || rawDeg >= 360.0f) {
        return false;
    }
    char valueKey[12];
    char validKey[12];
    snprintf(valueKey, sizeof(valueKey), "j%u_raw", axis);
    snprintf(validKey, sizeof(validKey), "j%u_valid", axis);

    if (prefs_.putBool(validKey, false) != 1) return false;
    if (prefs_.putFloat(valueKey, rawDeg) != sizeof(float)) return false;
    if (prefs_.putBool(validKey, true) != 1) return false;

    // Verify both marker and payload immediately. This catches a failed flash
    // commit instead of reporting a home as saved when boot would discard it.
    const float stored = prefs_.getFloat(valueKey, NAN);
    return prefs_.getBool(validKey, false) && std::isfinite(stored) &&
           fabsf(stored - rawDeg) < 0.001f;
}

void NvsStore::clearJointHome(uint8_t axis) {
    if (!ok_ || axis >= NUM_MOTORS) return;
    char key[12];
    snprintf(key, sizeof(key), "j%u_valid", axis);
    prefs_.putBool(key, false);
}

NvsStore::CalibData NvsStore::loadCalib(uint8_t axis) const {
    CalibData c;
    if (!ok_ || axis >= NUM_MOTORS) return c;
    char key[14];
    snprintf(key, sizeof(key), "j%u_cvalid", axis);
    c.valid = prefs_.getBool(key, false);
    if (c.valid) {
        snprintf(key, sizeof(key), "j%u_esign", axis);
        c.encSign = prefs_.getFloat(key, 1.0f);
        snprintf(key, sizeof(key), "j%u_mspd", axis);
        c.stepsPerDeg = prefs_.getFloat(key, 0.0f);
        if (!std::isfinite(c.encSign) || !std::isfinite(c.stepsPerDeg) || c.stepsPerDeg <= 0.0f) {
            c.valid = false;
        }
    }
    return c;
}

bool NvsStore::saveCalib(uint8_t axis, float encSign, float stepsPerDeg) {
    if (!ok_ || axis >= NUM_MOTORS) return false;
    char signKey[14];
    char spdKey[14];
    char validKey[14];
    snprintf(signKey, sizeof(signKey), "j%u_esign", axis);
    snprintf(spdKey, sizeof(spdKey), "j%u_mspd", axis);
    snprintf(validKey, sizeof(validKey), "j%u_cvalid", axis);

    if (prefs_.putBool(validKey, false) != 1) return false;
    if (prefs_.putFloat(signKey, encSign) != sizeof(float)) return false;
    if (prefs_.putFloat(spdKey, stepsPerDeg) != sizeof(float)) return false;
    return prefs_.putBool(validKey, true) == 1;
}

void NvsStore::clearCalib(uint8_t axis) {
    if (!ok_ || axis >= NUM_MOTORS) return;
    char key[14];
    snprintf(key, sizeof(key), "j%u_cvalid", axis);
    prefs_.putBool(key, false);
}
