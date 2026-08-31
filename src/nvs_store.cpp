#include "nvs_store.h"

namespace {
constexpr const char* NS = "arm-cfg";
}

void NvsStore::begin() {
    ok_ = prefs_.begin(NS, false);
    if (!ok_) Serial.printf("[NVS] LOI: khong mo duoc namespace %s\n", NS);
}

bool NvsStore::loadWifiCreds(String& ssid, String& pass) const {
    if (!ok_) return false;
    ssid = prefs_.getString("wf_ssid", "");
    pass = prefs_.getString("wf_pass", "");
    return ssid.length() > 0;
}

bool NvsStore::saveWifiCreds(const String& ssid, const String& pass) {
    if (!ok_ || ssid.isEmpty()) return false;
    const bool w1 = prefs_.putString("wf_ssid", ssid);
    const bool w2 = prefs_.putString("wf_pass", pass);
    return w1 && w2;
}

void NvsStore::clearWifiCreds() {
    if (ok_) {
        prefs_.remove("wf_ssid");
        prefs_.remove("wf_pass");
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
        if (h.rawDeg < 0.0f || h.rawDeg >= 360.0f) h.valid = false;
    }
    return h;
}

void NvsStore::saveJointHome(uint8_t axis, float rawDeg) {
    if (!ok_ || axis >= NUM_MOTORS) return;
    // Ghi giá trị TRƯỚC, valid flag SAU CÙNG: nếu mất nguồn giữa chừng,
    // valid vẫn false -> boot sau không phục hồi từ dữ liệu rác.
    char key[12];
    snprintf(key, sizeof(key), "j%u_raw", axis);
    prefs_.putFloat(key, rawDeg);
    snprintf(key, sizeof(key), "j%u_valid", axis);
    prefs_.putBool(key, true);
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
        if (c.stepsPerDeg <= 0.0f) c.valid = false;
    }
    return c;
}

void NvsStore::saveCalib(uint8_t axis, float encSign, float stepsPerDeg) {
    if (!ok_ || axis >= NUM_MOTORS) return;
    // Ghi giá trị TRƯỚC, valid flag SAU CÙNG (nguyên tắc commit-flag).
    char key[14];
    snprintf(key, sizeof(key), "j%u_esign", axis);
    prefs_.putFloat(key, encSign);
    snprintf(key, sizeof(key), "j%u_mspd", axis);
    prefs_.putFloat(key, stepsPerDeg);
    snprintf(key, sizeof(key), "j%u_cvalid", axis);
    prefs_.putBool(key, true);
}

void NvsStore::clearCalib(uint8_t axis) {
    if (!ok_ || axis >= NUM_MOTORS) return;
    char key[14];
    snprintf(key, sizeof(key), "j%u_cvalid", axis);
    prefs_.putBool(key, false);
}
