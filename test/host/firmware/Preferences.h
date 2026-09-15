#pragma once
#include <Arduino.h>
class Preferences {
public:
    inline static std::map<std::string, std::vector<uint8_t>> storage;
    inline static std::string failKey;
    inline static bool dropWrite = false;
    bool begin(const char*, bool) { return true; }
    size_t putBytes(const char* key, const void* p, size_t n) {
        if (failKey == key) return dropWrite ? n : 0;
        const auto b = static_cast<const uint8_t*>(p); storage[key] = {b, b + n}; return n;
    }
    size_t getBytesLength(const char* key) { return storage.count(key) ? storage[key].size() : 0; }
    size_t getBytes(const char* key, void* p, size_t n) {
        if (!storage.count(key) || storage[key].size() > n) return 0;
        memcpy(p, storage[key].data(), storage[key].size()); return storage[key].size();
    }
    size_t putBool(const char* key, bool v) { return putBytes(key, &v, sizeof(v)); }
    bool getBool(const char* key, bool fallback) {
        bool v; return getBytes(key, &v, sizeof(v)) == sizeof(v) ? v : fallback;
    }
    size_t putFloat(const char* key, float v) { return putBytes(key, &v, sizeof(v)); }
    float getFloat(const char* key, float fallback) {
        float v; return getBytes(key, &v, sizeof(v)) == sizeof(v) ? v : fallback;
    }
    size_t putString(const char* key, const String& v) { return putBytes(key, v.c_str(), v.size()); }
    String getString(const char* key, const char* fallback) {
        return storage.count(key) ? String(storage[key].begin(), storage[key].end()) : String(fallback);
    }
    bool isKey(const char* key) { return storage.count(key) != 0; }
};
