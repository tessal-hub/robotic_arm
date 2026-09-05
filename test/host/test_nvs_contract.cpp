#include <cstdio>
#include <fstream>
#include <string>

int main() {
    std::ifstream input("src/nvs_store.cpp");
    const std::string source{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    const bool loadRejectsNonFinite = source.find("!std::isfinite(h.rawDeg)") != std::string::npos;
    const bool saveRejectsNonFinite = source.find("!std::isfinite(rawDeg)") != std::string::npos;
    const bool verifiesWrite = source.find("prefs_.getBool(validKey, false)") != std::string::npos &&
                               source.find("prefs_.getFloat(valueKey, NAN)") != std::string::npos;
    if (loadRejectsNonFinite && saveRejectsNonFinite && verifiesWrite) {
        std::printf("ALL PASSED (NVS contract)\n");
        return 0;
    }
    std::printf("FAIL: NVS home raw-angle validation missing\n");
    return 1;
}
