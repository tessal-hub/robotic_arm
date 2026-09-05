#include <cstdio>
#include <fstream>
#include <string>

static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        std::printf("FAIL: %s (line %d)\n", msg, __LINE__); \
        ++g_fail; \
    } \
} while (0)

static std::string readSource(const char* path) {
    std::ifstream input(path);
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

int main() {
    const std::string header = readSource("src/sensor.h");
    const std::string source = readSource("src/sensor.cpp");
    CHECK(header.find("published_angles") != std::string::npos,
          "sensor keeps atomic fallback angle snapshot");
    CHECK(header.find("published_accumulated") != std::string::npos,
          "sensor keeps atomic fallback accumulated snapshot");
    CHECK(source.find("publishSample(i);") != std::string::npos,
          "scan publishes snapshot under data lock");
    CHECK(source.find("bitsToFloat(published_angles[ch].load") != std::string::npos,
          "angle timeout returns snapshot rather than unlocked array access");
    CHECK(source.find("bitsToFloat(published_accumulated[ch].load") != std::string::npos,
          "accumulated timeout returns snapshot rather than unlocked array access");

    if (g_fail == 0) {
        std::printf("ALL PASSED (sensor snapshot contract)\n");
        return 0;
    }
    std::printf("%d FAILED\n", g_fail);
    return 1;
}
