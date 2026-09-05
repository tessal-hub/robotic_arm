#include "drawing_workspace.h"

#include <cmath>
#include <cstdio>

static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { std::printf("FAIL: %s (line %d)\n", msg, __LINE__); ++g_fail; } \
} while (0)

int main() {
    const auto& profiles = DrawingWorkspace::recommendations();
    uint8_t validCount = 0;
    for (uint8_t i = 0; i < profiles.size(); ++i) {
        if (!profiles[i].valid) continue;
        ++validCount;
        CHECK(profiles[i].maxSquareSide >= 40.0f, "profile has usable inscribed square");
        if (i > 0 && profiles[i - 1].valid) {
            const float dz = profiles[i].z - profiles[i - 1].z;
            CHECK(dz >= 40.0f || dz <= -40.0f, "profiles are materially distinct in Z");
        }
        for (const auto shape : {DrawingWorkspace::Shape::LINE,
                                 DrawingWorkspace::Shape::CIRCLE,
                                 DrawingWorkspace::Shape::SQUARE}) {
            DrawingWorkspace::SuggestedJob job;
            CHECK(DrawingWorkspace::makeSuggestedJob(i, shape, job),
                  "recommended job stays reachable while drawing and lifting");
            CHECK(job.size >= 40.0f, "recommended job has meaningful size");
        }
        const float lineSize = std::floor(profiles[i].maxSquareSide * 0.60f / 5.0f) * 5.0f;
        DrawingWorkspace::SuggestedJob customLine;
        CHECK(DrawingWorkspace::makeSuggestedLine(i,
                                                  profiles[i].centerX - lineSize * 0.5f,
                                                  profiles[i].centerY,
                                                  lineSize,
                                                  customLine),
              "custom start point line stays reachable");
        CHECK(customLine.shape == DrawingWorkspace::Shape::LINE &&
              std::fabs(customLine.x - profiles[i].centerX) < 0.01f,
              "custom line preserves requested start and length");
        CHECK(!DrawingWorkspace::makeSuggestedLine(i, profiles[i].centerX, profiles[i].centerY,
                                                   1000.0f, customLine),
              "custom line rejects unreasonable length");
        DrawingWorkspace::SuggestedJob customSquare;
        CHECK(DrawingWorkspace::makeSuggestedShape(i, DrawingWorkspace::Shape::SQUARE,
                                                   profiles[i].centerX - lineSize * 0.5f,
                                                   profiles[i].centerY - lineSize * 0.5f,
                                                   lineSize, customSquare),
              "custom square preserves selectable start corner");
        DrawingWorkspace::SuggestedJob customCircle;
        CHECK(DrawingWorkspace::makeSuggestedShape(i, DrawingWorkspace::Shape::CIRCLE,
                                                   profiles[i].centerX + lineSize * 0.5f,
                                                   profiles[i].centerY,
                                                   lineSize, customCircle),
              "custom circle preserves selectable start point");
    }
    CHECK(validCount > 0, "at least one recommended drawing plane exists");

    if (g_fail == 0) {
        std::printf("ALL PASSED (drawing workspace, %u profiles)\n", validCount);
        return 0;
    }
    std::printf("%d FAILED\n", g_fail);
    return 1;
}
