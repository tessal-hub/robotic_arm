#include "drawing_workspace.h"
#include "config.h"
#include "kinematics.h"

#include <cmath>
#include <cstdio>

static int g_fail = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { std::printf("FAIL: %s (line %d)\n", msg, __LINE__); ++g_fail; } \
} while (0)

int main() {
    const auto& profiles = DrawingWorkspace::recommendations();
    CHECK(profiles.size() == 1 && std::fabs(profiles[0].z - DRAW_PLANE_Z_MM) < 0.01f &&
          profiles[0].maxSquareSide == 160.0f,
          "only drawing plane is low Z=20 mm with 160 mm arena");
    uint8_t validCount = 0;
    for (uint8_t i = 0; i < profiles.size(); ++i) {
        if (!profiles[i].valid) continue;
        ++validCount;
        CHECK(profiles[i].maxSquareSide >= 40.0f, "profile has usable inscribed square");
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
        CHECK(DrawingWorkspace::makeSuggestedShape(i, DrawingWorkspace::Shape::LINE,
                                                  profiles[i].centerX - lineSize * 0.5f,
                                                  profiles[i].centerY,
                                                  lineSize,
                                                  customLine),
              "custom start point line stays reachable");
        CHECK(customLine.shape == DrawingWorkspace::Shape::LINE &&
              std::fabs(customLine.x - profiles[i].centerX) < 0.01f,
              "custom line preserves requested start and length");
        CHECK(!DrawingWorkspace::makeSuggestedShape(i, DrawingWorkspace::Shape::LINE, profiles[i].centerX, profiles[i].centerY,
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

    // WRITE HELLO default: normalized width is 6.4 units, height 1.6 units.
    struct Stroke { float x1, y1, x2, y2; };
    constexpr Stroke hello[] = {
        {0,0,0,1.6f}, {1,0,1,1.6f}, {0,.8f,1,.8f},
        {1.35f,0,1.35f,1.6f}, {1.35f,1.6f,2.35f,1.6f}, {1.35f,.8f,2.15f,.8f}, {1.35f,0,2.35f,0},
        {2.7f,1.6f,2.7f,0}, {2.7f,0,3.7f,0},
        {4.05f,1.6f,4.05f,0}, {4.05f,0,5.05f,0},
        {5.4f,0,5.4f,1.6f}, {5.4f,1.6f,6.4f,1.6f}, {6.4f,1.6f,6.4f,0}, {6.4f,0,5.4f,0},
    };
    const float helloWidth = std::floor(profiles[0].maxSquareSide * 0.60f / 5.0f) * 5.0f;
    const float unit = helloWidth / 6.4f;
    const float helloX = profiles[0].centerX - helloWidth * 0.5f;
    const float helloY = profiles[0].centerY;
    const auto reachableWithLift = [](float x, float y, float z) {
        float joints[NUM_MOTORS];
        return kin::ikPenDown({x, y, z}, joints) &&
               kin::ikPenDown({x, y, z + PEN_LIFT_MM}, joints);
    };
    for (const auto& stroke : hello) {
        const float dx = (stroke.x2 - stroke.x1) * unit;
        const float dy = (stroke.y2 - stroke.y1) * unit;
        const int samples = static_cast<int>(std::ceil(std::hypot(dx, dy) / DRAW_LINE_SEGMENT_MM));
        for (int i = 0; i <= samples; ++i) {
            const float t = static_cast<float>(i) / samples;
            CHECK(reachableWithLift(helloX + (stroke.x1 * unit) + dx * t,
                                    helloY + (stroke.y1 * unit) + dy * t,
                                    profiles[0].z),
                  "default HELLO stroke stays reachable while drawing and lifting");
        }
    }

    if (g_fail == 0) {
        std::printf("ALL PASSED (drawing workspace, %u profiles)\n", validCount);
        return 0;
    }
    std::printf("%d FAILED\n", g_fail);
    return 1;
}
