#include "drawing_workspace.h"

#include <algorithm>
#include <cmath>

#include "config.h"
#include "kinematics.h"

namespace {
bool poseReachable(float x, float y, float z) {
    float joints[NUM_MOTORS];
    return kin::ikPenDown({x, y, z}, joints);
}
} // namespace

const std::array<DrawingWorkspace::Recommendation, DrawingWorkspace::kRecommendationCount>&
DrawingWorkspace::recommendations() {
    // Owner-selected single low plane: 160 mm arena at base Z=20 mm.
    static constexpr std::array<Recommendation, kRecommendationCount> values{{
        {DRAW_PLANE_Z_MM, 160.0f, 0.0f, 160.0f, true},
    }};
    return values;
}

bool DrawingWorkspace::isReachableWithLift(float x, float y, float z) {
    return poseReachable(x, y, z) && poseReachable(x, y, z + PEN_LIFT_MM);
}

bool DrawingWorkspace::verifySuggestedJob(const SuggestedJob& job) {
    const float half = job.size * 0.5f;
    const float step = DRAW_SEGMENT_MM;
    if (job.size < DRAW_PRESET_MIN_SQUARE_SIDE_MM) return false;

    if (job.shape == Shape::LINE) {
        for (float d = -half; d <= half + 0.001f; d += step) {
            if (!isReachableWithLift(job.x + std::min(d, half), job.y, job.z)) return false;
        }
        return true;
    }

    const uint16_t samples = job.shape == Shape::CIRCLE
        ? static_cast<uint16_t>(std::ceil(3.14159265f * job.size / step))
        : static_cast<uint16_t>(std::ceil(4.0f * job.size / step));
    for (uint16_t i = 0; i <= samples; ++i) {
        const float ratio = static_cast<float>(i) / samples;
        float x = job.x;
        float y = job.y;
        if (job.shape == Shape::CIRCLE) {
            const float angle = ratio * TWO_PI;
            x += half * cosf(angle);
            y += half * sinf(angle);
        } else {
            const float p = ratio * 4.0f * job.size;
            if (p <= job.size) { x += -half + p; y -= half; }
            else if (p <= 2.0f * job.size) { x += half; y += -half + (p - job.size); }
            else if (p <= 3.0f * job.size) { x += half - (p - 2.0f * job.size); y += half; }
            else { x -= half; y += half - (p - 3.0f * job.size); }
        }
        if (!isReachableWithLift(x, y, job.z)) return false;
    }
    return true;
}

bool DrawingWorkspace::makeSuggestedJob(uint8_t profile, Shape shape, SuggestedJob& out) {
    const auto& profiles = recommendations();
    if (profile >= profiles.size() || !profiles[profile].valid) return false;
    const Recommendation& recommendation = profiles[profile];
    const float size = floorf(recommendation.maxSquareSide * DRAW_PRESET_SAFE_SCALE_MM / 5.0f) * 5.0f;
    if (size < DRAW_PRESET_MIN_SQUARE_SIDE_MM) return false;
    out = {shape, recommendation.centerX, recommendation.centerY, recommendation.z, size};
    return verifySuggestedJob(out);
}

bool DrawingWorkspace::makeSuggestedShape(uint8_t profile, Shape shape, float startX,
                                           float startY, float size, SuggestedJob& out) {
    const auto& profiles = recommendations();
    if (profile >= profiles.size() || !profiles[profile].valid ||
        !std::isfinite(startX) || !std::isfinite(startY) ||
        !std::isfinite(size) || size < DRAW_PRESET_MIN_SQUARE_SIDE_MM ||
        size > DRAW_PRESET_MAX_SIZE_MM) return false;

    const Recommendation& recommendation = profiles[profile];
    const float half = size * 0.5f;
    if (shape == Shape::LINE) {
        out = {Shape::LINE, startX + half, startY, recommendation.z, size};
    } else if (shape == Shape::SQUARE) {
        out = {Shape::SQUARE, startX + half, startY + half, recommendation.z, size};
    } else if (shape == Shape::CIRCLE) {
        out = {Shape::CIRCLE, startX - half, startY, recommendation.z, size};
    } else {
        return false;
    }
    return verifySuggestedJob(out);
}
