#include "drawing_workspace.h"

#include <algorithm>
#include <cmath>

#include "config.h"
#include "kinematics.h"

namespace {
constexpr uint8_t X_CELLS = static_cast<uint8_t>(
    (DRAW_WORKSPACE_SCAN_X_MAX_MM - DRAW_WORKSPACE_SCAN_X_MIN_MM) / DRAW_WORKSPACE_SCAN_STEP_MM + 1.0f);
constexpr uint8_t Y_CELLS = static_cast<uint8_t>(
    (DRAW_WORKSPACE_SCAN_Y_MAX_MM - DRAW_WORKSPACE_SCAN_Y_MIN_MM) / DRAW_WORKSPACE_SCAN_STEP_MM + 1.0f);
constexpr uint8_t Z_SAMPLES = static_cast<uint8_t>(
    (DRAW_WORKSPACE_SCAN_Z_MAX_MM - DRAW_WORKSPACE_SCAN_Z_MIN_MM) / DRAW_WORKSPACE_SCAN_Z_STEP_MM + 1.0f);

static_assert(X_CELLS > 1 && Y_CELLS > 1, "Drawing workspace scan needs a 2D grid");

bool poseReachable(float x, float y, float z) {
    float joints[NUM_MOTORS];
    return kin::ikPenDown({x, y, z}, joints);
}

} // namespace

const std::array<DrawingWorkspace::Recommendation, DrawingWorkspace::kRecommendationCount>&
DrawingWorkspace::recommendations() {
    static const std::array<Recommendation, kRecommendationCount> values = analyze();
    return values;
}

bool DrawingWorkspace::isReachableWithLift(float x, float y, float z) {
    // Mỗi ô phải reachable cả lúc bút chạm giấy lẫn lúc planner nâng bút travel.
    return poseReachable(x, y, z) && poseReachable(x, y, z + PEN_LIFT_MM);
}

std::array<DrawingWorkspace::Recommendation, DrawingWorkspace::kRecommendationCount>
DrawingWorkspace::analyze() {
    std::array<Recommendation, kRecommendationCount> best{};
    std::array<Recommendation, Z_SAMPLES> candidates{};
    uint8_t candidateCount = 0;

    for (float z = DRAW_WORKSPACE_SCAN_Z_MIN_MM;
         z <= DRAW_WORKSPACE_SCAN_Z_MAX_MM + 0.001f;
         z += DRAW_WORKSPACE_SCAN_Z_STEP_MM) {
        uint8_t largest[Y_CELLS][X_CELLS]{};
        uint8_t bestCells = 0;
        uint8_t bestRow = 0;
        uint8_t bestCol = 0;

        for (uint8_t row = 0; row < Y_CELLS; ++row) {
            const float y = DRAW_WORKSPACE_SCAN_Y_MIN_MM + row * DRAW_WORKSPACE_SCAN_STEP_MM;
            for (uint8_t col = 0; col < X_CELLS; ++col) {
                const float x = DRAW_WORKSPACE_SCAN_X_MIN_MM + col * DRAW_WORKSPACE_SCAN_STEP_MM;
                if (!isReachableWithLift(x, y, z)) continue;

                const uint8_t top = row == 0 ? 0 : largest[row - 1][col];
                const uint8_t left = col == 0 ? 0 : largest[row][col - 1];
                const uint8_t diagonal = (row == 0 || col == 0) ? 0 : largest[row - 1][col - 1];
                largest[row][col] = static_cast<uint8_t>(1 + std::min({top, left, diagonal}));
                if (largest[row][col] > bestCells) {
                    bestCells = largest[row][col];
                    bestRow = row;
                    bestCol = col;
                }
            }
        }

        const float side = bestCells * DRAW_WORKSPACE_SCAN_STEP_MM;
        if (side < DRAW_PRESET_MIN_SQUARE_SIDE_MM) continue;
        const float firstX = DRAW_WORKSPACE_SCAN_X_MIN_MM +
                             (bestCol - bestCells + 1) * DRAW_WORKSPACE_SCAN_STEP_MM;
        const float firstY = DRAW_WORKSPACE_SCAN_Y_MIN_MM +
                             (bestRow - bestCells + 1) * DRAW_WORKSPACE_SCAN_STEP_MM;
        const Recommendation candidate{
            z,
            firstX + (bestCells - 1) * DRAW_WORKSPACE_SCAN_STEP_MM * 0.5f,
            firstY + (bestCells - 1) * DRAW_WORKSPACE_SCAN_STEP_MM * 0.5f,
            side,
            true,
        };

        if (candidateCount < candidates.size()) candidates[candidateCount++] = candidate;
    }

    // Chọn global maximum trước, sau đó mới ép các lựa chọn còn lại cách nhau đủ xa theo Z.
    for (uint8_t slot = 0; slot < best.size(); ++slot) {
        int selected = -1;
        for (uint8_t i = 0; i < candidateCount; ++i) {
            if (!candidates[i].valid) continue;
            bool distinctHeight = true;
            for (uint8_t previous = 0; previous < slot; ++previous) {
                if (fabsf(best[previous].z - candidates[i].z) < DRAW_PRESET_MIN_Z_SEPARATION_MM) {
                    distinctHeight = false;
                    break;
                }
            }
            if (distinctHeight && (selected < 0 ||
                candidates[i].maxSquareSide > candidates[static_cast<uint8_t>(selected)].maxSquareSide)) {
                selected = i;
            }
        }
        if (selected < 0) break;
        best[slot] = candidates[static_cast<uint8_t>(selected)];
        candidates[static_cast<uint8_t>(selected)].valid = false;
    }
    return best;
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

bool DrawingWorkspace::makeSuggestedLine(uint8_t profile, float startX, float startY,
                                          float length, SuggestedJob& out) {
    return makeSuggestedShape(profile, Shape::LINE, startX, startY, length, out);
}

bool DrawingWorkspace::makeSuggestedShape(uint8_t profile, Shape shape, float startX,
                                           float startY, float size, SuggestedJob& out) {
    const auto& profiles = recommendations();
    if (profile >= profiles.size() || !profiles[profile].valid ||
        !std::isfinite(startX) || !std::isfinite(startY) ||
        !std::isfinite(size) || size < DRAW_PRESET_MIN_SQUARE_SIDE_MM ||
        size > (DRAW_WORKSPACE_SCAN_X_MAX_MM - DRAW_WORKSPACE_SCAN_X_MIN_MM)) {
        return false;
    }
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
