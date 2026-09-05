#ifndef DRAWING_WORKSPACE_H
#define DRAWING_WORKSPACE_H

#include <array>
#include <cstdint>

/**
 * Gợi ý mặt phẳng vẽ ngang dựa trên đúng IK pen-down và giới hạn mềm hiện tại.
 * Không phụ thuộc Arduino/hardware, nên có thể kiểm thử trên host.
 */
class DrawingWorkspace {
public:
    static constexpr uint8_t kRecommendationCount = 3;

    struct Recommendation {
        float z{0.0f};
        float centerX{0.0f};
        float centerY{0.0f};
        float maxSquareSide{0.0f};
        bool valid{false};
    };

    enum class Shape : uint8_t { LINE, CIRCLE, SQUARE };

    struct SuggestedJob {
        Shape shape{Shape::LINE};
        float x{0.0f};
        float y{0.0f};
        float z{0.0f};
        float size{0.0f}; // line/square: chiều dài cạnh; circle: đường kính
    };

    [[nodiscard]] static const std::array<Recommendation, kRecommendationCount>& recommendations();
    [[nodiscard]] static bool makeSuggestedJob(uint8_t profile, Shape shape, SuggestedJob& out);
    [[nodiscard]] static bool makeSuggestedShape(uint8_t profile, Shape shape, float startX,
                                                  float startY, float size, SuggestedJob& out);
    [[nodiscard]] static bool makeSuggestedLine(uint8_t profile, float startX, float startY,
                                                 float length, SuggestedJob& out);

private:
    static std::array<Recommendation, kRecommendationCount> analyze();
    static bool isReachableWithLift(float x, float y, float z);
    static bool verifySuggestedJob(const SuggestedJob& job);
};

#endif // DRAWING_WORKSPACE_H
