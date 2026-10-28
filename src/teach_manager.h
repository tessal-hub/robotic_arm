#ifndef TEACH_MANAGER_H
#define TEACH_MANAGER_H

#include <Arduino.h>
#include <vector>
#include <mutex>
#include <atomic>
#include "config.h"

class Motor;
class JointModel;
class SafetyManager;
class NvsStore;

struct TrajectoryPoint {
    float j1{0.0f};
    float j2{0.0f};
    float j3{0.0f};
    bool penDown{true}; // true: nếp vẽ (chạm giấy); false: di chuyển nâng bút (travel)
};

class TeachManager {
public:
    static constexpr size_t MAX_POINTS = 250;

    enum class PlayState : uint8_t {
        IDLE = 0,
        LEAD_IN,
        PLAYING,
        PAUSED
    };

    TeachManager();

    void begin(Motor** motors, JointModel* joints, SafetyManager* safety);

    // ---- TEACH MODE ----
    void setFree(bool free);
    [[nodiscard]] bool isFree() const noexcept { return freeMode_.load(std::memory_order_acquire); }
    bool recordPoint(bool penDown = true);
    bool undoPoint();
    void clearPoints();
    [[nodiscard]] size_t pointCount() const;
    [[nodiscard]] TrajectoryPoint getPoint(size_t index) const;

    // JSON export / import
    String getPointsJson() const;
    bool loadPointsJson(const String& json);

    // NVS Flash persistence
    bool saveToNvs(NvsStore* nvs);
    bool loadFromNvs(NvsStore* nvs);

    // Bù lực đè bút Z khi vẽ nét (chống bay khỏi mặt giấy khi vươn ra ngoài)
    void setPenPressDeg(float deg) { penPressDeg_ = (deg >= 0.0f && deg <= 4.0f) ? deg : 0.35f; }
    [[nodiscard]] float penPressDeg() const noexcept { return penPressDeg_; }

    // ---- PLAYBACK MODE ----
    bool startPlayback(float stepDeg = 0.5f, uint32_t speedDelayMs = 15);
    void stopPlayback();
    void pausePlayback();
    void resumePlayback();
    [[nodiscard]] bool isPlaying() const noexcept { return playState_.load(std::memory_order_acquire) != PlayState::IDLE; }
    [[nodiscard]] PlayState playState() const noexcept { return playState_.load(std::memory_order_acquire); }
    [[nodiscard]] size_t currentPointIndex() const noexcept { return currentSegment_; }

    // Khóa cứng & giữ chủ động cổ tay J4..J6
    void lockWristAxes();

    // Gọi định kỳ trong motion task Core 1 (100Hz)
    void tick();

    String statusJson() const;

private:
    Motor* motors_[NUM_MOTORS]{};
    JointModel* jm_{nullptr};
    SafetyManager* safety_{nullptr};

    mutable std::mutex mutex_;
    std::vector<TrajectoryPoint> points_;
    std::atomic<bool> freeMode_{false};

    std::atomic<PlayState> playState_{PlayState::IDLE};
    PlayState stateBeforePause_{PlayState::PLAYING}; // Lưu trạng thái trước khi Pause
    size_t currentSegment_{0};
    uint32_t subStepIdx_{0};
    uint32_t totalSubSteps_{0};
    float stepDeg_{0.5f};
    uint32_t speedDelayMs_{15};
    float penPressDeg_{0.35f}; // Bù lực đè bút 0.35° (~1.5mm) êm ái khi penDown, không cày rách giấy khi rê ngang
    uint32_t lastStepMs_{0};

    TrajectoryPoint segStart_{};
    TrajectoryPoint segEnd_{};

    void setFreeInternal(bool free);
    bool executeJointTarget(float targetJ1, float targetJ2, float targetJ3, uint32_t durationMs);
};

#endif // TEACH_MANAGER_H
