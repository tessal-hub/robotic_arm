#include "teach_manager.h"
#include "motor.h"
#include "joint_model.h"
#include "safety_manager.h"
#include "nvs_store.h"
#include <cmath>
#include <algorithm>

TeachManager::TeachManager() = default;

void TeachManager::begin(Motor** motors, JointModel* joints, SafetyManager* safety) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors_[i] = motors[i];
    jm_ = joints;
    safety_ = safety;
    lockWristAxes();
}

void TeachManager::lockWristAxes() {
    // Khóa cứng tĩnh J4, J5, J6: dừng mọi xung STEP và bật driver giữ mô-men xoắn tối đa.
    // Tuyệt đối KHÔNG đọc encoder để phát xung bù góc (tránh trôi absSteps và tránh rung cổ tay).
    for (uint8_t a = 3; a < NUM_MOTORS; ++a) {
        if (motors_[a] != nullptr) {
            motors_[a]->stop();
            motors_[a]->enable(true);
        }
    }

    // Tăng dòng giữ tĩnh (ihold) cho TMC2209 của J4 (axis 3) lên ~90% (28/31)
    if (motors_[3] != nullptr && motors_[3]->isTmc()) {
        motors_[3]->setHoldCurrentScale(WRIST_J4_HOLD_SCALE);
    }
}

void TeachManager::setFree(bool free) {
    std::lock_guard<std::mutex> lock(mutex_);
    setFreeInternal(free);
}

void TeachManager::setFreeInternal(bool free) {
    if (playState_ != PlayState::IDLE) {
        playState_ = PlayState::IDLE;
        for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
            if (motors_[a] != nullptr) motors_[a]->stop();
        }
    }

    if (free) {
        for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
            if (motors_[a] != nullptr) motors_[a]->stop();
        }
        // Chỉ thả lỏng J1, J2, J3 (index 0, 1, 2) để kéo tay
        for (uint8_t a = 0; a < 3; ++a) {
            if (motors_[a] != nullptr) motors_[a]->enable(false);
        }
        // Khóa cứng tĩnh J4, J5, J6
        lockWristAxes();

        if (safety_ != nullptr) safety_->assertManualRelease(true);
        freeMode_ = true;
        Serial.println("[TEACH] J1..J3 tha long de keo tay. J4..J6 khoa cung tinh.");
    } else {
        // Khóa lại mô-men giữ cho J1..J3 và kiểm tra lỗi UART
        bool enableOk = true;
        for (uint8_t a = 0; a < 3; ++a) {
            if (motors_[a] != nullptr && !motors_[a]->enable(true)) {
                Serial.printf("[TEACH] LOI UART: Khong bat lai duoc dong giu J%u!\n", a + 1);
                enableOk = false;
            }
        }

        // Đồng bộ bước motor J1..J3 theo encoder thực tế vừa kéo
        if (jm_ != nullptr) {
            for (uint8_t a = 0; a < 3; ++a) {
                if (!jm_->resyncFromEncoder(a)) {
                    Serial.printf("[TEACH] CANH BAO: Khong resync duoc encoder J%u\n", a + 1);
                }
            }
        }
        lockWristAxes();
        if (safety_ != nullptr) {
            safety_->assertManualRelease(false);
            if (!enableOk) safety_->notifyFault("teach lock UART fail");
        }
        freeMode_ = false;
        Serial.println("[TEACH] J1..J3 da bat lai dong giu. J4..J6 khoa cung tinh.");
    }
}

bool TeachManager::recordPoint(bool penDown) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (jm_ == nullptr || points_.size() >= MAX_POINTS) return false;
    for (uint8_t a = 0; a < 3; ++a) {
        if (!jm_->isHomed(a)) {
            Serial.printf("[TEACH] J%u chua Home - tu choi ghi diem\n", a + 1);
            return false;
        }
    }

    TrajectoryPoint p;
    if (!freeMode_) {
        // Nguồn chính xác tuyệt đối khi Jog/Web: lấy trực tiếp từ bước máy
        p.j1 = jm_->angleFromSteps(0);
        p.j2 = jm_->angleFromSteps(1);
        p.j3 = jm_->angleFromSteps(2);
    } else {
        // Chỉ khi đang thả lỏng tay kéo tự do mới đọc từ encoder
        p.j1 = jm_->angleFromEncoder(0);
        p.j2 = jm_->angleFromEncoder(1);
        p.j3 = jm_->angleFromEncoder(2);
    }
    p.penDown = penDown;
    points_.push_back(p);

    Serial.printf("[TEACH] Da ghi diem #%u [%s]: J1=%.2f, J2=%.2f, J3=%.2f (nguon=%s)\n",
                  static_cast<unsigned>(points_.size()),
                  penDown ? "VE (DOWN)" : "NHAC BUT (UP)",
                  p.j1, p.j2, p.j3,
                  freeMode_ ? "ENCODER" : "STEPS");
    return true;
}

bool TeachManager::undoPoint() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (points_.empty()) return false;
    points_.pop_back();
    Serial.printf("[TEACH] Da xoa diem gan nhat. Con lai %u diem.\n",
                  static_cast<unsigned>(points_.size()));
    return true;
}

void TeachManager::clearPoints() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (playState_ != PlayState::IDLE) {
        playState_ = PlayState::IDLE;
        for (uint8_t a = 0; a < 3; ++a) {
            if (motors_[a] != nullptr) motors_[a]->stop();
        }
    }
    points_.clear();
    Serial.println("[TEACH] Da xoa toan bo danh sach diem.");
}

size_t TeachManager::pointCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return points_.size();
}

TrajectoryPoint TeachManager::getPoint(size_t index) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (index >= points_.size()) return {};
    return points_[index];
}

String TeachManager::getPointsJson() const {
    std::lock_guard<std::mutex> lock(mutex_);
    String s = "[";
    for (size_t i = 0; i < points_.size(); ++i) {
        if (i > 0) s += ",";
        char buf[64];
        snprintf(buf, sizeof(buf), "[%.2f,%.2f,%.2f,%d]",
                 points_[i].j1, points_[i].j2, points_[i].j3, points_[i].penDown ? 1 : 0);
        s += buf;
    }
    s += "]";
    return s;
}

bool TeachManager::loadPointsJson(const String& json) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (playState_ != PlayState::IDLE) {
        playState_ = PlayState::IDLE;
        for (uint8_t a = 0; a < 3; ++a) {
            if (motors_[a] != nullptr) motors_[a]->stop();
        }
    }

    std::vector<TrajectoryPoint> newPoints;
    const char* p = json.c_str();
    while (*p) {
        p = strchr(p, '[');
        if (!p) break;
        ++p;
        if (*p == '[') continue;
        char* end = nullptr;
        float v1 = strtof(p, &end);
        if (end == p) { p = end + 1; continue; }
        p = end;
        while (*p == ',' || *p == ' ') ++p;
        float v2 = strtof(p, &end);
        if (end == p) { p = end + 1; continue; }
        p = end;
        while (*p == ',' || *p == ' ') ++p;
        float v3 = strtof(p, &end);
        if (end == p) { p = end + 1; continue; }
        p = end;

        bool pen = true;
        while (*p == ',' || *p == ' ') ++p;
        if (*p >= '0' && *p <= '9') {
            int penVal = static_cast<int>(strtol(p, &end, 10));
            pen = (penVal != 0);
            p = end;
        }

        if (!std::isfinite(v1) || !std::isfinite(v2) || !std::isfinite(v3)) {
            p = strchr(p, ']');
            if (p) ++p;
            continue;
        }
        v1 = std::clamp(v1, J1_MIN_LIMIT, J1_MAX_LIMIT);
        v2 = std::clamp(v2, J2_MIN_LIMIT, J2_MAX_LIMIT);
        v3 = std::clamp(v3, J3_MIN_LIMIT, J3_MAX_LIMIT);

        newPoints.push_back({v1, v2, v3, pen});
        if (newPoints.size() >= MAX_POINTS) {
            Serial.printf("[TEACH] Dat gioi han toi da %u diem, bo qua cac diem tiep theo.\n",
                          static_cast<unsigned>(MAX_POINTS));
            break;
        }
        p = strchr(p, ']');
        if (p) ++p;
    }
    if (newPoints.empty()) return false;
    points_ = std::move(newPoints);
    Serial.printf("[TEACH] Nap thanh cong %u diem tu JSON\n",
                  static_cast<unsigned>(points_.size()));
    return true;
}

bool TeachManager::saveToNvs(NvsStore* nvs) {
    if (nvs == nullptr) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (points_.empty()) return nvs->clearTeachTrajectory();
    const size_t byteSize = points_.size() * sizeof(TrajectoryPoint);
    return nvs->saveTeachTrajectory(points_.data(), byteSize);
}

bool TeachManager::loadFromNvs(NvsStore* nvs) {
    if (nvs == nullptr) return false;
    std::vector<TrajectoryPoint> loaded(MAX_POINTS);
    const size_t bytesRead = nvs->loadTeachTrajectory(loaded.data(), MAX_POINTS * sizeof(TrajectoryPoint));
    if (bytesRead == 0 || (bytesRead % sizeof(TrajectoryPoint)) != 0) return false;
    const size_t count = bytesRead / sizeof(TrajectoryPoint);
    loaded.resize(count);

    for (auto& pt : loaded) {
        if (!std::isfinite(pt.j1) || !std::isfinite(pt.j2) || !std::isfinite(pt.j3)) {
            return false;
        }
        pt.j1 = std::clamp(pt.j1, J1_MIN_LIMIT, J1_MAX_LIMIT);
        pt.j2 = std::clamp(pt.j2, J2_MIN_LIMIT, J2_MAX_LIMIT);
        pt.j3 = std::clamp(pt.j3, J3_MIN_LIMIT, J3_MAX_LIMIT);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    points_ = std::move(loaded);
    Serial.printf("[TEACH] Nap thanh cong %u diem tu Flash (NVS)\n",
                  static_cast<unsigned>(points_.size()));
    return true;
}

bool TeachManager::startPlayback(float stepDeg, uint32_t speedDelayMs) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (points_.size() < 2 || jm_ == nullptr) {
        Serial.println("[PLAY] Can it nhat 2 diem de chay lai.");
        return false;
    }

    for (uint8_t a = 0; a < 3; ++a) {
        if (!jm_->isHomed(a)) {
            Serial.printf("[PLAY] Tu choi chay: J%u chua Home!\n", a + 1);
            return false;
        }
    }

    if (freeMode_) {
        setFreeInternal(false); // Khóa lại motor và resync encoder an toàn (khi trước đó đang kéo tay)
    }

    stepDeg_ = std::max(0.05f, stepDeg);
    speedDelayMs_ = std::max(5U, speedDelayMs);

    // Chuẩn bị pha LEAD_IN: Tiếp cận điểm đầu tiên
    segStart_ = {
        jm_->angleFromSteps(0),
        jm_->angleFromSteps(1),
        jm_->angleFromSteps(2),
        false
    };
    segEnd_ = points_[0];

    const float maxDelta = std::max({
        fabsf(segEnd_.j1 - segStart_.j1),
        fabsf(segEnd_.j2 - segStart_.j2),
        fabsf(segEnd_.j3 - segStart_.j3)
    });
    totalSubSteps_ = std::max(1U, static_cast<uint32_t>(ceilf(maxDelta / stepDeg_)));
    subStepIdx_ = 0;
    currentSegment_ = 0;
    playState_ = PlayState::LEAD_IN;
    lastStepMs_ = millis();

    Serial.printf("[PLAY] Bat dau chay lai %u diem (lead-in: %u buoc toi diem #1)...\n",
                  static_cast<unsigned>(points_.size()), totalSubSteps_);
    return true;
}

void TeachManager::stopPlayback() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (playState_ == PlayState::IDLE) return;
    playState_ = PlayState::IDLE;
    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
        if (motors_[a] != nullptr) motors_[a]->stop();
    }
    Serial.println("[PLAY] Da dung chay lai.");
}

void TeachManager::pausePlayback() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (playState_ == PlayState::PLAYING || playState_ == PlayState::LEAD_IN) {
        stateBeforePause_ = playState_; // Lưu đúng pha hiện tại (LEAD_IN hoặc PLAYING)
        playState_ = PlayState::PAUSED;
        for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
            if (motors_[a] != nullptr) motors_[a]->stop();
        }
        Serial.println("[PLAY] Tam dung.");
    }
}

void TeachManager::resumePlayback() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (playState_ == PlayState::PAUSED) {
        playState_ = stateBeforePause_; // Khôi phục đúng pha trước khi Pause
        subStepIdx_ = 0;
        lastStepMs_ = millis();
        Serial.println("[PLAY] Tiep tuc.");
    }
}

bool TeachManager::executeJointTarget(float targetJ1, float targetJ2, float targetJ3, uint32_t durationMs) {
    if (jm_ == nullptr) return false;
    const float targets[3] = {targetJ1, targetJ2, targetJ3};
    int64_t steps[3] = {0, 0, 0};
    float deltas[3] = {0.0f, 0.0f, 0.0f};
    uint32_t maxSteps = 1;
    bool anyMove = false;

    // Thời gian tối thiểu cần thiết để không trục nào vượt quá tốc độ Jog an toàn (DEFAULT_AXIS_JOG_SPEEDS)
    float requiredDurationUs = static_cast<float>(durationMs) * 1000.0f;

    for (uint8_t a = 0; a < 3; ++a) {
        if (motors_[a] == nullptr) continue;
        const float curDeg = jm_->angleFromSteps(a);
        deltas[a] = targets[a] - curDeg;
        steps[a] = JointModel::degreesToSteps(a, fabsf(deltas[a]));
        if (steps[a] > 0) {
            anyMove = true;
            if (static_cast<uint32_t>(steps[a]) > maxSteps) {
                maxSteps = static_cast<uint32_t>(steps[a]);
            }
            const float axisMinDurationUs =
                static_cast<float>(steps[a]) * static_cast<float>(DEFAULT_AXIS_JOG_SPEEDS[a]);
            if (axisMinDurationUs > requiredDurationUs) {
                requiredDurationUs = axisMinDurationUs;
            }
        }
    }

    if (!anyMove) return true;

    uint32_t dominantIntervalUs = static_cast<uint32_t>(requiredDurationUs / static_cast<float>(maxSteps));
    if (dominantIntervalUs < MIN_STEP_INTERVAL_US) dominantIntervalUs = MIN_STEP_INTERVAL_US;
    // Cho phép interval tới 25000 us (40 steps/s) trong chế độ vẽ quỹ đạo để nét vẽ đầm, không bị giật nảy
    constexpr uint32_t MAX_TRAJ_INTERVAL_US = 25000;
    if (dominantIntervalUs > MAX_TRAJ_INTERVAL_US) dominantIntervalUs = MAX_TRAJ_INTERVAL_US;

    // Tính lại tổng thời gian thực tế theo dominantIntervalUs đã kẹp
    const float actualDurationUs = static_cast<float>(dominantIntervalUs) * static_cast<float>(maxSteps);

    for (uint8_t a = 0; a < 3; ++a) {
        if (motors_[a] == nullptr || steps[a] <= 0) continue;
        const bool cw = JointModel::cwForDelta(a, deltas[a]);
        uint32_t intervalUs = static_cast<uint32_t>(actualDurationUs / static_cast<float>(steps[a]));
        if (intervalUs < DEFAULT_AXIS_JOG_SPEEDS[a]) intervalUs = DEFAULT_AXIS_JOG_SPEEDS[a];

        // Bật ramp S-curve đồng bộ nếu bước di chuyển dài (> 80 microsteps)
        const uint32_t rampStartUs = (maxSteps > 80) ? (intervalUs * 2U) : 0U;

        if (!motors_[a]->prepareCoordinatedRun(cw, static_cast<uint32_t>(steps[a]), intervalUs, rampStartUs)) {
            for (uint8_t j = 0; j < 3; ++j) {
                if (motors_[j] != nullptr) motors_[j]->stop();
            }
            playState_ = PlayState::IDLE;
            Serial.printf("[PLAY] LOI: J%u prepareCoordinatedRun that bai -> DUNG PLAYBACK\n", a + 1);
            return false;
        }
    }

    for (uint8_t a = 0; a < 3; ++a) {
        if (motors_[a] != nullptr && steps[a] > 0) {
            (void)motors_[a]->startPreparedRun();
        }
    }
    return true;
}

void TeachManager::tick() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (playState_ == PlayState::IDLE || playState_ == PlayState::PAUSED) return;

    // Chờ các motor chạy xong bước con trước đó
    for (uint8_t a = 0; a < 3; ++a) {
        if (motors_[a] != nullptr && motors_[a]->isRunning()) return;
    }

    const uint32_t now = millis();

    if (playState_ == PlayState::LEAD_IN) {
        if (subStepIdx_ == 0) {
            subStepIdx_ = totalSubSteps_;
            const uint32_t totalDurationMs = totalSubSteps_ * speedDelayMs_ * 2U;
            const float j2Comp = segEnd_.penDown ? penPressDeg_ : 0.0f;
            if (!executeJointTarget(segEnd_.j1, segEnd_.j2 + j2Comp, segEnd_.j3, totalDurationMs)) return;
            lastStepMs_ = now;
        } else {
            // Đã tới điểm đầu tiên an toàn -> Bắt đầu vẽ từ điểm 0 sang điểm 1
            playState_ = PlayState::PLAYING;
            currentSegment_ = 0;
            segStart_ = points_[0];
            segEnd_ = points_[1];
            const float maxDelta = std::max({
                fabsf(segEnd_.j1 - segStart_.j1),
                fabsf(segEnd_.j2 - segStart_.j2),
                fabsf(segEnd_.j3 - segStart_.j3)
            });
            totalSubSteps_ = std::max(1U, static_cast<uint32_t>(ceilf(maxDelta / stepDeg_)));
            subStepIdx_ = 0;
            lastStepMs_ = now;
            Serial.println("[PLAY] Da tiep can diem #1. Bat dau ve...");
        }
        return;
    }

    if (playState_ == PlayState::PLAYING) {
        if (subStepIdx_ == 0) {
            subStepIdx_ = totalSubSteps_;
            const uint32_t stepDur = segEnd_.penDown ? speedDelayMs_ : (speedDelayMs_ * 4U / 5U);
            const uint32_t totalDurationMs = totalSubSteps_ * stepDur;
            const float j2Comp = segEnd_.penDown ? penPressDeg_ : 0.0f;
            if (!executeJointTarget(segEnd_.j1, segEnd_.j2 + j2Comp, segEnd_.j3, totalDurationMs)) return;
            lastStepMs_ = now;
        } else {
            // Hoàn tất phân đoạn hiện tại -> chuyển sang phân đoạn kế tiếp
            ++currentSegment_;
            if (currentSegment_ >= points_.size() - 1) {
                playState_ = PlayState::IDLE;
                Serial.printf("[PLAY] Hoan tat toan bo quy dao (%u diem)!\n",
                              static_cast<unsigned>(points_.size()));
                return;
            }

            segStart_ = points_[currentSegment_];
            segEnd_ = points_[currentSegment_ + 1];
            const float maxDelta = std::max({
                fabsf(segEnd_.j1 - segStart_.j1),
                fabsf(segEnd_.j2 - segStart_.j2),
                fabsf(segEnd_.j3 - segStart_.j3)
            });
            totalSubSteps_ = std::max(1U, static_cast<uint32_t>(ceilf(maxDelta / stepDeg_)));
            subStepIdx_ = 0;
            lastStepMs_ = now;
        }
    }
}

String TeachManager::statusJson() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool curPen = (playState_ == PlayState::PLAYING && (currentSegment_ + 1) < points_.size()) ?
                        points_[currentSegment_ + 1].penDown : true;
    char buf[160];
    snprintf(buf, sizeof(buf),
             "{\"count\":%u,\"free\":%s,\"playing\":%s,\"state\":%u,\"segment\":%u,\"pen\":%s,\"pressDeg\":%.2f}",
             static_cast<unsigned>(points_.size()),
             freeMode_.load(std::memory_order_relaxed) ? "true" : "false",
             playState_.load(std::memory_order_relaxed) != PlayState::IDLE ? "true" : "false",
             static_cast<unsigned>(playState_.load(std::memory_order_relaxed)),
             static_cast<unsigned>(currentSegment_),
             curPen ? "true" : "false",
             penPressDeg_);
    return String(buf);
}
