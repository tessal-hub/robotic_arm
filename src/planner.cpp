#include "planner.h"
#include <cmath>
#include "differential_wrist.h"
#include "joint_model.h"
#include "kinematics.h"
#include "motor.h"
#include "work_plane.h"

namespace {
inline bool motorsBusy(Motor** m) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i)
        if (m[i]->isRunning()) return true;
    return false;
}
} // namespace

Planner::Planner() {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = nullptr;
}

void Planner::begin(Motor** motors_, JointModel* joints_) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = motors_[i];
    jm = joints_;
}

bool Planner::submit(const Job& job) {
    if (isActive()) return false;
    if (job.shape == Shape::NONE) return false;
    if (!syncWristFeedback()) return false;

    // Vị trí Cartesian xuất phát = TCP hiện tại theo FK (hoặc UCS nếu WorkPlane bật)
    float enc[6];
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) enc[i] = jm->angleFromSteps(i);
    const kin::FkResult fkNow = kin::forward(enc);
    if (workPlane != nullptr && workPlane->isEnabled()) {
        const Point3D ucsNow = workPlane->fromRobotXYZ({fkNow.tcp.x, fkNow.tcp.y, fkNow.tcp.z});
        curX_ = ucsNow.x;
        curY_ = ucsNow.y;
        curZ_ = ucsNow.z;
    } else {
        curX_ = fkNow.tcp.x;
        curY_ = fkNow.tcp.y;
        curZ_ = fkNow.tcp.z;
    }

    // Pre-flight lightweight B validation (§3.4) — reject out-of-reach BEFORE moving, HTTP 400
    job_ = job;

    switch (job_.shape) {
        case Shape::POINT: {
            totalLen_ = 0.0f;
            prog_ = 0.0f;
            break;
        }
        case Shape::LINE: {
            const float dx = job_.x2 - job_.x1;
            const float dy = job_.y2 - job_.y1;
            totalLen_ = sqrtf(dx * dx + dy * dy);
            if (totalLen_ < 1e-3f) return false;
            prog_ = 0.0f;
            break;
        }
        case Shape::CIRCLE: {
            if (job_.r <= 0.0f) return false;
            const float dxr = curX_ - job_.x1; // x1=cx
            const float dyr = curY_ - job_.y1; // y1=cy
            startAng_ = atan2f(dyr, dxr);
            sweep_ = TWO_PI; // vòng tròn đầy đủ, CCW
            totalLen_ = sweep_ * job_.r;
            prog_ = 0.0f;
            break;
        }
        case Shape::SQUARE: {
            if (job_.r <= 0.0f) return false;
            totalLen_ = 4.0f * job_.r;
            prog_ = 0.0f;
            break;
        }
        default:
            return false;
    }

    hasJob_ = true;
    state_ = State::LIFTING;
    segDone_ = 0;
    const char* shapeName = (job_.shape == Shape::LINE)    ? "LINE"
                            : (job_.shape == Shape::CIRCLE) ? "CIRCLE"
                            : (job_.shape == Shape::SQUARE) ? "SQUARE"
                                                            : "POINT";
    Serial.printf("[PLAN] Job %s: len=%.1fmm feed=%.1fmm/s (WorkPlane: %s)\n",
                  shapeName, totalLen_, job_.feedMmS,
                  (workPlane && workPlane->isEnabled()) ? "ENABLED" : "OFF");
    return true;
}

void Planner::stop() {
    if (!hasJob_) return;
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i]->stop();
    hasJob_ = false;
    state_ = State::IDLE;
    Serial.println("[PLAN] STOP");
}

bool Planner::syncWristFeedback() {
    if (!jm->isHomed(4) || !jm->isHomed(5) || !jm->encOK(4) || !jm->encOK(5)) {
        Serial.println("[PLAN] LOI: J5/J6 can Set Home va 2 encoder khoe de chay Cartesian/Draw");
        stop();
        return false;
    }
    return jm->resyncFromEncoder(4) && jm->resyncFromEncoder(5);
}

bool Planner::startMoveTo(float x, float y, float z, float feedMmS) {
    float rx = x;
    float ry = y;
    float rz = z;
    if (workPlane != nullptr && workPlane->isEnabled()) {
        // UCS transform: (x,y) are U,V on plane, wLift = z - job_.z is height above paper
        // job_.z = paper plane Z in UCS. Lift: job_.z + PEN_LIFT, so wLift = lift.
        // toRobotXYZ(u,v,wLift) = origin + u*uAxis + v*vAxis + wLift*normal
        const Point3D robotP = workPlane->toRobotXYZ(x, y, z - job_.z);
        rx = robotP.x;
        ry = robotP.y;
        rz = robotP.z;
    }

    float target[6];
    kin::Pose p{rx, ry, rz};
    if (!kin::ikPenDown(p, target)) {
        Serial.printf("[PLAN] LOI: diem (%.1f, %.1f, %.1f) [Robot: %.1f, %.1f, %.1f] ngoai vung lam viec\n",
                      x, y, z, rx, ry, rz);
        stop();
        return false;
    }

    // Encoder-anchored wrist: mọi waypoint bắt đầu từ vị trí actuator AS5600
    // thực tế, nên sai số/mất bước A4988 được bù ở segment kế tiếp thay vì tích lũy.
    if (!syncWristFeedback()) return false;

    // Bước step từng trục + trục chủ đạo (nhiều step nhất)
    int64_t steps[NUM_MOTORS];
    float deltaAct[NUM_MOTORS];
    uint32_t maxSteps = 1;
    bool anyMove = false;

    // Khớp J1..J4 (dẫn động trực tiếp)
    for (uint8_t i = 0; i < 4; ++i) {
        if (!jm->isHomed(i)) { stop(); return false; }
        const float curDeg = jm->actuatorAngleFromSteps(i);
        deltaAct[i] = target[i] - curDeg;
        steps[i] = JointModel::degreesToSteps(i, fabsf(deltaAct[i]));
        if (steps[i] > 0) anyMove = true;
        if (steps[i] > 0 && static_cast<uint32_t>(steps[i]) > maxSteps)
            maxSteps = static_cast<uint32_t>(steps[i]);
    }

    // Khớp J5, J6 qua cơ cấu Vi sai Bánh răng Côn (Differential Wrist)
    {
        const wrist::ActuatorState actTarget = wrist::inverse(target[4], target[5]);
        const float curM5 = jm->actuatorAngleFromSteps(4);
        const float curM6 = jm->actuatorAngleFromSteps(5);
        deltaAct[4] = actTarget.leftDeg - curM5;
        deltaAct[5] = actTarget.rightDeg - curM6;

        for (uint8_t i = 4; i < 6; ++i) {
            steps[i] = JointModel::degreesToSteps(i, fabsf(deltaAct[i]));
            if (steps[i] > 0) anyMove = true;
            if (steps[i] > 0 && static_cast<uint32_t>(steps[i]) > maxSteps)
                maxSteps = static_cast<uint32_t>(steps[i]);
        }
    }

    if (!anyMove) return true; // đã ở đúng vị trí

    // Thời gian segment từ feed (mm/s): T_us = 1e6 * segLen / feed.
    // Trục chủ đạo (nhiều step nhất) dùng interval = T_us / steps_max,
    // các trục khác scale để cùng kết thúc trong T_us.
    float segLenMm = DRAW_SEGMENT_MM;
    {
        float enc[6];
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) enc[i] = jm->angleFromSteps(i);
        const kin::FkResult fkNow = kin::forward(enc);
        const float dx = rx - fkNow.tcp.x;
        const float dy = ry - fkNow.tcp.y;
        const float dz = rz - fkNow.tcp.z;
        segLenMm = sqrtf(dx * dx + dy * dy + dz * dz);
        if (segLenMm < 0.05f) segLenMm = 0.05f;
    }

    uint32_t dominantIntervalUs =
        static_cast<uint32_t>(1000000.0f * segLenMm / (feedMmS * static_cast<float>(maxSteps)));
    if (dominantIntervalUs < MIN_STEP_INTERVAL_US) dominantIntervalUs = MIN_STEP_INTERVAL_US;
    if (dominantIntervalUs > MAX_STEP_INTERVAL_US) dominantIntervalUs = MAX_STEP_INTERVAL_US;

    uint32_t intervals[NUM_MOTORS]{};
    bool directions[NUM_MOTORS]{};
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (steps[i] <= 0) continue;
        const float scale = static_cast<float>(steps[i]) / static_cast<float>(maxSteps);
        uint32_t interval = static_cast<uint32_t>(dominantIntervalUs / scale);
        if (interval < MIN_STEP_INTERVAL_US) interval = MIN_STEP_INTERVAL_US;
        intervals[i] = interval;
        directions[i] = JointModel::cwForDelta(i, deltaAct[i]);
    }

    // Các đoạn vẽ ngắn phải khởi động đồng thời; nếu gọi run() tuần tự, mỗi trục
    // tự ramp và kết thúc lệch nhau tại từng waypoint 1 mm, tạo rung tuần hoàn.
    if (state_ == State::DRAWING) {
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
            if (steps[i] <= 0) continue;
            if (!motors[i]->prepareCoordinatedRun(
                    directions[i], static_cast<uint32_t>(steps[i]), intervals[i])) {
                stop();
                return false;
            }
        }
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
            if (steps[i] <= 0) continue;
            if (!motors[i]->startPreparedRun()) {
                stop();
                return false;
            }
        }
    } else {
        // Staging/nâng/hạ có hành trình dài: giữ S-curve của Motor::run().
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
            if (steps[i] <= 0) continue;
            motors[i]->setSpeed(intervals[i]);
            motors[i]->run(directions[i], static_cast<uint32_t>(steps[i]));
        }
    }
    return true;
}

bool Planner::nextDrawSegment() {
    // Sinh điểm kế tiếp cách DRAW_SEGMENT_MM
    float nx = curX_, ny = curY_;
    float remain = totalLen_ - prog_;

    if (job_.shape == Shape::LINE) {
        const float ux = (job_.x2 - job_.x1) / totalLen_;
        const float uy = (job_.y2 - job_.y1) / totalLen_;
        const float step = (remain < DRAW_SEGMENT_MM) ? remain : DRAW_SEGMENT_MM;
        nx += ux * step;
        ny += uy * step;
        prog_ += step;
    } else if (job_.shape == Shape::CIRCLE) {
        const float radius = job_.r;
        const float arcStep = (remain < DRAW_SEGMENT_MM) ? remain : DRAW_SEGMENT_MM;
        prog_ += arcStep;
        const float ang = startAng_ + prog_ / radius; // CCW
        nx = job_.x1 + radius * cosf(ang);
        ny = job_.y1 + radius * sinf(ang);
    } else { // SQUARE: bắt đầu ở góc dưới-trái, quét CCW bốn cạnh
        const float side = job_.r;
        const float half = side * 0.5f;
        const float step = (remain < DRAW_LINE_SEGMENT_MM) ? remain : DRAW_LINE_SEGMENT_MM;
        prog_ += step;
        const float p = prog_;
        if (p <= side) {
            nx = job_.x1 - half + p;
            ny = job_.y1 - half;
        } else if (p <= 2.0f * side) {
            nx = job_.x1 + half;
            ny = job_.y1 - half + (p - side);
        } else if (p <= 3.0f * side) {
            nx = job_.x1 + half - (p - 2.0f * side);
            ny = job_.y1 + half;
        } else {
            nx = job_.x1 - half;
            ny = job_.y1 + half - (p - 3.0f * side);
        }
    }

    curX_ = nx;
    curY_ = ny;
    const bool done = prog_ >= totalLen_ - 1e-4f;
    if (!startMoveTo(nx, ny, job_.z, job_.feedMmS)) return false;
    return !done;
}

void Planner::tick() {
    if (!hasJob_) return;

    switch (state_) {
        case State::LIFTING: {
            if (motorsBusy(motors)) return;
            const float safeZ = job_.z + PEN_LIFT_MM;
            // FK pose at HOME/park may not satisfy the fixed pen-down orientation.
            // Stage directly to the known IK-safe start point at pen-lift height,
            // instead of trying to re-solve the current parked TCP pose.
            if (job_.shape == Shape::CIRCLE) {
                curX_ = job_.x1 + job_.r * cosf(startAng_);
                curY_ = job_.y1 + job_.r * sinf(startAng_);
            } else if (job_.shape == Shape::SQUARE) {
                curX_ = job_.x1 - job_.r * 0.5f;
                curY_ = job_.y1 - job_.r * 0.5f;
            } else {
                curX_ = job_.x1;
                curY_ = job_.y1;
            }
            curZ_ = safeZ;
            if (!startMoveTo(curX_, curY_, curZ_, DRAW_FEED_MM_S)) return;
            const bool needDrop = job_.drawNow || (job_.shape == Shape::POINT);
            state_ = needDrop ? State::DROPPING : State::FINISHED_LIFT;
            break;
        }

        case State::DROPPING:
            if (motorsBusy(motors)) return;
            curZ_ = job_.z;
            if (!startMoveTo(curX_, curY_, job_.z, job_.feedMmS)) return;
            state_ = State::DRAWING;
            break;

        case State::DRAWING:
            if (motorsBusy(motors)) return;
            if (job_.shape == Shape::POINT) {
                finishAll(); // POINT kết thúc ngay tại đích (không nâng lại)
                return;
            }
            ++segDone_;
            if (!nextDrawSegment()) {
                if (!hasJob_) return; // lỗi trong nextDrawSegment -> đã stop()
                state_ = State::FINISHED_LIFT;
            }
            break;

        case State::FINISHED_LIFT: {
            if (motorsBusy(motors)) return;
            const float safeZ = job_.z + PEN_LIFT_MM;
            curZ_ = safeZ;
            if (!startMoveTo(curX_, curY_, safeZ, DRAW_FEED_MM_S)) return;
            state_ = State::WAIT_FINAL_LIFT;
            break;
        }

        case State::WAIT_FINAL_LIFT:
            if (motorsBusy(motors)) return;
            finishAll();
            break;

        default:
            break;
    }
}

void Planner::finishAll() {
    hasJob_ = false;
    state_ = State::IDLE;
    Serial.printf("[PLAN] Xong (%u segments)\n", static_cast<unsigned>(segDone_));
}
