#include "arm.h"
#include "differential_wrist.h"
#include "endstop.h"
#include "homing.h"
#include "joint_model.h"
#include "kinematics.h"
#include "motor.h"
#include "planner.h"
#include "safety_manager.h"
#include "sensor.h"
#include "trajectory_validator.h"
#include "wifi_manager.h"
#include "work_plane.h"

#include <esp_task_wdt.h>

namespace {
WifiManager* g_wifi = nullptr; // inject để statusJson đọc wifi (tránh include vòng)
} // namespace

void armSetWifiProvider(WifiManager* w) { g_wifi = w; }

ArmController::ArmController() {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = nullptr;
    mode_ = ArmMode::IDLE;
}

ArmController::~ArmController() = default;

void ArmController::begin(Motor** motors_, Sensor* sensor_, Endstops* endstops_,
                          JointModel* joints_, HomingController* homing_,
                          Planner* planner_) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = motors_[i];
    sensor = sensor_;
    es = endstops_;
    jm = joints_;
    hc = homing_;
    pl = planner_;

    // Create single owner SafetyManager and inject into all safety-dependent modules
    if (es != nullptr && jm != nullptr) {
        safety_ = std::make_unique<SafetyManager>(es, jm);
        es->setSafetyManager(safety_.get());
        for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
            if (motors[i] != nullptr) motors[i]->setSafetyManager(safety_.get());
        }
        if (hc != nullptr) hc->setSafetyManager(safety_.get());
    }

    queue = xQueueCreate(PLANNER_QUEUE_DEPTH, sizeof(ArmCommand));
    if (queue == nullptr) {
        Serial.println("[ARM] LOI: khong tao duoc command queue!");
        return;
    }
    const BaseType_t ok = xTaskCreatePinnedToCore(
        &ArmController::taskEntry, "arm_motion", MOTION_TASK_STACK_SIZE,
        this, MOTION_TASK_PRIORITY, &task, MOTION_TASK_CORE);
    if (ok != pdPASS) {
        Serial.println("[ARM] LOI: khong tao duoc motion task!");
    }
}

bool ArmController::submit(const ArmCommand& cmd, uint32_t timeoutMs) {
    if (queue == nullptr) return false;

    // STOP is an out-of-band command mailbox rather than a normal queue item.
    // It is therefore accepted even when the motion queue is full, and the motion
    // task handles it before any planner/homing tick or queued command.
    if (cmd.type == ArmCommand::STOP_ALL) {
        stopRequested_.store(true, std::memory_order_release);
        return true;
    }

    // Synchronous pre-flight validation for Cartesian jobs (§3.4 lightweight B) — HTTP 400 before moving
    if ((cmd.type == ArmCommand::MOVE_CART || cmd.type == ArmCommand::DRAW_LINE ||
         cmd.type == ArmCommand::DRAW_CIRCLE || cmd.type == ArmCommand::DRAW_SQUARE) &&
        pl != nullptr && jm != nullptr) {
        if (!busy()) {
            Planner::Job tjob;
            if (cmd.type == ArmCommand::MOVE_CART) {
                tjob.shape = Planner::Shape::POINT;
                tjob.x1 = cmd.p[0];
                tjob.y1 = cmd.p[1];
                tjob.z = cmd.p[2];
                if (cmd.p[5] > 1.0f && cmd.p[5] < 200.0f) tjob.feedMmS = cmd.p[5];
            } else if (cmd.type == ArmCommand::DRAW_LINE) {
                tjob.shape = Planner::Shape::LINE;
                tjob.x1 = cmd.p[0];
                tjob.y1 = cmd.p[1];
                tjob.x2 = cmd.p[2];
                tjob.y2 = cmd.p[3];
                tjob.z = cmd.p[4];
                if (cmd.p[5] > 1.0f && cmd.p[5] < 200.0f) tjob.feedMmS = cmd.p[5];
            } else if (cmd.type == ArmCommand::DRAW_CIRCLE) {
                tjob.shape = Planner::Shape::CIRCLE;
                tjob.x1 = cmd.p[0];
                tjob.y1 = cmd.p[1];
                tjob.z = cmd.p[2];
                tjob.r = cmd.p[3];
                if (cmd.p[5] > 1.0f && cmd.p[5] < 200.0f) tjob.feedMmS = cmd.p[5];
            } else {
                tjob.shape = Planner::Shape::SQUARE;
                tjob.x1 = cmd.p[0];
                tjob.y1 = cmd.p[1];
                tjob.z = cmd.p[2];
                tjob.r = cmd.p[3];
                if (cmd.p[5] > 1.0f && cmd.p[5] < 200.0f) tjob.feedMmS = cmd.p[5];
            }
            float enc[6];
            for (uint8_t i = 0; i < NUM_MOTORS; ++i) enc[i] = jm->angleFromSteps(i);
            const kin::FkResult fk = kin::forward(enc);
            kin::Pose curPose{fk.tcp.x, fk.tcp.y, fk.tcp.z};
            WorkPlane* wp = pl->getWorkPlane();
            if (wp != nullptr && wp->isEnabled()) {
                const Point3D ucs = wp->fromRobotXYZ({fk.tcp.x, fk.tcp.y, fk.tcp.z});
                curPose = {ucs.x, ucs.y, ucs.z};
            }
            TrajectoryValidator::Job vj;
            vj.type = static_cast<TrajectoryValidator::Job::Type>(tjob.shape);
            vj.x1 = tjob.x1;
            vj.y1 = tjob.y1;
            vj.x2 = tjob.x2;
            vj.y2 = tjob.y2;
            vj.z = tjob.z;
            vj.r = tjob.r;
            TrajectoryValidator vv(wp);
            ValidationResult vr = vv.validate(vj, curPose);
            if (!vr.ok) {
                lastPlannerError_ = vr.reason;
                lastPlannerFailIndex_ = vr.failIndex;
                Serial.printf("[ARM] REJECT pre-flight %s: %s at %d\n",
                              (tjob.shape == Planner::Shape::POINT)   ? "POINT"
                              : (tjob.shape == Planner::Shape::LINE) ? "LINE"
                              : (tjob.shape == Planner::Shape::CIRCLE) ? "CIRCLE"
                                                                     : "SQUARE",
                              vr.reason.c_str(), vr.failIndex);
                return false;
            }
            lastPlannerError_ = "OK";
            lastPlannerFailIndex_ = -1;
        }
    }
    return xQueueSend(queue, &cmd, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

bool ArmController::busy() const {
    if (hc != nullptr && hc->isActive()) return true;
    if (pl != nullptr && pl->isActive()) return true;
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (motors[i] != nullptr && motors[i]->isRunning()) return true;
    }
    return false;
}

ArmMode ArmController::mode() const { return mode_; }

void ArmController::taskEntry(void* param) {
    auto* self = static_cast<ArmController*>(param);
    self->taskLoop();
}

void ArmController::taskLoop() {
    TickType_t lastWake = xTaskGetTickCount();
    ArmCommand cmd;
    bool wasHoming{false};

    // Đăng ký Task WDT — task homing/planner/FAULT là an toàn nhất, phải có watchdog riêng
    esp_task_wdt_add(nullptr);

    const TickType_t period = pdMS_TO_TICKS(MOTION_TASK_PERIOD_MS);

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        if ((now - lastWake) > period * 2) {
            lastWake = now;
        }
        vTaskDelayUntil(&lastWake, period);
        esp_task_wdt_reset();

        // Highest command priority: cancel current motion and discard all queued
        // motion commands before they can restart the robot after STOP_ALL.
        if (stopRequested_.exchange(false, std::memory_order_acq_rel)) {
            stopAllAndDiscardQueuedMotion();
            continue;
        }

        // Safety poll: debounce ISR pending → latch/E_STOP (50ms)
        if (safety_ != nullptr) safety_->pollEndstops();

        // 1) Homing FSM trước (ưu tiên an toàn), rồi planner
        if (hc != nullptr) hc->tick();
        if (pl != nullptr) pl->tick();

        // Sau khi homing hoàn tất: endstop vẫn nhấn do backoff là bình thường → clear latch
        if (hc != nullptr && wasHoming && !hc->isActive()) {
            es->clearAllLatches();
        }
        wasHoming = (hc != nullptr && hc->isActive());

        // 2) Endstop bảo vệ + E-stop (homing tự xử lý endstop riêng)
        if (!manualRelease_.load(std::memory_order_acquire) && es != nullptr && hc != nullptr &&
            !hc->isActive() && mode_ != ArmMode::FAULT) {
            const bool estopPending = safety_ ? safety_->isEStop() : false;
            const bool latchPending = safety_ ? safety_->anyLatched() : es->anyLatched();

            if (estopPending || latchPending) {
                if (pl != nullptr) pl->stop();
                for (uint8_t j = 0; j < NUM_MOTORS; ++j) {
                    if (motors[j] != nullptr) motors[j]->stop();
                }
                mode_ = ArmMode::FAULT;
                Serial.println("[ARM] FAULT: endstop hit during motion (ISR/E-stop)");
            } else {
                // Backup: phát hiện endstop khi motor đang chạy (jog away khỏi công tắc)
                bool anyMotorRunning = false;
                for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
                    if (motors[i] != nullptr && motors[i]->isRunning()) {
                        anyMotorRunning = true;
                        break;
                    }
                }
                if (anyMotorRunning) {
                    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
                        if (es->hasPin(i, EndstopWhich::MIN) && es->isPressed(i, EndstopWhich::MIN)) {
                            const bool movingAway = (motors[i] != nullptr && motors[i]->isRunning() &&
                                                     motors[i]->getDirCW() == JointModel::cwForDelta(i, +1.0f));
                            if (!movingAway) {
                                if (pl != nullptr) pl->stop();
                                for (uint8_t j = 0; j < NUM_MOTORS; ++j) {
                                    if (motors[j] != nullptr) motors[j]->stop();
                                }
                                if (safety_) safety_->assertEStop("endstop MIN");
                                mode_ = ArmMode::FAULT;
                                Serial.printf("[ARM] FAULT: endstop J%u MIN pressed during motion\n", i + 1);
                                break;
                            }
                        }
                        if (es->hasPin(i, EndstopWhich::MAX) && es->isPressed(i, EndstopWhich::MAX)) {
                            const bool movingAway = (motors[i] != nullptr && motors[i]->isRunning() &&
                                                     motors[i]->getDirCW() == JointModel::cwForDelta(i, -1.0f));
                            if (!movingAway) {
                                if (pl != nullptr) pl->stop();
                                for (uint8_t j = 0; j < NUM_MOTORS; ++j) {
                                    if (motors[j] != nullptr) motors[j]->stop();
                                }
                                if (safety_) safety_->assertEStop("endstop MAX");
                                mode_ = ArmMode::FAULT;
                                Serial.printf("[ARM] FAULT: endstop J%u MAX pressed during motion\n", i + 1);
                                break;
                            }
                        }
                    }
                }
            }
        }

        // 3) Mode runtime (tính lại mỗi tick, tránh kẹt trạng thái)
        if (mode_ != ArmMode::FAULT) {
            if (manualRelease_.load(std::memory_order_acquire)) {
                mode_ = ArmMode::RELEASE;
            } else if (hc != nullptr && hc->isActive()) {
                mode_ = ArmMode::HOMING;
            } else if (pl != nullptr && pl->isActive()) {
                mode_ = pl->isDrawing() ? ArmMode::DRAW : ArmMode::CART;
            } else if (busy()) {
                mode_ = ArmMode::JOG;
            } else {
                mode_ = ArmMode::IDLE;
            }
        }

        // 4) Rút lệnh từ queue (không block)
        while (queue != nullptr && xQueueReceive(queue, &cmd, 0) == pdTRUE) {
            execute(cmd);
        }
    }

    esp_task_wdt_delete(nullptr);
}

bool ArmController::motionAllowed() const {
    if (safety_ && !safety_->isMotionAllowed()) return false;
    return mode_ != ArmMode::FAULT;
}

void ArmController::execute(const ArmCommand& cmd) {
    switch (cmd.type) {
        case ArmCommand::STOP_ALL:
            stopAllAndDiscardQueuedMotion();
            break;

        case ArmCommand::CLEAR_FAULT: {
            bool ok = true;
            if (safety_) {
                ok = safety_->tryClearFault();
            } else {
                if (es != nullptr) es->clearAllLatches();
                if (jm != nullptr) jm->clearAllDriftFaults();
            }
            if (ok) {
                mode_ = ArmMode::IDLE;
                Serial.println("[ARM] FAULT cleared -> IDLE");
            } else {
                Serial.println("[ARM] CLEAR_FAULT rejected: endstop still pressed");
            }
            break;
        }

        case ArmCommand::HOME_ALL:
            resumeManualRelease();
            if (!motionAllowed()) break;
            if (busy()) break;
            mode_ = ArmMode::HOMING;
            if (hc != nullptr && !hc->startAll()) mode_ = ArmMode::IDLE;
            break;

        case ArmCommand::HOME_AXIS:
            resumeManualRelease();
            if (!motionAllowed()) break;
            if (busy() || cmd.axis >= 4) break;
            mode_ = ArmMode::HOMING;
            if (hc != nullptr && !hc->startAxis(cmd.axis)) mode_ = ArmMode::IDLE;
            break;

        case ArmCommand::SET_HOME:
            resumeManualRelease();
            if (busy()) break;
            if (jm != nullptr) {
                if (cmd.axis == 255) {
                    // Set home toàn bộ cổ tay vi sai (J5 + J6)
                    jm->setHomeHere(4);
                    jm->setHomeHere(5);
                    Serial.println("[ARM] Set-Home Wrist (J5 + J6) tai vi tri hien tai");
                } else if (cmd.axis < NUM_MOTORS) {
                    jm->setHomeHere(cmd.axis);
                    Serial.printf("[ARM] Set-Home J%u tai vi tri hien tai\n", cmd.axis + 1);
                }
            }
            break;

        case ArmCommand::RELEASE_J1_J4:
            if (busy()) {
                Serial.println("[ARM] RELEASE J1-J4 bo qua: robot dang chay");
                break;
            }
            if (safety_ != nullptr) safety_->assertManualRelease(true);
            manualRelease_.store(true, std::memory_order_release);
            for (uint8_t axis = 0; axis < 4; ++axis) {
                if (motors[axis] == nullptr || !motors[axis]->enable(false)) {
                    Serial.printf("[ARM] RELEASE J%u FAIL (UART)\n", axis + 1);
                    continue;
                }
                Serial.printf("[ARM] RELEASE J%u OK — home/NVS giu nguyen\n", axis + 1);
            }
            break;

        case ArmCommand::JOG_REL:
            resumeManualRelease();
            if (!motionAllowed()) {
                Serial.printf("[ARM] TU CHOI JOG J%u: robot dang o mode=%u / FAULT (Bấm CLEAR FAULT để xóa lỗi)\n",
                              cmd.axis + 1, static_cast<unsigned>(mode_.load(std::memory_order_relaxed)));
                break;
            }
            if (cmd.axis >= NUM_MOTORS || hc == nullptr || jm == nullptr) break;
            if (motors[cmd.axis]->isRunning()) {
                Serial.printf("[ARM] JOG J%u bo qua: motor dang chay\n", cmd.axis + 1);
                break;
            }
            Serial.printf("[ARM] JOG J%u %+.2f deg\n", cmd.axis + 1, cmd.value);
            applyJog(cmd.axis, cmd.value);
            break;

        case ArmCommand::MOVE_CART:
        case ArmCommand::DRAW_LINE:
        case ArmCommand::DRAW_CIRCLE:
        case ArmCommand::DRAW_SQUARE: {
            resumeManualRelease();
            if (!motionAllowed() || pl == nullptr || jm == nullptr) break;
            if (busy()) break;
            if (!jm->allPositioningHomed()) {
                Serial.println("[ARM] TU CHOI: phai HOME J1-J4 truoc khi dieu khien Cartesian");
                break;
            }
            Planner::Job job;
            if (cmd.type == ArmCommand::MOVE_CART) {
                job.shape = Planner::Shape::POINT;
                job.x1 = cmd.p[0];
                job.y1 = cmd.p[1];
                job.z = cmd.p[2];
                job.drawNow = false;
            } else if (cmd.type == ArmCommand::DRAW_LINE) {
                job.shape = Planner::Shape::LINE;
                job.x1 = cmd.p[0];
                job.y1 = cmd.p[1];
                job.x2 = cmd.p[2];
                job.y2 = cmd.p[3];
                job.z = cmd.p[4];
            } else if (cmd.type == ArmCommand::DRAW_CIRCLE) {
                job.shape = Planner::Shape::CIRCLE;
                job.x1 = cmd.p[0]; // cx
                job.y1 = cmd.p[1]; // cy
                job.z = cmd.p[2];
                job.r = cmd.p[3];
            } else {
                job.shape = Planner::Shape::SQUARE;
                job.x1 = cmd.p[0]; // cx
                job.y1 = cmd.p[1]; // cy
                job.z = cmd.p[2];
                job.r = cmd.p[3];  // side
            }
            if (cmd.p[5] > 1.0f && cmd.p[5] < 200.0f) job.feedMmS = cmd.p[5];
            mode_ = (cmd.type == ArmCommand::MOVE_CART) ? ArmMode::CART : ArmMode::DRAW;
            if (!pl->submit(job)) {
                lastPlannerError_ = pl->lastError();
                lastPlannerFailIndex_ = pl->lastFailIndex();
                Serial.printf("[ARM] REJECT job submit: %s at %d\n", lastPlannerError_.c_str(),
                              lastPlannerFailIndex_);
                mode_ = ArmMode::IDLE;
            } else {
                lastPlannerError_ = "OK";
                lastPlannerFailIndex_ = -1;
            }
            break;
        }

        default:
            break;
    }
}

void ArmController::stopAllAndDiscardQueuedMotion() {
    if (hc != nullptr) hc->cancel();
    if (pl != nullptr) pl->stop();
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (motors[i] != nullptr) motors[i]->stop();
    }
    if (queue != nullptr) xQueueReset(queue);
    if (mode_ != ArmMode::FAULT) mode_ = ArmMode::IDLE;
    Serial.println("[ARM] STOP ALL: motion cancelled and queue cleared");
}

void ArmController::resumeManualRelease() {
    if (!manualRelease_.exchange(false, std::memory_order_acq_rel)) return;
    if (safety_ != nullptr) safety_->assertManualRelease(false);
    if (jm != nullptr) {
        for (uint8_t axis = 0; axis < 4; ++axis) jm->resyncFromEncoder(axis);
    }
    Serial.println("[ARM] RELEASE ket thuc — da resync J1-J4 tu encoder");
}

void ArmController::applyJog(uint8_t axis, float deltaDeg) {
    if (axis < 4) {
        // Khớp 1..4 dẫn động trực tiếp
        float delta = deltaDeg;
        if (jm->isHomed(axis)) {
            const float cur = (jm->encOK(axis)) ? jm->angleFromEncoder(axis) : jm->angleFromSteps(axis);
            float target = cur + delta;
            if (target > DEFAULT_AXIS_LIMIT_MAX[axis]) target = DEFAULT_AXIS_LIMIT_MAX[axis];
            if (target < DEFAULT_AXIS_LIMIT_MIN[axis]) target = DEFAULT_AXIS_LIMIT_MIN[axis];
            delta = target - cur; // clamp theo soft limit từ vị trí encoder thực tế
            if ((deltaDeg > 0.0f && delta < 0.0f) || (deltaDeg < 0.0f && delta > 0.0f)) {
                delta = 0.0f;
            }
        }
        const int64_t steps = JointModel::degreesToSteps(axis, fabsf(delta));
        Serial.printf("[JOG] J%u: delta=%.2f steps=%lld cw=%d (encOK=%d homed=%d)\n",
                      axis+1, delta, steps, (int)JointModel::cwForDelta(axis, delta),
                      (int)jm->encOK(axis), (int)jm->isHomed(axis));
        if (steps <= 0) return;
        const bool cw = JointModel::cwForDelta(axis, delta);
        motors[axis]->setSpeed(DEFAULT_AXIS_JOG_SPEEDS[axis]);
        motors[axis]->run(cw, static_cast<uint32_t>(steps));
    } else if (axis == 4) {
        // Jog J5 (Tilt): Chuyển động thuần Tilt qua Differential Wrist
        float delta = deltaDeg;
        if (jm->isHomed(4)) {
            const float cur = (jm->encOK(4) && jm->encOK(5)) ? jm->angleFromEncoder(4) : jm->angleFromSteps(4);
            float target = cur + delta;
            if (target > DEFAULT_AXIS_LIMIT_MAX[4]) target = DEFAULT_AXIS_LIMIT_MAX[4];
            if (target < DEFAULT_AXIS_LIMIT_MIN[4]) target = DEFAULT_AXIS_LIMIT_MIN[4];
            delta = target - cur;
            if ((deltaDeg > 0.0f && delta < 0.0f) || (deltaDeg < 0.0f && delta > 0.0f)) {
                delta = 0.0f;
            }
        }
        const DifferentialWrist::ActuatorSteps steps = g_diffWrist.computeIncrementalSteps(
            delta, 0.0f, JointModel::stepsPerDegree(4), JointModel::stepsPerDegree(5));
        Serial.printf("[JOG] J5 (Tilt): delta=%.2f leftSteps=%lld rightSteps=%lld\n",
                      delta, static_cast<long long>(steps.leftSteps), static_cast<long long>(steps.rightSteps));
        if (steps.leftSteps != 0 && motors[4] != nullptr) {
            motors[4]->setSpeed(DEFAULT_AXIS_JOG_SPEEDS[4]);
            motors[4]->run(JointModel::cwForDelta(4, static_cast<float>(steps.leftSteps)),
                           static_cast<uint32_t>(llabs(steps.leftSteps)));
        }
        if (steps.rightSteps != 0 && motors[5] != nullptr) {
            motors[5]->setSpeed(DEFAULT_AXIS_JOG_SPEEDS[5]);
            motors[5]->run(JointModel::cwForDelta(5, static_cast<float>(steps.rightSteps)),
                           static_cast<uint32_t>(llabs(steps.rightSteps)));
        }
    } else if (axis == 5) {
        // Jog J6 (Roll): Chuyển động thuần Roll qua Differential Wrist
        float delta = deltaDeg;
        if (jm->isHomed(5)) {
            const float cur = (jm->encOK(4) && jm->encOK(5)) ? jm->angleFromEncoder(5) : jm->angleFromSteps(5);
            float target = cur + delta;
            if (target > DEFAULT_AXIS_LIMIT_MAX[5]) target = DEFAULT_AXIS_LIMIT_MAX[5];
            if (target < DEFAULT_AXIS_LIMIT_MIN[5]) target = DEFAULT_AXIS_LIMIT_MIN[5];
            delta = target - cur;
            if ((deltaDeg > 0.0f && delta < 0.0f) || (deltaDeg < 0.0f && delta > 0.0f)) {
                delta = 0.0f;
            }
        }
        const DifferentialWrist::ActuatorSteps steps = g_diffWrist.computeIncrementalSteps(
            0.0f, delta, JointModel::stepsPerDegree(4), JointModel::stepsPerDegree(5));
        Serial.printf("[JOG] J6 (Roll): delta=%.2f leftSteps=%lld rightSteps=%lld\n",
                      delta, static_cast<long long>(steps.leftSteps), static_cast<long long>(steps.rightSteps));
        if (steps.leftSteps != 0 && motors[4] != nullptr) {
            motors[4]->setSpeed(DEFAULT_AXIS_JOG_SPEEDS[4]);
            motors[4]->run(JointModel::cwForDelta(4, static_cast<float>(steps.leftSteps)),
                           static_cast<uint32_t>(llabs(steps.leftSteps)));
        }
        if (steps.rightSteps != 0 && motors[5] != nullptr) {
            motors[5]->setSpeed(DEFAULT_AXIS_JOG_SPEEDS[5]);
            motors[5]->run(JointModel::cwForDelta(5, static_cast<float>(steps.rightSteps)),
                           static_cast<uint32_t>(llabs(steps.rightSteps)));
        }
    }
}

String ArmController::statusJson() {
    String j;
    j.reserve(3600);
    j = "{";
    j += "\"fw\":\"" FW_VERSION "\",";
    switch (mode_) {
        case ArmMode::IDLE:   j += "\"mode\":\"idle\",";   break;
        case ArmMode::RELEASE:j += "\"mode\":\"release\",";break;
        case ArmMode::HOMING: j += "\"mode\":\"homing\","; break;
        case ArmMode::JOG:    j += "\"mode\":\"jog\",";    break;
        case ArmMode::CART:   j += "\"mode\":\"cart\",";   break;
        case ArmMode::DRAW:   j += "\"mode\":\"draw\",";   break;
        case ArmMode::FAULT:  j += "\"mode\":\"fault\",";  break;
    }
    j += "\"busy\":" + String(busy() ? "true" : "false") + ",";

    if (g_wifi != nullptr) j += "\"wifi\":" + g_wifi->toJson() + ",";
    if (hc != nullptr) j += "\"homing\":" + hc->toJson() + ",";
    if (jm != nullptr) j += "\"joints\":" + jm->toJson() + ",";
    if (es != nullptr) j += "\"endstops\":" + es->toJson() + ",";

    // TCP pose hiện tại theo FK từ góc khớp thực tế (encoder)
    {
        float enc[6];
        for (uint8_t i = 0; i < NUM_MOTORS; ++i)
            enc[i] = (jm && jm->isHomed(i) && jm->encOK(i)) ? jm->angleFromEncoder(i) : (jm ? jm->angleFromSteps(i) : 0.0f);
        const kin::FkResult fk = kin::forward(enc);
        char buf[80];
        snprintf(buf, sizeof(buf), "\"pose\":{\"x\":%.1f,\"y\":%.1f,\"z\":%.1f},",
                 fk.tcp.x, fk.tcp.y, fk.tcp.z);
        j += buf;
    }
    if (pl != nullptr) {
        // Expose lastError/failIndex for pre-flight validator (§3.4) — HTTP 400 diagnostics via polling
        const String& pe = (lastPlannerError_ != "OK" && lastPlannerError_.c_str()[0] != '\0') ? lastPlannerError_ : pl->lastError();
        int pfi = (lastPlannerFailIndex_ != -1) ? lastPlannerFailIndex_ : pl->lastFailIndex();
        char pb[160];
        snprintf(pb, sizeof(pb),
                 "\"planner\":{\"active\":%s,\"state\":%u,\"segs\":%u,\"lastError\":\"%s\",\"failIndex\":%d},",
                 pl->isActive() ? "true" : "false", static_cast<unsigned>(pl->state()),
                 static_cast<unsigned>(pl->segmentsDone()), pe.c_str(), pfi);
        j += pb;
    }

    j += "\"motors\":[";
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (i > 0) j += ",";
        j += (motors[i] != nullptr) ? motors[i]->toJson() : "{}";
    }
    j += "]";
    j += "}";
    return j;
}
