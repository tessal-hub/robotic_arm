#include "arm.h"
#include "endstop.h"
#include "homing.h"
#include "joint_model.h"
#include "kinematics.h"
#include "motor.h"
#include "nvs_store.h"
#include "planner.h"
#include "safety_manager.h"
#include "sensor.h"
#include "trajectory_validator.h"
#include "wifi_manager.h"
#include "work_plane.h"

#include <esp_task_wdt.h>

namespace {
WifiManager* g_wifi = nullptr; // inject để statusJson đọc wifi (tránh include vòng)

Planner::Job plannerJobFor(const ArmCommand& cmd) {
    Planner::Job job;
    if (cmd.type == ArmCommand::MOVE_CART) {
        job.shape = Planner::Shape::POINT;
        job.x1 = cmd.p[0]; job.y1 = cmd.p[1]; job.z = cmd.p[2];
        job.drawNow = false;
    } else if (cmd.type == ArmCommand::DRAW_LINE) {
        job.shape = Planner::Shape::LINE;
        job.x1 = cmd.p[0]; job.y1 = cmd.p[1];
        job.x2 = cmd.p[2]; job.y2 = cmd.p[3]; job.z = cmd.p[4];
    } else {
        job.shape = cmd.type == ArmCommand::DRAW_CIRCLE
            ? Planner::Shape::CIRCLE : Planner::Shape::SQUARE;
        job.x1 = cmd.p[0]; job.y1 = cmd.p[1]; job.z = cmd.p[2]; job.r = cmd.p[3];
    }
    if (cmd.p[5] > 1.0f && cmd.p[5] < 200.0f) job.feedMmS = cmd.p[5];
    return job;
}

struct ShowOffPose {
    float dj[NUM_MOTORS];
    uint32_t durationMs;
};

constexpr uint8_t SHOW_OFF_STEP_COUNT = 8;
constexpr ShowOffPose SHOW_OFF_STEPS[SHOW_OFF_STEP_COUNT] = {
    { { +12.0f, +5.0f, -6.0f, +25.0f, +12.0f, +35.0f}, 1200 },
    { { +15.0f, -4.0f, +6.0f, -15.0f, -10.0f, -20.0f}, 1100 },
    { {   0.0f, -6.0f, +8.0f, -30.0f, -14.0f, -40.0f}, 1300 },
    { { -12.0f, -4.0f, +6.0f, -15.0f,  -8.0f, -20.0f}, 1100 },
    { { -15.0f, +6.0f, -6.0f, +20.0f, +12.0f, +30.0f}, 1200 },
    { {   0.0f, +7.0f, -8.0f, +30.0f, +15.0f, +45.0f}, 1300 },
    { {  +6.0f, -2.0f, +3.0f, -10.0f,  -6.0f, -15.0f}, 1000 },
    { {   0.0f,  0.0f,  0.0f,   0.0f,   0.0f,   0.0f}, 1200 }
};
} // namespace

void armSetWifiProvider(WifiManager* w) { g_wifi = w; }

ArmController::ArmController() {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = nullptr;
    mode_ = ArmMode::IDLE;
}

ArmController::~ArmController() = default;

void ArmController::begin(Motor** motors_, Sensor* sensor_, Endstops* endstops_,
                          JointModel* joints_, HomingController* homing_,
                          Planner* planner_, NvsStore* nvs_) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = motors_[i];
    sensor = sensor_;
    es = endstops_;
    jm = joints_;
    hc = homing_;
    pl = planner_;
    nvs = nvs_;
    if (nvs != nullptr) {
        for (uint8_t slot = 0; slot < TEACH_POINT_COUNT; ++slot) {
            teachPoints_[slot] = nvs->loadTeachPoint(slot);
        }
    }
    refreshTeachPoints();

    pinMode(GRIPPER_SERVO_PIN, OUTPUT);
    digitalWrite(GRIPPER_SERVO_PIN, LOW);
    gripperReady_ = ledcSetup(GRIPPER_PWM_CHANNEL, GRIPPER_PWM_HZ, GRIPPER_PWM_BITS) != 0;
    if (gripperReady_) {
        ledcWrite(GRIPPER_PWM_CHANNEL, 0); // no movement command at boot
        ledcAttachPin(GRIPPER_SERVO_PIN, GRIPPER_PWM_CHANNEL);
    }

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

    if (cmd.type == ArmCommand::SET_GRIPPER &&
        (!std::isfinite(cmd.value) || cmd.value < GRIPPER_MIN_DEG ||
         cmd.value > GRIPPER_MAX_DEG || !gripperAvailable())) return false;
    if (uxQueueMessagesWaiting(queue) != 0) return false;
    ArmCommand queued = cmd;
    queued.submittedAtUs = micros();

    // Synchronous pre-flight validation for Cartesian jobs (§3.4 lightweight B) — HTTP 400 before moving
    if ((cmd.type == ArmCommand::MOVE_CART || cmd.type == ArmCommand::DRAW_LINE ||
         cmd.type == ArmCommand::DRAW_CIRCLE || cmd.type == ArmCommand::DRAW_SQUARE) &&
        pl != nullptr && jm != nullptr) {
        if (!busy()) {
            Planner::Job tjob = plannerJobFor(cmd);
            WorkPlane* wp = pl->getWorkPlane();
            ValidationResult vr = validateTrajectory(tjob, wp);
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
    return xQueueSend(queue, &queued, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

bool ArmController::busy() const {
    if (queue != nullptr && uxQueueMessagesWaiting(queue) != 0) return true;
    if (hc != nullptr && hc->isActive()) return true;
    if (pl != nullptr && pl->isActive()) return true;
    if (showOffActive_) return true;
    if (teachPlaybackActive_) return true;
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (motors[i] != nullptr && motors[i]->isRunning()) return true;
    }
    return false;
}

bool ArmController::gripperAvailable() const {
    return gripperReady_ && !stopRequested_.load(std::memory_order_acquire) &&
           !manualRelease_.load(std::memory_order_acquire) && motionAllowed() && !busy();
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
        refreshTeachPoints();

        // Recovery Jog masks the held switch until that finite jog finishes. Then
        // re-arm it; if the switch is still held, restore FAULT for the next command.
        if (recoveryJogAxis_ < NUM_MOTORS && es != nullptr &&
            (motors[recoveryJogAxis_] == nullptr || !motors[recoveryJogAxis_]->isRunning())) {
            const bool stillPressed = es->isPhysicallyPressed(recoveryJogAxis_, recoveryJogSide_);
            es->setPinEnabled(recoveryJogAxis_, recoveryJogSide_, true);
            es->clearLatch(recoveryJogAxis_, recoveryJogSide_);
            if (stillPressed) {
                if (safety_ != nullptr) safety_->assertEStop("recovery jog incomplete");
                mode_ = ArmMode::FAULT;
                Serial.printf("[ARM] RECOVERY JOG J%u chua roi endstop — giu FAULT\n",
                              recoveryJogAxis_ + 1);
            }
            recoveryJogAxis_ = NUM_MOTORS;
        }

        // 1) Homing FSM trước (ưu tiên an toàn), rồi planner
        if (hc != nullptr) hc->tick();
        if (pl != nullptr) pl->tick();
        updateShowOff();
        updateTeachPlayback();

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
                stopShowOff();
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
            } else if (showOffActive_) {
                mode_ = ArmMode::SHOW_OFF;
            } else if (busy()) {
                mode_ = ArmMode::JOG;
            } else {
                mode_ = ArmMode::IDLE;
            }
        }

        if (!motionAllowed()) stopGripper();

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
    if (cmd.submittedAtUs != 0) {
        lastCommandLatencyUs_.store(micros() - cmd.submittedAtUs, std::memory_order_relaxed);
    }
    switch (cmd.type) {
        case ArmCommand::SET_GRIPPER: {
            if (!gripperAvailable() || !std::isfinite(cmd.value) ||
                cmd.value < GRIPPER_MIN_DEG || cmd.value > GRIPPER_MAX_DEG) break;
            const float ratio = (cmd.value - GRIPPER_MIN_DEG) / (GRIPPER_MAX_DEG - GRIPPER_MIN_DEG);
            const float pulseUs = GRIPPER_MIN_PULSE_US + ratio * (GRIPPER_MAX_PULSE_US - GRIPPER_MIN_PULSE_US);
            const uint32_t duty = static_cast<uint32_t>(lroundf(
                pulseUs * GRIPPER_PWM_HZ * (1UL << GRIPPER_PWM_BITS) / 1000000.0f));
            ledcWrite(GRIPPER_PWM_CHANNEL, duty);
            gripperTargetDeg_.store(cmd.value, std::memory_order_relaxed);
            gripperActive_.store(true, std::memory_order_release);
            break;
        }

        case ArmCommand::STOP_ALL:
            stopAllAndDiscardQueuedMotion();
            break;

        case ArmCommand::CLEAR_FAULT: {
            // Clearing includes encoder resync. Never change coordinates during motion.
            if (busy()) break;
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
            if (!resumeManualRelease()) break;
            if (!motionAllowed()) break;
            if (busy()) break;
            mode_ = ArmMode::HOMING;
            if (hc != nullptr && !hc->startAll()) mode_ = ArmMode::IDLE;
            break;

        case ArmCommand::HOME_AXIS:
            if (!resumeManualRelease()) break;
            if (!motionAllowed()) break;
            if (busy() || cmd.axis >= 4) break;
            mode_ = ArmMode::HOMING;
            if (hc != nullptr && !hc->startAxis(cmd.axis)) mode_ = ArmMode::IDLE;
            break;

        case ArmCommand::SET_HOME:
            if (!resumeManualRelease()) break;
            if (busy()) break;
            if (jm != nullptr) {
                if (cmd.axis == 255) {
                    // Set home đồng thời hai khớp cổ tay độc lập (J5 + J6)
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
            stopGripper();
            manualRelease_.store(true, std::memory_order_release);
            mode_ = ArmMode::RELEASE;
            for (uint8_t axis = 0; axis < 4; ++axis) {
                if (motors[axis] == nullptr || !motors[axis]->enable(false)) {
                    Serial.printf("[ARM] RELEASE J%u FAIL (UART)\n", axis + 1);
                    continue;
                }
                Serial.printf("[ARM] RELEASE J%u OK — home/NVS giu nguyen\n", axis + 1);
            }
            break;

        case ArmCommand::ENABLE_J1_J4:
            if (!resumeManualRelease()) break;
            break;

        case ArmCommand::JOG_REL:
            if (!resumeManualRelease()) break;
            if (cmd.axis >= NUM_MOTORS || hc == nullptr || jm == nullptr) break;
            {
                EndstopWhich pressedSide = EndstopWhich::MIN;
                const bool minPressed = es != nullptr && es->hasPin(cmd.axis, EndstopWhich::MIN) &&
                                        es->isPressed(cmd.axis, EndstopWhich::MIN);
                const bool maxPressed = es != nullptr && es->hasPin(cmd.axis, EndstopWhich::MAX) &&
                                        es->isPressed(cmd.axis, EndstopWhich::MAX);
                if (maxPressed) pressedSide = EndstopWhich::MAX;
            if (mode_ == ArmMode::FAULT &&
                (!safety_ || !safety_->tryBeginRecoveryJog(cmd.axis, cmd.value > 0.0f))) {
                Serial.printf("[ARM] TU CHOI RECOVERY JOG J%u %+.2f deg: chi duoc roi khoi endstop dang nhan\n",
                              cmd.axis + 1, cmd.value);
                break;
            }
            if (mode_ == ArmMode::FAULT) {
                if ((minPressed || maxPressed) && es != nullptr) {
                    recoveryJogAxis_ = cmd.axis;
                    recoveryJogSide_ = pressedSide;
                    es->setPinEnabled(cmd.axis, pressedSide, false);
                }
                mode_ = ArmMode::IDLE;
                Serial.printf("[ARM] FAULT auto-clear cho Recovery Jog J%u\n", cmd.axis + 1);
            }
            }
            if (!motionAllowed()) {
                Serial.printf("[ARM] TU CHOI JOG J%u: robot dang o mode=%u / FAULT (Bấm CLEAR FAULT để xóa lỗi)\n",
                              cmd.axis + 1, static_cast<unsigned>(mode_.load(std::memory_order_relaxed)));
                break;
            }
            if (motors[cmd.axis]->isRunning()) {
                Serial.printf("[ARM] JOG J%u bo qua: motor dang chay\n", cmd.axis + 1);
                break;
            }
            Serial.printf("[ARM] JOG J%u %+.2f deg\n", cmd.axis + 1, cmd.value);
            applyJog(cmd.axis, cmd.value);
            break;

        case ArmCommand::SHOW_OFF:
            if (!resumeManualRelease()) break;
            if (!motionAllowed() || jm == nullptr) break;
            if (busy()) {
                Serial.println("[ARM] SHOW_OFF bi bo qua: he thong dang ban");
                break;
            }
            startShowOff();
            break;

        case ArmCommand::SAVE_TEACH_POINT: {
            if (busy() || cmd.axis >= TEACH_POINT_COUNT || jm == nullptr || nvs == nullptr) break;
            NvsStore::TeachPoint point;
            for (uint8_t axis = 0; axis < NUM_MOTORS; ++axis) {
                if (!jm->isHomed(axis) || !jm->encOK(axis)) {
                    Serial.printf("[TEACH] SAVE %c rejected: J%u not homed/encoder unhealthy\n", 'A' + cmd.axis, axis + 1);
                    teachError_.store(TeachError::HOME);
                    return;
                }
                point.axes[axis] = {jm->angleFromEncoder(axis), jm->homeRawDeg(axis), jm->encSignOf(axis)};
            }
            if (nvs->saveTeachPoint(cmd.axis, point)) {
                teachError_.store(TeachError::NONE);
                Serial.printf("[TEACH] Saved point %c\n", 'A' + cmd.axis);
            } else {
                teachError_.store(TeachError::NVS);
                Serial.printf("[TEACH] SAVE %c failed (NVS)\n", 'A' + cmd.axis);
            }
            teachPoints_[cmd.axis] = nvs->loadTeachPoint(cmd.axis);
            refreshTeachPoints();
            break;
        }

        case ArmCommand::PLAY_TEACH_POINTS:
            if (busy() || jm == nullptr) break;
            refreshTeachPoints();
            if (teachValidMask_.load() == 0) {
                if (teachError_.load() != TeachError::FRAME) teachError_.store(TeachError::INVALID);
                break;
            }
            if (!resumeManualRelease() || !motionAllowed()) break;
            teachError_.store(TeachError::NONE);
            teachPlaybackActive_ = true;
            teachPlaybackSlot_ = 0;
            updateTeachPlayback();
            break;

        case ArmCommand::CLEAR_TEACH_POINTS:
            if (busy() || nvs == nullptr) break;
            if (nvs->clearTeachPoints()) {
                teachError_.store(TeachError::NONE);
                Serial.println("[TEACH] Cleared points A-C");
            } else {
                teachError_.store(TeachError::NVS);
                Serial.println("[TEACH] CLEAR failed (NVS); inspect remaining slots");
            }
            for (uint8_t slot = 0; slot < TEACH_POINT_COUNT; ++slot) {
                teachPoints_[slot] = nvs->loadTeachPoint(slot);
            }
            refreshTeachPoints();
            break;

        case ArmCommand::MOVE_CART:
        case ArmCommand::DRAW_LINE:
        case ArmCommand::DRAW_CIRCLE:
        case ArmCommand::DRAW_SQUARE: {
            if (!resumeManualRelease()) break;
            if (!motionAllowed() || pl == nullptr || jm == nullptr) break;
            if (busy()) break;
            if (!jm->allPositioningHomed()) {
                Serial.println("[ARM] TU CHOI: phai HOME J1-J4 truoc khi dieu khien Cartesian");
                break;
            }
            Planner::Job job = plannerJobFor(cmd);
            mode_ = (cmd.type == ArmCommand::MOVE_CART) ? ArmMode::CART : ArmMode::DRAW;
            if (!pl->submit(job)) {
                lastPlannerError_ = "INVALID_JOB";
                lastPlannerFailIndex_ = -1;
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
    stopGripper();
    stopShowOff();
    teachPlaybackActive_ = false;
    if (hc != nullptr) hc->cancel();
    if (pl != nullptr) pl->stop();
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (motors[i] != nullptr) motors[i]->stop();
    }
    if (queue != nullptr) xQueueReset(queue);
    if (mode_ != ArmMode::FAULT) mode_ = ArmMode::IDLE;
    Serial.println("[ARM] STOP ALL: motion cancelled and queue cleared");
}

void ArmController::stopGripper() {
    if (gripperActive_.exchange(false, std::memory_order_acq_rel)) {
        ledcWrite(GRIPPER_PWM_CHANNEL, 0);
    }
}

bool ArmController::resumeManualRelease() {
    if (!manualRelease_.load(std::memory_order_acquire)) return true;
    bool ok = true;
    for (uint8_t axis = 0; axis < 4; ++axis) {
        if (motors[axis] == nullptr || !motors[axis]->enable(true)) {
            Serial.printf("[ARM] ENABLE J%u FAIL (UART)\n", axis + 1);
            ok = false;
        }
    }
    if (!ok) {
        for (uint8_t axis = 0; axis < 4; ++axis) {
            if (motors[axis] != nullptr) motors[axis]->enable(false);
        }
        Serial.println("[ARM] ENABLE J1-J4 that bai — giu RELEASE");
        return false;
    }
    if (jm == nullptr) ok = false;
    for (uint8_t axis = 0; ok && axis < 4; ++axis) {
        if (!jm->resyncFromEncoder(axis)) {
            Serial.printf("[ARM] ENABLE J%u FAIL (encoder/home)\n", axis + 1);
            ok = false;
        }
    }
    if (!ok) {
        for (uint8_t axis = 0; axis < 4; ++axis) {
            if (motors[axis] != nullptr) motors[axis]->enable(false);
        }
        Serial.println("[ARM] ENABLE J1-J4 that bai — giu RELEASE");
        return false;
    }
    manualRelease_.store(false, std::memory_order_release);
    if (safety_ != nullptr) safety_->assertManualRelease(false);
    Serial.println("[ARM] ENABLE J1-J4 OK — da resync encoder va giu home/NVS");
    return true;
}

void ArmController::applyJog(uint8_t axis, float deltaDeg) {
    // Neo bộ đếm A4988 của khớp đang jog vào encoder thật.
    if (axis >= 4 && jm->isHomed(axis) && jm->encOK(axis)) {
        (void)jm->resyncFromEncoder(axis);
    }

    if (axis < NUM_MOTORS) {
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
        case ArmMode::DRAW:     j += "\"mode\":\"draw\",";     break;
        case ArmMode::SHOW_OFF: j += "\"mode\":\"show_off\","; break;
        case ArmMode::FAULT:    j += "\"mode\":\"fault\",";    break;
    }
    j += "\"busy\":" + String(busy() ? "true" : "false") + ",";
    char gripper[200];
    snprintf(gripper, sizeof(gripper),
             "\"gripper\":{\"ready\":%s,\"available\":%s,\"active\":%s,\"targetDeg\":%.1f,\"minDeg\":%.1f,\"maxDeg\":%.1f,\"pin\":%u},",
             gripperReady_ ? "true" : "false", gripperAvailable() ? "true" : "false",
             gripperActive_.load(std::memory_order_acquire) ? "true" : "false",
             gripperTargetDeg_.load(std::memory_order_relaxed), GRIPPER_MIN_DEG, GRIPPER_MAX_DEG, GRIPPER_SERVO_PIN);
    j += gripper;
    j += "\"commandLatencyUs\":" + String(lastCommandLatencyUs_.load(std::memory_order_relaxed)) + ",";
    j += "\"teachPoints\":[";
    const uint8_t teachMask = teachValidMask_.load();
    for (uint8_t slot = 0; slot < TEACH_POINT_COUNT; ++slot) {
        if (slot) j += ",";
        j += (teachMask & (1U << slot)) ? "true" : "false";
    }
    j += "],";
    j += "\"teachError\":" + String(static_cast<unsigned>(teachError_.load())) + ",";

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
        const String& pe = lastPlannerError_;
        const int pfi = lastPlannerFailIndex_;
        char pb[192];
        snprintf(pb, sizeof(pb),
                 "\"planner\":{\"active\":%s,\"state\":%u,\"segs\":%u,\"targetJ5Deg\":%.2f,\"lastError\":\"%s\",\"failIndex\":%d},",
                 pl->isActive() ? "true" : "false", static_cast<unsigned>(pl->state()),
                 static_cast<unsigned>(pl->segmentsDone()), pl->targetJ5Deg(), pe.c_str(), pfi);
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



void ArmController::refreshTeachPoints() {
    uint8_t mask = 0;
    for (uint8_t slot = 0; slot < TEACH_POINT_COUNT; ++slot) {
        auto& point = teachPoints_[slot];
        if (!point.valid) continue;
        for (uint8_t axis = 0; axis < NUM_MOTORS; ++axis) {
            if (jm == nullptr || !jm->isHomed(axis) ||
                point.axes[axis].homeRawDeg != jm->homeRawDeg(axis) ||
                point.axes[axis].encSign != jm->encSignOf(axis)) {
                point.valid = false;
                teachError_.store(TeachError::FRAME);
                break;
            }
        }
        if (point.valid) mask |= 1U << slot;
    }
    teachValidMask_.store(mask);
}

void ArmController::updateTeachPlayback() {
    if (!teachPlaybackActive_) return;
    if (!motionAllowed()) {
        teachPlaybackActive_ = false;
        return;
    }
    for (uint8_t axis = 0; axis < NUM_MOTORS; ++axis) {
        if (motors[axis] != nullptr && motors[axis]->isRunning()) return;
    }
    while (teachPlaybackSlot_ < TEACH_POINT_COUNT && !teachPoints_[teachPlaybackSlot_].valid) {
        ++teachPlaybackSlot_;
    }
    if (teachPlaybackSlot_ >= TEACH_POINT_COUNT) {
        teachPlaybackActive_ = false;
        mode_ = ArmMode::IDLE;
        Serial.println("[TEACH] Playback complete");
        return;
    }
    if (!moveToTeachPoint(teachPlaybackSlot_++)) {
        teachPlaybackActive_ = false;
        teachError_.store(TeachError::INVALID);
        if (safety_ != nullptr) safety_->assertEStop("teach point invalid");
        mode_ = ArmMode::FAULT;
        Serial.println("[TEACH] Playback rejected");
    }
}

bool ArmController::moveToTeachPoint(uint8_t slot) {
    refreshTeachPoints();
    if (slot >= TEACH_POINT_COUNT || !teachPoints_[slot].valid) return false;
    int64_t steps[NUM_MOTORS]{};
    float delta[NUM_MOTORS]{};
    uint32_t maxSteps = 1;
    for (uint8_t axis = 0; axis < NUM_MOTORS; ++axis) {
        const float target = teachPoints_[slot].axes[axis].deg;
        if (motors[axis] == nullptr || !jm->isHomed(axis) || !jm->encOK(axis) || !std::isfinite(target) ||
            target < DEFAULT_AXIS_LIMIT_MIN[axis] || target > DEFAULT_AXIS_LIMIT_MAX[axis]) return false;
        delta[axis] = target - jm->angleFromEncoder(axis);
        steps[axis] = JointModel::degreesToSteps(axis, fabsf(delta[axis]));
        if (steps[axis] > static_cast<int64_t>(maxSteps)) maxSteps = static_cast<uint32_t>(steps[axis]);
    }

    constexpr float MOVE_DURATION_US = 3000000.0f;
    uint32_t dominantInterval = static_cast<uint32_t>(MOVE_DURATION_US / maxSteps);
    if (dominantInterval < MIN_STEP_INTERVAL_US) dominantInterval = MIN_STEP_INTERVAL_US;
    const uint32_t rampStart = dominantInterval * 4 > MAX_STEP_INTERVAL_US
        ? dominantInterval * 4 : MAX_STEP_INTERVAL_US;
    for (uint8_t axis = 0; axis < NUM_MOTORS; ++axis) {
        if (steps[axis] <= 0) continue;
        const float scale = static_cast<float>(steps[axis]) / static_cast<float>(maxSteps);
        uint32_t interval = static_cast<uint32_t>(dominantInterval / scale);
        if (interval < MIN_STEP_INTERVAL_US) interval = MIN_STEP_INTERVAL_US;
        if (!motors[axis]->prepareCoordinatedRun(JointModel::cwForDelta(axis, delta[axis]),
                                                 static_cast<uint32_t>(steps[axis]), interval,
                                                 static_cast<uint32_t>(rampStart / scale))) {
            for (Motor* motor : motors) if (motor != nullptr) motor->stop();
            return false;
        }
    }
    for (uint8_t axis = 0; axis < NUM_MOTORS; ++axis) {
        if (steps[axis] > 0 && !motors[axis]->startPreparedRun()) {
            for (Motor* motor : motors) if (motor != nullptr) motor->stop();
            return false;
        }
    }
    mode_ = ArmMode::JOG;
    Serial.printf("[TEACH] Moving to point %c\n", 'A' + slot);
    return true;
}

void ArmController::startShowOff() {
    if (showOffActive_ || jm == nullptr) return;
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        showOffBaseAngles_[i] = (jm->isHomed(i) && jm->encOK(i))
            ? jm->angleFromEncoder(i)
            : jm->angleFromSteps(i);
    }
    showOffActive_ = true;
    showOffStep_ = 0;
    mode_ = ArmMode::SHOW_OFF;
    Serial.println("[ARM] Start SHOW OFF (dieu vu 6 truc dong bo)...");
    executeShowOffStep(0);
}

void ArmController::stopShowOff() {
    if (!showOffActive_) return;
    showOffActive_ = false;
    showOffStep_ = 0;
    Serial.println("[ARM] STOP SHOW OFF");
}

void ArmController::updateShowOff() {
    if (!showOffActive_) return;
    bool anyRunning = false;
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (motors[i] != nullptr && motors[i]->isRunning()) {
            anyRunning = true;
            break;
        }
    }
    if (anyRunning) return;

    showOffStep_++;
    if (showOffStep_ < SHOW_OFF_STEP_COUNT) {
        executeShowOffStep(showOffStep_);
    } else {
        showOffActive_ = false;
        showOffStep_ = 0;
        mode_ = ArmMode::IDLE;
        Serial.println("[ARM] SHOW OFF hoan tat thanh cong, tro ve IDLE");
    }
}

void ArmController::executeShowOffStep(uint8_t step) {
    if (step >= SHOW_OFF_STEP_COUNT || jm == nullptr) return;
    const auto& kf = SHOW_OFF_STEPS[step];

    int64_t steps[NUM_MOTORS]{};
    float deltaAct[NUM_MOTORS]{};
    uint32_t maxSteps = 1;
    bool anyMove = false;

    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (motors[i] == nullptr) continue;
        const float curDeg = (jm->isHomed(i) && jm->encOK(i))
            ? jm->angleFromEncoder(i)
            : jm->angleFromSteps(i);

        float target = showOffBaseAngles_[i] + kf.dj[i];
        if (jm->isHomed(i)) {
            if (target > DEFAULT_AXIS_LIMIT_MAX[i]) target = DEFAULT_AXIS_LIMIT_MAX[i];
            if (target < DEFAULT_AXIS_LIMIT_MIN[i]) target = DEFAULT_AXIS_LIMIT_MIN[i];
        }

        deltaAct[i] = target - curDeg;
        steps[i] = JointModel::degreesToSteps(i, fabsf(deltaAct[i]));
        if (steps[i] > 0) {
            anyMove = true;
            if (static_cast<uint32_t>(steps[i]) > maxSteps) {
                maxSteps = static_cast<uint32_t>(steps[i]);
            }
        }
    }

    if (!anyMove) return;

    const float durationUs = static_cast<float>(kf.durationMs) * 1000.0f;
    uint32_t dominantIntervalUs = static_cast<uint32_t>(durationUs / static_cast<float>(maxSteps));
    if (dominantIntervalUs < MIN_STEP_INTERVAL_US) dominantIntervalUs = MIN_STEP_INTERVAL_US;
    if (dominantIntervalUs > MAX_STEP_INTERVAL_US) dominantIntervalUs = MAX_STEP_INTERVAL_US;

    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (steps[i] <= 0 || motors[i] == nullptr) continue;
        const float scale = static_cast<float>(steps[i]) / static_cast<float>(maxSteps);
        uint32_t interval = static_cast<uint32_t>(static_cast<float>(dominantIntervalUs) / scale);
        if (interval < MIN_STEP_INTERVAL_US) interval = MIN_STEP_INTERVAL_US;
        const bool cw = JointModel::cwForDelta(i, deltaAct[i]);
        motors[i]->setSpeed(interval);
        motors[i]->run(cw, static_cast<uint32_t>(steps[i]));
    }
}
