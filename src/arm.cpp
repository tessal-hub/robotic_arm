#include "arm.h"
#include "endstop.h"
#include "homing.h"
#include "joint_model.h"
#include "kinematics.h"
#include "motor.h"
#include "nvs_store.h"
#include "safety_manager.h"
#include "sensor.h"
#include "wifi_manager.h"

#include <esp_task_wdt.h>

namespace {
WifiManager* g_wifi = nullptr; // inject để statusJson đọc wifi (tránh include vòng)

struct ShowOffPose {
    float dj[NUM_MOTORS];
    uint32_t durationMs;
};

constexpr uint8_t SHOW_OFF_STEP_COUNT = 8;
constexpr ShowOffPose SHOW_OFF_STEPS[SHOW_OFF_STEP_COUNT] = {
    { { +12.0f, +5.0f, -6.0f, 0.0f, 0.0f, 0.0f }, 1200 },
    { { +15.0f, -4.0f, +6.0f, 0.0f, 0.0f, 0.0f }, 1100 },
    { {   0.0f, -6.0f, +8.0f, 0.0f, 0.0f, 0.0f }, 1300 },
    { { -12.0f, -4.0f, +6.0f, 0.0f, 0.0f, 0.0f }, 1100 },
    { { -15.0f, +6.0f, -6.0f, 0.0f, 0.0f, 0.0f }, 1200 },
    { {   0.0f, +7.0f, -8.0f, 0.0f, 0.0f, 0.0f }, 1300 },
    { {  +6.0f, -2.0f, +3.0f, 0.0f, 0.0f, 0.0f }, 1000 },
    { {   0.0f,  0.0f,  0.0f, 0.0f, 0.0f, 0.0f }, 1200 }
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
                          NvsStore* nvs_) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = motors_[i];
    sensor = sensor_;
    es = endstops_;
    jm = joints_;
    hc = homing_;
    nvs = nvs_;

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

    teachMgr_.begin(motors, jm, safety_.get());

    wristLocked_.store(LOCK_WRIST_J4_J6, std::memory_order_relaxed);
    activeHold_.store(false, std::memory_order_relaxed);
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        targetHoldAngle_[i] = (jm && jm->isHomed(i) && jm->encOK(i))
            ? jm->angleFromEncoder(i)
            : (jm ? jm->angleFromSteps(i) : 0.0f);
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
    if (cmd.type == ArmCommand::STOP_ALL) {
        stopRequested_.store(true, std::memory_order_release);
        return true;
    }

    if (cmd.type == ArmCommand::SET_GRIPPER &&
        (!std::isfinite(cmd.value) || cmd.value < GRIPPER_MIN_DEG ||
         cmd.value > GRIPPER_MAX_DEG || !gripperAvailable())) return false;
    // FIX: Chỉ chặn queue 1 phần tử đối với các lệnh chuyển động vật lý (JOG/HOME/SHOW_OFF),
    // cho phép xếp hàng đợi với các lệnh trạng thái/teach (TEACH_RECORD, TEACH_UNDO, TEACH_FREE...)
    const bool isMotionCmd = (cmd.type == ArmCommand::JOG_REL ||
                              cmd.type == ArmCommand::HOME_ALL ||
                              cmd.type == ArmCommand::HOME_AXIS ||
                              cmd.type == ArmCommand::SHOW_OFF ||
                              cmd.type == ArmCommand::TEACH_PLAY);
    if (isMotionCmd && uxQueueMessagesWaiting(queue) != 0) return false;
    ArmCommand queued = cmd;
    queued.submittedAtUs = micros();

    return xQueueSend(queue, &queued, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

bool ArmController::busy() const {
    if (queue != nullptr && uxQueueMessagesWaiting(queue) != 0) return true;
    if (hc != nullptr && hc->isActive()) return true;
    if (showOffActive_) return true;
    if (teachMgr_.isPlaying()) return true;
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

void ArmController::setWristLocked(bool lock) {
    wristLocked_.store(lock, std::memory_order_relaxed);
    if (lock) {
        teachMgr_.lockWristAxes();
        if (motors[3] != nullptr) motors[3]->setHoldCurrentScale(WRIST_J4_HOLD_SCALE);
        Serial.println("[ARM] KHOA CUNG cum co tay (J4..J6)");
    } else {
        if (motors[3] != nullptr) motors[3]->setHoldCurrentScale(DEFAULT_HOLD_SCALE);
        Serial.println("[ARM] MO KHOA cum co tay (J4..J6) — san sang dieu khien");
    }
}

void ArmController::taskEntry(void* param) {
    auto* self = static_cast<ArmController*>(param);
    self->taskLoop();
}

void ArmController::taskLoop() {
    TickType_t lastWake = xTaskGetTickCount();
    ArmCommand cmd;
    bool wasHoming{false};

    // Đăng ký Task WDT an toàn
    const bool wdtRegistered = (esp_task_wdt_add(nullptr) == ESP_OK);

    const TickType_t period = pdMS_TO_TICKS(MOTION_TASK_PERIOD_MS);

    for (;;) {
        const TickType_t now = xTaskGetTickCount();
        if ((now - lastWake) > period * 2) {
            lastWake = now;
        }
        vTaskDelayUntil(&lastWake, period);
        if (wdtRegistered) esp_task_wdt_reset();

        // Highest command priority: cancel current motion and discard all queued motion commands
        if (stopRequested_.exchange(false, std::memory_order_acq_rel)) {
            stopAllAndDiscardQueuedMotion();
            continue;
        }

        // Safety poll: debounce ISR pending → latch/E_STOP (50ms)
        if (safety_ != nullptr) safety_->pollEndstops();

        // Recovery Jog masks the held switch until that finite jog finishes.
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

        // 1) Homing FSM trước (ưu tiên an toàn), rồi teach manager
        if (hc != nullptr) hc->tick();
        updateShowOff();
        teachMgr_.tick();

        // Sau khi homing hoàn tất: endstop vẫn nhấn do backoff là bình thường → clear latch
        if (hc != nullptr && wasHoming && !hc->isActive()) {
            es->clearAllLatches();
            if (wristLocked_.load(std::memory_order_relaxed)) {
                teachMgr_.lockWristAxes();
            }
            for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
                if (jm != nullptr && jm->isHomed(a)) {
                    targetHoldAngle_[a] = (jm->encOK(a)) ? jm->angleFromEncoder(a) : jm->angleFromSteps(a);
                }
            }
        }
        wasHoming = (hc != nullptr && hc->isActive());

        // 2) Endstop bảo vệ + E-stop (homing tự xử lý endstop riêng)
        if (!manualRelease_.load(std::memory_order_acquire) && !teachMgr_.isFree() && es != nullptr && hc != nullptr &&
            !hc->isActive() && mode_ != ArmMode::FAULT) {
            const bool estopPending = safety_ ? safety_->isEStop() : false;
            const bool latchPending = safety_ ? safety_->anyLatched() : es->anyLatched();

            if (estopPending || latchPending) {
                stopShowOff();
                teachMgr_.stopPlayback();
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
                        if (motors[i] == nullptr || !motors[i]->isRunning()) continue;
                        // SỬA LỖI: Dùng isPressed() thay vì isLatched() vì nhánh else chỉ chạy khi latchPending == false
                        if (es->hasPin(i, EndstopWhich::MIN) && es->isPressed(i, EndstopWhich::MIN)) {
                            const bool movingAway = (motors[i]->getDirCW() == JointModel::cwForDelta(i, +1.0f));
                            if (!movingAway) {
                                stopShowOff();
                                teachMgr_.stopPlayback();
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
                            const bool movingAway = (motors[i]->getDirCW() == JointModel::cwForDelta(i, -1.0f));
                            if (!movingAway) {
                                stopShowOff();
                                teachMgr_.stopPlayback();
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
            if (teachMgr_.isFree() || manualRelease_.load(std::memory_order_acquire)) {
                mode_ = ArmMode::RELEASE;
            } else if (hc != nullptr && hc->isActive()) {
                mode_ = ArmMode::HOMING;
            } else if (teachMgr_.isPlaying()) {
                mode_ = ArmMode::DRAW;
            } else if (showOffActive_) {
                mode_ = ArmMode::SHOW_OFF;
            } else if (busy()) {
                mode_ = ArmMode::JOG;
            } else {
                mode_ = ArmMode::IDLE;
            }
        }

        // 3.5) Closed-loop Active Joint Holding (chỉ chạy khi BẬT và hoàn toàn IDLE)
        if (mode_ == ArmMode::IDLE && activeHold_.load(std::memory_order_relaxed) &&
            !manualRelease_.load(std::memory_order_acquire) && !teachMgr_.isFree()) {
            static uint16_t holdTick = 0;
            if (++holdTick >= 50) { // Mỗi 500ms (Core 1 loop @100Hz = 10ms)
                holdTick = 0;
                bool anyRunning = false;
                for (uint8_t m = 0; m < NUM_MOTORS; ++m) {
                    if (motors[m] != nullptr && motors[m]->isRunning()) { anyRunning = true; break; }
                }
                if (!anyRunning) {
                    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
                        if (wristLocked_.load(std::memory_order_relaxed) && i >= 3) continue;
                        if (motors[i] == nullptr || motors[i]->isRunning()) continue;
                        if (jm == nullptr || !jm->isHomed(i) || !jm->encOK(i)) continue;

                        const float act = jm->angleFromEncoder(i);
                        const float err = targetHoldAngle_[i] - act;
                        // Vùng trễ (deadband) 1.2° tránh dao động vi bước do nhiễu số của encoder & độ rơ cơ khí
                        if (fabsf(err) > 1.2f) {
                            int32_t corrSteps = static_cast<int32_t>(roundf(fabsf(err) * JointModel::stepsPerDegree(i)));
                            if (corrSteps > 15) corrSteps = 15; // Tối đa 15 bước mỗi chu kỳ 500ms, kéo nhẹ nhàng không giật
                            if (corrSteps >= 3) {
                                const bool cw = JointModel::cwForDelta(i, err);
                                motors[i]->setSpeed(DEFAULT_AXIS_HOMING_SPEEDS[i]);
                                motors[i]->run(cw, static_cast<uint32_t>(corrSteps));
                                break; // Chỉ vi chỉnh 1 trục mỗi chu kỳ 500ms để không nghẽn bus UART
                            }
                        }
                    }
                }
            }
        }

        if (!motionAllowed()) stopGripper();

        // 4) Rút lệnh từ queue (không block)
        while (queue != nullptr && xQueueReceive(queue, &cmd, 0) == pdTRUE) {
            execute(cmd);
        }
    }

    if (wdtRegistered) esp_task_wdt_delete(nullptr);
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
            // FIX: Chỉ home chuỗi J1..J3 (bỏ J4 khỏi chuỗi auto-homing nếu dùng cơ cấu 3 trục dưới vẽ)
            if (hc != nullptr && !hc->startAll()) mode_ = ArmMode::IDLE;
            break;

        case ArmCommand::HOME_AXIS:
            if (wristLocked_.load(std::memory_order_relaxed) && cmd.axis >= 3) {
                Serial.printf("[ARM] J%u dang khoa cung (co tay), bo qua HOME_AXIS\n", cmd.axis + 1);
                break;
            }
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
                    jm->setHomeHere(4);
                    jm->setHomeHere(5);
                    targetHoldAngle_[4] = 0.0f;
                    targetHoldAngle_[5] = 0.0f;
                    Serial.println("[ARM] Set-Home Wrist (J5 + J6) tai vi tri hien tai");
                } else if (cmd.axis < NUM_MOTORS) {
                    jm->setHomeHere(cmd.axis);
                    targetHoldAngle_[cmd.axis] = 0.0f;
                    Serial.printf("[ARM] Set-Home J%u tai vi tri hien tai\n", cmd.axis + 1);
                }
            }
            break;

        case ArmCommand::JOG_REL:
            if (wristLocked_.load(std::memory_order_relaxed) && cmd.axis >= 3) {
                teachMgr_.lockWristAxes();
                Serial.printf("[ARM] TU CHOI JOG J%u: truc co tay J4..J6 dang khoa cung!\n", cmd.axis + 1);
                break;
            }
            if (teachMgr_.isFree()) {
                teachMgr_.setFree(false);
                Serial.println("[ARM] JOG tu dong khoa lai Teach Free mode de bao ve endstop");
            }
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
            if (es != nullptr && es->hasPin(cmd.axis, EndstopWhich::MIN) &&
                es->isPhysicallyPressed(cmd.axis, EndstopWhich::MIN) && cmd.value < 0.0f) {
                Serial.printf("[ARM] TU CHOI JOG J%u: endstop MIN dang nhan (muc vat ly)\n",
                              cmd.axis + 1);
                break;
            }
            if (es != nullptr && es->hasPin(cmd.axis, EndstopWhich::MAX) &&
                es->isPhysicallyPressed(cmd.axis, EndstopWhich::MAX) && cmd.value > 0.0f) {
                Serial.printf("[ARM] TU CHOI JOG J%u: endstop MAX dang nhan (muc vat ly)\n",
                              cmd.axis + 1);
                break;
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

        case ArmCommand::TEACH_FREE:
            if ((hc != nullptr && hc->isActive()) || showOffActive_) {
                Serial.println("[ARM] Tu choi TEACH_FREE khi dang Homing/ShowOff");
                break;
            }
            teachMgr_.setFree(cmd.value > 0.5f);
            break;

        case ArmCommand::TEACH_RECORD:
            teachMgr_.recordPoint(cmd.value >= 0.5f);
            break;

        case ArmCommand::TEACH_UNDO:
            teachMgr_.undoPoint();
            break;

        case ArmCommand::TEACH_CLEAR:
            teachMgr_.clearPoints();
            break;

        case ArmCommand::TEACH_PLAY:
            if (motionAllowed()) {
                teachMgr_.startPlayback(cmd.p[0] > 0.01f ? cmd.p[0] : 0.5f,
                                        cmd.p[1] > 0.01f ? static_cast<uint32_t>(cmd.p[1]) : 15);
            }
            break;

        case ArmCommand::TEACH_STOP:
            teachMgr_.stopPlayback();
            break;

        case ArmCommand::CLEAR_CALIB:
            if (busy()) break;
            if (jm != nullptr) {
                if (cmd.axis < NUM_MOTORS) {
                    jm->forgetHome(cmd.axis);
                    Serial.printf("[ARM] Cleared joint calibration / NVS home for J%u\n", cmd.axis + 1);
                } else {
                    for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
                        jm->forgetHome(a);
                    }
                    Serial.println("[ARM] Cleared all joint calibrations / NVS homes");
                }
            }
            break;

        default:
            break;
    }
}

void ArmController::stopAllAndDiscardQueuedMotion() {
    stopGripper();
    stopShowOff();
    teachMgr_.stopPlayback();
    if (hc != nullptr) hc->cancel();
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
    // FIX: Đồng bộ thoát cả trạng thái Teach Free trước khi chạy lệnh mới
    if (teachMgr_.isFree()) {
        teachMgr_.setFree(false);
    }
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
    if (axis >= 4 && jm->isHomed(axis) && jm->encOK(axis)) {
        (void)jm->resyncFromEncoder(axis);
    }

    if (axis < NUM_MOTORS) {
        float delta = deltaDeg;
        if (jm->isHomed(axis) && recoveryJogAxis_ != axis) {
            const float cur = (jm->encOK(axis)) ? jm->angleFromEncoder(axis) : jm->angleFromSteps(axis);
            const float minLim = DEFAULT_AXIS_LIMIT_MIN[axis];
            const float maxLim = DEFAULT_AXIS_LIMIT_MAX[axis];

            if (deltaDeg > 0.0f) {
                if (cur >= maxLim) {
                    delta = 0.0f;
                } else if (cur + deltaDeg > maxLim) {
                    delta = maxLim - cur;
                } else {
                    delta = deltaDeg;
                }
            } else if (deltaDeg < 0.0f) {
                if (cur <= minLim) {
                    delta = 0.0f;
                } else if (cur + deltaDeg < minLim) {
                    delta = minLim - cur;
                } else {
                    delta = deltaDeg;
                }
            } else {
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
        const float curAngle = (jm->isHomed(axis) && jm->encOK(axis)) ? jm->angleFromEncoder(axis) : jm->angleFromSteps(axis);
        targetHoldAngle_[axis] = curAngle + delta;
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
        case ArmMode::DRAW:     j += "\"mode\":\"draw\",";     break;
        case ArmMode::SHOW_OFF: j += "\"mode\":\"show_off\","; break;
        case ArmMode::FAULT:    j += "\"mode\":\"fault\",";    break;
    }
    j += (busy() ? "\"busy\":true," : "\"busy\":false,");
    char gripper[200];
    snprintf(gripper, sizeof(gripper),
             "\"gripper\":{\"ready\":%s,\"available\":%s,\"active\":%s,\"targetDeg\":%.1f,\"minDeg\":%.1f,\"maxDeg\":%.1f,\"pin\":%u},",
             gripperReady_ ? "true" : "false", gripperAvailable() ? "true" : "false",
             gripperActive_.load(std::memory_order_acquire) ? "true" : "false",
             gripperTargetDeg_.load(std::memory_order_relaxed), GRIPPER_MIN_DEG, GRIPPER_MAX_DEG, GRIPPER_SERVO_PIN);
    j += gripper;

    char metaBuf[128];
    snprintf(metaBuf, sizeof(metaBuf),
             "\"commandLatencyUs\":%lu,\"wristLocked\":%s,\"activeHold\":%s,\"toolLength\":%.1f,\"targetAngles\":[",
             static_cast<unsigned long>(lastCommandLatencyUs_.load(std::memory_order_relaxed)),
             isWristLocked() ? "true" : "false",
             isActiveHoldEnabled() ? "true" : "false",
             kin::getToolLength());
    j += metaBuf;

    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (i > 0) j += ",";
        char fBuf[16];
        snprintf(fBuf, sizeof(fBuf), "%.2f", targetHoldAngle_[i]);
        j += fBuf;
    }
    j += "],";
    j += "\"teachTrajectory\":" + teachMgr_.statusJson() + ",";

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

    j += "\"motors\":[";
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
        if (i > 0) j += ",";
        j += (motors[i] != nullptr) ? motors[i]->toJson() : "{}";
    }
    j += "]";
    j += "}";
    return j;
}

void ArmController::startShowOff() {
    if (showOffActive_ || jm == nullptr) return;
    const uint8_t activeAxes = wristLocked_.load(std::memory_order_relaxed) ? 3 : NUM_MOTORS;
    for (uint8_t i = 0; i < activeAxes; ++i) {
        if (jm->isHomed(i) && jm->encOK(i)) {
            (void)jm->resyncFromEncoder(i);
        }
        showOffBaseAngles_[i] = jm->angleFromSteps(i);
    }
    showOffActive_ = true;
    showOffStep_ = 0;
    mode_ = ArmMode::SHOW_OFF;
    Serial.println("[ARM] Start SHOW OFF (dieu vu 3 truc chinh dong bo)...");
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
    const uint8_t activeAxes = wristLocked_.load(std::memory_order_relaxed) ? 3 : NUM_MOTORS;
    float requiredDurationUs = static_cast<float>(kf.durationMs) * 1000.0f;

    for (uint8_t i = 0; i < activeAxes; ++i) {
        if (motors[i] == nullptr) continue;
        const float curDeg = jm->angleFromSteps(i);

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
            const float axisMinDurationUs =
                static_cast<float>(steps[i]) * static_cast<float>(DEFAULT_AXIS_JOG_SPEEDS[i]);
            if (axisMinDurationUs > requiredDurationUs) {
                requiredDurationUs = axisMinDurationUs;
            }
        }
    }

    if (!anyMove) return;

    uint32_t dominantIntervalUs = static_cast<uint32_t>(requiredDurationUs / static_cast<float>(maxSteps));
    if (dominantIntervalUs < MIN_STEP_INTERVAL_US) dominantIntervalUs = MIN_STEP_INTERVAL_US;
    if (dominantIntervalUs > MAX_STEP_INTERVAL_US) dominantIntervalUs = MAX_STEP_INTERVAL_US;

    const float actualDurationUs = static_cast<float>(dominantIntervalUs) * static_cast<float>(maxSteps);

    for (uint8_t i = 0; i < activeAxes; ++i) {
        if (steps[i] <= 0 || motors[i] == nullptr) continue;
        uint32_t interval = static_cast<uint32_t>(actualDurationUs / static_cast<float>(steps[i]));
        if (interval < DEFAULT_AXIS_JOG_SPEEDS[i]) interval = DEFAULT_AXIS_JOG_SPEEDS[i];
        const uint32_t rampStartUs = (maxSteps > 80) ? (interval * 2U) : 0U;
        const bool cw = JointModel::cwForDelta(i, deltaAct[i]);
        if (!motors[i]->prepareCoordinatedRun(cw, static_cast<uint32_t>(steps[i]), interval, rampStartUs)) {
            for (uint8_t j = 0; j < NUM_MOTORS; ++j) {
                if (motors[j] != nullptr) motors[j]->stop();
            }
            stopShowOff();
            Serial.printf("[ARM] SHOW OFF huy: J%u prepareCoordinatedRun that bai\n", i + 1);
            return;
        }
    }
    for (uint8_t i = 0; i < activeAxes; ++i) {
        if (steps[i] > 0 && motors[i] != nullptr) {
            (void)motors[i]->startPreparedRun();
        }
    }
}
