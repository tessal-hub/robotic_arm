#include "arm.h"
#include "differential_wrist.h"
#include "endstop.h"
#include "homing.h"
#include "joint_model.h"
#include "kinematics.h"
#include "motor.h"
#include "planner.h"
#include "sensor.h"
#include "wifi_manager.h"

#include <esp_task_wdt.h>

namespace {
WifiManager* g_wifi = nullptr; // inject để statusJson đọc wifi (tránh include vòng)
} // namespace

void armSetWifiProvider(WifiManager* w) { g_wifi = w; }

ArmController::ArmController() {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = nullptr;
    mode_ = ArmMode::IDLE;
}

void ArmController::begin(Motor** motors_, Sensor* sensor_, Endstops* endstops_,
                          JointModel* joints_, HomingController* homing_,
                          Planner* planner_) {
    for (uint8_t i = 0; i < NUM_MOTORS; ++i) motors[i] = motors_[i];
    sensor = sensor_;
    es = endstops_;
    jm = joints_;
    hc = homing_;
    pl = planner_;

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

        // 1) Homing FSM trước (ưu tiên an toàn), rồi planner
        if (hc != nullptr) hc->tick();
        if (pl != nullptr) pl->tick();

        // Sau khi homing hoàn tất: endstop vẫn nhấn do backoff là bình thường → clear latch
        if (hc != nullptr && wasHoming && !hc->isActive()) {
            es->clearAllLatches();
        }
        wasHoming = (hc != nullptr && hc->isActive());

        // 2) Endstop bảo vệ + E-stop (homing tự xử lý endstop riêng)
        if (es != nullptr && hc != nullptr && !hc->isActive() && mode_ != ArmMode::FAULT) {
            const bool estopPending = g_emergencyStop.load(std::memory_order_acquire);
            const bool latchPending = es->anyLatched();

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
                                g_emergencyStop.store(true, std::memory_order_release);
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
                                g_emergencyStop.store(true, std::memory_order_release);
                                mode_ = ArmMode::FAULT;
                                Serial.printf("[ARM] FAULT: endstop J%u MAX pressed during motion\n", i + 1);
                                break;
                            }
                        }
                    }
                }
            }
        }

        // 3) Drift watchdog ~ mỗi 500ms (chỉ chạy khi KHÔNG homing)
        if (++driftTickCounter >= (500 / MOTION_TASK_PERIOD_MS)) {
            driftTickCounter = 0;
            if (jm != nullptr && (hc == nullptr || !hc->isActive()) && mode_ != ArmMode::FAULT) {
                for (uint8_t i = 0; i < NUM_MOTORS; ++i) jm->updateDriftCheck(i);
                if (jm->hasAnyDriftFault()) {
                    if (pl != nullptr) pl->stop();
                    for (uint8_t j = 0; j < NUM_MOTORS; ++j) {
                        if (motors[j] != nullptr) motors[j]->stop();
                    }
                    mode_ = ArmMode::FAULT;
                    Serial.println("[ARM] FAULT: step/encoder drift exceeded threshold");
                }
            }
        }

        // 4) Mode runtime (tính lại mỗi tick, tránh kẹt trạng thái)
        if (mode_ != ArmMode::FAULT) {
            if (hc != nullptr && hc->isActive()) {
                mode_ = ArmMode::HOMING;
            } else if (pl != nullptr && pl->isActive()) {
                mode_ = pl->isDrawing() ? ArmMode::DRAW : ArmMode::CART;
            } else if (busy()) {
                mode_ = ArmMode::JOG;
            } else {
                mode_ = ArmMode::IDLE;
            }
        }

        // 5) Rút lệnh từ queue (không block)
        while (queue != nullptr && xQueueReceive(queue, &cmd, 0) == pdTRUE) {
            execute(cmd);
        }
    }

    esp_task_wdt_delete(nullptr);
}

bool ArmController::motionAllowed() const { return mode_ != ArmMode::FAULT; }

void ArmController::execute(const ArmCommand& cmd) {
    switch (cmd.type) {
        case ArmCommand::STOP_ALL:
            if (hc != nullptr) hc->cancel();
            if (pl != nullptr) pl->stop();
            for (uint8_t i = 0; i < NUM_MOTORS; ++i) {
                if (motors[i] != nullptr) motors[i]->stop();
            }
            if (mode_ != ArmMode::FAULT) mode_ = ArmMode::IDLE;
            Serial.println("[ARM] STOP ALL");
            break;

        case ArmCommand::CLEAR_FAULT:
            if (es != nullptr) {
                es->clearAllLatches();
            }
            g_emergencyStop.store(false, std::memory_order_release);
            if (jm != nullptr) jm->clearAllDriftFaults();
            mode_ = ArmMode::IDLE;
            Serial.println("[ARM] FAULT cleared -> IDLE");
            break;

        case ArmCommand::HOME_ALL:
            if (!motionAllowed()) break;
            if (busy()) break;
            mode_ = ArmMode::HOMING;
            if (hc != nullptr && !hc->startAll()) mode_ = ArmMode::IDLE;
            break;

        case ArmCommand::HOME_AXIS:
            if (!motionAllowed()) break;
            if (busy() || cmd.axis >= 4) break;
            mode_ = ArmMode::HOMING;
            if (hc != nullptr && !hc->startAxis(cmd.axis)) mode_ = ArmMode::IDLE;
            break;

        case ArmCommand::SET_HOME:
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

        case ArmCommand::JOG_REL:
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
        case ArmCommand::DRAW_CIRCLE: {
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
            } else {
                job.shape = Planner::Shape::CIRCLE;
                job.x1 = cmd.p[0]; // cx
                job.y1 = cmd.p[1]; // cy
                job.z = cmd.p[2];
                job.r = cmd.p[3];
            }
            if (cmd.p[5] > 1.0f && cmd.p[5] < 200.0f) job.feedMmS = cmd.p[5];
            mode_ = (cmd.type == ArmCommand::MOVE_CART) ? ArmMode::CART : ArmMode::DRAW;
            if (!pl->submit(job)) mode_ = ArmMode::IDLE;
            break;
        }

        default:
            break;
    }
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
    j.reserve(3500);
    j = "{";
    j += "\"fw\":\"" FW_VERSION "\",";
    switch (mode_) {
        case ArmMode::IDLE:   j += "\"mode\":\"idle\",";   break;
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
        char pb[64];
        snprintf(pb, sizeof(pb), "\"planner\":{\"active\":%s,\"state\":%u,\"segs\":%u},",
                 pl->isActive() ? "true" : "false",
                 static_cast<unsigned>(pl->state()),
                 static_cast<unsigned>(pl->segmentsDone()));
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
