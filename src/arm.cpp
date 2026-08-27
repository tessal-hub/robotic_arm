#include "arm.h"
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
}

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

void ArmController::estopFromISR() {
    // ISR endstop ngoài ngữ cảnh homing => dừng tất cả + FAULT
    mode_ = ArmMode::FAULT;
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

    for (;;) {
        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(MOTION_TASK_PERIOD_MS));
        esp_task_wdt_reset();

        // 1) Homing FSM trước (ưu tiên an toàn), rồi planner
        if (hc != nullptr) hc->tick();
        if (pl != nullptr) pl->tick();

        // Sau khi homing hoàn tất: endstop vẫn nhấn do backoff là bình thường → clear latch
        if (hc != nullptr && wasHoming && !hc->isActive()) {
            es->clearAllLatches();
        }
        wasHoming = (hc != nullptr && hc->isActive());

        // 2) Endstop bảo vệ: chỉ khi motor đang chạy VÀ endstop vật lý thực sự nhấn
        //    Homing tự xử lý endstop riêng — arm không can thiệp trong quá trình homing.
        //    Endstop pressed at boot = OK (resting against switch), không fault.
        if (es != nullptr && hc != nullptr && !hc->isActive()) {
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
                        for (uint8_t j = 0; j < NUM_MOTORS; ++j)
                            if (motors[j] != nullptr) motors[j]->stop();
                        es->clearAllLatches();
                        mode_ = ArmMode::FAULT;
                        Serial.printf("[ARM] FAULT: endstop J%u MIN pressed during motion\n", i + 1);
                        break;
                    }
                    if (es->hasPin(i, EndstopWhich::MAX) && es->isPressed(i, EndstopWhich::MAX)) {
                        for (uint8_t j = 0; j < NUM_MOTORS; ++j)
                            if (motors[j] != nullptr) motors[j]->stop();
                        es->clearAllLatches();
                        mode_ = ArmMode::FAULT;
                        Serial.printf("[ARM] FAULT: endstop J%u MAX pressed during motion\n", i + 1);
                        break;
                    }
                }
            }
        }

        // 3) Drift watchdog ~ mỗi 500ms
        if (++driftTickCounter >= (500 / MOTION_TASK_PERIOD_MS)) {
            driftTickCounter = 0;
            if (jm != nullptr) {
                for (uint8_t i = 0; i < NUM_MOTORS; ++i) jm->updateDriftCheck(i);
            }
        }

        // 4) Mode runtime (tính lại mỗi tick, tránh kẹt trạng thái)
        if (mode_ != ArmMode::FAULT) {
            if (hc != nullptr && hc->isActive()) {
                mode_ = ArmMode::HOMING;
            } else if (pl != nullptr && pl->isActive()) {
                mode_ = ArmMode::CART;
            } else if (mode_ == ArmMode::HOMING || mode_ == ArmMode::CART ||
                       mode_ == ArmMode::DRAW) {
                mode_ = ArmMode::IDLE;
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
            if (mode_ == ArmMode::FAULT) mode_ = ArmMode::IDLE;
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
            if (jm != nullptr) jm->setHomeHere(cmd.axis);
            Serial.printf("[ARM] Set-Home J%u tai vi tri hien tai\n", cmd.axis + 1);
            break;

        case ArmCommand::JOG_REL:
            if (!motionAllowed()) break;
            if (cmd.axis >= NUM_MOTORS || hc == nullptr || jm == nullptr) break;
            if (motors[cmd.axis]->isRunning()) break; // bỏ qua nếu trục đang chạy
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
            mode_ = ArmMode::CART;
            if (!pl->submit(job)) mode_ = ArmMode::IDLE;
            break;
        }

        default:
            break;
    }
}

void ArmController::applyJog(uint8_t axis, float deltaDeg) {
    float delta = deltaDeg;
    if (jm->isHomed(axis)) {
        const float cur = jm->angleFromSteps(axis);
        float target = cur + delta;
        if (target > DEFAULT_AXIS_LIMIT_MAX[axis]) target = DEFAULT_AXIS_LIMIT_MAX[axis];
        if (target < DEFAULT_AXIS_LIMIT_MIN[axis]) target = DEFAULT_AXIS_LIMIT_MIN[axis];
        delta = target - cur; // clamp theo soft limit
    }
    const int64_t steps = JointModel::degreesToSteps(axis, fabsf(delta));
    if (steps <= 0) return;
    const bool cw = JointModel::cwForDelta(axis, delta);
    motors[axis]->run(cw, static_cast<uint32_t>(steps));
}

String ArmController::statusJson() {
    String j = "{";
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

    // TCP pose hiện tại theo FK từ góc khớp (step count)
    {
        float enc[6];
        for (uint8_t i = 0; i < NUM_MOTORS; ++i)
            enc[i] = jm ? jm->angleFromSteps(i) : 0.0f;
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
