#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <atomic>
#include <memory>
#include <mutex>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "config.h"
#include "safety_manager.h"
#include "nvs_store.h"
#include "teach_manager.h"

class Motor;
class Sensor;
class Endstops;
class JointModel;
class HomingController;
class NvsStore;

enum class ArmMode : uint8_t { IDLE = 0, RELEASE, HOMING, JOG, DRAW, SHOW_OFF, FAULT };

class WifiManager;
void armSetWifiProvider(WifiManager* w); // inject provider cho statusJson (tránh vòng include)

struct ArmCommand {
    enum Type : uint8_t {
        NONE = 0,
        JOG_REL,     // axis di chuyển tương đối value (độ)
        STOP_ALL,    // dừng mọi chuyển động + huỷ homing
        HOME_ALL,    // homing chuỗi J1..J4
        HOME_AXIS,   // homing 1 khớp (axis)
        SET_HOME,    // đặt home thủ công tại vị trí hiện tại (axis)
        CLEAR_FAULT, // thoát trạng thái FAULT
        SHOW_OFF,    // trình diễn đồng bộ 6 khớp múa nhẹ nhàng trong khoảng an toàn
        TEACH_FREE,  // thả lỏng J1..J3 để kéo tay, khóa cứng J4..J6
        TEACH_RECORD,// ghi điểm quỹ đạo (value >= 0.5f: Hạ bút; < 0.5f: Nhấc bút)
        TEACH_UNDO,  // xóa điểm vừa ghi
        TEACH_CLEAR, // xóa toàn bộ điểm quỹ đạo
        TEACH_PLAY,  // chạy lại quỹ đạo dạy (p[0]=stepDeg, p[1]=speedDelayMs)
        TEACH_STOP,  // dừng chạy lại
        CLEAR_CALIB, // xóa calib/NVS home
        SET_GRIPPER  // value: góc servo gripper [0..180]
    };
    Type type{Type::NONE};
    uint8_t axis{0};
    float value{0.0f};
    float p[8]{0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t submittedAtUs{0};
};

/**
 * Bộ điều phối trung tâm: nhận lệnh từ Web/Serial, tuần tự hoá và thực thi trong
 * motion task (Core 1 @100Hz). Web handler chỉ enqueue — không đụng phần cứng.
 */
class ArmController {
public:
    ArmController();
    ~ArmController();

    ArmController(const ArmController&) = delete;
    ArmController& operator=(const ArmController&) = delete;

    void begin(Motor** motors, Sensor* sensor, Endstops* endstops,
               JointModel* joints, HomingController* homing, NvsStore* nvs);

    // Đưa lệnh vào hàng đợi. Trả false nếu đầy/đang bận với lệnh không thể trộn.
    [[nodiscard]] bool submit(const ArmCommand& cmd, uint32_t timeoutMs = 10);
    [[nodiscard]] bool busy() const;
    [[nodiscard]] bool gripperAvailable() const;

    [[nodiscard]] ArmMode mode() const;
    [[nodiscard]] SafetyManager* safety() noexcept { return safety_.get(); }
    [[nodiscard]] TeachManager* teachManager() noexcept { return &teachMgr_; }
    String statusJson();

    [[nodiscard]] bool isWristLocked() const noexcept { return wristLocked_.load(std::memory_order_relaxed); }
    void setWristLocked(bool lock);

    [[nodiscard]] bool isActiveHoldEnabled() const noexcept { return activeHold_.load(std::memory_order_relaxed); }
    void setActiveHold(bool en) noexcept { activeHold_.store(en, std::memory_order_relaxed); }

    void setTargetHoldAngle(uint8_t axis, float deg) noexcept {
        if (axis < NUM_MOTORS) targetHoldAngle_[axis] = deg;
    }
    [[nodiscard]] float getTargetHoldAngle(uint8_t axis) const noexcept {
        return (axis < NUM_MOTORS) ? targetHoldAngle_[axis] : 0.0f;
    }

private:
    static void taskEntry(void* param);
    void taskLoop();
    void execute(const ArmCommand& cmd);
    void stopAllAndDiscardQueuedMotion();
    void stopGripper();
    [[nodiscard]] bool resumeManualRelease();
    void applyJog(uint8_t axis, float deltaDeg);
    [[nodiscard]] bool motionAllowed() const;

    // Show Off — múa đồng bộ 6 khớp
    void startShowOff();
    void updateShowOff();
    void stopShowOff();
    void executeShowOffStep(uint8_t step);

    Motor* motors[NUM_MOTORS]{};
    Sensor* sensor{nullptr};
    Endstops* es{nullptr};
    JointModel* jm{nullptr};
    HomingController* hc{nullptr};
    NvsStore* nvs{nullptr};

    QueueHandle_t queue{nullptr};
    TaskHandle_t task{nullptr};
    std::unique_ptr<SafetyManager> safety_{nullptr};
    std::atomic<ArmMode> mode_{ArmMode::IDLE};
    std::atomic<bool> stopRequested_{false};
    bool gripperReady_{false}; // initialized before motion/web tasks use it
    std::atomic<bool> gripperActive_{false};
    std::atomic<float> gripperTargetDeg_{90.0f}; // target only; no servo feedback
    std::atomic<bool> manualRelease_{false};
    std::atomic<uint32_t> lastCommandLatencyUs_{0};
    uint8_t recoveryJogAxis_{NUM_MOTORS};
    EndstopWhich recoveryJogSide_{EndstopWhich::MIN};

    // Trạng thái Show Off
    std::atomic<bool> showOffActive_{false};
    uint8_t showOffStep_{0};
    float showOffBaseAngles_[NUM_MOTORS]{};

    TeachManager teachMgr_;

    std::atomic<bool> wristLocked_{true};
    std::atomic<bool> activeHold_{true};
    float targetHoldAngle_[NUM_MOTORS]{};
};

#endif // ARM_H
