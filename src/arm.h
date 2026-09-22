#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <atomic>
#include <memory>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "config.h"
#include "safety_manager.h"
#include "nvs_store.h"

class Motor;
class Sensor;
class Endstops;
class JointModel;
class HomingController;
class Planner;
class NvsStore;

enum class ArmMode : uint8_t { IDLE = 0, RELEASE, HOMING, JOG, CART, DRAW, SHOW_OFF, FAULT };

class WifiManager;
void armSetWifiProvider(WifiManager* w); // inject provider cho statusJson (tránh vòng include)

struct ArmCommand {
    enum Type : uint8_t {
        NONE = 0,
        JOG_REL,     // axis di chuyển tương đối value (độ)
        STOP_ALL,    // dừng mọi chuyển động + huỷ homing/planner
        HOME_ALL,    // homing chuỗi J1..J4
        HOME_AXIS,   // homing 1 khớp (axis)
        SET_HOME,    // đặt home thủ công tại vị trí hiện tại (axis)
        RELEASE_J1_J4,
        ENABLE_J1_J4,
        CLEAR_FAULT, // thoát trạng thái FAULT
        MOVE_CART,   // p[0..2]=x,y,z — di chuyển TCP tới điểm (bút xuống)
        DRAW_LINE,   // p[0..4]=x1,y1,x2,y2,z ; p[5]=feed
        DRAW_CIRCLE, // p[0..2]=cx,cy,z ; p[3]=r ; p[5]=feed
        DRAW_SQUARE, // p[0..2]=cx,cy,z ; p[3]=side ; p[5]=feed
        SHOW_OFF,    // trình diễn đồng bộ 6 khớp múa nhẹ nhàng trong khoảng an toàn
        SAVE_TEACH_POINT,
        PLAY_TEACH_POINTS,
        CLEAR_TEACH_POINTS,
        SET_GRIPPER // value: commanded servo angle, independent of J1..J6/Teach
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
               JointModel* joints, HomingController* homing, Planner* planner, NvsStore* nvs);

    // Đưa lệnh vào hàng đợi. Trả false nếu đầy/đang bận với lệnh không thể trộn.
    [[nodiscard]] bool submit(const ArmCommand& cmd, uint32_t timeoutMs = 10);
    [[nodiscard]] bool busy() const;
    [[nodiscard]] bool gripperAvailable() const;

    [[nodiscard]] ArmMode mode() const;
    [[nodiscard]] SafetyManager* safety() noexcept { return safety_.get(); }
    [[nodiscard]] const String& lastPlannerError() const noexcept { return lastPlannerError_; }
    [[nodiscard]] int lastPlannerFailIndex() const noexcept { return lastPlannerFailIndex_; }
    String statusJson();

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
    void updateTeachPlayback();
    bool moveToTeachPoint(uint8_t slot);
    void refreshTeachPoints();

    Motor* motors[NUM_MOTORS]{};
    Sensor* sensor{nullptr};
    Endstops* es{nullptr};
    JointModel* jm{nullptr};
    HomingController* hc{nullptr};
    Planner* pl{nullptr};
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
    String lastPlannerError_{"OK"};
    int lastPlannerFailIndex_{-1};

    // Trạng thái Show Off
    bool showOffActive_{false};
    uint8_t showOffStep_{0};
    float showOffBaseAngles_[NUM_MOTORS]{};
    static constexpr uint8_t TEACH_POINT_COUNT = 3;
    NvsStore::TeachPoint teachPoints_[TEACH_POINT_COUNT]{};
    std::atomic<uint8_t> teachValidMask_{0};
    enum class TeachError : uint8_t { NONE, NVS, HOME, FRAME, INVALID };
    std::atomic<TeachError> teachError_{TeachError::NONE};
    bool teachPlaybackActive_{false};
    uint8_t teachPlaybackSlot_{0};
};

#endif // ARM_H
