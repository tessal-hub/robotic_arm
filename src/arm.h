#ifndef ARM_H
#define ARM_H

#include <Arduino.h>
#include <atomic>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "config.h"

class Motor;
class Sensor;
class Endstops;
class JointModel;
class HomingController;
class Planner;

enum class ArmMode : uint8_t { IDLE = 0, HOMING, JOG, CART, DRAW, FAULT };

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
        CLEAR_FAULT, // thoát trạng thái FAULT
        MOVE_CART,   // p[0..2]=x,y,z — di chuyển TCP tới điểm (bút xuống)
        DRAW_LINE,   // p[0..4]=x1,y1,x2,y2,z ; p[5]=feed
        DRAW_CIRCLE  // p[0..2]=cx,cy,z ; p[3]=r ; p[5]=feed
    };
    Type type{Type::NONE};
    uint8_t axis{0};
    float value{0.0f};
    float p[8]{0, 0, 0, 0, 0, 0, 0, 0};
};

/**
 * Bộ điều phối trung tâm: nhận lệnh từ Web/Serial, tuần tự hoá và thực thi trong
 * motion task (Core 1 @100Hz). Web handler chỉ enqueue — không đụng phần cứng.
 */
class ArmController {
public:
    ArmController();
    ~ArmController() = default;

    ArmController(const ArmController&) = delete;
    ArmController& operator=(const ArmController&) = delete;

    void begin(Motor** motors, Sensor* sensor, Endstops* endstops,
               JointModel* joints, HomingController* homing, Planner* planner);

    // Đưa lệnh vào hàng đợi. Trả false nếu đầy/đang bận với lệnh không thể trộn.
    [[nodiscard]] bool submit(const ArmCommand& cmd, uint32_t timeoutMs = 10);
    [[nodiscard]] bool busy() const;

    [[nodiscard]] ArmMode mode() const;
    String statusJson();

private:
    static void taskEntry(void* param);
    void taskLoop();
    void execute(const ArmCommand& cmd);
    void applyJog(uint8_t axis, float deltaDeg);
    [[nodiscard]] bool motionAllowed() const;

    Motor* motors[NUM_MOTORS]{};
    Sensor* sensor{nullptr};
    Endstops* es{nullptr};
    JointModel* jm{nullptr};
    HomingController* hc{nullptr};
    Planner* pl{nullptr};

    QueueHandle_t queue{nullptr};
    TaskHandle_t task{nullptr};
    std::atomic<ArmMode> mode_{ArmMode::IDLE};
    uint32_t driftTickCounter{0};
};

#endif // ARM_H
