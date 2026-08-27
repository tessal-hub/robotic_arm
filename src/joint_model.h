#ifndef JOINT_MODEL_H
#define JOINT_MODEL_H

#include <Arduino.h>
#include <cmath>
#include "config.h"

class Motor;
class Sensor;
class NvsStore;

/**
 * Mô hình vị trí khớp tuyệt đối (đơn vị: độ, gốc tại HOME đã calib).
 * - Nguồn chính: bộ đếm step tuyệt đối trong Motor (motor.getAbsoluteSteps()).
 * - Nguồn tham chiếu: encoder AS5600 (zeroRef chụp lúc set home) để phát hiện drift.
 * - Quy đổi: stepsPerDegree = FULL_STEPS * microstep * gearRatio / 360.
 *   Khi đã home bằng kiến trúc quét Min/Max, bước/độ và dấu encoder được ĐO thực tế
 *   (s_encSign / s_measuredSpd) và áp dụng thay hằng số cố định.
 * - Bền vững qua nguồn: raw góc encoder lúc home được lưu NVS; khi bật nguồn,
 *   restoreFromNVS() đối chiếu raw hiện tại vs đã lưu -> khôi phục vị trí.
 */
class JointModel {
public:
    JointModel();

    JointModel(const JointModel&) = delete;
    JointModel& operator=(const JointModel&) = delete;

    void begin(Motor** motors, Sensor* sensor);
    void attachNvs(NvsStore* nvs);

    [[nodiscard]] static float stepsPerDegree(uint8_t axis);
    [[nodiscard]] static int64_t degreesToSteps(uint8_t axis, float deg);
    [[nodiscard]] static float stepsToDegrees(uint8_t axis, int64_t steps);
    [[nodiscard]] static float wrap180(float deg);

    // Chiều quay logic (cw=true => absSteps tăng) để góc thay đổi theo deltaDeg.
    [[nodiscard]] static bool cwForDelta(uint8_t axis, float deltaDeg);

    // Góc khớp hiện tại theo bước máy (độ so với home).
    [[nodiscard]] float angleFromSteps(uint8_t axis) const;
    // Góc khớp theo encoder (chỉ có nghĩa sau khi đã set home).
    [[nodiscard]] float angleFromEncoder(uint8_t axis);
    // Góc thô tuyệt đối từ encoder (raw accumulated, không cần home) — homing dùng.
    [[nodiscard]] float rawEncoder(uint8_t axis);

    // Đặt vị trí HIỆN TẠI làm mốc home (góc = 0) + lưu NVS nếu encoder khoẻ.
    void setHomeHere(uint8_t axis);
    // Đồng bộ step counter với encoder — dùng khi cancel/STOP để tránh drift.
    void resyncFromEncoder(uint8_t axis);
    void clearHome(uint8_t axis);
    void forgetHome(uint8_t axis);  // xoá cả NVS (nút CLEAR CALIB trên web)

    // Khôi phục vị trí từ NVS + encoder sau boot. Trả về số khớp khôi phục OK.
    uint8_t restoreFromNVS();

    [[nodiscard]] bool isHomed(uint8_t axis) const noexcept { return axis < NUM_MOTORS && homed[axis]; }
    [[nodiscard]] bool wasRestored(uint8_t axis) const noexcept { return axis < NUM_MOTORS && restored[axis]; }
    [[nodiscard]] uint8_t homedCount() const noexcept;
    [[nodiscard]] bool allPositioningHomed() const noexcept; // J1..J4

    // So sánh step-count vs encoder. Trả về true NẾU phát hiện lệch quá ngưỡng (latch fault).
    bool updateDriftCheck(uint8_t axis);
    [[nodiscard]] bool hasDriftFault(uint8_t axis) const noexcept { return axis < NUM_MOTORS && driftFault[axis]; }
    void clearDriftFault(uint8_t axis) { if (axis < NUM_MOTORS) driftFault[axis] = false; }

    [[nodiscard]] bool encOK(uint8_t axis) const;

    // Đo và áp dụng hiệu chuẩn động (gọi từ homing sau cross-check):
    // encSign = dấu encoder (±1) thay AXIS_ENC_SIGN, stepsPerDeg = bước/độ đo được.
    void applyHomingCalibration(uint8_t axis, float encSign, float stepsPerDeg);

    String toJson();

private:
    Motor* motors[NUM_MOTORS]{};
    Sensor* sensor{nullptr};
    NvsStore* nvs{nullptr};
    float encZeroRef[NUM_MOTORS]{};
    bool homed[NUM_MOTORS]{};
    bool restored[NUM_MOTORS]{};
    bool driftFault[NUM_MOTORS]{};

    // Hiệu chuẩn đo được (thay hằng số cố định khi đã home). Static: 1 instance duy nhất.
    static float s_encSign[NUM_MOTORS];      // dấu encoder (±1)
    static float s_measuredSpd[NUM_MOTORS];  // bước/độ đo được
    static bool  s_hasMeasured[NUM_MOTORS];
};

#endif // JOINT_MODEL_H
