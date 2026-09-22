// Compile with -DARDUINO and -I test/host/firmware: only peripheral/RTOS APIs
// are fakes. Arm, Motor, Sensor, JointModel, NVS and SafetyManager are production.
#include <Arduino.h>
#include <cassert>
#include <limits>
#define private public
#include "arm.h"
#include "motor.h"
#include "sensor.h"
#include "joint_model.h"
#include "homing.h"
#include "planner.h"
#include "wifi_manager.h"
#undef private

String WifiManager::toJson() const { return "{}"; } // Unused network boundary.

struct Rig {
    Motor motors[NUM_MOTORS] = {
        {&Serial1, R_SENSE, 0, 1}, {&Serial1, R_SENSE, 1, 2},
        {&Serial1, R_SENSE, 2, 3}, {&Serial1, R_SENSE, 3, 4},
        {nullptr, R_SENSE, 0, 38, 39}, {nullptr, R_SENSE, 0, 40, 47}
    };
    Motor* pointers[NUM_MOTORS]{};
    SemaphoreHandle_t mutex = reinterpret_cast<void*>(1);
    Sensor sensor;
    NvsStore nvs;
    JointModel joints;
    Endstops endstops;
    HomingController homing;
    Planner planner;
    ArmController arm;
    explicit Rig(bool restore = false) {
        nvs.begin();
        sensor.begin();
        for (uint8_t a = 0; a < NUM_MOTORS; ++a) {
            pointers[a] = &motors[a]; motors[a].setUartMutex(&mutex); motors[a].begin();
        }
        joints.begin(pointers, &sensor); joints.attachNvs(&nvs);
        if (restore) assert(joints.restoreFromNVS() == NUM_MOTORS);
        else for (uint8_t a = 0; a < NUM_MOTORS; ++a) joints.setHomeHere(a);
        endstops.begin(pointers);
        homing.begin(pointers, &endstops, &joints);
        planner.begin(pointers, &joints);
        arm.begin(pointers, &sensor, &endstops, &joints, &homing, &planner, &nvs);
    }
    ~Rig() { delete arm.queue; }
    void command(ArmCommand::Type type, uint8_t axis = 0) {
        ArmCommand c; c.type = type; c.axis = axis; arm.execute(c);
    }
    void save(uint8_t slot) {
        command(ArmCommand::SAVE_TEACH_POINT, slot);
        assert(nvs.loadTeachPoint(slot).valid);
        assert(arm.teachValidMask_.load() & (1U << slot));
    }
};

static void direction_failure_stops_all_start_paths() {
    Rig r;
    auto& m = r.motors[0]; auto* driver = m.getDriver();
    assert(m.setDirection(true));
    driver->writeOK = false;
    assert(!m.setDirection(false));
    assert(m.getDirCW()); // Failed write cannot update the logical direction.
    m.run(false, 100); assert(!m.isRunning());
    m.runContinuous(false); assert(!m.isRunning());
    assert(!m.prepareCoordinatedRun(false, 100, 1000));
    driver->writeOK = true;
    assert(m.setDirection(false));
    driver->readOK = false;
    assert(!m.setDirection(false)); // False-valued timeout is not valid CCW.
    driver->readOK = true;
    driver->failReadAt = driver->reads + 2;
    assert(!m.setDirection(true)); // Read-back failure after a successful write.
    driver->failReadAt = 0;
    assert(m.setDirection(true));
    driver->direction = false; // Driver reset must not be hidden by a direction cache.
    assert(m.setDirection(true)); assert(driver->direction);
    fakeMutexOK = false;
    assert(!m.setDirection(true));
    fakeMutexOK = true;
}

static void clear_fault_and_resync_require_stopped_motors() {
    Rig r;
    auto& m = r.motors[0];
    m.setAbsoluteSteps(1234); m.run(true, 500);
    r.command(ArmCommand::CLEAR_FAULT);
    assert(m.isRunning() && m.getAbsoluteSteps() == 1234);
    assert(!r.joints.resyncFromEncoder(0));
    r.joints.clearAllDriftFaults();
    assert(m.getAbsoluteSteps() == 1234);
    r.joints.setHomeHere(0); assert(m.getAbsoluteSteps() == 1234);
    r.command(ArmCommand::STOP_ALL);
    r.arm.safety()->assertEStop("test"); r.arm.mode_ = ArmMode::FAULT;
    r.command(ArmCommand::CLEAR_FAULT);
    assert(r.arm.mode() == ArmMode::IDLE && !r.arm.safety()->isEStop());
    assert(m.getAbsoluteSteps() == 0);
}

static void sensor_health_requires_a_published_sample_and_task() {
    Sensor sensor;
    for (uint8_t a = 0; a < NUM_SENSORS; ++a) assert(!sensor.isSensorOK(a));
    Wire.ok = false; sensor.begin();
    assert(!sensor.isSensorOK(0));
    Wire.ok = true;
    sensor.dataMutex = reinterpret_cast<void*>(2); fakeBlockedMutex = sensor.dataMutex;
    sensor.scanOnce(); assert(!sensor.isSensorOK(0));
    fakeBlockedMutex = nullptr;
    sensor.scanOnce(); assert(sensor.isSensorOK(0));
    assert(sensor.getAccumulatedAngle(0) == 90.0f);
    fakeTaskOK = false;
    Sensor noTask; noTask.begin(); assert(!noTask.isSensorOK(0));
    fakeTaskOK = true;
    fakeMutexOK = false;
    Sensor noMutex; noMutex.begin(); assert(!noMutex.isSensorOK(0));
    fakeMutexOK = true;
}

static void teach_frame_is_bound_to_home_across_reboot() {
    Preferences::storage.clear();
    {
        Rig r; r.save(0);
    }
    {
        Rig restored(true);
        assert(restored.arm.teachValidMask_.load() == 1);
        restored.sensor.accumulated_angles[0] += 15.0f;
        restored.sensor.filtered_angles[0] += 15.0f;
        restored.joints.setHomeHere(0);
        assert(!restored.arm.moveToTeachPoint(0));
        assert(restored.arm.teachValidMask_.load() == 0);
        restored.command(ArmCommand::RELEASE_J1_J4);
        restored.command(ArmCommand::PLAY_TEACH_POINTS);
        assert(restored.arm.teachError_.load() == ArmController::TeachError::FRAME);
        assert(restored.arm.manualRelease_.load());
        for (uint8_t a = 0; a < 4; ++a) assert(!restored.motors[a].enabled.load());
        for (auto& m : restored.motors) assert(!m.isRunning());
    }
    {
        Rig reboot(true); assert(reboot.arm.teachValidMask_.load() == 0);
        reboot.save(0); // New record explicitly acknowledges the new home.
        JointModel::s_encSign[0] *= -1;
        assert(!reboot.arm.moveToTeachPoint(0));
    }
    // Legacy points lack a home signature and must never be migrated by guessing.
    Preferences::storage.clear(); Preferences p;
    float oldAngles[NUM_MOTORS]{};
    p.putBool("tp0_valid", true); p.putBytes("tp0_deg", oldAngles, sizeof(oldAngles));
    NvsStore legacy; legacy.begin(); assert(!legacy.loadTeachPoint(0).valid);
}

static void teach_nvs_failures_remain_visible_and_reboot_consistent() {
    Preferences::storage.clear(); Rig r;
    r.save(0); r.save(1); r.save(2);
    Preferences::failKey = "tp1_valid";
    r.command(ArmCommand::CLEAR_TEACH_POINTS);
    assert(r.arm.teachError_.load() == ArmController::TeachError::NVS);
    assert(r.arm.teachValidMask_.load() == 2);
    NvsStore reboot; reboot.begin();
    assert(!reboot.loadTeachPoint(0).valid && reboot.loadTeachPoint(1).valid && !reboot.loadTeachPoint(2).valid);
    Preferences::dropWrite = true; // Even a reported success must be read back.
    assert(!r.nvs.clearTeachPoints());
    Preferences::failKey.clear(); Preferences::dropWrite = false;
    r.command(ArmCommand::CLEAR_TEACH_POINTS);
    assert(r.arm.teachValidMask_.load() == 0);
    assert(r.arm.teachError_.load() == ArmController::TeachError::NONE);
    r.save(0);
    Preferences::failKey = "tp0_pose";
    r.command(ArmCommand::SAVE_TEACH_POINT);
    assert(r.arm.teachError_.load() == ArmController::TeachError::NVS);
    assert(r.arm.teachValidMask_.load() == 0 && !reboot.loadTeachPoint(0).valid);
    Preferences::failKey.clear();
}

static void teach_ramps_and_drawing_keeps_constant_rate() {
    Rig r; r.save(0);
    r.arm.teachPoints_[0].axes[0].deg = 40;
    assert(r.arm.moveToTeachPoint(0));
    auto& m = r.motors[0];
    const uint32_t start = m.getCurrentInterval(), cruise = m.getStepInterval();
    assert(start > cruise && m.accelSteps > 0 && m.decelSteps > 0);
    uint32_t minimum = start, final = 0;
    while (m.isRunning()) {
        Motor::onStepTimer(&m);
        if (m.isRunning()) { final = m.getCurrentInterval(); minimum = std::min(minimum, final); }
    }
    assert(minimum == cruise && final > cruise);
    assert(m.getStepCounter() == m.getTargetSteps());
    assert(m.prepareCoordinatedRun(true, 100, 800));
    assert(m.startPreparedRun());
    while (m.isRunning()) { Motor::onStepTimer(&m); assert(m.getCurrentInterval() == 800); }
    // Very slow axes must keep their scaled interval, even below one step/sec.
    assert(m.prepareCoordinatedRun(true, 20, 2000000, 8000000));
    assert(m.startPreparedRun()); Motor::onStepTimer(&m);
    assert(m.getCurrentInterval() > 2000000);
    r.command(ArmCommand::STOP_ALL);
    r.arm.teachPlaybackActive_ = true; r.arm.teachPlaybackSlot_ = 3;
    r.arm.updateTeachPlayback();
    assert(!r.arm.teachPlaybackActive_ && r.arm.teachError_.load() == ArmController::TeachError::NONE);
}

static void teach_rejects_bad_axes_before_start_and_stop_cancels_sequence() {
    Preferences::storage.clear();
    Rig r;
    r.command(ArmCommand::RELEASE_J1_J4);
    r.command(ArmCommand::PLAY_TEACH_POINTS);
    assert(r.arm.manualRelease_.load());
    assert(r.arm.teachError_.load() == ArmController::TeachError::INVALID);
    for (uint8_t a = 0; a < 4; ++a) assert(!r.motors[a].enabled.load());
    r.save(0); r.save(1);
    r.arm.teachPoints_[0].axes[0].deg = 20;
    r.arm.teachPoints_[0].axes[1].deg = 20;
    r.sensor.sensor_error[1].store(true);
    assert(!r.arm.moveToTeachPoint(0));
    for (auto& m : r.motors) assert(!m.isRunning());
    r.sensor.sensor_error[1].store(false);
    auto* driver = r.motors[1].getDriver();
    driver->readOK = false;
    assert(!r.arm.moveToTeachPoint(0));
    for (auto& m : r.motors) assert(!m.isRunning() && !m.preparedRun.load());
    driver->readOK = true;
    r.command(ArmCommand::PLAY_TEACH_POINTS);
    assert(!r.arm.manualRelease_.load());
    assert(r.arm.teachPlaybackActive_ && r.motors[0].isRunning());
    r.command(ArmCommand::STOP_ALL);
    r.arm.updateTeachPlayback();
    for (auto& m : r.motors) assert(!m.isRunning());
    assert(!r.arm.teachPlaybackActive_);
}

static void gripper_queue_limits_and_stop() {
    Rig r;
    assert(fakeLedcPin == GRIPPER_SERVO_PIN && fakeLedcHz == 50 && fakeLedcBits == 14);
    assert(fakeLedcDuty == 0 && !r.arm.gripperActive_.load());
    ArmCommand c; c.type = ArmCommand::SET_GRIPPER;
    for (float angle : {0.0f, 90.0f, 180.0f}) {
        c.value = angle;
        assert(r.arm.submit(c));
        ArmCommand queued;
        assert(xQueueReceive(r.arm.queue, &queued, 0) == pdTRUE);
        r.arm.execute(queued);
        const uint32_t expected = static_cast<uint32_t>(lroundf((1000.0f + angle / 180.0f * 1000.0f) * 50 * 16384 / 1000000));
        assert(fakeLedcDuty == expected && r.arm.gripperTargetDeg_.load() == angle);
    }
    const auto duty = fakeLedcDuty;
    for (float bad : {-1.0f, 181.0f, NAN, INFINITY}) {
        c.value = bad; assert(!r.arm.submit(c)); r.arm.execute(c);
        assert(fakeLedcDuty == duty);
    }
    c.value = 90;
    assert(r.arm.submit(c));
    r.command(ArmCommand::STOP_ALL);
    assert(fakeLedcDuty == 0 && !r.arm.gripperActive_.load());
    assert(uxQueueMessagesWaiting(r.arm.queue) == 0);
    r.arm.safety()->notifyFault("test");
    assert(!r.arm.submit(c)); r.arm.execute(c); assert(fakeLedcDuty == 0);
    r.command(ArmCommand::CLEAR_FAULT);
    r.motors[0].run(true, 100);
    assert(!r.arm.submit(c)); r.arm.execute(c); assert(fakeLedcDuty == 0);
    r.command(ArmCommand::STOP_ALL);
    r.arm.execute(c); assert(fakeLedcDuty != 0);
    r.command(ArmCommand::RELEASE_J1_J4);
    assert(fakeLedcDuty == 0 && !r.arm.submit(c));
    r.arm.execute(c); assert(fakeLedcDuty == 0 && r.arm.manualRelease_.load());
    fakeLedcOK = false;
    { Rig failed; assert(!failed.arm.gripperAvailable()); assert(!failed.arm.submit(c)); }
    fakeLedcOK = true;
}

int main() {
    gripper_queue_limits_and_stop();
    direction_failure_stops_all_start_paths();
    clear_fault_and_resync_require_stopped_motors();
    sensor_health_requires_a_published_sample_and_task();
    teach_frame_is_bound_to_home_across_reboot();
    teach_nvs_failures_remain_visible_and_reboot_consistent();
    teach_ramps_and_drawing_keeps_constant_rate();
    teach_rejects_bad_axes_before_start_and_stop_cancels_sequence();
    std::puts("ALL PASSED (production firmware failure regressions)");
}
