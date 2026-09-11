#include "homing_fsm_mocks.h"
#include <cassert>
#include <limits>
#define private public
#include "homing.h"
#undef private
#define HOMING_HOST_TEST
#include "../../src/homing.cpp"

struct Rig {
    Motor motor;
    Motor* motors[NUM_MOTORS]{};
    JointModel joints;
    Endstops endstops;
    HomingController home;
    float position = 0, encoderSign = 1;
    bool jam = false, badSg = false, missingStop = false;
    int forcedSg = -1;
    Rig() {
        fakeNow = 1000;
        for (uint8_t a = 0; a < NUM_MOTORS; ++a) for (uint8_t w = 0; w < 2; ++w) {
            fakeHasEndstop[a][w] = false;
            fakeEndstopPressed[a][w] = false;
            fakeEndstopEnabled[a][w] = true;
        }
        motors[3] = &motor;
        home.begin(motors, &endstops, &joints);
    }
    void tick() {
        fakeNow += 10;
        if (motor.running) {
            const unsigned count = std::max(1U, 10000U / motor.speed);
            for (unsigned i = 0; i < count && motor.running; ++i) {
                motor.steps += motor.cw ? 1 : -1;
                if (!jam) {
                    position += (motor.cw ? -1.0f : 1.0f) / JointModel::stepsPerDegree(3);
                    if (!missingStop) position = std::max(-20.0f, std::min(20.0f, position));
                }
                if (!motor.continuous && --motor.remaining == 0) motor.running = false;
            }
        }
        joints.raw = position * encoderSign;
        motor.sg = badSg ? 1023 : (!missingStop && std::fabs(position) >= 19.999f ? 0 : 200);
        if (forcedSg >= 0) motor.sg = static_cast<uint16_t>(forcedSg);
        home.tick();
    }
    void run() { for (int i = 0; i < 30000 && home.isActive(); ++i) tick(); assert(!home.isActive()); }
};

int main() {
    { // Contact-release bounce must not let the ISR stop the initial backoff.
        Rig r;
        r.motors[0] = &r.motor;
        r.home.motors[0] = &r.motor;
        fakeHasEndstop[0][0] = true;
        fakeEndstopPressed[0][0] = true;
        r.home.curAxis_ = 0;
        r.home.active_ = true;
        r.home.approachSide_ = EndstopWhich::MIN;
        r.home.cwApproach_ = false;
        r.home.enterScanBackoff();
        assert(!fakeEndstopEnabled[0][0]);
        fakeEndstopPressed[0][0] = false;
        fakeNow += HOMING_BACKOFF_SETTLE_MS + 1;
        r.home.tick();
        r.motor.running = false;
        r.home.tick();
        assert(fakeEndstopEnabled[0][0] && r.home.phase() == HomePhase::SCAN_SLOW);
    }
    { // The second switch stays masked while centering away, then is re-armed.
        Rig r;
        r.home.motors[0] = &r.motor;
        fakeHasEndstop[0][1] = true;
        fakeEndstopPressed[0][1] = true;
        r.home.curAxis_ = 0;
        r.home.active_ = true;
        r.home.approachSide_ = EndstopWhich::MAX;
        r.home.cwApproach_ = true;
        r.home.contactSpan_ = 1000;
        r.home.encFirstRaw_ = -10.0f;
        r.home.encSecondRaw_ = 10.0f;
        r.home.enterCenteringScan();
        assert(!fakeEndstopEnabled[0][1] && r.home.phase() == HomePhase::CENTERING);
        fakeEndstopPressed[0][1] = false;
        r.motor.steps = r.home.centeringTargetSteps_;
        r.motor.running = false;
        r.home.tick();
        assert(fakeEndstopEnabled[0][1] && r.home.phase() == HomePhase::VERIFY_SETTLE_WAIT);
    }
    { // No movement when UART is unavailable; a later request rechecks the bus.
        Rig r; r.motor.uartAvailable = false;
        assert(r.home.startAxis(3));
        assert(!r.home.isActive() && !r.motor.running && r.motor.runCalls == 0);
        assert(r.motor.uartChecks == HOMING_MAX_ATTEMPTS && r.joints.homes == 0);
        r.motor.uartAvailable = true;
        assert(r.home.startAxis(3)); r.run();
        assert(r.home.lastRunOK() && r.joints.homes == 1);
    }
    for (float sign : {-1.0f, 1.0f}) {
        Rig r; r.encoderSign = sign;
        assert(r.home.startAxis(3)); r.run();
        assert(r.home.lastRunOK());
        assert(r.joints.homes == 1 && r.joints.calibrations == 1);
        assert(std::fabs(r.position) < 1.0f);
    }
    { // Start at the stop that blocks initial warmup: one reverse probe escapes.
        Rig r; r.position = 20; r.joints.raw = 20;
        assert(r.home.startAxis(3)); r.run(); assert(r.home.lastRunOK());
    }
    { Rig r; r.jam = true; assert(r.home.startAxis(3)); r.run();
      assert(!r.home.lastRunOK() && r.joints.homes == 0 && r.joints.calibrations == 0); }
    { Rig r; r.badSg = true; assert(r.home.startAxis(3)); r.run();
      assert(!r.home.lastRunOK() && r.joints.homes == 0); }
    { Rig r; r.forcedSg = 0; assert(r.home.startAxis(3)); r.run();
      assert(r.home.lastRunOK() && std::fabs(r.position) < 1.0f); }
    { Rig r; r.forcedSg = 200; assert(r.home.startAxis(3)); r.run();
      assert(!r.home.lastRunOK() && r.joints.homes == 0); }
    { Rig r; r.missingStop = true; assert(r.home.startAxis(3)); r.run();
      assert(!r.home.lastRunOK() && r.joints.homes == 0); }
    { // Contact is found, but the commanded backoff cannot move the rotor.
        Rig r; assert(r.home.startAxis(3));
        while (r.home.phase() != HomePhase::BACKOFF_SETTLE_WAIT && r.home.isActive()) r.tick();
        assert(r.home.isActive()); r.jam = true;
        r.home.attempt_ = HOMING_MAX_ATTEMPTS - 1; r.run();
        assert(!r.home.lastRunOK() && r.joints.homes == 0 && r.joints.calibrations == 0);
    }
    { // Second stop is absent even though first contact succeeded.
        Rig r; assert(r.home.startAxis(3));
        while (r.home.phase() != HomePhase::SCAN_MAX && r.home.isActive()) r.tick();
        assert(r.home.isActive()); r.missingStop = true;
        r.home.attempt_ = HOMING_MAX_ATTEMPTS - 1; r.run();
        assert(!r.home.lastRunOK() && r.joints.homes == 0);
    }
    { Rig r; assert(r.home.startAxis(3));
      while (r.home.phase() != HomePhase::SCAN_SLOW && r.home.isActive()) r.tick();
      assert(r.home.isActive()); r.missingStop = true;
      r.home.attempt_ = HOMING_MAX_ATTEMPTS - 1; r.run();
      assert(!r.home.lastRunOK() && r.joints.homes == 0); }
    { // Compensation handles decreasing step counters; confirmations require distinct windows.
        Rig r; r.home.curAxis_ = 3; r.home.encDirMult_ = 1;
        r.motor.steps = 100; r.home.resetStallWindow(r.motor);
        float delta = 0;
        r.motor.steps = -20;
        assert(!r.home.stallWindowCheck(3, r.motor, delta));
        assert(!r.home.stallWindowCheck(3, r.motor, delta));
        r.motor.steps = -140;
        assert(r.home.stallWindowCheck(3, r.motor, delta));
        assert(r.home.compensatedContactStep(r.motor, 1.0f) == 64);
        r.motor.steps = 340;
        assert(r.home.compensatedContactStep(r.motor, 1.0f) == 136);
    }
    { Rig r; assert(r.home.startAxis(3)); r.joints.healthy = false; r.home.tick();
      assert(!r.motor.running && r.joints.homes == 0); }
    { Rig r; assert(r.home.startAxis(3)); r.joints.raw = std::numeric_limits<float>::quiet_NaN(); r.home.tick();
      assert(!r.motor.running && r.joints.homes == 0); }
    { // Encoder jump beyond the commissioned 40-degree envelope is invalid.
        Rig r; assert(r.home.startAxis(3));
        r.home.attempt_ = HOMING_MAX_ATTEMPTS - 1;
        r.motor.stop(); r.home.phase_ = HomePhase::WARMUP_SETTLE_WAIT;
        r.home.settleStartMs_ = fakeNow - WARMUP_ENC_SETTLE_MS;
        r.joints.raw = 42.26f; r.home.tick();
        assert(!r.home.lastRunOK() && !r.motor.running && r.joints.calibrations == 0 && r.joints.homes == 0);
    }
    { // Real VERIFY must not save calibration until the measured center is reached.
        Rig r; assert(r.home.startAxis(3));
        r.home.attempt_ = HOMING_MAX_ATTEMPTS - 1;
        r.motor.stop(); r.home.encCenterRaw_ = 0; r.home.encFirstRaw_ = -20; r.home.encSecondRaw_ = 20;
        r.home.encDirMult_ = 1; r.joints.raw = 8; r.home.enterVerify();
        assert(r.home.phase() == HomePhase::VERIFY && r.joints.homes == 0 && r.joints.calibrations == 0);
        r.home.tick(); assert(r.motor.running);
        r.motor.steps += JointModel::degreesToSteps(3, HOMING_TRIM_MAX_TRAVEL_DEG + 1);
        r.home.tick(); assert(!r.home.lastRunOK() && !r.motor.running && r.joints.homes == 0);
    }
    { Rig r; assert(r.home.startAxis(3)); r.home.cancel();
      assert(!r.motor.running && !r.home.isActive() && r.joints.homes == 0); }
    std::puts("ALL PASSED (production J4 homing FSM)");
}
