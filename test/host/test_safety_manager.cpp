// Host tests for SafetyManager — 8 cases as per Sprint 0 Task 1 spec
#include "safety_manager.h"
#include "test_mocks.h"
#include <cstdio>

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { \
    std::printf("FAIL: %s (line %d)\n", msg, __LINE__); \
    ++g_fail; \
  } \
} while(0)

#define PASS(msg) do { std::printf("PASS: %s\n", msg); ++g_pass; } while(0)

static void test_pending_high_no_latch() {
  MockEndstops es; es.setGpio(0, EndstopWhich::MIN, true); // HIGH = not pressed
  MockJointModel jm;
  SafetyManager sm(&es, &jm);
  sm.isrNotify(0, EndstopWhich::MIN, 1000000);
  sm.pollEndstops(1000000 + 10000); // 10ms < 50ms debounce + GPIO HIGH
  bool ok = (sm.state() == SafetyState::NORMAL) && (!sm.anyLatched());
  // also poll after debounce with HIGH should still not latch
  sm.pollEndstops(1000000 + 60000);
  ok = ok && (sm.state() == SafetyState::NORMAL) && (!sm.anyLatched());
  if (ok) PASS("pending_high_no_latch");
  else { CHECK(false, "pending_high_no_latch"); }
}

static void test_low_before_50ms_no_latch() {
  MockEndstops es; es.setGpio(0, EndstopWhich::MIN, false); // LOW = pressed
  MockJointModel jm;
  SafetyManager sm(&es, &jm);
  sm.isrNotify(0, EndstopWhich::MIN, 2000000);
  sm.pollEndstops(2000000 + 40000); // 40ms < debounce
  bool ok = (sm.state() == SafetyState::NORMAL) && (!sm.anyLatched());
  if (ok) PASS("low_before_50ms_no_latch");
  else CHECK(false, "low_before_50ms_no_latch should not latch yet");
}

static void test_low_after_50ms_latches_estop() {
  MockEndstops es; es.setGpio(0, EndstopWhich::MIN, false);
  MockJointModel jm;
  SafetyManager sm(&es, &jm);
  sm.isrNotify(0, EndstopWhich::MIN, 2000000);
  sm.pollEndstops(2000000 + 60000); // 60ms > debounce
  bool ok = (sm.state() == SafetyState::E_STOP) && sm.isEStop() && sm.anyLatched() && !sm.isMotionAllowed();
  if (ok) PASS("low_after_50ms_latches_estop");
  else CHECK(false, "low_after_50ms_latches_estop");
}

static void test_homing_no_estop() {
  MockEndstops es; es.setGpio(1, EndstopWhich::MAX, false);
  MockJointModel jm; SafetyManager sm(&es,&jm);
  sm.assertHoming(true);
  CHECK(sm.state() == SafetyState::HOMING, "homing state after assert");
  CHECK(sm.isMotionAllowed(), "homing isMotionAllowed true");
  sm.isrNotify(1, EndstopWhich::MAX, 3000000);
  sm.pollEndstops(3000000+60000);
  bool ok = (sm.state() == SafetyState::HOMING) && (!sm.isEStop()) && sm.anyLatched() && sm.isMotionAllowed();
  if (ok) PASS("homing_no_estop");
  else CHECK(false, "homing_no_estop: should stay HOMING not E_STOP");
}

static void test_tryClear_reject_when_pressed() {
  MockEndstops es; es.setGpio(0, EndstopWhich::MIN, false); // still pressed (LOW)
  MockJointModel jm; SafetyManager sm(&es,&jm);
  sm.assertEStop("test");
  CHECK(sm.state()==SafetyState::E_STOP, "pre E_STOP");
  bool cleared = sm.tryClearFault();
  bool ok = (!cleared) && (sm.state()==SafetyState::E_STOP);
  if (ok) PASS("tryClear_reject_when_pressed");
  else CHECK(false, "tryClear_reject_when_pressed");
}

static void test_tryClear_reject_when_drift() {
  MockEndstops es; es.setGpio(0, EndstopWhich::MIN, true); // not pressed
  MockJointModel jm; jm.setDriftFault(true);
  SafetyManager sm(&es,&jm);
  sm.assertEStop("drift test");
  bool cleared = sm.tryClearFault();
  bool ok = (!cleared) && (sm.state()==SafetyState::E_STOP) && jm.hasAnyDriftFault();
  if (ok) PASS("tryClear_reject_when_drift");
  else CHECK(false, "tryClear_reject_when_drift");
}

static void test_tryClear_success_clears() {
  MockEndstops es; es.setGpio(0, EndstopWhich::MIN, false); // pressed initially
  MockJointModel jm; SafetyManager sm(&es,&jm);
  sm.isrNotify(0, EndstopWhich::MIN, 4000000);
  sm.pollEndstops(4000000+60000);
  CHECK(sm.state()==SafetyState::E_STOP, "should be E_STOP before clear");
  CHECK(sm.anyLatched(), "latched before clear");
  // now release switch
  es.setGpio(0, EndstopWhich::MIN, true); // HIGH = released
  bool cleared = sm.tryClearFault();
  bool ok = cleared && (sm.state()==SafetyState::NORMAL) && (!sm.anyLatched()) && sm.isMotionAllowed() && !sm.isEStop();
  // also ensure mock's latches cleared (via SafetyManager clearLatches call)
  ok = ok && (!es.anyLatched());
  if (ok) PASS("tryClear_success_clears");
  else CHECK(false, "tryClear_success_clears");
}

static void test_isMotionAllowed_matrix() {
  // NORMAL -> allowed
  {
    MockEndstops es; MockJointModel jm; SafetyManager sm(&es,&jm);
    CHECK(sm.isMotionAllowed(), "NORMAL allowed");
    CHECK(!sm.isEStop(), "NORMAL not estop");
  }
  // HOMING -> allowed
  {
    MockEndstops es; MockJointModel jm; SafetyManager sm(&es,&jm);
    sm.assertHoming(true);
    CHECK(sm.isMotionAllowed(), "HOMING allowed");
    CHECK(sm.state()==SafetyState::HOMING, "HOMING state");
  }
  // E_STOP -> not allowed
  {
    MockEndstops es; MockJointModel jm; SafetyManager sm(&es,&jm);
    sm.assertEStop("test");
    CHECK(!sm.isMotionAllowed(), "E_STOP not allowed");
    CHECK(sm.isEStop(), "E_STOP isEStop true");
  }
  // After clear -> allowed again
  {
    MockEndstops es; MockJointModel jm; SafetyManager sm(&es,&jm);
    sm.assertEStop("x");
    es.setGpio(0, EndstopWhich::MIN, true);
    sm.tryClearFault();
    CHECK(sm.isMotionAllowed(), "after clear allowed");
  }
  PASS("isMotionAllowed_matrix");
}

static void test_anyLatched_and_assertEStop() {
  MockEndstops es; es.setGpio(2, EndstopWhich::MAX, false);
  MockJointModel jm; SafetyManager sm(&es,&jm);
  CHECK(!sm.anyLatched(), "initial not latched");
  sm.assertEStop("manual");
  CHECK(sm.state()==SafetyState::E_STOP, "assertEStop state");
  CHECK(sm.isEStop(), "assertEStop isEStop");
  // isr + poll should also latch
  MockEndstops es2; es2.setGpio(2, EndstopWhich::MAX, false);
  MockJointModel jm2; SafetyManager sm2(&es2,&jm2);
  sm2.isrNotify(2, EndstopWhich::MAX, 5000000);
  sm2.pollEndstops(5000000+60000);
  CHECK(sm2.anyLatched(), "anyLatched after poll");
  PASS("anyLatched_and_assertEStop");
}

static void test_debounce_exact_boundary_and_glitch_recovery() {
  // At exactly 50ms should latch
  MockEndstops es; es.setGpio(0, EndstopWhich::MIN, false);
  MockJointModel jm; SafetyManager sm(&es,&jm);
  sm.isrNotify(0, EndstopWhich::MIN, 1000000);
  sm.pollEndstops(1000000 + 50000);
  bool ok1 = (sm.state()==SafetyState::E_STOP) && sm.anyLatched();
  // Glitch: HIGH at poll time after debounce should not latch and should clear pending
  MockEndstops es2; es2.setGpio(0, EndstopWhich::MIN, true); // HIGH
  MockJointModel jm2; SafetyManager sm2(&es2,&jm2);
  sm2.isrNotify(0, EndstopWhich::MIN, 2000000);
  sm2.pollEndstops(2000000+60000); // debounce passed but HIGH => no latch
  bool ok2 = (sm2.state()==SafetyState::NORMAL) && (!sm2.anyLatched());
  // Next isr with LOW should latch after debounce
  es2.setGpio(0, EndstopWhich::MIN, false);
  sm2.isrNotify(0, EndstopWhich::MIN, 3000000);
  sm2.pollEndstops(3000000+60000);
  bool ok3 = (sm2.state()==SafetyState::E_STOP) && sm2.anyLatched();
  if (ok1 && ok2 && ok3) PASS("debounce_exact_boundary_and_glitch_recovery");
  else CHECK(false, "debounce_exact_boundary_and_glitch_recovery");
}

int main() {
  test_pending_high_no_latch();
  test_low_before_50ms_no_latch();
  test_low_after_50ms_latches_estop();
  test_homing_no_estop();
  test_tryClear_reject_when_pressed();
  test_tryClear_reject_when_drift();
  test_tryClear_success_clears();
  test_isMotionAllowed_matrix();
  test_anyLatched_and_assertEStop();
  test_debounce_exact_boundary_and_glitch_recovery();

  if (g_fail==0) {
    std::printf("ALL PASSED (%d tests)\n", g_pass);
    return 0;
  }
  std::printf("%d FAILED, %d PASSED\n", g_fail, g_pass);
  return 1;
}
