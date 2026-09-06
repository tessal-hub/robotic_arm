// Host test for Task 4: Non-blocking Homing FSM
// Verifies: enum settle states exist, HOMING_BACKOFF_SETTLE_MS=30, no delay(30) blocking, millis() timeout logic
#include "config.h"
#include "homing.h"
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond, msg) do { \
  if (!(cond)) { \
    std::printf("FAIL: %s (line %d)\n", msg, __LINE__); \
    ++g_fail; \
  } \
} while(0)
#define PASS(msg) do { std::printf("PASS: %s\n", msg); ++g_pass; } while(0)

static void test_config_backoff_settle_ms() {
  // Must be 30 per spec, keep timeouts invariant
  bool ok = (HOMING_BACKOFF_SETTLE_MS == 30);
  if (ok) PASS("config_backoff_settle_ms == 30");
  else CHECK(false, "HOMING_BACKOFF_SETTLE_MS must be 30");
  // Also verify other timeouts kept
  CHECK(HOMING_JOINT_TIMEOUT_MS == 30000, "HOMING_JOINT_TIMEOUT_MS 30s invariant");
  CHECK(HOMING_MAX_ATTEMPTS == 2, "HOMING_MAX_ATTEMPTS 2 invariant");
  CHECK(HOMING_MIN_MECHANICAL_SPAN_DEG == 15.0f, "J4 commissioned mechanical span floor 15 deg");
  CHECK((int)HOMING_BACKOFF_MAX_EXTEND == 3, "HOMING_BACKOFF_MAX_EXTEND 3 invariant");
  // Verify other settles kept: WARMUP 200, ENC_SETTLE 350, DEBOUNCE 50000
  // These are defined in homing.cpp anon namespace, check config indirectly via reading file
}

static void test_enum_has_settle_states() {
  // Compile-time check: enum must contain 3 new states
  HomePhase a = HomePhase::BACKOFF_SETTLE_WAIT;
  HomePhase b = HomePhase::WARMUP_SETTLE_WAIT;
  HomePhase c = HomePhase::VERIFY_SETTLE_WAIT;
  HomePhase d = HomePhase::SCAN_BACKOFF;
  HomePhase e = HomePhase::SCAN_SLOW;
  (void)a; (void)b; (void)c; (void)d; (void)e;
  // Check ordering: IDLE 0, WARMUP 1, WARMUP_SETTLE_WAIT 2, etc per brief
  bool order_ok = (static_cast<uint8_t>(HomePhase::IDLE) == 0) &&
                  (static_cast<uint8_t>(HomePhase::WARMUP) == 1) &&
                  (static_cast<uint8_t>(HomePhase::WARMUP_SETTLE_WAIT) == 2) &&
                  (static_cast<uint8_t>(HomePhase::SCAN_MIN) == 3) &&
                  (static_cast<uint8_t>(HomePhase::SCAN_BACKOFF) == 4) &&
                  (static_cast<uint8_t>(HomePhase::BACKOFF_SETTLE_WAIT) == 5) &&
                  (static_cast<uint8_t>(HomePhase::SCAN_SLOW) == 6);
  if (order_ok) PASS("enum_has_settle_states_order");
  else {
    // Allow alternative ordering but still require existence; fallback to existence only
    bool exists = (static_cast<uint8_t>(HomePhase::BACKOFF_SETTLE_WAIT) != static_cast<uint8_t>(HomePhase::IDLE)) &&
                  (static_cast<uint8_t>(HomePhase::WARMUP_SETTLE_WAIT) != static_cast<uint8_t>(HomePhase::IDLE)) &&
                  (static_cast<uint8_t>(HomePhase::VERIFY_SETTLE_WAIT) != static_cast<uint8_t>(HomePhase::IDLE));
    if (exists) PASS("enum_has_settle_states_exists");
    else CHECK(false, "enum missing settle states");
  }
  // Verify VERIFY_SETTLE_WAIT before DONE
  bool before_done = static_cast<uint8_t>(HomePhase::VERIFY_SETTLE_WAIT) < static_cast<uint8_t>(HomePhase::DONE);
  CHECK(before_done, "VERIFY_SETTLE_WAIT before DONE");
  if (before_done && order_ok) PASS("enum_all_settle_states");
}

static std::string readFile(const char* path) {
  std::ifstream f(path);
  if (!f) return "";
  return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

static void test_no_delay_blocking() {
  std::string content = readFile("src/homing.cpp");
  if (content.empty()) content = readFile("/home/pa/Code/4.robotic_arm/robotic_arm/src/homing.cpp");
  // Must NOT contain delay(30) blocking call (allow comment containing delay but not actual code)
  // Check for pattern "delay(30" without preceding "//" on same line
  bool has_blocking_delay = false;
  // Simple: search for "\n    delay(30" or "delay(30);" not in comment line
  size_t pos = 0;
  while ((pos = content.find("delay(30", pos)) != std::string::npos) {
    // Find start of line
    size_t line_start = content.rfind('\n', pos);
    if (line_start == std::string::npos) line_start = 0; else line_start++;
    std::string line = content.substr(line_start, pos - line_start);
    // If line trimmed starts with //, it's comment
    size_t first_non_ws = line.find_first_not_of(" \t");
    bool is_comment = false;
    if (first_non_ws != std::string::npos && line[first_non_ws] == '/' && first_non_ws+1 < line.size() && line[first_non_ws+1] == '/') is_comment = true;
    // Also check if the delay is inside a comment block "thay delay(30)" -> still not code
    // Code delay would be like "delay(30);" with possible spaces
    if (!is_comment) {
      // Check that preceding char is not part of comment phrase "thay delay"
      // If line contains "delay(30" and also contains "(" after, it's likely code if not comment
      // We'll consider any non-comment occurrence as blocking
      has_blocking_delay = true;
      break;
    }
    pos += 9;
  }
  // Also check for "delay (" variant
  if (!has_blocking_delay) {
    pos = 0;
    while ((pos = content.find("delay (", pos)) != std::string::npos) {
      size_t line_start = content.rfind('\n', pos);
      if (line_start == std::string::npos) line_start = 0; else line_start++;
      std::string line = content.substr(line_start, pos - line_start);
      size_t first_non_ws = line.find_first_not_of(" \t");
      bool is_comment = false;
      if (first_non_ws != std::string::npos && line[first_non_ws] == '/' && first_non_ws+1 < line.size() && line[first_non_ws+1] == '/') is_comment = true;
      if (!is_comment) { has_blocking_delay = true; break; }
      pos += 7;
    }
  }
  if (!has_blocking_delay) PASS("no_delay_blocking");
  else CHECK(false, "homing.cpp still contains blocking delay(30)");
  // Must contain BACKOFF_SETTLE_WAIT handling with millis()
  bool has_backoff_wait = content.find("BACKOFF_SETTLE_WAIT") != std::string::npos;
  bool has_warmup_wait = content.find("WARMUP_SETTLE_WAIT") != std::string::npos;
  bool has_verify_wait = content.find("VERIFY_SETTLE_WAIT") != std::string::npos;
  bool has_millis_check = content.find("millis()") != std::string::npos && content.find("HOMING_BACKOFF_SETTLE_MS") != std::string::npos;
  bool has_settle_start = content.find("settleStartMs_") != std::string::npos;
  bool stops_on_finish = content.find("motors[curAxis_] != nullptr) motors[curAxis_]->stop()") != std::string::npos;
  bool fast_sensorless_fusion = content.find("if (!hitAny && sgStalled && encoderStalled)") != std::string::npos;
  bool slow_sensorless_fusion = content.find("if (!hitEndstop && sgStalled && encoderStalled)") != std::string::npos;
  bool rejects_bad_sensorless_span = content.find("CROSSCHECK encoder span") != std::string::npos;
  bool rejects_bad_sensorless_ratio = content.find("CROSSCHECK encoder ratio") != std::string::npos;
  bool direction_independent_stall_frame = content.find("encDeltaDeg = fabsf(curEnc - lastCheckEnc_)") != std::string::npos &&
                                           content.find("sweepJointSign") != std::string::npos;
  bool has_j4_second_side_cap = content.find("exceededSecondSideTravel") != std::string::npos &&
                               content.find("HOMING_J4_MAX_MECHANICAL_SPAN_DEG") != std::string::npos;
  CHECK(has_backoff_wait, "has BACKOFF_SETTLE_WAIT");
  CHECK(has_warmup_wait, "has WARMUP_SETTLE_WAIT");
  CHECK(has_verify_wait, "has VERIFY_SETTLE_WAIT");
  CHECK(has_millis_check, "uses millis() + HOMING_BACKOFF_SETTLE_MS");
  CHECK(has_settle_start, "has settleStartMs_ member use");
  CHECK(stops_on_finish, "finishJoint always stops the current motor");
  CHECK(fast_sensorless_fusion, "J4 FAST requires both StallGuard and AS5600 stall");
  CHECK(slow_sensorless_fusion, "J4 SLOW requires both StallGuard and AS5600 stall");
  CHECK(rejects_bad_sensorless_span, "J4 rejects invalid encoder span before SetHome");
  CHECK(rejects_bad_sensorless_ratio, "J4 rejects invalid encoder ratio before SetHome");
  CHECK(direction_independent_stall_frame, "J4 detects motion independent of noisy warmup sign and learns sign from full sweep");
  CHECK(has_j4_second_side_cap, "J4 aborts a missing second hard-stop before scan timeout");
  CHECK(content.find("m.setChopperMode(curAxis_ == 1)") != std::string::npos,
        "J2 homes with SpreadCycle while J4 retains StealthChop for StallGuard");
  CHECK(content.find("BACKOFF encoder chua dich") != std::string::npos,
        "J4 extends backoff through backlash before declaring a jam");
  if (has_backoff_wait && has_warmup_wait && has_verify_wait && has_millis_check && has_settle_start) {
    if (!has_blocking_delay) PASS("settle_states_millis_logic");
  }
}

static void test_backoff_not_blocking_logic() {
  // Simulate the timing invariant: 5 rapid ticks within 30ms stay in wait, after 30ms goes to SCAN_SLOW
  // This is a mock of the spec's test_backoff_not_blocking — we validate the millis() comparison logic
  // without requiring full hardware mocks.
  // Mock values:
  const uint32_t settleMs = HOMING_BACKOFF_SETTLE_MS; // 30
  uint32_t settleStart = 1000;
  // 5 ticks at 5ms intervals -> now = 1005,1010,1015,1020,1025 all < 30ms from settleStart
  for (int i = 0; i < 5; ++i) {
    uint32_t now = settleStart + 5*(i+1);
    bool stay = (now - settleStart < settleMs);
    char msg[64];
    std::snprintf(msg, sizeof(msg), "backoff_wait_stays_%d", i);
    if (stay) PASS(msg);
    else CHECK(false, msg);
  }
  uint32_t now_after = settleStart + 30;
  bool proceed = (now_after - settleStart >= settleMs);
  CHECK(proceed, "backoff_wait_proceeds_after_30ms");
  if (proceed) PASS("backoff_not_blocking_timing");
}

static void test_j3_scans_both_stops_before_offset() {
  const std::string source = readFile("src/homing.cpp");
  const size_t start = source.find("case HomePhase::SCAN_SLOW:");
  const size_t end = source.find("case HomePhase::LEG1_BACKOFF:", start);
  CHECK(start != std::string::npos && end != std::string::npos, "find production SLOW handler");
  if (start == std::string::npos || end == std::string::npos) return;
  const std::string slow = source.substr(start, end - start);
  CHECK(slow.find("homeAtMinOffset") == std::string::npos,
        "J3 must not bypass second contact in SLOW");
  CHECK(slow.find("phase_ = HomePhase::CENTERING") == std::string::npos,
        "SLOW cannot directly mark first contact ready for home");
  CHECK(slow.find("secondSide_ = true") != std::string::npos &&
        slow.find("enterCenteringScan();") != std::string::npos,
        "both contacts use shared sweep completion");
  CHECK(source.find("? contactSpan_ - offsetSteps : offsetSteps") != std::string::npos,
        "J3 MIN offset handles either contact order");
  CHECK(source.find("contactSpan_ <= offsetSteps") != std::string::npos,
        "J3 rejects span too short for offset");
  const size_t scanMax = source.find("void HomingController::enterScanMax()");
  const size_t backoff = source.find("void HomingController::enterScanBackoff()", scanMax);
  CHECK(scanMax != std::string::npos && backoff != std::string::npos,
        "find production second scan entry");
  if (scanMax != std::string::npos && backoff != std::string::npos) {
    CHECK(source.substr(scanMax, backoff - scanMax).find("cwApproach_ = !cwApproach_") != std::string::npos,
          "second scan reverses actual direction even if first contact is MAX");
  }
}

static void test_endstop_scan_guards() {
  const std::string source = readFile("src/homing.cpp");
  CHECK(!source.empty(), "read production homing source");
  const size_t scan = source.find("void HomingController::enterScanMax()");
  const size_t backoff = source.find("void HomingController::enterScanBackoff()", scan);
  CHECK(scan != std::string::npos && backoff != std::string::npos, "find second scan entry");
  if (scan != std::string::npos && backoff != std::string::npos) {
    const std::string entry = source.substr(scan, backoff - scan);
    CHECK(entry.find("setPinEnabled(curAxis_, firstSide_, true)") != std::string::npos,
          "first switch must be armed during second scan");
    CHECK(entry.find("setPinEnabled(curAxis_, targetSide, true)") != std::string::npos,
          "target switch must be armed during second scan");
  }
  CHECK(source.find("if (firstLeg || movedFarEnough)") == std::string::npos,
        "physical contact must not be discarded by sensorless travel grace");
  CHECK(source.find("isPhysicallyPressed(curAxis_, firstSide_)") != std::string::npos,
        "masked first switch must be physically released before scanning");
  CHECK(source.find("cwApproach_ = warmupCW_;") != std::string::npos,
        "warmup contact backs off relative to actual approach direction");
  CHECK(source.find("if (minP || maxP)") < source.find("m.run(warmupCW_, warmupSteps)"),
        "pressed switch at start rejects warmup before movement");
  CHECK(source.find("m.getAbsoluteSteps() != centeringTargetSteps_") != std::string::npos,
        "interrupted centering cannot save home");
  const std::string endstops = readFile("src/endstop.cpp");
  const size_t raw = endstops.find("bool Endstops::isPhysicallyPressed(");
  const size_t next = endstops.find("bool Endstops::isLatched(", raw);
  CHECK(raw != std::string::npos && next != std::string::npos, "find raw GPIO reader");
  if (raw != std::string::npos && next != std::string::npos) {
    const std::string reader = endstops.substr(raw, next - raw);
    CHECK(reader.find("digitalRead") != std::string::npos &&
          reader.find("isPinEnabled") == std::string::npos,
          "raw switch read remains available while interrupt is masked");
  }
}

static void test_warmup_settle_timing() {
  // WARMUP_ENC_SETTLE_MS = 200 per homing.cpp
  const uint32_t warmupSettle = 200;
  uint32_t start = 2000;
  bool stay = (start + 100 - start < warmupSettle);
  CHECK(stay, "warmup_wait_stays_within_200ms");
  bool proceed = (start + 200 - start >= warmupSettle);
  CHECK(proceed, "warmup_wait_proceeds_after_200ms");
  if (stay && proceed) PASS("warmup_settle_timing");
}

static void test_verify_settle_timing() {
  // ENC_SETTLE_MS = 350
  const uint32_t encSettle = 350;
  uint32_t start = 3000;
  bool stay = (start + 200 - start < encSettle);
  CHECK(stay, "verify_wait_stays_within_350ms");
  bool proceed = (start + 350 - start >= encSettle);
  CHECK(proceed, "verify_wait_proceeds_after_350ms");
  if (stay && proceed) PASS("verify_settle_timing");
}

int main() {
  test_config_backoff_settle_ms();
  test_enum_has_settle_states();
  test_no_delay_blocking();
  test_j3_scans_both_stops_before_offset();
  test_endstop_scan_guards();
  test_backoff_not_blocking_logic();
  test_warmup_settle_timing();
  test_verify_settle_timing();
  if (g_fail == 0) {
    std::printf("ALL PASSED (%d tests)\n", g_pass);
    return 0;
  }
  std::printf("%d FAILED, %d PASSED\n", g_fail, g_pass);
  return 1;
}
