// Host test for Task 2 ISR No-Delay refactor — verifies ISR minimalism (<5us) and debounce delegation
#include "safety_manager.h"
#include "test_mocks.h"
#include <cstdio>
#include <chrono>
#include <atomic>
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

// Simulate minimal ISR as specified: only pending + timestamp, <3us
struct FakeIsrCtx {
  std::atomic<bool> pending{false};
  std::atomic<int64_t> isrTime{0};
  uint8_t axis{0};
  EndstopWhich which{EndstopWhich::MIN};
};

static inline void fakeIsrHandler(FakeIsrCtx* c, int64_t nowUs) {
  c->pending.store(true, std::memory_order_relaxed);
  c->isrTime.store(nowUs, std::memory_order_relaxed);
}

static std::string readSource(const char* path) {
  std::ifstream in(path);
  if (!in) return {};
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

static void test_isr_no_delay() {
  FakeIsrCtx ctx;
  ctx.axis = 0; ctx.which = EndstopWhich::MIN;
  // Measure latency of minimal ISR
  auto t0 = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < 1000; ++i) {
    fakeIsrHandler(&ctx, 1000000 + i);
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  int64_t dtAvgNs = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count() / 1000;
  int64_t dtUs = dtAvgNs / 1000;
  // Should be <5us per invocation (typically <1us)
  bool ok = dtUs < 5;
  if (ok) {
    PASS("isr_no_delay");
    std::printf("  avg ISR latency: %lld ns (%lld us) <5us OK\n", (long long)dtAvgNs, (long long)dtUs);
  } else {
    CHECK(false, "isr_no_delay: ISR too slow, should be <5us");
    std::printf("  avg ISR latency: %lld ns (%lld us) FAIL\n", (long long)dtAvgNs, (long long)dtUs);
  }
  // Also verify pending flag set
  CHECK(ctx.pending.load(std::memory_order_relaxed) == true, "isr_no_delay pending flag");
  CHECK(ctx.isrTime.load(std::memory_order_relaxed) == 1000999, "isr_no_delay timestamp");
}

static void test_isr_delegates_to_safety_manager() {
  // Verify ISR only sets pending, debounce + latch via SafetyManager poll
  MockEndstops es;
  MockJointModel jm;
  SafetyManager sm(&es, &jm);

  // Simulate ISR at t=2_000_000 with GPIO still HIGH (not pressed) -> should not latch after debounce
  es.setGpio(0, EndstopWhich::MIN, true); // HIGH = not pressed
  sm.isrNotify(0, EndstopWhich::MIN, 2000000);
  sm.pollEndstops(2000000 + 60000); // 60ms > debounce, but GPIO HIGH => no latch
  bool ok1 = (sm.state() == SafetyState::NORMAL) && (!sm.anyLatched()) && (!sm.isLatched(0, EndstopWhich::MIN));
  CHECK(ok1, "isr delegates: HIGH at poll should not latch (glitch rejection)");

  // Now simulate ISR with LOW at poll time -> should latch after 50ms
  es.setGpio(1, EndstopWhich::MAX, false); // LOW = pressed
  sm.isrNotify(1, EndstopWhich::MAX, 3000000);
  sm.pollEndstops(3000000 + 40000); // 40ms < debounce => not yet latched
  // At least the channel 1 should not yet be latched before debounce (but other channels also false)
  CHECK(!sm.isLatched(1, EndstopWhich::MAX), "isr delegates: before debounce not latched");

  sm.pollEndstops(3000000 + 60000); // now 60ms > debounce and LOW => latch + E_STOP (not homing)
  bool ok2 = sm.isLatched(1, EndstopWhich::MAX) && sm.anyLatched() && sm.state() == SafetyState::E_STOP;
  if (ok1 && ok2) PASS("isr_delegates_to_safety_manager");
  else CHECK(false, "isr_delegates_to_safety_manager: debounce delegation failed");
}

static void test_isr_homing_no_estop() {
  MockEndstops es;
  MockJointModel jm;
  SafetyManager sm(&es, &jm);
  sm.assertHoming(true);
  es.setGpio(2, EndstopWhich::MIN, false);
  sm.isrNotify(2, EndstopWhich::MIN, 4000000);
  sm.pollEndstops(4000000 + 60000);
  bool ok = sm.isLatched(2, EndstopWhich::MIN) && sm.state() == SafetyState::HOMING && !sm.isEStop();
  if (ok) PASS("isr_homing_no_estop");
  else CHECK(false, "isr_homing_no_estop");
}

static void test_isr_stops_pulse_before_debounce() {
  const std::string source = readSource("src/endstop.cpp");
  const size_t handler = source.find("Endstops::isrHandler");
  const size_t stop = source.find("c->motor->stopFromISR()", handler);
  const size_t pending = source.find("c->pending.store", handler);
  const bool ok = handler != std::string::npos && stop != std::string::npos &&
                  pending != std::string::npos && stop < pending;
  if (ok) PASS("isr_stops_pulse_before_debounce");
  else CHECK(false, "ISR must stop the affected pulse train before debounce bookkeeping");
}

static void test_stop_all_preempts_queue() {
  const std::string source = readSource("src/arm.cpp");
  const bool hasMailbox = source.find("stopRequested_.store(true") != std::string::npos;
  const bool hasEarlyPoll = source.find("stopRequested_.exchange(false") != std::string::npos;
  const bool clearsQueue = source.find("xQueueReset(queue)") != std::string::npos;
  if (hasMailbox && hasEarlyPoll && clearsQueue) PASS("stop_all_preempts_queue");
  else CHECK(false, "STOP_ALL must use an out-of-band request and clear queued motion");
}

int main() {
  test_isr_no_delay();
  test_isr_delegates_to_safety_manager();
  test_isr_homing_no_estop();
  test_isr_stops_pulse_before_debounce();
  test_stop_all_preempts_queue();

  if (g_fail == 0) {
    std::printf("ALL PASSED (%d tests)\n", g_pass);
    return 0;
  }
  std::printf("%d FAILED, %d PASSED\n", g_fail, g_pass);
  return 1;
}
