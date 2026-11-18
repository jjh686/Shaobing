// These checks must run in release builds too.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdint>
#include <limits>

#include "rm_chassis_comm/gimbal_clock.hpp"

using rm_chassis_comm::GimbalClock;

static bool near(double a, double b)
{
  return std::abs(a - b) < 1e-7;
}

int main()
{
  {
    GimbalClock clock;
    const auto r = clock.observe(100, 98, 99, 98.0, 100.0);
    assert(r && near(r->sample_host_ms, 99.0) && near(r->uncertainty_ms, 0.0));
  }
  {
    // Offset PC-MCU = 1000 ms, symmetric 3 ms one-way delay.
    GimbalClock clock;
    const auto r = clock.observe(100, 98, 95, 1095.0, 1103.0);
    assert(r && near(r->sample_host_ms, 1095.0) && near(r->uncertainty_ms, 3.0));
  }
  {
    // Asymmetric delays 1/9 ms: midpoint estimate is +4 ms from truth;
    // the RTT/2 uncertainty bound is 5 ms.
    GimbalClock clock;
    const auto r = clock.observe(100, 98, 95, 1097.0, 1109.0);
    assert(r && near(r->sample_host_ms, 1099.0));
    assert(std::abs(r->sample_host_ms - 1095.0) <= r->uncertainty_ms);
    assert(near(r->uncertainty_ms, 5.0));
  }
  {
    // tx, rx and sample all cross the 32-bit uptime boundary.
    GimbalClock clock;
    constexpr uint32_t end = UINT32_C(0xffffffff);
    auto a = clock.observe(end - 4, end - 6, end - 5, 997.0, 1001.0);
    auto b = clock.observe(4, end, 2, 1003.0, 1010.0);
    assert(a && b && near(b->sample_host_ms - a->sample_host_ms, 8.0));
  }
  {
    GimbalClock clock;
    assert(clock.observe(100, 98, 95, 1097.0, 1101.0));
    // Duplicate sample despite advancing tx, replay, source clock rollback.
    assert(!clock.observe(102, 100, 95, 1099.0, 1103.0));
    assert(!clock.observe(100, 98, 95, 1097.0, 1101.0));
    assert(!clock.observe(99, 97, 96, 1096.0, 1100.0));
    auto good = clock.observe(101, 99, 96, 1098.0, 1102.0);
    assert(good && near(good->sample_host_ms, 1096.0));
    clock.reset();
    assert(clock.observe(2, 0, 1, 1999.0, 2003.0));
  }
  {
    GimbalClock clock;
    assert(!clock.observe(100, 98, 95, 0.0, 101.0));  // host RTT
    assert(!clock.observe(100, 98, 95, 0.0, 23.0));   // network RTT
    assert(!clock.observe(300, 99, 295, 0.0, 10.0)); // processing > 200
    assert(!clock.observe(100, 101, 95, 0.0, 10.0)); // rx in future
    assert(!clock.observe(100, 98, 101, 0.0, 10.0)); // sample in future
    assert(!clock.observe(100, 98, 49, 0.0, 10.0));  // sample age > 50
    assert(!clock.observe(100, 98, 95, 10.0, 0.0));  // host rollback
    assert(!clock.observe(100, 98, 95, 0.0, 1.0));   // negative network RTT
    assert(!clock.observe(100, 98, 95,
      std::numeric_limits<double>::quiet_NaN(), 10.0));
    assert(!clock.observe(100, 98, 95, 0.0,
      std::numeric_limits<double>::infinity()));
    assert(clock.observe(1, 0, 0, 9.0, 12.0)); // rejects did not seed state
  }
  {
    GimbalClock clock;
    assert(clock.observe(100, 98, 95, 1097.0, 1101.0)); // offset 1000, RTT 2
    // A higher-RTT sample's asymmetric queue must not shift the clock.
    auto queued = clock.observe(110, 108, 105, 1107.0, 1121.0);
    assert(queued && near(queued->sample_host_ms, 1105.0));
    assert(near(queued->uncertainty_ms, 1.0));
    // Valid ticks but incompatible clock mapping: too old / future mapped.
    assert(!clock.observe(120, 118, 115, 1220.0, 1226.0));
    assert(!clock.observe(120, 118, 115, 1000.0, 1006.0));
    // Earlier than rejected tx/sample remains admissible; window unchanged.
    auto good = clock.observe(111, 109, 106, 1108.0, 1112.0);
    assert(good && near(good->sample_host_ms, 1106.0));
    assert(near(good->uncertainty_ms, 1.0));
  }
  {
    // The minimum-RTT estimate expires after exactly 16 accepted exchanges.
    GimbalClock clock;
    assert(clock.observe(100, 98, 95, 1097.0, 1101.0)); // offset 1000, RTT 2
    for (uint32_t i = 1; i <= 16; ++i) {
      const uint32_t tick = 100 + 10 * i;
      auto r = clock.observe(tick, tick - 2, tick - 5,
        1000.0 + tick - 3.0, 1000.0 + tick + 5.0); // offset 1002, RTT 6
      assert(r);
      assert(near(r->sample_host_ms, 1000.0 + tick - 5.0 + (i == 16 ? 2.0 : 0.0)));
      assert(near(r->uncertainty_ms, i == 16 ? 3.0 : 1.0));
    }
  }
  {
    // A better offset is rejected if it would move the mapped sample backwards.
    GimbalClock clock;
    assert(clock.observe(100, 98, 95, 1097.0, 1101.0));
    assert(!clock.observe(101, 99, 96, 1000.0, 1002.0));
    auto good = clock.observe(102, 100, 97, 1099.0, 1103.0);
    assert(good && near(good->sample_host_ms, 1097.0));
  }
}
