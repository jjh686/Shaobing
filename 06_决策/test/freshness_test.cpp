// Deterministic clock injection: no ROS, sleeps or wall-clock dependency.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <chrono>
#include <cstdint>

#include "rm2027_decision/freshness.hpp"

using namespace rm2027_decision;
using namespace std::chrono_literals;

int main()
{
  constexpr int64_t second = 1000000000LL;
  constexpr double timeout = 2.5;
  const auto start = Steady::time_point{} + 10s;
  SourceProgress source;
  assert(source.stamp == -1 && !source.advanced);
  assert(!source.fresh(second, true, second, start, timeout));

  source.observe(second, true, second, start);
  assert(source.stamp == second && source.advanced == start);
  assert(source.fresh(second, true, second, start, timeout));
  assert(source.fresh(second, true, 2 * second, start + 1s, timeout));

  // An aggregate heartbeat republishes the same source stamp during a clock pause.
  source.observe(second, true, second, start + 2s);
  assert(source.advanced == start);
  assert(source.fresh(second, true, second, start + 2499ms, timeout));
  assert(!source.fresh(second, true, second, start + 2500ms, timeout));
  assert(!source.fresh(second, true, second, start + 3s, timeout));

  // Source progress refreshes both its high-water stamp and steady receipt age.
  source.observe(2 * second, true, 2 * second, start + 3s);
  assert(source.stamp == 2 * second && source.advanced == start + 3s);
  assert(source.fresh(2 * second, true, 2 * second, start + 3s, timeout));
  source.observe(second, true, 2 * second, start + 4s);
  assert(source.stamp == 2 * second && source.advanced == start + 3s);
  assert(!source.fresh(second, true, 2 * second, start + 4s, timeout));
  assert(source.fresh(2 * second, true, 2 * second, start + 4s, timeout));

  // Invalid inputs cannot advance a valid source, and are never fresh themselves.
  source.observe(3 * second, false, 3 * second, start + 4s);
  source.observe(4 * second, true, 3 * second, start + 4s);
  source.observe(0, true, 3 * second, start + 4s);
  source.observe(-1, true, 3 * second, start + 4s);
  assert(source.stamp == 2 * second && source.advanced == start + 3s);
  assert(!source.fresh(2 * second, false, 3 * second, start + 4s, timeout));
  assert(!source.fresh(4 * second, true, 3 * second, start + 4s, timeout));
  assert(!source.fresh(0, true, 3 * second, start + 4s, timeout));
  assert(!source.fresh(-1, true, 3 * second, start + 4s, timeout));
  // Previously observed stamps also fail if ROS time moves behind them.
  assert(!source.fresh(2 * second, true, second, start + 4s, timeout));

  // Either clock alone can expire the source, including the exact timeout edge.
  assert(source.fresh(2 * second, true, 4 * second + 499000000LL,
    start + 4s, timeout));
  assert(!source.fresh(2 * second, true, 4 * second + 500000000LL,
    start + 4s, timeout));
  assert(!source.fresh(2 * second, true, 5 * second, start + 4s, timeout));
  assert(!source.fresh(2 * second, true, 2 * second, start + 5500ms, timeout));

  SourceProgress rejected_first;
  rejected_first.observe(0, true, second, start);
  rejected_first.observe(-1, true, second, start);
  rejected_first.observe(2 * second, true, second, start);
  rejected_first.observe(second, false, second, start);
  assert(rejected_first.stamp == -1 && !rejected_first.advanced);

  // Owner resets progress after a ROS clock boundary; lower new-epoch stamps work.
  source = SourceProgress{};
  source.observe(second, true, second, start + 6s);
  assert(source.stamp == second && source.advanced == start + 6s);
  assert(source.fresh(second, true, second, start + 6s, timeout));
  return 0;
}
