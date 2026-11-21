#ifndef RM2027_DECISION__FRESHNESS_HPP_
#define RM2027_DECISION__FRESHNESS_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

namespace rm2027_decision
{
using Steady = std::chrono::steady_clock;

// Aggregate heartbeats must not refresh the receipt age of an unchanged source.
struct SourceProgress
{
  int64_t stamp = -1;
  std::optional<Steady::time_point> advanced;

  void observe(int64_t value, bool valid, int64_t now, Steady::time_point wall)
  {
    if (!valid || value <= 0 || value > now) return;
    if (value > stamp) { stamp = value; advanced = wall; }
  }

  bool fresh(int64_t value, bool valid, int64_t now, Steady::time_point wall,
    double timeout) const
  {
    return valid && value > 0 && value <= now && value == stamp && advanced &&
      (now - value) * 1e-9 < timeout &&
      std::chrono::duration<double>(wall - *advanced).count() < timeout;
  }
};

}  // namespace rm2027_decision

#endif  // RM2027_DECISION__FRESHNESS_HPP_
