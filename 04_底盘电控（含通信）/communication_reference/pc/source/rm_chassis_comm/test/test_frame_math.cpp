#ifdef NDEBUG
#undef NDEBUG  // Assertions remain executable in Release test builds.
#endif
#include "rm_chassis_comm/frame_math.hpp"
#include <cassert>
#include <cmath>
#include <limits>

namespace fm = rm_chassis_comm::frame_math;
static bool near(double a, double b) {return std::fabs(a - b) < 1e-10;}

int main()
{
  const std::array<double, 3> origin{{0, 0, 0}};
  fm::PlanarTwist forward;
  forward.vx = 1;
  auto left = fm::to_body(forward, fm::kPi / 2, origin);
  auto right = fm::to_body(forward, -fm::kPi / 2, origin);
  assert(left && near(left->vx, 0) && near(left->vy, 1));
  assert(right && near(right->vx, 0) && near(right->vy, -1));

  fm::PlanarTwist spin;
  spin.wz = fm::kPi;
  auto centred = fm::to_body(spin, 1.2, origin);
  assert(centred && near(centred->vx, 0) && near(centred->vy, 0));
  assert(centred->wz == spin.wz);  // No second spin contribution.
  auto offset = fm::to_body(spin, 0.7, {{0.2, -0.1, 0.5}});
  assert(offset && near(offset->vx, -0.1 * fm::kPi));
  assert(near(offset->vy, -0.2 * fm::kPi) && offset->wz == spin.wz);

  fm::PlanarTwist invalid = forward;
  invalid.wx = 0.1;
  assert(!fm::to_body(invalid, 0, origin));
  invalid = forward;
  invalid.vz = 0.01;
  assert(!fm::to_body(invalid, 0, origin));
  invalid = forward;
  invalid.vy = std::numeric_limits<double>::quiet_NaN();
  assert(!fm::to_body(invalid, 0, origin));
  assert(!fm::to_body(forward, std::numeric_limits<double>::infinity(), origin));
  assert(!fm::to_body(forward, 0, {{0, std::numeric_limits<double>::infinity(), 0}}));

  fm::JointHistory history;
  assert(!history.at(100));
  assert(history.push({100, fm::kPi - 0.1, 0.2}));
  assert(history.push({200, -fm::kPi + 0.1, 0.4}));
  auto midpoint = history.at(150);
  assert(midpoint && near(std::fabs(midpoint->yaw), fm::kPi));
  assert(near(midpoint->pitch, 0.3));
  assert(history.at(100) && history.at(200));
  assert(!history.at(99) && !history.at(201));  // No extrapolation.
  assert(!history.push({200, 0, 0}) && !history.push({50, 0, 0}));
  assert(!history.push({300, std::numeric_limits<double>::quiet_NaN(), 0}));
  history.clear();
  for (std::int64_t i = 1; i <= 100; ++i) {
    assert(history.push({i, 0.01 * static_cast<double>(i), 0}));
  }
  assert(history.size() == fm::JointHistory::kCapacity);
  assert(history.first()->stamp_ns == 37 && history.last()->stamp_ns == 100);
  assert(!history.at(36) && history.at(37));
  history.clear();
  assert(!history.at(100));
  assert(history.push({100, 0, 1e308}));
  assert(history.push({200, 0, -1e308}));
  assert(history.at(150) && near(history.at(150)->pitch, 0));
}
