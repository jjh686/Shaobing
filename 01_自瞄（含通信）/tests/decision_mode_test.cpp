#include "tools/decision_mode.hpp"

#include <cmath>
#include <cstdlib>

namespace
{
void require(bool ok)
{
  if (!ok) std::exit(1);
}
}  // namespace

int main()
{
  using tools::VisionMode;
  using tools::GimbalMaster;

  require(tools::parse_vision_mode("alert") == VisionMode::Alert);
  require(tools::parse_vision_mode("search") == VisionMode::Search);
  require(tools::parse_vision_mode("track") == VisionMode::Track);
  require(tools::parse_vision_mode("idle") == VisionMode::Idle);
  require(tools::parse_vision_mode("other") == VisionMode::Unknown);

  require(tools::valid_tracking("tracking"));
  require(tools::valid_tracking("temp_lost"));
  require(tools::valid_tracking("switching"));
  require(!tools::valid_tracking("detecting"));
  require(!tools::valid_tracking("lost"));
  require(!tools::valid_tracking("unavailable"));

  // Alert must not steal the gimbal from a valid lock.
  for (const char * state : {"tracking", "temp_lost", "switching"}) {
    const auto locked = tools::arbitrate_gimbal(state, true, true, VisionMode::Alert);
    require(locked.master == GimbalMaster::AutoAim);
    require(locked.send_auto_aim);
    require(!locked.send_scan);
    const auto search = tools::arbitrate_gimbal(state, true, true, VisionMode::Search);
    require(search.master == GimbalMaster::AutoAim && !search.send_scan);
    const auto idle = tools::arbitrate_gimbal(state, true, true, VisionMode::Idle);
    require(idle.master == GimbalMaster::AutoAim && !idle.send_scan);
    const auto no_plan = tools::arbitrate_gimbal(state, true, false, VisionMode::Alert);
    require(no_plan.master == GimbalMaster::AutoAim);
    require(!no_plan.send_auto_aim && !no_plan.send_scan);
  }

  // Without a lock, alert and search may move the gimbal; idle and track do not.
  const auto alert = tools::arbitrate_gimbal("lost", true, false, VisionMode::Alert);
  require(alert.master == GimbalMaster::DecisionScan && alert.send_scan && !alert.send_auto_aim);
  const auto search = tools::arbitrate_gimbal("detecting", true, false, VisionMode::Search);
  require(search.master == GimbalMaster::DecisionScan && search.send_scan);
  const auto idle = tools::arbitrate_gimbal("lost", true, false, VisionMode::Idle);
  require(idle.master == GimbalMaster::None && !idle.send_scan && !idle.send_auto_aim);
  const auto track = tools::arbitrate_gimbal("lost", true, false, VisionMode::Track);
  require(track.master == GimbalMaster::None && !track.send_scan);

  // Firmware not in control mode: no command from either source.
  const auto disabled = tools::arbitrate_gimbal("tracking", false, true, VisionMode::Alert);
  require(disabled.master == GimbalMaster::None && !disabled.send_auto_aim && !disabled.send_scan);

  const auto scan = tools::scan_command(0.2f, -0.1f, 0.0, true);
  require(std::abs(scan.yaw - 0.2f) < 1e-6f);
  require(std::abs(scan.pitch + 0.1f) < 1e-6f);
  require(std::abs(scan.yaw_vel - 1.6f) < 1e-5f);
  const auto search_scan = tools::scan_command(0.0f, 0.0f, 0.0, false);
  require(std::abs(search_scan.yaw_vel - 0.48f) < 1e-5f);  // amplitude 0.6 * omega 0.8
  return 0;
}
