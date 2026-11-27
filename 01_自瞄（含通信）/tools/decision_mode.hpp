#ifndef TOOLS__DECISION_MODE_HPP
#define TOOLS__DECISION_MODE_HPP

#include <cmath>
#include <string_view>

namespace tools
{
// Decision publishes idle/search/track/alert on /decision2027/vision_mode_request.
enum class VisionMode { Idle, Search, Track, Alert, Unknown };

// One gimbal master per frame. Auto-aim keeps the gimbal whenever tracking is valid;
// search/alert may move it only after that lock is gone. Idle never commands motion.
enum class GimbalMaster { None, AutoAim, DecisionScan };

struct GimbalArbitration
{
  GimbalMaster master = GimbalMaster::None;
  bool send_auto_aim = false;
  bool send_scan = false;
};

struct ScanTarget
{
  float yaw = 0, yaw_vel = 0, yaw_acc = 0;
  float pitch = 0, pitch_vel = 0, pitch_acc = 0;
};

inline VisionMode parse_vision_mode(std::string_view text)
{
  if (text == "idle") return VisionMode::Idle;
  if (text == "search") return VisionMode::Search;
  if (text == "track") return VisionMode::Track;
  if (text == "alert") return VisionMode::Alert;
  return VisionMode::Unknown;
}

inline const char * vision_mode_name(VisionMode mode)
{
  switch (mode) {
    case VisionMode::Idle: return "idle";
    case VisionMode::Search: return "search";
    case VisionMode::Track: return "track";
    case VisionMode::Alert: return "alert";
    default: return "unknown";
  }
}

// A confirmed lock: tracking, a brief drop (temp_lost), or a priority switch.
// detecting has not reached min_detect_count; lost/unavailable are not a lock.
inline bool valid_tracking(std::string_view tracker_state)
{
  return tracker_state == "tracking" || tracker_state == "temp_lost" ||
         tracker_state == "switching";
}

inline GimbalArbitration arbitrate_gimbal(
  std::string_view tracker_state, bool command_enabled, bool plan_control, VisionMode mode)
{
  GimbalArbitration out;
  if (!command_enabled) return out;
  if (valid_tracking(tracker_state)) {
    out.master = GimbalMaster::AutoAim;
    out.send_auto_aim = plan_control;
    return out;
  }
  if (mode == VisionMode::Alert || mode == VisionMode::Search) {
    out.master = GimbalMaster::DecisionScan;
    out.send_scan = true;
    return out;
  }
  return out;
}

// Sweep yaw about the heading where search/alert began; keep pitch still. Never fire.
inline ScanTarget scan_command(
  float center_yaw, float center_pitch, double elapsed_s, bool alert)
{
  const double amplitude = alert ? 1.0 : 0.6;
  const double omega = alert ? 1.6 : 0.8;
  const double phase = omega * elapsed_s;
  const double offset = amplitude * std::sin(phase);
  ScanTarget target;
  target.yaw = static_cast<float>(center_yaw + offset);
  target.yaw_vel = static_cast<float>(amplitude * omega * std::cos(phase));
  target.yaw_acc = static_cast<float>(-amplitude * omega * omega * std::sin(phase));
  target.pitch = center_pitch;
  return target;
}
}  // namespace tools

#endif  // TOOLS__DECISION_MODE_HPP
