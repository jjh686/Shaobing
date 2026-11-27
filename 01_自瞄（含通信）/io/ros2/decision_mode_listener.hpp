#ifndef IO__ROS2__DECISION_MODE_LISTENER_HPP
#define IO__ROS2__DECISION_MODE_LISTENER_HPP

#include "tools/decision_mode.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace io
{
// Optional ROS 2 listener for /decision2027/vision_mode_request.
// Missing, invalid or stale requests fall back to track, which keeps current
// auto-aim behaviour when the decision node is not running.
class DecisionModeListener
{
public:
  static constexpr const char * kTopic = "/decision2027/vision_mode_request";

  DecisionModeListener();
  ~DecisionModeListener();

  DecisionModeListener(const DecisionModeListener &) = delete;
  DecisionModeListener & operator=(const DecisionModeListener &) = delete;

  void spin_some();
  tools::VisionMode mode() const;
  const char * mode_name() const { return tools::vision_mode_name(mode()); }

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
}  // namespace io

#endif  // IO__ROS2__DECISION_MODE_LISTENER_HPP
