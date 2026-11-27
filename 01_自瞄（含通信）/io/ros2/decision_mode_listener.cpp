#include "io/ros2/decision_mode_listener.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace io
{
namespace
{
constexpr auto kFreshness = std::chrono::seconds(2);
}

struct DecisionModeListener::Impl
{
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub;
  mutable std::mutex mutex;
  tools::VisionMode mode{tools::VisionMode::Track};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  bool received{false};
};

DecisionModeListener::DecisionModeListener() : impl_(std::make_unique<Impl>())
{
  if (!rclcpp::ok()) {
    int argc = 0;
    rclcpp::init(argc, nullptr);
  }
  impl_->node = std::make_shared<rclcpp::Node>("auto_aim_decision_mode");
  impl_->sub = impl_->node->create_subscription<std_msgs::msg::String>(
    kTopic, rclcpp::QoS(1), [this](const std_msgs::msg::String & msg) {
      const auto parsed = tools::parse_vision_mode(msg.data);
      if (parsed == tools::VisionMode::Unknown) return;
      std::lock_guard<std::mutex> lock(impl_->mutex);
      impl_->mode = parsed;
      impl_->stamp = impl_->node->now();
      impl_->received = true;
    });
}

DecisionModeListener::~DecisionModeListener()
{
  impl_->sub.reset();
  impl_->node.reset();
}

void DecisionModeListener::spin_some()
{
  if (impl_->node) rclcpp::spin_some(impl_->node);
}

tools::VisionMode DecisionModeListener::mode() const
{
  std::lock_guard<std::mutex> lock(impl_->mutex);
  if (!impl_->received) return tools::VisionMode::Track;
  const auto now = impl_->node->now();
  if (now < impl_->stamp || (now - impl_->stamp) > rclcpp::Duration(kFreshness)) {
    return tools::VisionMode::Track;
  }
  return impl_->mode;
}
}  // namespace io
