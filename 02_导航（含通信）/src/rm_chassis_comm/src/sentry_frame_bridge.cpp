#include "rm_chassis_comm/frame_math.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace fm = rm_chassis_comm::frame_math;
using Steady = std::chrono::steady_clock;
using TwistStamped = geometry_msgs::msg::TwistStamped;

class SentryFrameBridge : public rclcpp::Node
{
public:
  explicit SentryFrameBridge(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("sentry_frame_bridge", options)
  {
    rcl_interfaces::msg::ParameterDescriptor fixed;
    fixed.read_only = true;
    fixed.description = "Startup-only measured geometry/frame configuration; restart to change.";
    body_frame_ = declare_parameter<std::string>("body_frame", "base_footprint", fixed);
    yaw_frame_ = declare_parameter<std::string>("yaw_frame", "gimbal_yaw", fixed);
    pitch_frame_ = declare_parameter<std::string>("pitch_frame", "gimbal_pitch", fixed);
    yaw_joint_ = declare_parameter<std::string>("yaw_joint", "gimbal_yaw_joint", fixed);
    pitch_joint_ = declare_parameter<std::string>("pitch_joint", "gimbal_pitch_joint", fixed);
    // Zero translations are placeholders. Only set confirmed=true after the
    // real pivots, joint signs, and fixed Z-yaw/Y-pitch axes are verified.
    geometry_confirmed_ = declare_parameter<bool>("geometry_confirmed", false, fixed);
    publish_tf_ = declare_parameter<bool>("publish_tf", true, fixed);
    yaw_pivot_ = pivot_parameter("yaw_pivot_xyz", fixed);
    pitch_pivot_ = pivot_parameter("pitch_pivot_xyz", fixed);
    if (!valid_frame(body_frame_) || !valid_frame(yaw_frame_) || !valid_frame(pitch_frame_) ||
      body_frame_ == yaw_frame_ || body_frame_ == pitch_frame_ || yaw_frame_ == pitch_frame_ ||
      yaw_joint_.empty() || pitch_joint_.empty() || yaw_joint_ == pitch_joint_)
    {
      throw std::invalid_argument("Frame IDs/joint names must be nonempty and distinct; no leading '/'.");
    }

    body_pub_ = create_publisher<TwistStamped>("cmd_vel_body", 10);
    ready_pub_ = create_publisher<std_msgs::msg::Bool>("frame_ready", 1);
    if (publish_tf_) {
      tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "gimbal_joint_state", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {on_joint(*msg);});
    valid_sub_ = create_subscription<std_msgs::msg::Bool>(
      "gimbal_state_valid", 10,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {on_valid(msg->data);});
    command_sub_ = create_subscription<TwistStamped>(
      "cmd_vel_gimbal", 10,
      [this](TwistStamped::ConstSharedPtr msg) {on_command(*msg);});
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] {on_timer();});
    publish_ready(false);
    if (!geometry_confirmed_) {
      RCLCPP_WARN(get_logger(), "Geometry is unconfirmed: TF and motion output are inhibited.");
    }
  }

private:
  static constexpr std::int64_t kStateGapNs = 50'000'000;
  static constexpr std::int64_t kInputAgeNs = 200'000'000;
  static constexpr std::int64_t kFutureNs = 5'000'000;
  static constexpr std::int64_t kClockDiscontinuityNs = 100'000'000;
  struct Pending {
    TwistStamped message;
    std::int64_t stamp_ns;
    Steady::time_point arrived;
  };

  static bool valid_frame(const std::string & frame)
  {
    return !frame.empty() && frame.front() != '/';
  }

  std::array<double, 3> pivot_parameter(
    const std::string & name, const rcl_interfaces::msg::ParameterDescriptor & descriptor)
  {
    const auto values = declare_parameter<std::vector<double>>(
      name, std::vector<double>{0.0, 0.0, 0.0}, descriptor);
    if (values.size() != 3 || !std::all_of(values.begin(), values.end(),
      [](double value) {return std::isfinite(value);}))
    {
      throw std::invalid_argument(name + " must contain exactly three finite metres.");
    }
    return {{values[0], values[1], values[2]}};
  }

  static std::optional<std::int64_t> stamp_ns(const builtin_interfaces::msg::Time & stamp)
  {
    if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000u) {
      return std::nullopt;
    }
    const auto value = static_cast<std::int64_t>(stamp.sec) * 1'000'000'000 + stamp.nanosec;
    return value > 0 ? std::optional<std::int64_t>(value) : std::nullopt;
  }

  static bool stamp_fresh(std::int64_t stamp, std::int64_t now)
  {
    return stamp > 0 && now > 0 && stamp - now <= kFutureNs && now - stamp <= kInputAgeNs;
  }

  static bool arrival_fresh(Steady::time_point arrived, Steady::time_point now, std::int64_t age)
  {
    return now >= arrived &&
           std::chrono::duration_cast<std::chrono::nanoseconds>(now - arrived).count() < age;
  }

  void publish_ready(bool ready)
  {
    std_msgs::msg::Bool msg;
    msg.data = ready;
    ready_pub_->publish(msg);
  }

  void stop_commands(std::int64_t now_ros, bool move_floor)
  {
    pending_.reset();
    command_active_ = false;
    if (move_floor) {
      minimum_command_ns_ = std::max(minimum_command_ns_, now_ros);
    }
    TwistStamped stop;
    stop.header.frame_id = body_frame_;
    stop.header.stamp = get_clock()->now();
    body_pub_->publish(stop);  // A new safety STOP, never a restamped old command.
  }

  void invalidate_state(const char * reason, std::int64_t now_ros, bool new_clock_epoch = false)
  {
    const bool had_data = have_joint_ || command_active_ || pending_.has_value();
    history_.clear();
    have_joint_ = false;
    have_status_ = false;
    status_valid_ = false;
    // Measurement time normally precedes arrival time. A new state must be
    // received after this invalidation and advance the last measured sample;
    // comparing it to each false Bool's receive time would reject healthy,
    // consistently delayed joint samples forever.
    if (new_clock_epoch) {
      last_measurement_ns_ = 0;
    }
    minimum_joint_ns_ = last_measurement_ns_;
    minimum_command_ns_ = std::max<std::int64_t>(0, now_ros);
    last_command_ns_ = 0;
    stop_commands(now_ros, false);
    publish_ready(false);
    if (had_data) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", reason);
    }
  }

  bool maintain(Steady::time_point now_steady, std::int64_t now_ros)
  {
    bool jumped = false;
    if (clock_started_) {
      const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now_steady - previous_steady_).count();
      const auto ros_elapsed = now_ros - previous_ros_;
      jumped = ros_elapsed < 0 || std::llabs(ros_elapsed - elapsed) > kClockDiscontinuityNs;
    }
    previous_steady_ = now_steady;
    previous_ros_ = now_ros;
    clock_started_ = true;
    if (jumped || now_ros <= 0) {
      if (jumped || have_joint_ || command_active_ || pending_) {
        invalidate_state("ROS clock discontinuity: joint/command history cleared.", now_ros, true);
      }
      return false;
    }
    if ((have_status_ && (!status_valid_ ||
      !arrival_fresh(last_status_arrival_, now_steady, kStateGapNs))) ||
      (have_joint_ && (!arrival_fresh(last_joint_arrival_, now_steady, kStateGapNs) ||
      !stamp_fresh(history_.last()->stamp_ns, now_ros))))
    {
      invalidate_state("Gimbal state interrupted or stale: motion stopped.", now_ros);
    }
    if (pending_ && (!stamp_fresh(pending_->stamp_ns, now_ros) ||
      !arrival_fresh(pending_->arrived, now_steady, kInputAgeNs)))
    {
      stop_commands(now_ros, true);
    }
    if (command_active_ && (!stamp_fresh(active_command_ns_, now_ros) ||
      !arrival_fresh(active_command_arrival_, now_steady, kInputAgeNs)))
    {
      stop_commands(now_ros, true);
    }
    return true;
  }

  bool state_ready(Steady::time_point now_steady, std::int64_t now_ros) const
  {
    return geometry_confirmed_ && have_status_ && status_valid_ && have_joint_ &&
           arrival_fresh(last_status_arrival_, now_steady, kStateGapNs) &&
           arrival_fresh(last_joint_arrival_, now_steady, kStateGapNs) &&
           history_.last() && stamp_fresh(history_.last()->stamp_ns, now_ros);
  }

  void on_valid(bool valid)
  {
    const auto steady = Steady::now();
    const auto ros = get_clock()->now().nanoseconds();
    if (!maintain(steady, ros)) {
      return;
    }
    if (!valid) {
      invalidate_state("Gimbal validity is false: a new state and command are required.", ros);
      return;
    }
    have_status_ = true;
    status_valid_ = true;
    last_status_arrival_ = steady;
    publish_ready(state_ready(steady, ros));
  }

  void on_joint(const sensor_msgs::msg::JointState & msg)
  {
    const auto steady = Steady::now();
    const auto ros = get_clock()->now().nanoseconds();
    if (!maintain(steady, ros) || !geometry_confirmed_) {
      return;
    }
    if (!have_status_ || !status_valid_) {
      return;
    }
    const auto stamp = stamp_ns(msg.header.stamp);
    const auto yaw = std::find(msg.name.begin(), msg.name.end(), yaw_joint_);
    const auto pitch = std::find(msg.name.begin(), msg.name.end(), pitch_joint_);
    if (!stamp || !stamp_fresh(*stamp, ros) || *stamp <= minimum_joint_ns_ ||
      yaw == msg.name.end() || pitch == msg.name.end() ||
      std::count(msg.name.begin(), msg.name.end(), yaw_joint_) != 1 ||
      std::count(msg.name.begin(), msg.name.end(), pitch_joint_) != 1)
    {
      invalidate_state("Invalid joint names or measurement timestamp.", ros);
      return;
    }
    const auto yi = static_cast<std::size_t>(yaw - msg.name.begin());
    const auto pi = static_cast<std::size_t>(pitch - msg.name.begin());
    if (yi >= msg.position.size() || pi >= msg.position.size() ||
      !std::isfinite(msg.position[yi]) || !std::isfinite(msg.position[pi]))
    {
      invalidate_state("Missing or non-finite gimbal joint positions.", ros);
      return;
    }
    if (history_.last() && *stamp == history_.last()->stamp_ns) {
      return;  // Repeated measurements do not refresh state freshness or TF.
    }
    if (!history_.push({*stamp, msg.position[yi], msg.position[pi]})) {
      invalidate_state("Out-of-order joint measurement: history cleared.", ros);
      return;
    }
    have_joint_ = true;
    last_measurement_ns_ = *stamp;
    last_joint_arrival_ = steady;
    if (publish_tf_) {
      publish_sample_tf(msg.header.stamp, *history_.last());
    }
    publish_ready(state_ready(steady, ros));
    if (pending_) {
      resolve_pending(steady, ros);
    }
  }

  void on_command(const TwistStamped & msg)
  {
    const auto steady = Steady::now();
    const auto ros = get_clock()->now().nanoseconds();
    if (!maintain(steady, ros)) {
      return;
    }
    const auto stamp = stamp_ns(msg.header.stamp);
    const fm::PlanarTwist twist{
      msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
      msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z};
    if (!state_ready(steady, ros) || msg.header.frame_id != yaw_frame_ ||
      !stamp || !stamp_fresh(*stamp, ros) || *stamp < minimum_command_ns_ ||
      !fm::finite_planar(twist))
    {
      stop_commands(ros, false);
      return;
    }
    if (*stamp <= last_command_ns_) {
      return;  // Duplicate/late commands never renew a moving output.
    }
    last_command_ns_ = *stamp;
    pending_ = Pending{msg, *stamp, steady};
    resolve_pending(steady, ros);
  }

  void resolve_pending(Steady::time_point now_steady, std::int64_t now_ros)
  {
    if (!pending_) {
      return;
    }
    if (!state_ready(now_steady, now_ros) || !stamp_fresh(pending_->stamp_ns, now_ros) ||
      !arrival_fresh(pending_->arrived, now_steady, kInputAgeNs))
    {
      stop_commands(now_ros, true);
      return;
    }
    if (pending_->stamp_ns > history_.last()->stamp_ns) {
      return;  // Wait for a later measured sample, bounded by freshness limits.
    }
    const auto sample = history_.at(pending_->stamp_ns);
    if (!sample) {
      stop_commands(now_ros, false);  // Before the buffer: no backward extrapolation.
      return;
    }
    const auto & msg = pending_->message;
    const auto body = fm::to_body(
      {msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
        msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z}, sample->yaw, yaw_pivot_);
    if (!body) {
      stop_commands(now_ros, true);
      return;
    }
    TwistStamped output;
    output.header = msg.header;  // Preserve command measurement time exactly.
    output.header.frame_id = body_frame_;
    output.twist.linear.x = body->vx;
    output.twist.linear.y = body->vy;
    output.twist.angular.z = body->wz;
    body_pub_->publish(output);
    command_active_ = true;
    active_command_ns_ = pending_->stamp_ns;
    active_command_arrival_ = pending_->arrived;
    pending_.reset();
  }

  void publish_sample_tf(const builtin_interfaces::msg::Time & stamp, const fm::JointSample & sample)
  {
    geometry_msgs::msg::TransformStamped yaw, pitch;
    yaw.header.stamp = stamp;
    yaw.header.frame_id = body_frame_;
    yaw.child_frame_id = yaw_frame_;
    yaw.transform.translation.x = yaw_pivot_[0];
    yaw.transform.translation.y = yaw_pivot_[1];
    yaw.transform.translation.z = yaw_pivot_[2];
    yaw.transform.rotation.z = std::sin(sample.yaw / 2.0);
    yaw.transform.rotation.w = std::cos(sample.yaw / 2.0);
    pitch.header.stamp = stamp;
    pitch.header.frame_id = yaw_frame_;
    pitch.child_frame_id = pitch_frame_;
    pitch.transform.translation.x = pitch_pivot_[0];
    pitch.transform.translation.y = pitch_pivot_[1];
    pitch.transform.translation.z = pitch_pivot_[2];
    pitch.transform.rotation.y = std::sin(sample.pitch / 2.0);
    pitch.transform.rotation.w = std::cos(sample.pitch / 2.0);
    tf_->sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{yaw, pitch});
  }

  void on_timer()
  {
    const auto steady = Steady::now();
    const auto ros = get_clock()->now().nanoseconds();
    const bool clock_ok = maintain(steady, ros);
    publish_ready(clock_ok && state_ready(steady, ros));
  }

  std::string body_frame_, yaw_frame_, pitch_frame_, yaw_joint_, pitch_joint_;
  bool geometry_confirmed_{false}, publish_tf_{true};
  std::array<double, 3> yaw_pivot_{}, pitch_pivot_{};
  bool have_status_{false}, status_valid_{false}, have_joint_{false};
  bool clock_started_{false}, command_active_{false};
  Steady::time_point last_status_arrival_{}, last_joint_arrival_{}, previous_steady_{};
  Steady::time_point active_command_arrival_{};
  std::int64_t previous_ros_{}, minimum_joint_ns_{}, minimum_command_ns_{}, last_measurement_ns_{};
  std::int64_t last_command_ns_{}, active_command_ns_{};
  fm::JointHistory history_;
  std::optional<Pending> pending_;
  rclcpp::Publisher<TwistStamped>::SharedPtr body_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr valid_sub_;
  rclcpp::Subscription<TwistStamped>::SharedPtr command_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#ifndef RM_CHASSIS_COMM_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<SentryFrameBridge>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("sentry_frame_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
#endif
