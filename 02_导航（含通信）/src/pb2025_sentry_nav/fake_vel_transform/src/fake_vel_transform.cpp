// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "fake_vel_transform/fake_vel_transform.hpp"

#include <stdexcept>
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

constexpr double EPSILON = 1e-5;
constexpr double CONTROLLER_TIMEOUT = 0.5;

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_link");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_link_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("local_plan_topic", "local_plan");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<float>("init_spin_speed", 0.0);
  const auto stamped_topic=this->declare_parameter<std::string>("output_stamped_topic","cmd_vel_gimbal_stamped");
  stamped_timeout_ms_=this->declare_parameter<double>("stamped_command_timeout_ms",200.0);
  if(stamped_timeout_ms_<20.0 || stamped_timeout_ms_>500.0)
    throw std::invalid_argument("Invalid stamped command timeout");
  stamped_pub_=this->create_publisher<geometry_msgs::msg::TwistStamped>(stamped_topic,1);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("local_plan_topic", local_plan_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_filter_.subscribe(this, odom_topic_);
  local_plan_sub_filter_.subscribe(this, local_plan_topic_);
  odom_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));
  local_plan_sub_filter_.registerCallback(
    std::bind(&FakeVelTransform::localPlanCallback, this, std::placeholders::_1));

  // In Navigation2 Humble release, the velocity is published by the controller without timestamped.
  // We consider the velocity is published at the same time as local_plan.
  // Therefore, we use ApproximateTime policy to synchronize `cmd_vel` and `odometry`.
  sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(100), odom_sub_filter_, local_plan_sub_filter_);
  sync_->registerCallback(
    std::bind(&FakeVelTransform::syncCallback, this, std::placeholders::_1, std::placeholders::_2));

  // 50Hz Timer to send transform from `robot_base_frame` to `fake_robot_base_frame`
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_ = msg->data;
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // NOTE: Haven't synced with local_plan
  if ((this->now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
    angle_stamp_=rclcpp::Time(msg->header.stamp,RCL_ROS_TIME);have_angle_stamp_=true;
  }
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  command_received_=std::chrono::steady_clock::now();++command_generation_;
  const bool is_zero_vel = std::abs(msg->linear.x) < EPSILON && std::abs(msg->linear.y) < EPSILON &&
                           std::abs(msg->angular.z) < EPSILON;
  if (
    is_zero_vel ||
    (this->now() - last_controller_activate_time_).seconds() > CONTROLLER_TIMEOUT) {
    // A direct STOP/new input must discard any earlier buffered motion.
    latest_cmd_vel_.reset();
    // If received velocity cannot be synchronized, publish it directly
    auto aft_tf_vel = transformVelocity(msg, current_robot_base_angle_);
    cmd_vel_chassis_pub_->publish(aft_tf_vel);
    if(have_angle_stamp_)publishStamped(aft_tf_vel,angle_stamp_);
  } else {
    latest_cmd_vel_ = msg;
  }
}

void FakeVelTransform::localPlanCallback(const nav_msgs::msg::Path::ConstSharedPtr & /*msg*/)
{
  // Consider nav2_controller_server is activated when receiving local_plan
  last_controller_activate_time_ = this->now();
}

void FakeVelTransform::syncCallback(
  const nav_msgs::msg::Odometry::ConstSharedPtr & odom_msg,
  const nav_msgs::msg::Path::ConstSharedPtr & /*local_plan_msg*/)
{
  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  geometry_msgs::msg::Twist::SharedPtr current_cmd_vel;
  {
    if (!latest_cmd_vel_) {
      return;
    }
    current_cmd_vel = latest_cmd_vel_;
    latest_cmd_vel_.reset();  // Consume once; a later plan cannot restamp it.
  }
  if(std::chrono::duration<double,std::milli>(
      std::chrono::steady_clock::now()-command_received_).count()>stamped_timeout_ms_) {
    cmd_vel_chassis_pub_->publish(geometry_msgs::msg::Twist{});
    return;
  }

  current_robot_base_angle_ = tf2::getYaw(odom_msg->pose.pose.orientation);
  float yaw_diff = current_robot_base_angle_;
  geometry_msgs::msg::Twist aft_tf_vel = transformVelocity(current_cmd_vel, yaw_diff);

  cmd_vel_chassis_pub_->publish(aft_tf_vel);
  angle_stamp_=rclcpp::Time(odom_msg->header.stamp,RCL_ROS_TIME);have_angle_stamp_=true;
  publishStamped(aft_tf_vel,angle_stamp_);
}

void FakeVelTransform::publishStamped(
  const geometry_msgs::msg::Twist & velocity,const rclcpp::Time & stamp)
{
  // A new odometry/local-plan pair must not refresh an old velocity command.
  // Preserve the time of the orientation actually used for this conversion.
  if(command_generation_==stamped_generation_ || stamp.nanoseconds()<=0)return;
  const auto age_ms=std::chrono::duration<double,std::milli>(
    std::chrono::steady_clock::now()-command_received_).count();
  const auto source_age_ms=(this->now()-stamp).seconds()*1000.0;
  if(age_ms>stamped_timeout_ms_ || source_age_ms>stamped_timeout_ms_ || source_age_ms < -5.0)return;
  geometry_msgs::msg::TwistStamped message;
  message.header.stamp=stamp;message.header.frame_id=robot_base_frame_;message.twist=velocity;
  stamped_pub_->publish(message);stamped_generation_=command_generation_;
}

void FakeVelTransform::publishTransform()
{
  if(!have_angle_stamp_ || angle_stamp_.nanoseconds()<=0 ||
      angle_stamp_.nanoseconds()==published_tf_stamp_)return;
  const auto age=(this->now()-angle_stamp_).seconds()*1000.0;
  if(age<0.0 || age>stamped_timeout_ms_)return;
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = angle_stamp_;
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -current_robot_base_angle_);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
  published_tf_stamp_=angle_stamp_.nanoseconds();
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist::SharedPtr & twist, float yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist->angular.z + spin_speed_;
  aft_tf_vel.linear.x = twist->linear.x * cos(yaw_diff) + twist->linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist->linear.x * sin(yaw_diff) + twist->linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
