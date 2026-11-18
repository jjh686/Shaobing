#ifdef NDEBUG
#undef NDEBUG
#endif
#include "fake_vel_transform/fake_vel_transform.hpp"
#include <cassert>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <thread>
#include <vector>
#include <iostream>
int main(int argc,char**argv){rclcpp::init(argc,argv);{
 rclcpp::NodeOptions options;options.arguments({"--ros-args","-r","__ns:=/source_check"});
 options.parameter_overrides({rclcpp::Parameter("robot_base_frame","gimbal_yaw"),
  rclcpp::Parameter("input_cmd_vel_topic","nav_in"),rclcpp::Parameter("output_cmd_vel_topic","legacy_out"),
  rclcpp::Parameter("init_spin_speed",3.14)});
 auto source=std::make_shared<fake_vel_transform::FakeVelTransform>(options);
 auto test=std::make_shared<rclcpp::Node>("fake_source_test");
 auto odom=test->create_publisher<nav_msgs::msg::Odometry>("/source_check/odom",10);
 auto plan=test->create_publisher<nav_msgs::msg::Path>("/source_check/local_plan",10);
 auto input=test->create_publisher<geometry_msgs::msg::Twist>("/source_check/nav_in",10);
 std::vector<geometry_msgs::msg::TwistStamped> messages;
 std::vector<geometry_msgs::msg::TransformStamped> fake_tf;
 auto tf_sub=test->create_subscription<tf2_msgs::msg::TFMessage>("/tf",100,
   [&](tf2_msgs::msg::TFMessage::ConstSharedPtr m){for(const auto&t:m->transforms)if(t.child_frame_id=="gimbal_link_fake")fake_tf.push_back(t);});
 auto sub=test->create_subscription<geometry_msgs::msg::TwistStamped>("/source_check/cmd_vel_gimbal_stamped",10,
  [&](geometry_msgs::msg::TwistStamped::ConstSharedPtr m){messages.push_back(*m);});
 rclcpp::executors::SingleThreadedExecutor exec;exec.add_node(source);exec.add_node(test);
 auto pump=[&](int ms){for(int i=0;i<ms;++i){exec.spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}};
 nav_msgs::msg::Odometry o;o.pose.pose.orientation.w=std::cos(.25*3.141592653589793);
 o.pose.pose.orientation.z=std::sin(.25*3.141592653589793);
 pump(500);o.header.stamp=test->now();odom->publish(o);pump(20);
 geometry_msgs::msg::Twist velocity;velocity.linear.x=1;input->publish(velocity);pump(30);
 assert(messages.size()==1&&messages.back().header.frame_id=="gimbal_yaw");
 assert(messages.back().header.stamp==o.header.stamp);
 assert(std::abs(messages.back().twist.linear.x)<1e-6&&std::abs(messages.back().twist.linear.y+1)<1e-6);
 assert(std::abs(messages.back().twist.angular.z-3.14)<1e-6);
 assert(!fake_tf.empty()&&fake_tf.back().header.stamp==o.header.stamp);
 const auto tf_count=fake_tf.size();pump(50);assert(fake_tf.size()==tf_count);
 nav_msgs::msg::Path p;p.header.stamp=test->now();plan->publish(p);pump(10);
 input->publish(velocity);pump(10);
 o.header.stamp=test->now();p.header.stamp=o.header.stamp;odom->publish(o);plan->publish(p);pump(30);
 assert(messages.size()==2);auto stamp=messages.back().header.stamp;
 for(int i=0;i<12;++i){o.header.stamp=test->now();p.header.stamp=o.header.stamp;odom->publish(o);plan->publish(p);pump(25);}
 assert(messages.size()==2&&messages.back().header.stamp==stamp); // No timestamp laundering.
 // Zero arriving after odometry stalls must clear the older moving payload.
 pump(220);geometry_msgs::msg::Twist zero;input->publish(zero);pump(10);
 o.header.stamp=test->now();p.header.stamp=o.header.stamp;odom->publish(o);plan->publish(p);pump(30);
 assert(messages.size()==2);

 // A newly queued command that misses the 200ms source budget must also drop.
 p.header.stamp=test->now();plan->publish(p);pump(5);input->publish(velocity);pump(220);
 o.header.stamp=test->now();p.header.stamp=o.header.stamp;odom->publish(o);plan->publish(p);pump(30);
 assert(messages.size()==2);
 std::cout<<"PASS real navigation source: exact frame/sample stamp/spin and no old-command restamping\n";
 exec.remove_node(source);exec.remove_node(test);
 }rclcpp::shutdown();}
