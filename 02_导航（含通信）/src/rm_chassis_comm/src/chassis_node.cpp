// SPDX-License-Identifier: Apache-2.0
#include "rm_chassis_comm/protocol.h"
#include "rm_chassis_comm/stream_mux.hpp"
#include "rm_chassis_comm/gimbal_clock.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <fcntl.h>
#include <memory>
#include <optional>
#include <pty.h>
#include <poll.h>
#include <random>
#include <stdexcept>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace rm_chassis_comm {
using Steady=std::chrono::steady_clock;
using Time=Steady::time_point;
class ChassisNode : public rclcpp::Node {
public:
  explicit ChassisNode(const rclcpp::NodeOptions & options=rclcpp::NodeOptions()) : Node("chassis_comm",options),boot_(Steady::now()) {
    connect_=fixed<bool>("connect",false);
    if(fixed<bool>("enabled",false)) throw std::invalid_argument("Start disabled; use ~/set_enabled after handshake");
    route_=fixed<int>("route",1);port_=fixed<std::string>("serial_port","/dev/ttyACM0");
    baud_=fixed<int>("baud_rate",115200);hz_=fixed<int>("send_hz",50);
    command_ms_=fixed<int>("command_timeout_ms",200);feedback_ms_=fixed<int>("feedback_timeout_ms",300);
    max_linear_=fixed<double>("max_linear_speed",1.0);max_angular_=fixed<double>("max_angular_speed",3.5);
    frame_=fixed<std::string>("frame_id","base_link");sp_enabled_=fixed<bool>("sp_proxy_enabled",false);
    stamped_commands_=fixed<bool>("use_stamped_commands",false);
    require_gimbal_=fixed<bool>("require_gimbal_state",false);
    require_frame_=fixed<bool>("require_frame_ready",false);
    gimbal_ms_=fixed<int>("gimbal_state_timeout_ms",60);
    frame_ms_=fixed<int>("frame_ready_timeout_ms",100);
    yaw_joint_=fixed<std::string>("yaw_joint_name","gimbal_yaw_joint");
    pitch_joint_=fixed<std::string>("pitch_joint_name","gimbal_pitch_joint");
    if(gimbal_ms_<20||gimbal_ms_>100||frame_ms_<20||frame_ms_>200||
       yaw_joint_.empty()||pitch_joint_.empty()||yaw_joint_==pitch_joint_||
       ((require_gimbal_||require_frame_)&&route_!=2))throw std::invalid_argument("Invalid gimbal state gate");
    if((route_!=1&&route_!=2)||hz_<10||hz_>100||command_ms_<20||command_ms_>250||
      command_ms_<2000.0/hz_||feedback_ms_<50||feedback_ms_>2000||frame_.empty()||
      !std::isfinite(max_linear_)||max_linear_<=0||max_linear_>20||
      !std::isfinite(max_angular_)||max_angular_<=0||max_angular_>100||
      (sp_enabled_&&route_!=2)) throw std::invalid_argument("Invalid chassis transport/rate/timeout/limit configuration");
    baudValue();
    preview_=create_publisher<std_msgs::msg::UInt8MultiArray>("tx_preview",1);
    measured_=create_publisher<geometry_msgs::msg::TwistStamped>("measured_velocity",1);
    diagnostics_=create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics",1);
    proxy_path_=create_publisher<std_msgs::msg::String>("sp_proxy",rclcpp::QoS(1).transient_local());
    joints_=create_publisher<sensor_msgs::msg::JointState>("gimbal_joint_state",10);
    gimbal_valid_pub_=create_publisher<std_msgs::msg::Bool>("gimbal_state_valid",10);
    frame_subscription_=create_subscription<std_msgs::msg::Bool>("frame_ready",1,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        frame_ready_=msg->data;frame_received_=Steady::now();
        if(require_frame_&&!frame_ready_) {enabled_=false;command_.reset();stop_ack_seen_=false;}
      });
    stamped_subscription_=create_subscription<geometry_msgs::msg::TwistStamped>("cmd_vel_stamped",rclcpp::QoS(1),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {stampedCommand(*msg);});
    commands_=create_subscription<geometry_msgs::msg::Twist>("cmd_vel",rclcpp::QoS(1),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr input) { if(!stamped_commands_)command(*input); });
    enable_=create_service<std_srvs::srv::SetBool>("set_enabled",
      [this](std_srvs::srv::SetBool::Request::SharedPtr req,std_srvs::srv::SetBool::Response::SharedPtr res) {
        command_.reset();
        if(!req->data) {enabled_=false;res->success=true;res->message="Disabled; STOP heartbeat";return;}
        if(!ready(Steady::now())) {
          enabled_=false;res->success=false;
          res->message="Requires fresh chassis READY/STOP ACK, configured gimbal/frame gates, and no ESTOP/manual/fault";return;
        }
        enable_stamp_floor_=this->now().nanoseconds();last_command_stamp_=0;
        enabled_=true;res->success=true;res->message="Enabled; waiting for a NEW cmd_vel";
      });
    newSession();
    if(sp_enabled_) openProxy();
    timer_=create_wall_timer(std::chrono::milliseconds(5),[this] {tick();});
    RCLCPP_INFO(get_logger(),"Chassis route=%d, connect=%d; no motion until feedback handshake, enable and new cmd_vel",route_,connect_);
  }
  ~ChassisNode() override {
    enabled_=false;command_.reset();
    if(fd_>=0) {
      // Best effort only. The independent MCU watchdog is the hard stop guarantee.
      if(tx_size_==0) {
        auto bytes=makeCommand(Steady::now());
        const ssize_t stopped=::write(fd_,bytes.data(),bytes.size());
        (void)stopped; // MCU TTL covers a short/failed best-effort shutdown write.
      }
      ::close(fd_);
    }
    if(pty_master_>=0)::close(pty_master_);
    if(pty_slave_>=0)::close(pty_slave_);
  }
private:
  template<class T>T fixed(const std::string & name,const T & value) {
    rcl_interfaces::msg::ParameterDescriptor d;d.read_only=true;
    return declare_parameter<T>(name,value,d);
  }
  struct Command {double x,y,w;Time received;};
  uint32_t uptime(Time now) const {
    return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now-boot_).count());
  }
  void newSession() {
    std::random_device random;
    session_=static_cast<uint32_t>(random());if(!session_)session_=1;
    sequence_=0;have_feedback_=false;enabled_=false;command_.reset();
    sent_={};stop_ack_seen_=false;tx_expiry_.reset();latest_expiry_.reset();
    last_uptime_.reset();mux_.reset();sp_parser_.reset();sp_pending_=false;
    gimbal_clock_.reset();gimbal_sample_.reset();gimbal_received_.reset();
    frame_ready_=false;frame_received_.reset();
    tx_size_=tx_offset_=0;next_chassis_=Time{};
  }
  speed_t baudValue() const {
    switch(baud_) {case 115200:return B115200;case 230400:return B230400;
      case 460800:return B460800;case 921600:return B921600;
      default:throw std::invalid_argument("baud_rate: 115200/230400/460800/921600");}
  }
  bool ready(Time now) const {
    if(fd_<0||!have_feedback_||!feedback_time_||
      std::chrono::duration_cast<std::chrono::milliseconds>(now-*feedback_time_).count()>=feedback_ms_)return false;
    if(require_gimbal_&&!gimbalFresh(now))return false;
    if(require_frame_&&(!frame_ready_||!frame_received_||now-*frame_received_>=std::chrono::milliseconds(frame_ms_)))return false;
    const auto forbidden=RM_STATUS_ESTOP|RM_STATUS_TIMEOUT|RM_STATUS_MANUAL;
    return feedback_.route==route_&&feedback_.session==session_&&
      (feedback_.status&RM_STATUS_READY)&&(feedback_.status&RM_STATUS_ACK_VALID)&&
      !(feedback_.status&forbidden)&&feedback_.fault_mask==0&&stop_ack_seen_;
  }
  void command(const geometry_msgs::msg::Twist & msg) {
    if(!enabled_)return;
    const std::array<double,6> components{msg.linear.x,msg.linear.y,msg.linear.z,
      msg.angular.x,msg.angular.y,msg.angular.z};
    if(!std::all_of(components.begin(),components.end(),[](double n){return std::isfinite(n);})||
      std::abs(msg.linear.z)>1e-9||std::abs(msg.angular.x)>1e-9||std::abs(msg.angular.y)>1e-9) {
      command_.reset();RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Rejected non-planar or non-finite cmd_vel");return;
    }
    const double norm=std::hypot(msg.linear.x,msg.linear.y);
    const double factor=norm>max_linear_?max_linear_/norm:1.0;
    command_=Command{msg.linear.x*factor,msg.linear.y*factor,
      std::clamp(msg.angular.z,-max_angular_,max_angular_),Steady::now()};
  }
  void stampedCommand(const geometry_msgs::msg::TwistStamped & msg) {
    if(!stamped_commands_||!enabled_)return;
    const auto stamp=rclcpp::Time(msg.header.stamp,RCL_ROS_TIME);
    const auto age=(this->now()-stamp).nanoseconds();
    if(msg.header.frame_id!=frame_||stamp.nanoseconds()<=0||age<0||age>=int64_t(command_ms_)*1000000) {
      command_.reset();return;
    }
    if(stamp.nanoseconds()<enable_stamp_floor_ || stamp.nanoseconds()<=last_command_stamp_)return;
    last_command_stamp_=stamp.nanoseconds();
    command(msg.twist);
    if(command_)command_->received-=std::chrono::nanoseconds(age);
  }
  static double steadyMs(Time value) {
    return std::chrono::duration<double,std::milli>(value.time_since_epoch()).count();
  }
  bool gimbalFresh(Time now) const {
    return gimbal_sample_&&gimbal_received_&&now-*gimbal_received_<std::chrono::milliseconds(gimbal_ms_)&&
      now-*gimbal_sample_<std::chrono::milliseconds(gimbal_ms_);
  }
  void invalidateGimbal() {
    gimbal_sample_.reset();gimbal_received_.reset();
    if(require_gimbal_) {enabled_=false;command_.reset();stop_ack_seen_=false;}
  }
  void gimbalFeedback(const rm_gimbal_t & value) {
    ++decoded_gimbal_;
    if(value.route!=route_||value.session!=session_)return;
    const auto & sent=sent_[value.sequence%sent_.size()];
    const auto arrived=Steady::now();
    if(!sent.valid||sent.sequence!=value.sequence||arrived-sent.at>=std::chrono::milliseconds(100))return;
    last_gimbal_status_=value.status;
    const auto required=RM_GIMBAL_YAW_VALID|RM_GIMBAL_PITCH_VALID|RM_GIMBAL_CALIBRATED;
    if((value.status&required)!=required || (value.status&RM_GIMBAL_FAULT)) {invalidateGimbal();return;}
    auto mapped=gimbal_clock_.observe(value.uptime_ms,value.command_received_ms,value.sample_ms,
      steadyMs(sent.at),steadyMs(arrived));
    if(!mapped)return; // Repeated or out-of-order samples cannot renew validity.
    gimbal_uncertainty_ms_=mapped->uncertainty_ms;
    const auto age=steadyMs(arrived)-mapped->sample_host_ms;
    if(age<0.0||age>=gimbal_ms_) {invalidateGimbal();return;}
    gimbal_received_=arrived;
    gimbal_sample_=Time(std::chrono::duration_cast<Steady::duration>(
      std::chrono::duration<double,std::milli>(mapped->sample_host_ms)));
    sensor_msgs::msg::JointState msg;
    msg.header.stamp=this->now()-rclcpp::Duration::from_seconds(age*0.001);
    msg.header.frame_id=frame_;msg.name={yaw_joint_,pitch_joint_};
    msg.position={value.yaw,value.pitch};msg.velocity={value.yaw_rate,value.pitch_rate};
    joints_->publish(msg);
  }
  std::array<uint8_t,RM_FRAME_SIZE> makeCommand(Time now,std::optional<Time> * expiry=nullptr) {
    if(expiry)expiry->reset();
    rm_command_t command{};command.route=static_cast<uint8_t>(route_);
    command.session=session_;command.sequence=sequence_++;command.uptime_ms=uptime(now);
    if(enabled_&&ready(now)&&command_&&
      std::chrono::duration_cast<std::chrono::milliseconds>(now-command_->received).count()<command_ms_) {
      const auto age=std::chrono::duration_cast<std::chrono::milliseconds>(now-command_->received).count();
      const auto remaining=command_ms_-age;
      if(remaining>=20) {
        command.flags=RM_FLAG_ENABLE;command.ttl_ms=static_cast<uint16_t>(remaining);
        command.vx=static_cast<float>(command_->x);command.vy=static_cast<float>(command_->y);command.wz=static_cast<float>(command_->w);
        if(expiry)*expiry=command_->received+std::chrono::milliseconds(command_ms_);
      }
    }
    std::array<uint8_t,RM_FRAME_SIZE> out{};
    if(!rm_encode_command(&command,out.data()))throw std::runtime_error("Command encoding failed");
    return out;
  }
  void feedback(const rm_feedback_t & value) {
    ++decoded_feedback_;
    if(value.route!=route_||value.session!=session_||!(value.status&RM_STATUS_ACK_VALID))return;
    const auto & sent=sent_[value.sequence%sent_.size()];
    if(!sent.valid||sent.sequence!=value.sequence)return;
    if(last_uptime_) {
      const uint32_t delta=value.uptime_ms-*last_uptime_;
      if(delta==0||delta>=0x80000000U)return; // A replayed heartbeat cannot renew freshness.
    }
    last_uptime_=value.uptime_ms;feedback_=value;feedback_time_=Steady::now();have_feedback_=true;
    if(sent.stop)stop_ack_seen_=true;
    if(value.status&RM_STATUS_MEASURED) {
      geometry_msgs::msg::TwistStamped msg;msg.header.stamp=this->now();msg.header.frame_id=frame_;
      msg.twist.linear.x=value.vx;msg.twist.linear.y=value.vy;msg.twist.angular.z=value.wz;
      measured_->publish(msg);
    }
  }
  void openProxy() {
    char name[128]{};
    if(::openpty(&pty_master_,&pty_slave_,name,nullptr,nullptr)<0)throw std::runtime_error("openpty failed");
    termios attrs{};
    if(::tcgetattr(pty_slave_,&attrs)<0)throw std::runtime_error("PTY attributes failed");
    ::cfmakeraw(&attrs);::tcsetattr(pty_slave_,TCSANOW,&attrs);
    ::fcntl(pty_master_,F_SETFL,::fcntl(pty_master_,F_GETFL)|O_NONBLOCK);
    ::fcntl(pty_slave_,F_SETFL,::fcntl(pty_slave_,F_GETFL)|O_NONBLOCK);
    std_msgs::msg::String msg;msg.data=name;proxy_path_->publish(msg);
    RCLCPP_INFO(get_logger(),"SP vision virtual serial: %s (this node owns physical USB)",name);
  }
  void openSerial(Time now) {
    if(!connect_||fd_>=0||now<retry_after_)return;
    retry_after_=now+std::chrono::seconds(1);
    int candidate=::open(port_.c_str(),O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if(candidate<0)return;
    termios attributes{};
    if(::flock(candidate,LOCK_EX|LOCK_NB)<0||::ioctl(candidate,TIOCEXCL)<0||
      ::tcgetattr(candidate,&attributes)<0) {::close(candidate);return;}
    ::cfmakeraw(&attributes);::cfsetispeed(&attributes,baudValue());::cfsetospeed(&attributes,baudValue());
    attributes.c_cflag|=CLOCAL|CREAD;attributes.c_cflag&=~(CSTOPB|PARENB|CRTSCTS);
    attributes.c_cc[VMIN]=0;attributes.c_cc[VTIME]=0;
    if(::tcsetattr(candidate,TCSANOW,&attributes)<0) {::close(candidate);return;}
    ::tcflush(candidate,TCIOFLUSH);fd_=candidate;newSession();feedback_time_.reset();
    if(pty_slave_>=0)::tcflush(pty_slave_,TCIOFLUSH);
    RCLCPP_INFO(get_logger(),"USB connected; STOP handshake only, chassis remains disabled");
  }
  void disconnect(Time now) {
    if(fd_>=0)::close(fd_);
    fd_=-1;newSession();feedback_time_.reset();retry_after_=now+std::chrono::seconds(1);
    RCLCPP_WARN(get_logger(),"USB disconnected; re-enable required after reconnection");
  }
  void readSerial(Time now) {
    if(fd_<0)return;
    pollfd poll_state{fd_,POLLIN,0};
    if(::poll(&poll_state,1,0)>0&&(poll_state.revents&(POLLHUP|POLLERR|POLLNVAL))) {
      disconnect(now);return;
    }
    std::array<uint8_t,256> bytes{};
    for(unsigned iteration=0;iteration<4;++iteration) {
      const ssize_t size=::read(fd_,bytes.data(),bytes.size());
      if(size>0) {
        received_bytes_+=static_cast<uint64_t>(size);
        for(ssize_t i=0;i<size;++i) mux_.push(bytes[static_cast<size_t>(i)],
          [this](const rm_feedback_t & f){feedback(f);},
          [this](const uint8_t * data,size_t n){
            if(pty_master_>=0) {
              // One nonblocking whole-frame enqueue. Drop if the vision reader stalls;
              // never append another feedback after a partial write.
              const ssize_t written=::write(pty_master_,data,n);
              if(written>0&&static_cast<size_t>(written)!=n) {::tcflush(pty_slave_,TCIOFLUSH);}
            }
          },[this](const rm_gimbal_t & g){gimbalFeedback(g);},sp_enabled_);
      } else if(size<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR) {disconnect(now);return;}
      else return;
    }
  }
  void readProxy() {
    if(pty_master_<0)return;
    std::array<uint8_t,256> bytes{};
    for(unsigned iteration=0;iteration<4;++iteration) {
      const ssize_t size=::read(pty_master_,bytes.data(),bytes.size());
      if(size<=0)return;
      for(ssize_t i=0;i<size;++i) {
        std::array<uint8_t,29> packet{};
        if(sp_parser_.push(bytes[static_cast<size_t>(i)],packet)) {
          sp_command_=packet;sp_pending_=true;sp_received_=Steady::now();
        }
      }
    }
  }
  void queue(const uint8_t * bytes,size_t n,Time now,std::optional<Time> expiry=std::nullopt) {
    std::copy(bytes,bytes+n,tx_.begin());tx_size_=n;tx_offset_=0;tx_started_=now;
    tx_expiry_=expiry;
  }
  void flush(Time now) {
    if(fd_<0||!tx_size_)return;
    if(tx_expiry_) {
      const auto remaining=std::chrono::duration_cast<std::chrono::milliseconds>(*tx_expiry_-now).count();
      if(!enabled_||!ready(now)||!command_||remaining<20||
        (tx_offset_>0&&now-tx_started_>=std::chrono::milliseconds(5))) {
        if(tx_offset_>0) {disconnect(now);return;}
        tx_size_=tx_offset_=0;tx_expiry_.reset();chassis_pending_=false;next_chassis_=Time{};return;
      }
      if(tx_offset_==0) {
        rm_command_t command{};
        if(!rm_decode_command(tx_.data(),tx_size_,&command))throw std::runtime_error("Pending command corrupted");
        command.ttl_ms=static_cast<uint16_t>(remaining);
        if(!rm_encode_command(&command,tx_.data()))throw std::runtime_error("Pending TTL encoding failed");
      }
    }
    if(now-tx_started_>=std::chrono::milliseconds(50)) {disconnect(now);return;}
    const ssize_t result=::write(fd_,tx_.data()+tx_offset_,tx_size_-tx_offset_);
    if(result>0) {
      tx_offset_+=static_cast<size_t>(result);
      if(tx_offset_==tx_size_) {
        if(tx_size_==RM_FRAME_SIZE&&tx_[0]==0x5a) {
          rm_command_t command{};
          if(rm_decode_command(tx_.data(),tx_size_,&command))
            sent_[command.sequence%sent_.size()]={true,command.sequence,command.flags==0,Steady::now()};
        }
        tx_size_=tx_offset_=0;tx_expiry_.reset();
      }
    } else if(result<0&&errno!=EAGAIN&&errno!=EWOULDBLOCK&&errno!=EINTR)disconnect(now);
  }
  void diagnose(Time now) {
    if(now<next_diagnostic_)return;
    next_diagnostic_=now+std::chrono::seconds(1);
    diagnostic_msgs::msg::DiagnosticArray array;array.header.stamp=this->now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name=get_fully_qualified_name();status.hardware_id=port_;
    status.level=ready(now)?0:1;
    status.message=!connect_?"Preview: hardware not opened":fd_<0?"USB disconnected":
      require_gimbal_&&!gimbalFresh(now)?"Waiting for calibrated, fresh gimbal joints":
      require_frame_&&(!frame_ready_||!frame_received_||now-*frame_received_>=std::chrono::milliseconds(frame_ms_))?
        "Waiting for measured geometry and fresh frame_ready":
      !ready(now)?"Waiting for actual chassis READY/STOP ACK":enabled_?"Enabled (source stamp watchdog applies)":"Ready, disabled";
    auto add=[&](const std::string & k,const std::string & v) {
      diagnostic_msgs::msg::KeyValue item;item.key=k;item.value=v;status.values.push_back(item);
    };
    add("route",std::to_string(route_));add("enabled",enabled_?"true":"false");
    add("frame_id",frame_);add("use_stamped_commands",stamped_commands_?"true":"false");
    add("require_gimbal_state",require_gimbal_?"true":"false");
    add("require_frame_ready",require_frame_?"true":"false");
    add("session",std::to_string(session_));add("fault_mask",have_feedback_?std::to_string(feedback_.fault_mask):"unknown");
    add("received_bytes",std::to_string(received_bytes_));
    add("decoded_feedback",std::to_string(decoded_feedback_));
    add("decoded_gimbal",std::to_string(decoded_gimbal_));
    add("gimbal_status",std::to_string(last_gimbal_status_));
    add("gimbal_fresh",gimbalFresh(now)?"true":"false");
    add("gimbal_clock_uncertainty_ms",std::to_string(gimbal_uncertainty_ms_));
    add("frame_ready",frame_ready_?"true":"false");
    add("feedback_age_ms",feedback_time_?std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(now-*feedback_time_).count()):"unknown");
    array.status.push_back(status);diagnostics_->publish(array);
  }
  void tick() {
    auto time=Steady::now();openSerial(time);readSerial(time);readProxy();time=Steady::now();
    const auto ros_time=this->now().nanoseconds();
    if(last_ros_time_&&last_clock_check_) {
      const auto elapsed=std::chrono::duration_cast<std::chrono::nanoseconds>(time-*last_clock_check_).count();
      if(ros_time<*last_ros_time_ || std::llabs((ros_time-*last_ros_time_)-elapsed)>100000000LL) {
        invalidateGimbal();gimbal_clock_.reset();frame_ready_=false;command_.reset();enabled_=false;
      }
    }
    last_ros_time_=ros_time;last_clock_check_=time;
    if(time>=next_gimbal_valid_) {
      std_msgs::msg::Bool valid;valid.data=gimbalFresh(time);gimbal_valid_pub_->publish(valid);
      next_gimbal_valid_=time+std::chrono::milliseconds(10);
    }
    if(enabled_&&!ready(time)) {
      enabled_=false;command_.reset();stop_ack_seen_=false;sent_={};
    }
    flush(time);
    if(time>=next_chassis_) {
      next_chassis_=time+std::chrono::microseconds(1000000/hz_);
      latest_chassis_=makeCommand(time,&latest_expiry_);chassis_pending_=true;
      std_msgs::msg::UInt8MultiArray msg;msg.data.assign(latest_chassis_.begin(),latest_chassis_.end());preview_->publish(msg);
    }
    if(fd_>=0&&!tx_size_&&chassis_pending_) {
      queue(latest_chassis_.data(),latest_chassis_.size(),time,latest_expiry_);chassis_pending_=false;flush(time);
    }
    if(fd_>=0&&!tx_size_&&sp_pending_) {
      if(time-sp_received_<std::chrono::milliseconds(50)) {
        queue(sp_command_.data(),sp_command_.size(),time);flush(time);
      }
      sp_pending_=false;
    }
    diagnose(time);
  }
  bool connect_=false,enabled_=false,sp_enabled_=false,have_feedback_=false;
  int route_=1,baud_=115200,hz_=50,command_ms_=200,feedback_ms_=300;
  double max_linear_=1,max_angular_=3.5;
  std::string port_,frame_;
  int fd_=-1,pty_master_=-1,pty_slave_=-1;
  uint32_t session_=0;uint16_t sequence_=0;
  uint64_t received_bytes_=0,decoded_feedback_=0,decoded_gimbal_=0;
  bool stamped_commands_=false,require_gimbal_=false,require_frame_=false,frame_ready_=false;
  int64_t enable_stamp_floor_=0,last_command_stamp_=0;
  int gimbal_ms_=60,frame_ms_=100;uint16_t last_gimbal_status_=0;
  double gimbal_uncertainty_ms_=0;
  std::string yaw_joint_,pitch_joint_;
  GimbalClock gimbal_clock_;
  std::optional<Time> gimbal_sample_,gimbal_received_,frame_received_,last_clock_check_;
  std::optional<int64_t> last_ros_time_;
  Time next_gimbal_valid_{};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gimbal_valid_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr frame_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr stamped_subscription_;
  struct Sent {bool valid=false;uint16_t sequence=0;bool stop=false;Time at{};};
  std::array<Sent,128> sent_{};bool stop_ack_seen_=false;
  rm_feedback_t feedback_{};
  std::optional<uint32_t> last_uptime_;
  std::optional<Time> feedback_time_;
  std::optional<Time> tx_expiry_,latest_expiry_;
  std::optional<Command> command_;
  Time boot_,retry_after_{},next_chassis_{},next_diagnostic_{},tx_started_{},sp_received_{};
  StreamMux<43> mux_;SpCommandParser sp_parser_;
  std::array<uint8_t,43> tx_{};size_t tx_size_=0,tx_offset_=0;
  std::array<uint8_t,RM_FRAME_SIZE> latest_chassis_{};
  std::array<uint8_t,29> sp_command_{};
  bool chassis_pending_=false,sp_pending_=false;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr commands_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr preview_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr measured_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr proxy_path_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace rm_chassis_comm
#ifndef RM_CHASSIS_COMM_NO_MAIN
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);int result=0;
  try {rclcpp::spin(std::make_shared<rm_chassis_comm::ChassisNode>());}
  catch(const std::exception & e) {RCLCPP_ERROR(rclcpp::get_logger("chassis_comm"),"%s",e.what());result=1;}
  rclcpp::shutdown();return result;
}

#endif
