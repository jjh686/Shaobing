#ifdef NDEBUG
#undef NDEBUG
#endif
#define RM_CHASSIS_COMM_NO_MAIN
#include "../src/chassis_node.cpp"
#include "../src/sentry_frame_bridge.cpp"
#include <tf2_msgs/msg/tf_message.hpp>
#include <cassert>
#include <thread>
#include <iostream>

class Rig {
public:
  Rig() : start(std::chrono::steady_clock::now()) {
    char name[128]{};assert(::openpty(&master,&slave,name,nullptr,nullptr)==0);
    termios a{};assert(::tcgetattr(slave,&a)==0);::cfmakeraw(&a);assert(::tcsetattr(slave,TCSANOW,&a)==0);
    ::fcntl(master,F_SETFL,::fcntl(master,F_GETFL)|O_NONBLOCK);
    rclcpp::NodeOptions d;
    d.arguments({"--ros-args","-r","__ns:=/sentry_check","-r","cmd_vel_stamped:=cmd_vel_body"});
    d.parameter_overrides({rclcpp::Parameter("connect",true),rclcpp::Parameter("serial_port",std::string(name)),
      rclcpp::Parameter("route",2),rclcpp::Parameter("frame_id","base_footprint"),
      rclcpp::Parameter("require_gimbal_state",true),rclcpp::Parameter("require_frame_ready",true),
      rclcpp::Parameter("use_stamped_commands",true)});
    driver=std::make_shared<rm_chassis_comm::ChassisNode>(d);
    rclcpp::NodeOptions b;b.arguments({"--ros-args","-r","__ns:=/sentry_check"});
    b.parameter_overrides({rclcpp::Parameter("geometry_confirmed",true)});
    bridge=std::make_shared<SentryFrameBridge>(b);
    observer=std::make_shared<rclcpp::Node>("sentry_test_observer");
    publisher=observer->create_publisher<geometry_msgs::msg::TwistStamped>("/sentry_check/cmd_vel_gimbal",10);
    ready=observer->create_subscription<std_msgs::msg::Bool>("/sentry_check/frame_ready",10,
      [this](std_msgs::msg::Bool::ConstSharedPtr m){frame_ready=m->data;});
    output=observer->create_subscription<geometry_msgs::msg::TwistStamped>("/sentry_check/cmd_vel_body",10,
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr m){body.push_back(*m);});
    joint=observer->create_subscription<sensor_msgs::msg::JointState>("/sentry_check/gimbal_joint_state",10,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr m){joints.push_back(*m);});
    tf=observer->create_subscription<tf2_msgs::msg::TFMessage>("/tf",100,
      [this](tf2_msgs::msg::TFMessage::ConstSharedPtr m){for(const auto&t:m->transforms)if(t.child_frame_id=="gimbal_yaw")++yaw_tf;});
    service=observer->create_client<std_srvs::srv::SetBool>("/sentry_check/set_enabled");
    exec.add_node(driver);exec.add_node(bridge);exec.add_node(observer);
  }
  ~Rig(){exec.remove_node(driver);exec.remove_node(bridge);exec.remove_node(observer);driver.reset();bridge.reset();observer.reset();::close(master);::close(slave);}
  uint32_t mcu_now(){return 1000u+static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count());}
  void serial(){
    uint8_t data[2048];ssize_t n;
    while((n=::read(master,data,sizeof(data)))>0)rx.insert(rx.end(),data,data+n);
    while(rx.size()>=RM_FRAME_SIZE){
      rm_command_t c{};
      if(!rm_decode_command(rx.data(),RM_FRAME_SIZE,&c)){rx.erase(rx.begin());continue;}
      last=c;received=mcu_now();commands.push_back(c);rx.erase(rx.begin(),rx.begin()+RM_FRAME_SIZE);
    }
    const auto now=mcu_now();if(!last.session || now-last_tx<10)return;last_tx=now;
    rm_feedback_t f{};f.route=2;f.session=last.session;f.sequence=last.sequence;
    f.uptime_ms=now;f.status=RM_STATUS_READY|RM_STATUS_ACK_VALID;
    uint8_t bytes[RM_FRAME_SIZE];assert(rm_encode_feedback(&f,bytes));assert(::write(master,bytes,sizeof(bytes))==sizeof(bytes));
    if(!send_gimbal)return;
    rm_gimbal_t g{};g.route=2;g.session=last.session;g.sequence=last.sequence;
    g.uptime_ms=now;g.command_received_ms=received;g.sample_ms=replay_sample?frozen_sample:now;
    g.sample_age_ms=static_cast<uint16_t>(now-g.sample_ms);
    g.status=calibrated?7:3;g.yaw=float(3.14159265358979323846/2);g.pitch=0;
    uint8_t joint_bytes[RM_GIMBAL_FRAME_SIZE];
    if(rm_encode_gimbal(&g,joint_bytes))assert(::write(master,joint_bytes,sizeof(joint_bytes))==sizeof(joint_bytes));
  }
  void pump(int ms){auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(ms);
    while(std::chrono::steady_clock::now()<end){exec.spin_some();serial();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    exec.spin_some();serial();}
  bool enable(bool yes){auto req=std::make_shared<std_srvs::srv::SetBool::Request>();req->data=yes;
    assert(service->service_is_ready());auto future=service->async_send_request(req);
    for(int i=0;i<1000&&future.wait_for(std::chrono::seconds(0))!=std::future_status::ready;++i)pump(1);
    assert(future.wait_for(std::chrono::seconds(0))==std::future_status::ready);return future.get()->success;}
  void command(double x=1,double y=0,double w=3.14,const std::string& frame="gimbal_yaw"){
    geometry_msgs::msg::TwistStamped m;m.header.stamp=observer->now()-rclcpp::Duration::from_seconds(.015);
    m.header.frame_id=frame;m.twist.linear.x=x;m.twist.linear.y=y;m.twist.angular.z=w;publisher->publish(m);
  }
  bool moving()const{return !commands.empty()&&commands.back().flags==1&&std::hypot(commands.back().vx,commands.back().vy)>.1;}
  int master=-1,slave=-1;bool frame_ready=false,send_gimbal=true,calibrated=true,replay_sample=false;
  uint32_t received=0,last_tx=0,frozen_sample=0;unsigned yaw_tf=0;
  rm_command_t last{};std::vector<uint8_t>rx;std::vector<rm_command_t>commands;
  std::vector<geometry_msgs::msg::TwistStamped>body;std::vector<sensor_msgs::msg::JointState>joints;
  std::chrono::steady_clock::time_point start;
  std::shared_ptr<rm_chassis_comm::ChassisNode> driver;std::shared_ptr<SentryFrameBridge> bridge;
  std::shared_ptr<rclcpp::Node> observer;rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ready;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr output;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr service;
};
int main(int argc,char**argv){rclcpp::init(argc,argv);{
 Rig r;r.pump(1200);
 std::cerr<<"ROS state: frames="<<r.commands.size()<<" joints="<<r.joints.size()<<" TF="<<r.yaw_tf<<" ready="<<r.frame_ready<<"\n";
 assert(!r.commands.empty()&&!r.joints.empty()&&r.yaw_tf>2&&r.frame_ready);
 for(const auto& c:r.commands)assert(c.flags==0);
 r.command();r.pump(30);assert(!r.moving());assert(r.enable(true));r.pump(30);assert(!r.moving());
 geometry_msgs::msg::TwistStamped delayed;
 delayed.header.frame_id="base_footprint";delayed.header.stamp=r.observer->now()-rclcpp::Duration::from_seconds(.08);
 delayed.twist.linear.x=.5;
 auto direct=r.observer->create_publisher<geometry_msgs::msg::TwistStamped>("/sentry_check/cmd_vel_body",10);
 direct->publish(delayed);r.pump(20);assert(!r.moving());
 for(int i=0;i<10;++i){r.command();r.pump(20);}
 assert(r.moving());assert(std::abs(r.commands.back().vx)<1e-4);
 assert(std::abs(r.commands.back().vy-1)<1e-4&&std::abs(r.commands.back().wz-3.14)<1e-4);
 r.pump(280);assert(!r.moving()); // Original stamp is not refreshed by the bridge.
 r.command(1,0,0,"wrong_frame");r.pump(40);assert(!r.moving());
 r.command();r.pump(40);assert(r.moving());
 r.calibrated=false;r.pump(100);assert(!r.moving()&&!r.frame_ready);assert(!r.enable(true));
 r.calibrated=true;r.pump(150);assert(r.frame_ready&&!r.moving());
 r.command();r.pump(30);assert(!r.moving());assert(r.enable(true));r.pump(20);
 r.command();r.pump(40);assert(r.moving());
 r.replay_sample=true;r.frozen_sample=r.mcu_now();r.pump(150);assert(!r.moving()&&!r.frame_ready);
 r.replay_sample=false;r.pump(150);assert(r.frame_ready&&!r.moving());
 assert(r.enable(true));r.pump(20);r.command();r.pump(40);assert(r.moving());
 r.send_gimbal=false;r.pump(150);assert(!r.moving()&&!r.frame_ready);
 std::cout<<"PASS real ROS nodes + PTY: joint feedback/TF/+90deg/spin/enable/stale/invalid/replay/dropout\n";
 }rclcpp::shutdown();}
