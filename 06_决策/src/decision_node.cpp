// Copyright 2026 Qingyun contributors
// SPDX-License-Identifier: Apache-2.0
// New entry using the FZSD/COD ROS -> blackboard -> BT -> intent architecture.
#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rm2027_decision/msg/decision_intent.hpp>
#include <rm2027_decision/msg/decision_state.hpp>
#include "rm2027_decision/policy.hpp"
#include "rm2027_decision/freshness.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <std_msgs/msg/string.hpp>
#include <unordered_set>

namespace rm2027_decision {
using WireState = msg::DecisionState;

struct Context {
  explicit Context(Config cfg) : policy(cfg) {}
  Policy policy;
  State state;
  Intent intent{Mode::Disabled,"not_evaluated",""};
  uint8_t vision_status = WireState::VISION_UNKNOWN;
};

class ReadState : public BT::SyncActionNode {
public:
  ReadState(const std::string & name, const BT::NodeConfig & cfg,
            std::shared_ptr<Context> ctx)
    : BT::SyncActionNode(name,cfg), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {BT::OutputPort<State>("state")}; }
  BT::NodeStatus tick() override {
    setOutput("state",ctx_->state);
    return BT::NodeStatus::SUCCESS;
  }
private:
  std::shared_ptr<Context> ctx_;
};

class SelectIntent : public BT::SyncActionNode {
public:
  SelectIntent(const std::string & name, const BT::NodeConfig & cfg,
               std::shared_ptr<Context> ctx)
    : BT::SyncActionNode(name,cfg), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() {
    return {BT::InputPort<State>("state"),BT::OutputPort<Intent>("intent")};
  }
  BT::NodeStatus tick() override {
    const auto state = getInput<State>("state");
    if (!state) throw BT::RuntimeError("Missing normalized state");
    ctx_->intent = ctx_->policy.evaluate(state.value());
    setOutput("intent",ctx_->intent);
    return BT::NodeStatus::SUCCESS;
  }
private:
  std::shared_ptr<Context> ctx_;
};

class PublishIntent : public BT::SyncActionNode {
public:
  using Publisher = rclcpp::Publisher<msg::DecisionIntent>;
  using ModePublisher = rclcpp::Publisher<std_msgs::msg::String>;
  PublishIntent(const std::string & name, const BT::NodeConfig & cfg,
                Publisher::SharedPtr pub, ModePublisher::SharedPtr mode_pub,
                rclcpp::Clock::SharedPtr clock,
                std::shared_ptr<Context> ctx)
    : BT::SyncActionNode(name,cfg),pub_(std::move(pub)),mode_pub_(std::move(mode_pub)),
      clock_(std::move(clock)), ctx_(std::move(ctx)) {}
  static BT::PortsList providedPorts() { return {BT::InputPort<Intent>("intent")}; }
  BT::NodeStatus tick() override {
    const auto result = getInput<Intent>("intent");
    if (!result) throw BT::RuntimeError("Missing decision intent");
    const auto & in = result.value();
    msg::DecisionIntent out;
    out.header.stamp = clock_->now();
    out.mode = mode_name(in.mode); out.reason = in.reason; out.target = in.target;
    out.fire_allowed = in.fire_allowed;
    out.free_revive_requested = in.free_revive_requested;
    out.team_situation = in.team_situation;
    out.coverage_requested = in.coverage_requested;
    out.vision_status = ctx_->vision_status;
    out.vision_mode_request = in.vision_mode_request;
    pub_->publish(out);
    std_msgs::msg::String mode;
    mode.data = in.vision_mode_request.empty() ? "idle" : in.vision_mode_request;
    mode_pub_->publish(mode);
    return BT::NodeStatus::SUCCESS;
  }
private:
  Publisher::SharedPtr pub_;
  ModePublisher::SharedPtr mode_pub_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<Context> ctx_;
};

class DecisionNode : public rclcpp::Node {
public:
  DecisionNode() : Node("rm2027_tree") {
    const auto profile = fixedParameter<std::string>("profile","rmul_3v3");
    Config cfg;
    if (profile == "rmul_3v3") cfg.profile = Profile::Rmul3v3;
    else if (profile == "rmuc") cfg.profile = Profile::Rmuc;
    else if (profile != "unconfigured") throw std::invalid_argument("profile: rmul_3v3/rmuc/unconfigured");
    enabled_ = fixedParameter<bool>("enabled",false);
    map_frame_ = fixedParameter<std::string>("map_frame","map");
    if (map_frame_.empty() || map_frame_.size() > 64) throw std::invalid_argument("map_frame required, max 64 characters");
    timeout_ = fixedParameter<double>("source_timeout",2.5);
    if (!std::isfinite(timeout_) || timeout_ <= 0) throw std::invalid_argument("source_timeout > 0 required");
    cfg.low_hp_ratio = fixedParameter<double>("low_hp_ratio",0.30);
    cfg.resume_hp_ratio = fixedParameter<double>("resume_hp_ratio",0.80);
    cfg.max_task_risk = fixedParameter<double>("max_task_risk",0.50);
    cfg.max_recovery_risk = fixedParameter<double>("max_recovery_risk",0.50);
    cfg.cautious_capture_risk = fixedParameter<double>("cautious_capture_risk",0.25);
    config_=cfg;
    ctx_ = std::make_shared<Context>(cfg);
    pub_ = create_publisher<msg::DecisionIntent>("intent",rclcpp::QoS(1));
    mode_pub_ = create_publisher<std_msgs::msg::String>("vision_mode_request",rclcpp::QoS(1));
    sub_ = create_subscription<WireState>("state",rclcpp::QoS(1).best_effort(),
      [this](WireState::ConstSharedPtr data) { receive(*data); });
    factory_.registerBuilder<ReadState>("ReadState",[this](const auto & n,const auto & c) {
      return std::make_unique<ReadState>(n,c,ctx_);
    });
    factory_.registerBuilder<SelectIntent>("SelectIntent",[this](const auto & n,const auto & c) {
      return std::make_unique<SelectIntent>(n,c,ctx_);
    });
    factory_.registerBuilder<PublishIntent>("PublishIntent",[this](const auto & n,const auto & c) {
      return std::make_unique<PublishIntent>(n,c,pub_,mode_pub_,get_clock(),ctx_);
    });
    const auto file = fixedParameter<std::string>("tree_file","");
    if (file.empty()) throw std::invalid_argument("tree_file required; use intents.launch.py");
    tree_ = factory_.createTreeFromFile(file);
    timer_ = create_wall_timer(std::chrono::milliseconds(100),[this] { tick(); });
    RCLCPP_INFO(get_logger(),"Intent inspection only: profile=%s, enabled=%d",profile.c_str(),enabled_);
  }
private:
  template<class T> T fixedParameter(const std::string & name, const T & value) {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only=true;
    return declare_parameter<T>(name,value,descriptor);
  }
  static int64_t ns(const builtin_interfaces::msg::Time & value) {
    if (value.sec < 0 || value.nanosec >= 1000000000U) return -1;
    return int64_t(value.sec)*1000000000LL+value.nanosec;
  }
  static std::array<int64_t,14> stamps(const WireState & s) {
    return {ns(s.game_stamp),ns(s.robot_stamp),ns(s.ammunition_stamp),
            ns(s.localization_stamp),ns(s.objectives_stamp),ns(s.sentry_stamp),
            ns(s.heavy.health_stamp),ns(s.heavy.position_stamp),
            ns(s.infantry.health_stamp),ns(s.infantry.position_stamp),ns(s.vision_stamp),
            ns(s.control_degree_stamp),ns(s.central_activation_stamp),ns(s.hurt_stamp)};
  }
  static std::array<bool,14> valid(const WireState & s) {
    return {s.game_valid,s.robot_valid,s.ammunition_valid,
            s.localization_valid,s.objectives_valid,s.sentry_valid,
            s.heavy.health_valid,s.heavy.position_valid,
            s.infantry.health_valid,s.infantry.position_valid,
            s.vision_status >= WireState::VISION_LOST && s.vision_status <= WireState::VISION_SWITCHING,
            s.control_degree_valid,s.central_activation_valid,s.hurt_valid};
  }
  void clockBoundary(int64_t ros_now) {
    if (previous_clock_ && ros_now < *previous_clock_) {
      // 时钟回退时清除输入及补给、守点等历史状态。
      sample_.reset(); received_.reset(); sources_ = {}; sample_stamp_ = -1;
      ctx_->policy = Policy(config_);
    }
    previous_clock_ = ros_now;
  }
  void receive(const WireState & data) {
    const auto ros_now = now().nanoseconds(); clockBoundary(ros_now);
    const auto stamp = ns(data.header.stamp);
    // 只接收时间递增、地图帧一致的最新状态。
    if (stamp <= sample_stamp_ || stamp <= 0 || stamp > ros_now ||
        (ros_now-stamp)*1e-9 >= timeout_ || data.objectives.size() > 16 ||
        data.header.frame_id != map_frame_) return;
    std::unordered_set<std::string> ids;
    for (const auto & candidate : data.objectives) {
      if (candidate.id.empty() || candidate.id.size() > 64 || !ids.insert(candidate.id).second) return;
    }
    const auto times = stamps(data); const auto flags = valid(data);
    const auto wall = Steady::now();
    for (size_t i=0;i<sources_.size();++i) sources_[i].observe(times[i],flags[i],stamp,wall);
    sample_ = data; sample_stamp_ = stamp; received_ = wall;
  }
  void tick() {
    const auto ros_now = now().nanoseconds(); clockBoundary(ros_now);
    State state; state.enabled = enabled_;
    ctx_->vision_status = WireState::VISION_UNKNOWN;
    const auto wall = Steady::now();
    if (sample_ && received_ && (ros_now-sample_stamp_)*1e-9 < timeout_ &&
        ros_now >= sample_stamp_ && std::chrono::duration<double>(wall-*received_).count() < timeout_) {
      const auto & in = *sample_;
      const auto times = stamps(in); const auto flags = valid(in);
      std::array<bool,14> fresh{};
      // 汇总消息中的每组来源按自身时间戳检查时效。
      for (size_t i=0;i<fresh.size();++i) {
        const double limit = (i==10 ? 0.5 : (i==13 ? 1.0 : timeout_));
        fresh[i] = times[i] <= sample_stamp_ &&
          sources_[i].fresh(times[i],flags[i],ros_now,wall,limit);
      }
      if (fresh[10]) ctx_->vision_status=in.vision_status;
      state.game_fresh=fresh[0]; state.robot_fresh=fresh[1]; state.ammo_fresh=fresh[2];
      state.localization_fresh=fresh[3]; state.objectives_fresh=fresh[4]; state.sentry_fresh=fresh[5];
      state.control_fresh=fresh[11];
      state.central_activation_fresh=fresh[12];
      state.hurt_fresh=fresh[13];
      state.team_color=in.team_color;
      state.ally_control=in.ally_control_degree;
      state.enemy_control=in.enemy_control_degree;
      state.stage_remaining_s=in.stage_remaining_s;
      state.central_ally_active=in.central_ally_active;
      state.central_enemy_active=in.central_enemy_active;
      state.hurt_armor_id=in.hurt_armor_id;
      state.hurt_reason=in.hurt_reason;
      state.manual_override=in.manual_override;
      state.game_type=in.game_type;
      state.game_running=in.game_progress == 4;
      state.hp=in.hp; state.max_hp=in.max_hp; state.ammo=in.ammunition;
      state.life_state_valid=in.life_state_valid; state.weakened=in.weakened;
      state.heat=in.heat; state.heat_limit=in.heat_limit; state.shooter_enabled=in.shooter_enabled;
      state.free_revive_available=in.free_revive_available;
      const auto teammate = [](const msg::TeammateState & source, bool health, bool position) {
        Teammate ally;
        ally.health_fresh=health; ally.position_fresh=position;
        ally.hp=source.hp; ally.max_hp=source.max_hp;
        ally.life_state_valid=source.life_state_valid; ally.weakened=source.weakened;
        ally.x=source.x; ally.y=source.y;
        return ally;
      };
      state.heavy=teammate(in.heavy,fresh[6],fresh[7]);
      state.infantry=teammate(in.infantry,fresh[8],fresh[9]);
      for (const auto & source : in.objectives) {
        Objective candidate;
        candidate.id=source.id;
        candidate.kind=static_cast<ObjectiveKind>(source.kind);
        candidate.terrain=static_cast<Terrain>(source.terrain);
        candidate.reachable=source.reachable; candidate.route_verified=source.route_verified;
        candidate.route_uses_highland=source.route_uses_highland;
        candidate.route_uses_tunnel=source.route_uses_tunnel;
        candidate.risk_known=source.risk_known; candidate.risk=source.risk;
        candidate.travel_time_s=source.travel_time_s;
        candidate.x=source.x; candidate.y=source.y; candidate.radius=source.radius;
        candidate.owner=source.owner; candidate.maintain_required=source.maintain_required;
        candidate.self_present=source.self_present; candidate.vacancy_confirmed=source.vacancy_confirmed;
        candidate.withdrawal_support=source.withdrawal_support;
        candidate.handover_confirmed=source.handover_confirmed;
        state.objectives.push_back(std::move(candidate));
      }
    }
    // 把本轮状态交给行为树，意图在树中计算和发布。
    ctx_->state=std::move(state);
    ctx_->intent={Mode::Disabled,"tree_not_evaluated",""};
    bool tree_ok = false;
    try {
      tree_ok = tree_.tickOnce() == BT::NodeStatus::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_ERROR_THROTTLE(get_logger(),*get_clock(),2000,"Decision tree failed: %s",error.what());
      tree_.haltTree();
    }
    if (!tree_ok) {
      ctx_->intent={Mode::Disabled,"tree_failed",""};
      msg::DecisionIntent out;
      out.header.stamp=now(); out.mode=mode_name(ctx_->intent.mode); out.reason=ctx_->intent.reason;
      out.vision_status=ctx_->vision_status;
      pub_->publish(out);
    }
    const auto & intent = ctx_->intent;
    const auto * mode = mode_name(intent.mode);
    const std::string label = std::string(mode) + "|" + intent.target + "|" +
      intent.team_situation + "|" + intent.reason;
    if (label != last_decision_) {
      RCLCPP_INFO(get_logger(), "Decision mode=%s task=%s reason=%s team=%s", mode,
        intent.target.empty() ? "-" : intent.target.c_str(), intent.reason.c_str(),
        intent.team_situation.empty() ? "-" : intent.team_situation.c_str());
      last_decision_ = label;
    }
  }
  bool enabled_=false;
  double timeout_=2.5;
  Config config_;
  std::shared_ptr<Context> ctx_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  rclcpp::Publisher<msg::DecisionIntent>::SharedPtr pub_;
  rclcpp::Subscription<WireState>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  std::array<SourceProgress,14> sources_{};
  std::optional<WireState> sample_;
  std::optional<Steady::time_point> received_;
  std::optional<int64_t> previous_clock_;
  int64_t sample_stamp_=-1;
  std::string last_decision_,map_frame_;
};
}  // namespace rm2027_decision

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  int result=0;
  try { rclcpp::spin(std::make_shared<rm2027_decision::DecisionNode>()); }
  catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rm2027_tree"),"%s",e.what()); result=1;
  }
  rclcpp::shutdown();
  return result;
}
