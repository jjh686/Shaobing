#ifndef RM2027_DECISION__POLICY_HPP_
#define RM2027_DECISION__POLICY_HPP_

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace rm2027_decision
{

enum class Profile { Unconfigured, Rmul3v3, Rmuc };
enum class Mode {
  Disabled, WaitConfig, WaitState, WaitGame, Dead, Revive,
  LocalizationLost, Resupply, Capture, Patrol, Hold, Cover, WaitPlan
};

struct Config
{
  Profile profile = Profile::Unconfigured;
  double low_hp_ratio = .3;
  double resume_hp_ratio = .8;
  double max_task_risk = .5, max_recovery_risk = .5, cautious_capture_risk = .25;
};

enum class Terrain { Ground = 0, Highland = 1, Tunnel = 2 };
enum class ObjectiveKind { Recovery = 0, Zone = 1, Defence = 2, Patrol = 3 };

struct Teammate
{
  bool health_fresh = false, position_fresh = false, life_state_valid = false, weakened = false;
  double hp = 0, max_hp = 0, x = 0, y = 0;
};

struct Objective
{
  std::string id;
  ObjectiveKind kind = ObjectiveKind::Zone;
  Terrain terrain = Terrain::Ground;
  bool reachable = false, route_verified = false;
  bool route_uses_highland = false, route_uses_tunnel = false, risk_known = false;
  double risk = 1, travel_time_s = 0, x = 0, y = 0, radius = .5;
  unsigned owner = 0;
  bool maintain_required = false, self_present = false, vacancy_confirmed = false;
  bool withdrawal_support = false, handover_confirmed = false;
};

// The adapter supplies independent source freshness. Goals are semantic IDs only.
// Owner: 0 unknown, 1 neutral, 2 ally, 3 enemy, 4 contested.
struct State
{
  bool enabled = false, manual_override = false;
  bool game_fresh = false, robot_fresh = false, ammo_fresh = false;
  bool localization_fresh = false, sentry_fresh = false;
  bool game_running = false;
  unsigned game_type = 0;
  bool life_state_valid = false, weakened = false;
  double hp = 0, max_hp = 0, heat = 0, heat_limit = 0;
  unsigned ammo = 0;
  bool shooter_enabled = false, free_revive_available = false;
  Teammate heavy, infantry;
  bool objectives_fresh = false;
  std::vector<Objective> objectives;
  // 0 unknown, 1 red (robot_id 7), 2 blue (robot_id 107). Never a match-side lock in config.
  unsigned team_color = 0;
  bool control_fresh = false;
  unsigned ally_control = 0, enemy_control = 0, stage_remaining_s = 0;
  bool central_activation_fresh = false, central_ally_active = false, central_enemy_active = false;
  bool hurt_fresh = false;
  unsigned hurt_armor_id = 0, hurt_reason = 0;
};

struct Intent
{
  Mode mode;
  std::string reason, target;
  bool fire_allowed = false, free_revive_requested = false;
  std::string team_situation{};
  bool coverage_requested = false;
  // Advisory only: idle, search, track or alert. Not a gimbal or MCU command.
  std::string vision_mode_request{};
};

inline const char * mode_name(Mode mode)
{
  switch (mode) {
    case Mode::Disabled: return "Disabled";
    case Mode::WaitConfig: return "WaitConfig";
    case Mode::WaitState: return "WaitState";
    case Mode::WaitGame: return "WaitGame";
    case Mode::Dead: return "Dead";
    case Mode::Revive: return "Revive";
    case Mode::LocalizationLost: return "LocalizationLost";
    case Mode::Resupply: return "Resupply";
    case Mode::Capture: return "Capture";
    case Mode::Patrol: return "Patrol";
    case Mode::Hold: return "Hold";
    case Mode::Cover: return "Cover";
    case Mode::WaitPlan: return "WaitPlan";
  }
  return "Unknown";
}

class Policy
{
public:
  explicit Policy(Config config) : config_(config)
  {
    if (!std::isfinite(config_.low_hp_ratio) || !std::isfinite(config_.resume_hp_ratio) ||
      config_.low_hp_ratio <= 0 || config_.low_hp_ratio >= config_.resume_hp_ratio ||
      config_.resume_hp_ratio > 1)
    {
      throw std::invalid_argument("Require finite 0 < low_hp_ratio < resume_hp_ratio <= 1");
    }
    for (double risk : {config_.max_task_risk, config_.max_recovery_risk,
      config_.cautious_capture_risk})
    {
      if (!std::isfinite(risk) || risk < 0 || risk > 1)
        throw std::invalid_argument("Risk budgets must be finite and within [0, 1]");
    }
    if (config_.cautious_capture_risk > config_.max_task_risk)
      throw std::invalid_argument("Cautious capture risk must not exceed the normal task budget");
  }

  Intent evaluate(const State & state)
  {
    // 按优先级检查条件，命中后返回本轮意图。
    const Health heavy = health(state.heavy, heavy_progress_);
    const Health infantry = health(state.infantry, infantry_progress_);
    auto emit = [&](Mode mode, const std::string & reason, const std::string & target = "",
      bool fire = false, bool recovery = false, bool coverage = false) {
        Intent out{mode, reason, target};
        out.fire_allowed = fire;
        out.team_situation = situation(heavy, infantry, recovery,
          valid_hp(state.hp, state.max_hp) && state.hp > 0 &&
          state.hp / state.max_hp < config_.low_hp_ratio);
        out.coverage_requested = coverage;
        out.vision_mode_request = vision_mode(mode, fire, state.hurt_fresh);
        return out;
      };
    // 首先确认决策使能、比赛状态、本车生命状态和定位。
    if (state.manual_override || !state.enabled) {
      resupplying_ = false;
      held_id_.clear();
      heavy_progress_ = {}; infantry_progress_ = {};
      return emit(Mode::Disabled, state.manual_override ? "manual_override" : "disabled");
    }
    if (config_.profile != Profile::Rmul3v3) {
      return emit(Mode::WaitConfig, "profile_unconfigured_or_unsupported");
    }
    if (!state.game_fresh || !state.robot_fresh) {
      return emit(Mode::WaitState, "critical_state_missing_or_stale");
    }
    if (state.game_type != 4) {
      return emit(Mode::WaitState, "game_type_not_rmul_3v3");
    }
    if (!valid_hp(state.hp, state.max_hp)) {
      return emit(Mode::WaitState, "invalid_hp");
    }
    if (!state.game_running) {
      resupplying_ = false;
      held_id_.clear();
      heavy_progress_ = {}; infantry_progress_ = {};
      return emit(Mode::WaitGame, "game_not_running");
    }
    if (state.hp == 0) {
      resupplying_ = false;
      held_id_.clear();
      heavy_progress_ = {}; infantry_progress_ = {};
      return emit(Mode::Dead, "automatic_revive_wait");
    }
    if (!state.life_state_valid) {
      return emit(Mode::WaitState, "life_state_unknown");
    }
    if (!state.localization_fresh) {
      return emit(Mode::LocalizationLost, "localization_missing_or_stale");
    }

    // 用不同的进入和退出阈值保持补给任务稳定。
    const double hp_ratio = state.hp / state.max_hp;
    if (hp_ratio < config_.low_hp_ratio) {
      resupplying_ = true;
    } else if (hp_ratio >= config_.resume_hp_ratio) {
      resupplying_ = false;
    }
    const bool recovery = state.weakened || resupplying_;
    bool pending_maintenance = false, current_unreleased = false;
    if (state.objectives_fresh) {
      for (const auto & goal : state.objectives) {
        if (goal.kind == ObjectiveKind::Zone && goal.owner == 2 &&
          goal.maintain_required && !released(state, goal))
        {
          pending_maintenance = true;
          current_unreleased = current_unreleased || goal.self_present;
        }
      }
    }
    const bool request_coverage = recovery && (pending_maintenance || !held_id_.empty());
    if (!state.objectives_fresh) {
      return emit(Mode::WaitPlan, "objectives_missing_or_stale", "", false,
        recovery, request_coverage);
    }
    // 自身恢复优先，补给路线仍需满足风险阈值。
    if (recovery) {
      const Objective * best = choose(state, ObjectiveKind::Recovery, config_.max_recovery_risk);
      if (!best) return emit(Mode::WaitPlan, "no_recovery_route_within_risk_budget", "", false,
          true, request_coverage);
      return emit(Mode::Resupply, state.weakened ? "post_revive_weakness" :
        "low_hp_recovery_latched", best->id, false, true, request_coverage);
    }
    // 检查上一轮守点任务是否仍需维护或已交接。
    const Objective * previous_guard = nullptr;
    if (!held_id_.empty()) {
      for (const auto & goal : state.objectives) {
        if (goal.id == held_id_) { previous_guard = &goal; break; }
      }
      if (!previous_guard || previous_guard->kind != ObjectiveKind::Zone ||
        previous_guard->owner == 0 || previous_guard->owner == 4 || previous_guard->owner > 4)
      {
        return emit(Mode::WaitPlan, "previous_guard_evidence_missing");
      }
      if (previous_guard->owner != 2 || !previous_guard->maintain_required ||
        released(state, *previous_guard))
      {
        held_id_.clear(); previous_guard = nullptr;
      }
    }
    const bool fire_allowed = state.ammo_fresh && state.shooter_enabled && state.ammo > 0 &&
      std::isfinite(state.heat) && std::isfinite(state.heat_limit) &&
      state.heat >= 0 && state.heat_limit > 0 && state.heat < state.heat_limit;
    // 离开仍需维护的岗位时返回，已经在位时继续守点。
    if (previous_guard && !previous_guard->self_present) {
      if (!allowed(*previous_guard, config_.max_task_risk)) return emit(Mode::WaitPlan, "previous_guard_route_unverified");
      return emit(Mode::Cover, "maintain_previous_guard_assignment", previous_guard->id, fire_allowed);
    }
    for (const auto & goal : state.objectives) {
      if (allowed(goal, config_.max_task_risk) && goal.kind == ObjectiveKind::Zone && goal.owner == 2 &&
        goal.maintain_required && goal.self_present && !released(state, goal))
      {
        held_id_ = goal.id;
        return emit(Mode::Hold, "maintain_current_allied_zone", goal.id, fire_allowed);
      }
    }
    if (current_unreleased) {
      return emit(Mode::WaitPlan, "current_guard_route_or_risk_unverified");
    }
    // 岗位空缺需要明确证据，并确认没有队友已经在位。
    for (const auto & goal : state.objectives) {
      if (allowed(goal, config_.max_task_risk) && goal.kind == ObjectiveKind::Zone && goal.owner == 2 &&
        goal.maintain_required && !released(state, goal) &&
        (goal.vacancy_confirmed || positions_confirm_empty(state, goal)) &&
        !teammate_present(state, goal))
      {
        return emit(Mode::Cover, "confirmed_allied_guard_vacancy", goal.id, fire_allowed);
      }
    }
    if (recovering_ally(heavy) || recovering_ally(infantry)) {
      const Objective * near = cover_near_teammate(state,
        recovering_ally(heavy) ? state.heavy : state.infantry,
        recovering_ally(heavy) ? heavy : infantry);
      if (near) return emit(Mode::Cover, "cover_near_recovering_ally", near->id, fire_allowed);
      const Objective * best = choose(state, ObjectiveKind::Defence, config_.max_task_risk, true);
      if (best) return emit(Mode::Cover, recovering_ally(heavy) && recovering_ally(infantry) ?
          "allies_down_withdrawal_support" : "ally_withdrawal_support", best->id, fire_allowed);
    }
    // 队友状态不明或尚未恢复时保守占点；两人均确认阵亡或虚弱时哨兵改为承担进攻。
    const bool team_unknown = heavy == Health::Unknown || infantry == Health::Unknown;
    const bool both_out = !team_unknown &&
      (heavy == Health::Dead || heavy == Health::Weakened) &&
      (infantry == Health::Dead || infantry == Health::Weakened);
    const bool sole_fighter = both_out;
    const bool losing = state.control_fresh && state.ally_control < state.enemy_control;
    const bool winning = state.control_fresh && state.ally_control > state.enemy_control;
    const bool time_closing = state.stage_remaining_s > 0 && state.stage_remaining_s <= 30;
    const bool cautious = team_unknown ||
      (heavy != Health::Healthy && infantry != Health::Healthy && !both_out);
    const double capture_budget = cautious ? config_.cautious_capture_risk : config_.max_task_risk;
    const bool prefer_attack = !cautious && !(winning && time_closing);
    if (!(winning && time_closing && !losing)) {
      if (const Objective * best = choose_capture(state, capture_budget, prefer_attack,
            losing || sole_fighter || (state.central_activation_fresh &&
              (state.central_enemy_active || !state.central_ally_active))))
      {
        return emit(Mode::Capture, capture_reason(*best, losing, sole_fighter), best->id, fire_allowed);
      }
    }
    if (!pending_maintenance) {
      const Objective * best = choose(state, ObjectiveKind::Patrol, config_.max_task_risk);
      if (best) return emit(Mode::Patrol, "explicit_verified_patrol", best->id, fire_allowed);
    }
    return emit(Mode::WaitPlan, pending_maintenance ? "allied_maintenance_unresolved" :
      "no_verified_tactical_objective");
  }

private:
  enum class Health { Unknown, Healthy, Low, Recovering, Dead, Weakened, Alive };
  struct AllyProgress {
    bool seen = false, recovering = false;
    Health status = Health::Unknown;
  };
  static bool valid_hp(double hp, double max_hp)
  {
    return std::isfinite(hp) && std::isfinite(max_hp) && max_hp > 0 && hp >= 0 && hp <= max_hp;
  }
  Health health(const Teammate & robot, AllyProgress & progress)
  {
    if (!robot.health_fresh || !std::isfinite(robot.hp) || robot.hp < 0) {
      return progress.status = Health::Unknown;
    }
    // Official 0x0003 HP 0 means the robot is absent, penalized or eliminated.
    if (robot.hp == 0) {
      progress.seen = progress.recovering = true;
      return progress.status = Health::Dead;
    }
    if (!robot.life_state_valid || !valid_hp(robot.hp, robot.max_hp)) {
      return progress.status = Health::Alive;
    }
    if (robot.weakened) {
      progress.seen = progress.recovering = true;
      return progress.status = Health::Weakened;
    }
    const double ratio = robot.hp / robot.max_hp;
    if (!progress.seen) progress.recovering = ratio < config_.resume_hp_ratio;
    if (ratio < config_.low_hp_ratio) progress.recovering = true;
    else if (ratio >= config_.resume_hp_ratio) progress.recovering = false;
    progress.seen = true;
    return progress.status = ratio < config_.low_hp_ratio ? Health::Low :
      (progress.recovering ? Health::Recovering : Health::Healthy);
  }
  static bool recovering_ally(Health value)
  {
    return value == Health::Low || value == Health::Recovering ||
      value == Health::Dead || value == Health::Weakened;
  }
  static const char * health_name(Health value)
  {
    switch (value) {
      case Health::Healthy: return "healthy";
      case Health::Low: return "low";
      case Health::Recovering: return "recovering";
      case Health::Dead: return "dead";
      case Health::Weakened: return "weakened";
      case Health::Alive: return "alive";
      default: return "unknown";
    }
  }
  static std::string situation(Health heavy, Health infantry, bool recovery, bool self_low)
  {
    const char * category = "normal";
    if (self_low && heavy == Health::Low && infantry == Health::Low) category = "all_low";
    else if (recovery) category = "self_recovery";
    else if (heavy == Health::Unknown || infantry == Health::Unknown) category = "unknown";
    else if (recovering_ally(heavy) && recovering_ally(infantry)) category = "allies_recovering";
    else if (recovering_ally(heavy) || recovering_ally(infantry)) category = "partial";
    return std::string(category) + "|heavy=" + health_name(heavy) +
      "|infantry=" + health_name(infantry);
  }
  static bool allowed(const Objective & goal, double budget)
  {
    return !goal.id.empty() && goal.id.size() <= 64 && goal.reachable && goal.route_verified &&
      goal.terrain == Terrain::Ground && !goal.route_uses_highland && !goal.route_uses_tunnel &&
      goal.risk_known && std::isfinite(goal.risk) && goal.risk >= 0 && goal.risk <= budget &&
      std::isfinite(goal.travel_time_s) && goal.travel_time_s >= 0 &&
      std::isfinite(goal.x) && std::isfinite(goal.y) &&
      std::isfinite(goal.radius) && goal.radius > 0;
  }
  bool teammate_present(const State & state, const Objective & goal, bool ready_only = false) const
  {
    unsigned index = 0;
    for (const auto * robot : {&state.heavy, &state.infantry}) {
      const Health status = (index++ == 0 ? heavy_progress_ : infantry_progress_).status;
      const bool alive = status == Health::Healthy || status == Health::Low ||
        status == Health::Recovering;
      if (robot->position_fresh && alive && (!ready_only || status == Health::Healthy) &&
        std::isfinite(robot->x) && std::isfinite(robot->y) &&
        std::hypot(robot->x - goal.x, robot->y - goal.y) <= goal.radius)
      {
        return true;
      }
    }
    return false;
  }
  bool released(const State & state, const Objective & goal) const
  {
    return goal.handover_confirmed && teammate_present(state, goal, true);
  }
  static bool opponent_supply(const State & state, const Objective & goal)
  {
    return goal.kind == ObjectiveKind::Recovery &&
      ((state.team_color == 1 && goal.id == "blue_supply") ||
       (state.team_color == 2 && goal.id == "red_supply"));
  }
  bool positions_confirm_empty(const State & state, const Objective & goal) const
  {
    // Only when BOTH teammates have fresh coordinates. Missing one pose is not vacancy.
    return state.heavy.position_fresh && state.infantry.position_fresh &&
      !teammate_present(state, goal);
  }
  const Objective * cover_near_teammate(const State & state, const Teammate & robot,
    Health status) const
  {
    if (!recovering_ally(status) || !robot.position_fresh ||
      !std::isfinite(robot.x) || !std::isfinite(robot.y))
    {
      return nullptr;
    }
    const Objective * best = nullptr;
    double best_range = 0;
    for (const auto & goal : state.objectives) {
      const bool usable = (goal.kind == ObjectiveKind::Defence && goal.withdrawal_support) ||
        (goal.kind == ObjectiveKind::Zone && (goal.owner == 1 || goal.owner == 2));
      if (!usable || !allowed(goal, config_.max_task_risk) || opponent_supply(state, goal)) continue;
      const double range = std::hypot(robot.x - goal.x, robot.y - goal.y);
      // 队友已经站在该点时，不再把它当成要去补位的目标。
      if (range <= goal.radius) continue;
      if (!best || range < best_range) { best = &goal; best_range = range; }
    }
    return best;
  }
  static const char * capture_reason(const Objective & goal, bool losing, bool sole_fighter)
  {
    if (goal.id == "central_zone") return losing || sole_fighter ?
      "press_central_zone" : "eligible_central_zone";
    if (goal.owner == 3) return losing || sole_fighter ?
      "press_enemy_ground_zone" : "eligible_enemy_ground_zone";
    return "eligible_neutral_ground_zone";
  }
  const Objective * choose_capture(const State & state, double budget, bool prefer_attack,
    bool press_central)
  {
    // Without fresh control scores, keep the original first-eligible-in-list rule.
    if (!state.control_fresh) {
      for (const auto & goal : state.objectives) {
        if (goal.kind != ObjectiveKind::Zone || !allowed(goal, budget) ||
          teammate_present(state, goal) || opponent_supply(state, goal)) continue;
        if (goal.owner == 1 || (goal.owner == 3 && prefer_attack)) return &goal;
      }
      return nullptr;
    }
    const Objective * best = nullptr;
    int best_rank = -1;
    for (const auto & goal : state.objectives) {
      if (goal.kind != ObjectiveKind::Zone || !allowed(goal, budget) ||
        teammate_present(state, goal) || opponent_supply(state, goal)) continue;
      if (!(goal.owner == 1 || (goal.owner == 3 && prefer_attack))) continue;
      int rank = 0;
      if (press_central && goal.id == "central_zone") rank += 5;
      else if (goal.id == "central_zone") rank += 3;
      if (prefer_attack && goal.owner == 3) rank += 2;
      if (goal.owner == 1) rank += 1;
      if (!best || rank > best_rank || (rank == best_rank && (goal.risk < best->risk ||
        (goal.risk == best->risk && goal.travel_time_s < best->travel_time_s))))
      {
        best = &goal; best_rank = rank;
      }
    }
    return best;
  }
  static const char * vision_mode(Mode mode, bool fire, bool hurt)
  {
    if (hurt) return "alert";
    switch (mode) {
      case Mode::Capture:
      case Mode::Hold:
      case Mode::Cover:
      case Mode::Patrol:
        return fire ? "track" : "search";
      default:
        return "idle";
    }
  }
  static const Objective * choose(const State & state, ObjectiveKind kind, double budget,
    bool require_support = false)
  {
    const Objective * best = nullptr;
    for (const auto & goal : state.objectives) {
      if (goal.kind != kind || !allowed(goal, budget) || opponent_supply(state, goal) ||
        (require_support && !goal.withdrawal_support)) continue;
      if (!best || goal.risk < best->risk ||
        (goal.risk == best->risk && goal.travel_time_s < best->travel_time_s))
      {
        best = &goal;
      }
    }
    return best;
  }
  Config config_;
  bool resupplying_ = false;
  std::string held_id_;
  AllyProgress heavy_progress_, infantry_progress_;
};

}  // namespace rm2027_decision

#endif  // RM2027_DECISION__POLICY_HPP_
