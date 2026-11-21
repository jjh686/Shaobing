// Standalone assertion tests: no ROS, serial device, motion output or simulation.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <limits>
#include <string>

#include "rm2027_decision/policy.hpp"

using namespace rm2027_decision;

namespace
{
Objective objective(std::string id, ObjectiveKind kind, unsigned owner = 0)
{
  Objective goal;
  goal.id = std::move(id); goal.kind = kind; goal.owner = owner;
  goal.reachable = goal.route_verified = goal.risk_known = true;
  goal.risk = .1; goal.travel_time_s = 10; goal.x = 5; goal.y = 5;
  return goal;
}

Teammate teammate(double hp = 400, double max_hp = 400)
{
  Teammate robot;
  robot.health_fresh = robot.life_state_valid = true;
  robot.hp = hp; robot.max_hp = max_hp;
  return robot;
}

State ready()
{
  State state;
  state.enabled = true;
  state.game_fresh = state.robot_fresh = state.ammo_fresh = true;
  state.localization_fresh = state.game_running = true;
  state.game_type = 4;
  state.life_state_valid = true;
  state.hp = state.max_hp = 400;
  state.heat = 0;
  state.heat_limit = 100;
  state.ammo = 100;
  state.shooter_enabled = true;
  state.heavy = teammate(); state.infantry = teammate();
  state.objectives_fresh = true;
  state.objectives = {objective("home", ObjectiveKind::Recovery),
    objective("patrol", ObjectiveKind::Patrol)};
  return state;
}

void assert_stopped(const Intent & intent, Mode expected)
{
  assert(intent.mode == expected);
  assert(intent.target.empty());
  assert(!intent.fire_allowed);
  assert(!intent.free_revive_requested);
}

void expect_invalid(Config config)
{
  bool rejected = false;
  try { Policy policy(config); } catch (const std::invalid_argument &) { rejected = true; }
  assert(rejected);
}

void test_rmul_3v3()
{
  Policy policy(Config{Profile::Rmul3v3});
  State state = ready();
  auto result = policy.evaluate(state);
  assert(result.mode == Mode::Patrol && result.target == "patrol" && result.fire_allowed);

  state.enabled = false;
  assert_stopped(policy.evaluate(state), Mode::Disabled);
  state = ready();
  state.manual_override = true;
  state.game_fresh = false;
  assert_stopped(policy.evaluate(state), Mode::Disabled);
  state = ready();
  state.game_fresh = false;
  assert_stopped(policy.evaluate(state), Mode::WaitState);
  state = ready();
  state.robot_fresh = false;
  assert_stopped(policy.evaluate(state), Mode::WaitState);
  state = ready();
  state.game_running = false;
  assert_stopped(policy.evaluate(state), Mode::WaitGame);
  for (unsigned type : {0U, 1U, 5U}) {
    state = ready(); state.game_type = type;
    assert_stopped(policy.evaluate(state), Mode::WaitState);
  }
  state = ready(); state.life_state_valid = false;
  assert_stopped(policy.evaluate(state), Mode::WaitState);

  state = ready();
  state.hp = 0;
  result = policy.evaluate(state);
  assert_stopped(result, Mode::Dead);
  assert(result.reason == "automatic_revive_wait");
  state.free_revive_available = true;
  assert_stopped(policy.evaluate(state), Mode::Dead);
  state.sentry_fresh = true;
  result = policy.evaluate(state);
  assert_stopped(result, Mode::Dead);  // Even fresh free-revival permission is irrelevant to 3V3.
  state.localization_fresh = false;
  state.life_state_valid = false;
  assert_stopped(policy.evaluate(state), Mode::Dead);  // Dead branch precedes life/localization.
  state = ready();
  state.localization_fresh = false;
  assert_stopped(policy.evaluate(state), Mode::LocalizationLost);

  state = ready(); state.weakened = true;
  state.objectives.push_back(objective("neutral", ObjectiveKind::Zone, 1));
  result = policy.evaluate(state);
  assert(result.mode == Mode::Resupply && result.target == "home");
  assert(result.reason == "post_revive_weakness" && !result.fire_allowed);
  assert(!result.free_revive_requested);  // Full HP still cannot unlock capture or shooting.
  state.weakened = false;
  assert(policy.evaluate(state).mode == Mode::Capture);
  state.weakened = true; state.localization_fresh = false;
  assert_stopped(policy.evaluate(state), Mode::LocalizationLost);

  state = ready();
  state.hp = 120;  // Exactly 30% does not enter recovery.
  assert(policy.evaluate(state).mode == Mode::Patrol);
  state.hp = 119;
  result = policy.evaluate(state);
  assert(result.mode == Mode::Resupply && result.target == "home" && !result.fire_allowed);
  state.hp = 200;
  assert(policy.evaluate(state).mode == Mode::Resupply);
  state.robot_fresh = false;
  assert_stopped(policy.evaluate(state), Mode::WaitState);
  state.robot_fresh = true;
  assert(policy.evaluate(state).mode == Mode::Resupply);  // Dropout preserves the latch.
  state.localization_fresh = false;
  assert_stopped(policy.evaluate(state), Mode::LocalizationLost);
  state.localization_fresh = true;
  assert(policy.evaluate(state).mode == Mode::Resupply);
  state.hp = 319;
  assert(policy.evaluate(state).mode == Mode::Resupply);
  state.hp = 320;  // Exactly 80% resumes.
  assert(policy.evaluate(state).mode == Mode::Patrol);

  // Disabled, stopped game, manual takeover and death reset the recovery latch.
  for (unsigned reset = 0; reset < 4; ++reset) {
    state = ready(); state.hp = 100;
    assert(policy.evaluate(state).mode == Mode::Resupply);
    if (reset == 0) state.enabled = false;
    if (reset == 1) state.game_running = false;
    if (reset == 2) state.manual_override = true;
    if (reset == 3) state.hp = 0;
    policy.evaluate(state);
    state = ready(); state.hp = 200;
    assert(policy.evaluate(state).mode == Mode::Patrol);
  }

  // Missing ammo/heat evidence forbids firing but does not manufacture a resupply request.
  for (unsigned failure = 0; failure < 11; ++failure) {
    state = ready();
    if (failure == 0) state.ammo = 0;
    if (failure == 1) state.ammo_fresh = false;
    if (failure == 2) state.shooter_enabled = false;
    if (failure == 3) state.heat = state.heat_limit;
    if (failure == 4) state.heat = state.heat_limit + 1;
    if (failure == 5) state.heat = std::numeric_limits<double>::quiet_NaN();
    if (failure == 6) state.heat_limit = 0;
    if (failure == 7) state.heat = -1;
    if (failure == 8) state.heat_limit = std::numeric_limits<double>::quiet_NaN();
    if (failure == 9) state.heat_limit = std::numeric_limits<double>::infinity();
    if (failure == 10) state.heat = std::numeric_limits<double>::infinity();
    result = policy.evaluate(state);
    assert(result.mode == Mode::Patrol && !result.fire_allowed && !result.free_revive_requested);
  }

  state = ready();
  state.objectives = {objective("unknown", ObjectiveKind::Zone, 0),
    objective("allied", ObjectiveKind::Zone, 2), objective("contested", ObjectiveKind::Zone, 4),
    objective("invalid", ObjectiveKind::Zone, 99), objective("", ObjectiveKind::Zone, 1),
    objective("first_enemy", ObjectiveKind::Zone, 3), objective("second_neutral", ObjectiveKind::Zone, 1)};
  state.objectives_fresh = false;
  assert_stopped(policy.evaluate(state), Mode::WaitPlan);
  state.objectives_fresh = true;
  result = policy.evaluate(state);
  assert(result.mode == Mode::Capture && result.target == "first_enemy" && result.fire_allowed);
  state.objectives = {objective("first_neutral", ObjectiveKind::Zone, 1),
    objective("enemy", ObjectiveKind::Zone, 3)};
  assert(policy.evaluate(state).target == "first_neutral");
  state.objectives = {objective("unknown", ObjectiveKind::Zone, 0),
    objective("allied", ObjectiveKind::Zone, 2), objective("contested", ObjectiveKind::Zone, 4),
    objective("invalid", ObjectiveKind::Zone, 99)};
  assert_stopped(policy.evaluate(state), Mode::WaitPlan);

  for (unsigned failure = 0; failure < 7; ++failure) {
    state = ready();
    if (failure == 0) state.hp = std::numeric_limits<double>::quiet_NaN();
    if (failure == 1) state.hp = std::numeric_limits<double>::infinity();
    if (failure == 2) state.max_hp = std::numeric_limits<double>::infinity();
    if (failure == 3) state.max_hp = 0;
    if (failure == 4) state.hp = -1;
    if (failure == 5) state.hp = state.max_hp + 1;
    if (failure == 6) state.max_hp = std::numeric_limits<double>::quiet_NaN();
    assert_stopped(policy.evaluate(state), Mode::WaitState);
  }
}

void test_team_matrix_and_hysteresis()
{
  for (unsigned mask = 0; mask < 8; ++mask) {
    Policy policy(Config{Profile::Rmul3v3});
    auto s = ready();
    s.hp = mask & 1 ? 100 : 400;
    s.heavy.hp = mask & 2 ? 100 : 400;
    s.infantry.hp = mask & 4 ? 100 : 400;
    auto defence = objective("withdrawal", ObjectiveKind::Defence);
    defence.withdrawal_support = true;
    s.objectives.push_back(defence);
    s.objectives.push_back(objective("neutral", ObjectiveKind::Zone, 1));
    auto out = policy.evaluate(s);
    assert(out.mode == (mask & 1 ? Mode::Resupply : (mask & 6 ? Mode::Cover : Mode::Capture)));
    assert(!out.free_revive_requested);
    if (mask == 7) assert(out.team_situation.find("all_low|") == 0);
    if (mask == 6) assert(out.team_situation.find("allies_recovering|") == 0);
  }
  Policy p(Config{Profile::Rmul3v3});
  auto s = ready();
  s.objectives = {objective("enemy", ObjectiveKind::Zone, 3)};
  s.heavy = teammate(100,450); s.infantry = teammate(200,400);
  assert_stopped(p.evaluate(s), Mode::WaitPlan);
  s.heavy.hp = 140;  // 31% is not ready after recovery began.
  auto out = p.evaluate(s);
  assert(out.team_situation.find("heavy=recovering") != std::string::npos);
  s.heavy.health_fresh = false;
  assert_stopped(p.evaluate(s), Mode::WaitPlan);
  s.heavy.health_fresh = true;
  assert_stopped(p.evaluate(s), Mode::WaitPlan);
  s.heavy.hp = 360;
  assert(p.evaluate(s).mode == Mode::Capture);  // Own max HP, not a 400-HP constant.
  s.heavy = teammate(100,100);
  assert(p.evaluate(s).team_situation.find("heavy=healthy") != std::string::npos);
  s.heavy.hp = 0; s.heavy.life_state_valid = false;
  out = p.evaluate(s);
  assert(out.team_situation.find("heavy=unknown") != std::string::npos);
  s.heavy.life_state_valid = true;
  out = p.evaluate(s);
  assert(out.team_situation.find("heavy=dead") != std::string::npos);
  s.heavy.hp = 100; s.heavy.life_state_valid = false;
  assert(p.evaluate(s).team_situation.find("heavy=unknown") != std::string::npos);
  s.heavy.life_state_valid = true; s.heavy.weakened = true;
  assert(p.evaluate(s).team_situation.find("heavy=weakened") != std::string::npos);
  s.heavy.health_fresh = false;
  s.objectives = {objective("neutral", ObjectiveKind::Zone,1)};
  assert(p.evaluate(s).mode == Mode::Capture);
  s.objectives[0].risk = .26;
  assert_stopped(p.evaluate(s),Mode::WaitPlan);
}

void test_hold_handover_and_missing_guard()
{
  auto guard = objective("guard", ObjectiveKind::Zone,2);
  guard.maintain_required = guard.self_present = true;
  for (unsigned failure=0; failure<6; ++failure) {
    Policy p(Config{Profile::Rmul3v3}); auto s=ready();
    s.objectives.push_back(guard);
    assert(p.evaluate(s).mode == Mode::Hold);
    if(failure==0) s.objectives.pop_back();
    if(failure==1) s.objectives.back().owner=0;
    if(failure==2) s.objectives.back().owner=4;
    if(failure==3) s.objectives.back().reachable=false;
    if(failure==4) s.objectives.back().risk_known=false;
    if(failure==5) s.objectives.back().kind=ObjectiveKind::Patrol;
    if(failure==1 || failure==2){
      s.objectives.back().handover_confirmed=true;
      s.heavy.position_fresh=true;s.heavy.x=s.heavy.y=5;
    }
    assert_stopped(p.evaluate(s),Mode::WaitPlan);
    s.hp=100;
    auto out=p.evaluate(s);
    assert(out.mode==Mode::Resupply && out.coverage_requested);
  }
  Policy p(Config{Profile::Rmul3v3}); auto s=ready(); s.objectives.push_back(guard);
  assert(p.evaluate(s).mode==Mode::Hold);
  s.objectives.back().handover_confirmed=true;
  assert(p.evaluate(s).mode==Mode::Hold);  // Flag alone cannot release.
  s.heavy.position_fresh=true; s.heavy.x=s.heavy.y=5;
  s.heavy.hp=100;
  assert(p.evaluate(s).mode==Mode::Hold);
  s.heavy.hp=124;
  assert(p.evaluate(s).mode==Mode::Hold);
  s.heavy.hp=320; s.heavy.x=std::numeric_limits<double>::quiet_NaN();
  assert(p.evaluate(s).mode==Mode::Hold);
  s.heavy.x=5; s.heavy.position_fresh=false;
  assert(p.evaluate(s).mode==Mode::Hold);
  s.heavy.position_fresh=true;
  assert(p.evaluate(s).mode==Mode::Patrol);
  for(unsigned release=0;release<3;++release){
    Policy q(Config{Profile::Rmul3v3}); auto t=ready();t.objectives.push_back(guard);
    assert(q.evaluate(t).mode==Mode::Hold);
    if(release==0)t.objectives.back().owner=1;
    if(release==1)t.objectives.back().owner=3;
    if(release==2)t.objectives.back().maintain_required=false;
    assert(q.evaluate(t).mode!=Mode::Hold);
  }
}

void test_coverage_and_occupancy()
{
  auto goal=objective("zone",ObjectiveKind::Zone,2);goal.maintain_required=true;
  Policy p(Config{Profile::Rmul3v3});auto s=ready();s.objectives={goal};
  assert_stopped(p.evaluate(s),Mode::WaitPlan);  // Missing position is not vacancy.
  s.objectives[0].vacancy_confirmed=true;
  assert(p.evaluate(s).mode==Mode::Cover);
  s.heavy.position_fresh=true;s.heavy.x=s.heavy.y=5;
  assert_stopped(p.evaluate(s),Mode::WaitPlan);
  s.heavy.x=std::numeric_limits<double>::quiet_NaN();
  assert(p.evaluate(s).mode==Mode::Cover);
  s.objectives[0].owner=1;s.heavy.x=5;
  assert_stopped(p.evaluate(s),Mode::WaitPlan);
  s.heavy.hp=100;  // Low alive robot still occupies the point.
  assert_stopped(p.evaluate(s),Mode::WaitPlan);
  s.heavy.hp=0;
  assert(p.evaluate(s).mode==Mode::Capture);
  s=ready(); s.objectives.push_back(goal);s.objectives.back().reachable=false;
  assert_stopped(p.evaluate(s),Mode::WaitPlan); // Unresolved allied maintenance blocks patrol.
}

void test_routes_risk_and_no_map()
{
  for(auto kind : {ObjectiveKind::Recovery,ObjectiveKind::Zone,ObjectiveKind::Defence,ObjectiveKind::Patrol}){
    for(unsigned failure=0;failure<15;++failure){
      Policy p(Config{Profile::Rmul3v3});auto s=ready();
      auto goal=objective("candidate",kind,1);goal.withdrawal_support=true;
      s.objectives={goal};if(kind==ObjectiveKind::Recovery)s.hp=100;
      if(kind==ObjectiveKind::Defence)s.heavy.hp=100;
      auto &g=s.objectives[0];
      if(failure==0)g.terrain=Terrain::Highland;
      if(failure==1)g.terrain=Terrain::Tunnel;
      if(failure==2)g.route_uses_highland=true;
      if(failure==3)g.route_uses_tunnel=true;
      if(failure==4)g.reachable=false;
      if(failure==5)g.route_verified=false;
      if(failure==6)g.risk_known=false;
      if(failure==7)g.risk=.51;
      if(failure==8)g.x=std::numeric_limits<double>::quiet_NaN();
      if(failure==9)g.radius=0;
      if(failure==10)g.travel_time_s=-1;
      if(failure==11)g.risk=-.1;
      if(failure==12)g.risk=std::numeric_limits<double>::infinity();
      if(failure==13)g.id="";
      if(failure==14)s.objectives_fresh=false;
      assert_stopped(p.evaluate(s),Mode::WaitPlan);
    }
  }
  Policy p(Config{Profile::Rmul3v3});auto s=ready();s.objectives.clear();
  assert_stopped(p.evaluate(s),Mode::WaitPlan);
  s.hp=100;s.objectives={Objective{}};
  assert_stopped(p.evaluate(s),Mode::WaitPlan);
  auto a=objective("slow",ObjectiveKind::Recovery);a.risk=.2;a.travel_time_s=30;
  auto b=a;b.id="fast";b.travel_time_s=2;
  s.objectives={a,b};s.heavy.position_fresh=true;s.heavy.x=s.heavy.y=5;
  auto out=p.evaluate(s);
  assert(out.mode==Mode::Resupply && out.target=="fast" && !out.fire_allowed);
  s.objectives[0].risk=.1;
  assert(p.evaluate(s).target=="slow");
  s.objectives[0].risk=s.objectives[1].risk=.9;
  assert_stopped(p.evaluate(s),Mode::WaitPlan); // All-low never loosens risk budget.
  s.heavy.hp=s.infantry.hp=100;
  assert_stopped(p.evaluate(s),Mode::WaitPlan);
  for(unsigned field=0;field<3;++field){
    for(double invalid : {-0.1,1.1,std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}){
      Config c{Profile::Rmul3v3};
      if(field==0)c.max_task_risk=invalid;
      if(field==1)c.max_recovery_risk=invalid;
      if(field==2)c.cautious_capture_risk=invalid;
      expect_invalid(c);
    }
  }
}
}  // namespace

int main()
{
  expect_invalid(Config{Profile::Rmul3v3, .3, .8, .2, .5, .3});
  expect_invalid(Config{Profile::Rmul3v3, 0, .8});
  expect_invalid(Config{Profile::Rmul3v3, -.1, .8});
  expect_invalid(Config{Profile::Rmul3v3, .8, .8});
  expect_invalid(Config{Profile::Rmul3v3, .9, .8});
  expect_invalid(Config{Profile::Rmul3v3, .3, 1.1});
  expect_invalid(Config{Profile::Rmul3v3, std::numeric_limits<double>::quiet_NaN(), .8});
  expect_invalid(Config{Profile::Rmul3v3, .3, std::numeric_limits<double>::infinity()});

  Policy unconfigured(Config{});
  assert_stopped(unconfigured.evaluate(ready()), Mode::WaitConfig);
  Policy invalid_profile(Config{static_cast<Profile>(99)});
  assert_stopped(invalid_profile.evaluate(ready()), Mode::WaitConfig);
  assert_stopped(unconfigured.evaluate(State{}), Mode::Disabled);
  Policy unsupported_rmuc(Config{Profile::Rmuc});
  assert_stopped(unsupported_rmuc.evaluate(ready()), Mode::WaitConfig);
  Policy inclusive_upper_bound(Config{Profile::Rmul3v3, .3, 1});
  assert(inclusive_upper_bound.evaluate(ready()).mode == Mode::Patrol);
  test_rmul_3v3();
  test_team_matrix_and_hysteresis();
  test_hold_handover_and_missing_guard();
  test_coverage_and_occupancy();
  test_routes_risk_and_no_map();
  assert(std::string(mode_name(Mode::Resupply)) == "Resupply");
  assert(std::string(mode_name(static_cast<Mode>(99))) == "Unknown");
  return 0;
}
