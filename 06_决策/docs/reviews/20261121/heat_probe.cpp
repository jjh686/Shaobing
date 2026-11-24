#include <iostream>
#include "rm2027_decision/policy.hpp"
using namespace rm2027_decision;
int main() {
  Policy policy(Config{Profile::Rmul3v3});
  State s;
  s.enabled=s.game_fresh=s.robot_fresh=s.ammo_fresh=true;
  s.localization_fresh=s.game_running=s.life_state_valid=s.objectives_fresh=true;
  s.game_type=4; s.hp=s.max_hp=400;
  s.ammo=100; s.shooter_enabled=true; s.heat=255; s.heat_limit=260;
  Objective goal;
  goal.id="patrol"; goal.kind=ObjectiveKind::Patrol;
  goal.reachable=goal.route_verified=goal.risk_known=true;
  goal.risk=0.1;
  s.objectives.push_back(goal);
  const auto intent=policy.evaluate(s);
  std::cout << "{\"heat\":255,\"limit\":260,\"next_17mm_shot_adds\":10,\"fire_allowed\":"
    << (intent.fire_allowed ? "true" : "false") << "}\n";
}
