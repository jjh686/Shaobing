# Decision intent for inspection; never a serial packet or navigation command.
std_msgs/Header header
string mode
string reason
string target
bool fire_allowed
bool free_revive_requested
string team_situation
bool coverage_requested
# Validated status for inspection, using DecisionState VISION_* values.
uint8 vision_status
# coverage_requested is an advisory for local inspection/operator display.
# It sends no order to the human-controlled heavy or infantry robot.
# Permission/request outputs are advisory. Execution requires a separate arbiter.
# free_revive_requested is a LEVEL, not an incrementing request/cost counter.
