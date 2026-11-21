# Observations only. This is not a control/communication link to a teammate.
bool health_valid
builtin_interfaces/Time health_stamp
uint16 hp
uint16 max_hp
bool life_state_valid
bool weakened
# health_stamp covers the oldest dynamic evidence needed for hp and life state.
# max_hp is specific to this robot/performance state, not a fixed team-wide 400.

bool position_valid
builtin_interfaces/Time position_stamp
float64 x
float64 y
# Coordinates use DecisionState.header.frame_id, in metres.
# Missing/outdated coordinates never establish that a post is vacant.
