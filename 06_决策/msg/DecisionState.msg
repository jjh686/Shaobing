# Normalized software interface, NOT an official referee or MCU wire packet.
# Source timestamps are local ROS receive times of the ORIGINAL decoded packets.
# Republishing a snapshot must preserve each source timestamp and validity.
# All stamps must use this node's ROS clock; simulation /clock must be shared.
std_msgs/Header header
bool manual_override

# Local camera tracker observation. UNKNOWN covers missing, stale or unavailable tracking.
uint8 VISION_UNKNOWN=0
uint8 VISION_LOST=1
uint8 VISION_DETECTING=2
uint8 VISION_TRACKING=3
uint8 VISION_TEMP_LOST=4
uint8 VISION_SWITCHING=5
uint8 vision_status
builtin_interfaces/Time vision_stamp

bool game_valid
builtin_interfaces/Time game_stamp
uint8 game_type  # Official serial 0x0001: 4 = RMUL 3V3; 5 = infantry event.
uint8 game_progress  # Official serial 0x0001: 4 = running (not simulator stage=2).
uint16 stage_remaining_s  # Valid with game_valid; time belongs to the reported phase.

bool control_degree_valid
builtin_interfaces/Time control_degree_stamp  # Official 0x0003
uint16 ally_control_degree
uint16 enemy_control_degree

# Activation and ownership are independent. These bits do not provide occupation progress.
bool central_activation_valid
builtin_interfaces/Time central_activation_stamp  # Official 0x0101
bool central_ally_active
bool central_enemy_active

bool robot_valid
builtin_interfaces/Time robot_stamp
uint16 hp
uint16 max_hp
# Normalized rule state, NOT an extra official 0x0201 field.
# Unknown life/weakness state must leave life_state_valid=false.
# robot_stamp covers the oldest original evidence for HP and these flags.
bool life_state_valid
bool weakened
# Combined firing eligibility from hardware AND current rules/status.
# Unknown, dead, penalized or rule-blocked -> false. A power-output bit alone
# is insufficient. robot_stamp must preserve the oldest evidence used here.
bool shooter_enabled

bool ammunition_valid
builtin_interfaces/Time ammunition_stamp
uint16 ammunition
float32 heat
float32 heat_limit
# ammunition_stamp must be the OLDEST source timestamp needed for all 3 values.

bool localization_valid
builtin_interfaces/Time localization_stamp
# Adapter must validate localization/TF before setting localization_valid.

uint8 OWNER_UNKNOWN=0
uint8 OWNER_NEUTRAL=1
uint8 OWNER_ALLY=2
uint8 OWNER_ENEMY=3
uint8 OWNER_CONTESTED=4
TeammateState heavy
TeammateState infantry

bool objectives_valid
builtin_interfaces/Time objectives_stamp
TacticalObjective[<=16] objectives
# All XY positions use header.frame_id, matched to the configured map_frame.
# objectives_stamp is the oldest dynamic zone/risk/route/occupancy evidence.
# Static map/calibration data are validated by revision, not periodically re-dated.
# Repackaging candidates cannot refresh old vacancy/control/risk observations.

bool sentry_valid
builtin_interfaces/Time sentry_stamp
bool free_revive_available
uint8 posture
bool posture_powered
# These posture fields are feedback only; this version never commands posture.
