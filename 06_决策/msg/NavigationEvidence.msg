# Produced on this robot by the localization/route validation layer.
# Source timestamps come from observations, not time of republishing this message.
std_msgs/Header header
bool localization_valid
builtin_interfaces/Time localization_stamp
bool objectives_valid
builtin_interfaces/Time objectives_stamp
TacticalObjective[<=16] objectives
# Optional local observations. Do not populate from robot-to-robot communication
# or from a generic referee position payload without RMUL positioning hardware.
TeammateState heavy
TeammateState infantry
