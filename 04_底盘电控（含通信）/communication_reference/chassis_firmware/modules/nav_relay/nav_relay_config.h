#ifndef NAV_RELAY_CONFIG_H
#define NAV_RELAY_CONFIG_H
#ifndef NAV_RELAY_ALLOW_MOTION
#define NAV_RELAY_ALLOW_MOTION 0
#endif
#ifndef NAV_AXES_VERIFIED
#define NAV_AXES_VERIFIED 0
#endif
/* Defaults are theoretical; verify ROS x forward, y left, yaw CCW on real wheels. */
#define NAV_RELAY_VX_SIGN 1.0f
#define NAV_RELAY_VY_SIGN 1.0f
#define NAV_RELAY_WZ_SIGN 1.0f
#endif
