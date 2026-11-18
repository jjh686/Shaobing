#ifndef STUB_ROBOT_DEF_H
#define STUB_ROBOT_DEF_H
#include <stdint.h>
#define TEMP 0
#define RADIUS_WHEEL 60
#define REDUCTION_RATIO_WHEEL 19.0f
typedef enum { CHASSIS_ZERO_FORCE,CHASSIS_ROTATE,CHASSIS_NO_FOLLOW,CHASSIS_FOLLOW_GIMBAL_YAW } chassis_mode_e;
typedef struct { float vx,vy,wz,offset_angle;chassis_mode_e chassis_mode;int chassis_speed_buff;uint8_t posture_cmd;} Chassis_Ctrl_Cmd_s;
typedef struct {float real_vx,real_vy,real_wz;} Chassis_Upload_Data_s;
#endif
