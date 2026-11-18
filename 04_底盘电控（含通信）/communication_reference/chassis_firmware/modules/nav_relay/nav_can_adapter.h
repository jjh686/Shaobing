#ifndef NAV_CAN_ADAPTER_H
#define NAV_CAN_ADAPTER_H
#include "robot_def.h"
void NavCanInit(void);
void NavCanCommand(Chassis_Ctrl_Cmd_s *,int motors_online);
void NavCanFeedback(const Chassis_Upload_Data_s *,int motors_online);
#endif
