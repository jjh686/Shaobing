#ifndef CHASSIS_COMM_PORT_H
#define CHASSIS_COMM_PORT_H
#include "robot_def.h"

/* Call Init once in ChassisInit; Command/Feedback from the 5 ms chassis task. */
void ChassisCommPortInit(void);
void ChassisCommPortCommand(Chassis_Ctrl_Cmd_s *command, int motors_online);
void ChassisCommPortFeedback(const Chassis_Upload_Data_s *measured, int motors_online);

/* Debugger counters only; never use them to grant motion permission. */
extern volatile uint32_t chassis_comm_rx_frames;
extern volatile uint32_t chassis_comm_accepted_commands;
extern volatile uint32_t chassis_comm_rx_overruns;
extern volatile uint32_t chassis_comm_decode_errors;
#endif
