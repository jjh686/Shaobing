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
extern volatile uint32_t chassis_nav_rx_frames, chassis_remote_rx_frames;
extern volatile uint32_t chassis_feedback_tx_mailbox_loaded_frames;
extern volatile uint32_t chassis_feedback_tx_addtx_errors;
extern volatile uint32_t chassis_feedback_tx_timeout_groups;
extern volatile uint32_t chassis_feedback_tx_aborted_groups;
extern volatile uint32_t chassis_feedback_tx_stall_recoveries;
extern volatile uint32_t chassis_rc_online, chassis_rc_navigation, chassis_rc_estop;
extern volatile uint32_t chassis_rc_accepted, chassis_rc_rejected;
extern volatile int16_t chassis_rc_channels[10];
extern volatile uint32_t chassis_comm_can_esr, chassis_comm_can_tsr;
extern volatile uint32_t chassis_comm_task_ticks, chassis_comm_feedback_groups;
#endif
