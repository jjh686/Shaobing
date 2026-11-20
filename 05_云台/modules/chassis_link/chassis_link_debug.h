#ifndef CHASSIS_LINK_DEBUG_H
#define CHASSIS_LINK_DEBUG_H

#include <stdint.h>

/* 保留原nav_*符号供SWD观察；诊断计数不参与控制和许可判断。 */
extern volatile uint32_t nav_usb_received, nav_can_groups, nav_feedback_received, nav_gateway_drops;
extern volatile uint32_t nav_rc_groups, nav_rc_online, nav_rc_estop, nav_rc_age_ms;
extern volatile uint32_t nav_can_esr, nav_can_tsr, nav_can_rx_frames, nav_can_decode_errors;
extern volatile uint32_t nav_rx_last_id, nav_rx_foreign_frames;
extern volatile uint32_t nav_rx_groups_formed, nav_rx_push_fail, nav_rx_prev_id;
extern volatile uint32_t nav_can_task_ticks;
extern volatile uint32_t nav_command_tx_mailbox_loaded_frames, nav_command_tx_addtx_errors;
extern volatile uint32_t nav_command_tx_timeout_groups, nav_command_tx_aborted_groups;
extern volatile uint32_t nav_remote_tx_mailbox_loaded_frames, nav_remote_tx_addtx_errors;
extern volatile uint32_t nav_remote_tx_timeout_groups, nav_remote_tx_aborted_groups;
extern volatile uint32_t nav_tx_stall_recoveries;

#endif
