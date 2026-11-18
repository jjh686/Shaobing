#ifndef RM_CHASSIS_COMM_MCU_LINK_H
#define RM_CHASSIS_COMM_MCU_LINK_H
#include <stdbool.h>
#include "rm_chassis_comm/protocol.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {bool enabled;float vx,vy,wz;} rm_output_t;
/* Single-owner API: caller serializes receive/safety/output/feedback across tasks/ISRs. */
typedef struct {uint8_t route;bool ready,estop,manual,ack_valid,requires_stop,active,timed_out;
 uint16_t fault_mask;uint32_t received_ms;rm_command_t command;} rm_mcu_link_t;
void rm_mcu_init(rm_mcu_link_t*,uint8_t route);
int rm_mcu_receive(rm_mcu_link_t*,const uint8_t*,size_t,uint32_t now_ms);
void rm_mcu_set_safety(rm_mcu_link_t*,bool ready,bool estop,bool manual,uint16_t fault_mask);
void rm_mcu_get_output(rm_mcu_link_t*,uint32_t now_ms,rm_output_t*);
void rm_mcu_feedback(rm_mcu_link_t*,uint32_t now_ms,bool measured_valid,float vx,float vy,float wz,rm_feedback_t*);
#ifdef __cplusplus
}
#endif
#endif
