#ifndef CHASSIS_COMM_H
#define CHASSIS_COMM_H

/* Platform-independent CAN protocol and safety endpoint. One task owns state. */
#include "nav_can.h"
#include "rm_chassis_comm/mcu_link.h"

typedef struct {
    bool remote_online;
    bool navigation_selected;
    bool motors_online;
} ChassisCommSafety;

typedef struct {
    rm_mcu_link_t link;
    nc_rx_t receiver;
    ChassisCommSafety safety;
    bool rx_fault;
    uint32_t accepted_commands;
    uint32_t decode_errors;
} ChassisComm;

void ChassisCommInit(ChassisComm *state);
void ChassisCommSetSafety(ChassisComm *state, ChassisCommSafety safety);
void ChassisCommRxFault(ChassisComm *state);
bool ChassisCommReceive(ChassisComm *state, uint16_t id, unsigned dlc,
                       const uint8_t *data, uint32_t received_ms, uint32_t now_ms);
void ChassisCommGetCommand(ChassisComm *state, uint32_t now_ms, rm_output_t *command);
bool ChassisCommMakeFeedback(ChassisComm *state, uint32_t now_ms,
                            float vx_mps, float vy_mps, float wz_radps,
                            nc_group_t *group);

#endif
