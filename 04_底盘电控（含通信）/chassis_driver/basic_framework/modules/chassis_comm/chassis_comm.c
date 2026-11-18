#include "chassis_comm.h"
#include "chassis_comm_config.h"
#include <math.h>
#include <string.h>

static void apply_safety(ChassisComm *state)
{
    uint16_t fault = (!CHASSIS_COMM_ALLOW_MOTION ? NC_FAULT_LOCK : 0) |
        (!CHASSIS_COMM_AXES_VERIFIED ? NC_FAULT_UNITS : 0) |
        (!state->safety.remote_online || !state->safety.navigation_selected ? NC_FAULT_RC : 0) |
        (!state->safety.motors_online ? NC_FAULT_MOTOR : 0) |
        (state->rx_fault ? NC_FAULT_RX : 0);
    rm_mcu_set_safety(&state->link, fault == 0, !state->safety.remote_online,
                      !state->safety.remote_online || !state->safety.navigation_selected, fault);
}

void ChassisCommInit(ChassisComm *state)
{
    memset(state, 0, sizeof(*state));
    rm_mcu_init(&state->link, CHASSIS_COMM_ROUTE);
    apply_safety(state);
}

void ChassisCommSetSafety(ChassisComm *state, ChassisCommSafety safety)
{
    state->safety = safety;
    apply_safety(state);
}

void ChassisCommRxFault(ChassisComm *state)
{
    state->receiver.active = 0;
    state->rx_fault = true;
    apply_safety(state);
}

bool ChassisCommReceive(ChassisComm *state, uint16_t id, unsigned dlc,
                       const uint8_t *data, uint32_t received_ms, uint32_t now_ms)
{
    nc_group_t group;
    rm_command_t command;
    uint8_t frame[RM_FRAME_SIZE];
    if ((uint32_t)(now_ms - received_ms) > NC_GROUP_MS) {
        ChassisCommRxFault(state);
        return false;
    }
    if (!nc_rx_push(&state->receiver, NC_COMMAND_BASE, id, false, false, dlc,
                     data, received_ms, &group)) {
        return false;
    }
    if (!nc_command_decode(&group, &command)) {
        state->decode_errors++;
        return false;
    }
    if (!(command.flags & RM_FLAG_ENABLE)) {
        state->rx_fault = false;
        apply_safety(state);
    }
    if (!rm_encode_command(&command, frame) ||
        !rm_mcu_receive(&state->link, frame, sizeof(frame), state->receiver.started_ms)) {
        return false;
    }
    state->accepted_commands++;
    return true;
}

void ChassisCommGetCommand(ChassisComm *state, uint32_t now_ms, rm_output_t *command)
{
    apply_safety(state);
    rm_mcu_get_output(&state->link, now_ms, command);
}

bool ChassisCommMakeFeedback(ChassisComm *state, uint32_t now_ms,
                            float vx_mps, float vy_mps, float wz_radps, nc_group_t *group)
{
    rm_feedback_t feedback;
    apply_safety(state);
    const bool valid = state->safety.motors_online &&
        isfinite(vx_mps) && isfinite(vy_mps) && isfinite(wz_radps) &&
        fabsf(vx_mps) <= 20 && fabsf(vy_mps) <= 20 && fabsf(wz_radps) <= 32.767f;
    rm_mcu_feedback(&state->link, now_ms, valid, vx_mps, vy_mps, wz_radps, &feedback);
    return nc_feedback_group(&feedback, group);
}
