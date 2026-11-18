#include "chassis_comm.h"
#include "chassis_comm_config.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static bool send(ChassisComm *state, const rm_command_t *command, uint32_t received, uint32_t now)
{
    nc_group_t group;
    assert(nc_command_group(command, &group));
    bool accepted = false;
    for (unsigned i = 0; i < 3; i++) {
        accepted = ChassisCommReceive(state, NC_COMMAND_BASE + i, 8, group.data[i], received, now);
    }
    return accepted;
}
static rm_feedback_t feedback(ChassisComm *state, uint32_t now)
{
    nc_group_t group;
    rm_feedback_t result;
    assert(ChassisCommMakeFeedback(state, now, 0.12f, -0.13f, 0.14f, &group));
    assert(nc_feedback_decode(&group, &result));
    return result;
}
static void stopped(ChassisComm *state, uint32_t now)
{
    rm_output_t out;
    ChassisCommGetCommand(state, now, &out);
    assert(!out.enabled && out.vx == 0 && out.vy == 0 && out.wz == 0);
}
int main(void)
{
    ChassisComm a, b;
    ChassisCommInit(&a);
    ChassisCommInit(&b);
    ChassisCommSafety ready = {true, true, true};
    ChassisCommSetSafety(&a, ready);
    ChassisCommSetSafety(&b, ready);
    rm_command_t command = {.route = 2, .session = 99, .sequence = 65534};
    assert(send(&a, &command, 10, 10));
    assert(feedback(&a, 10).session == 99);
    assert(feedback(&b, 10).session == 0);
    assert(!send(&a, &command, 11, 11));
    command.sequence++;
    assert(send(&a, &command, 12, 12));
    command.sequence = 0;
    command.flags = RM_FLAG_ENABLE;
    command.ttl_ms = 100;
    command.vx = 0.1f;
#if CHASSIS_COMM_ALLOW_MOTION && CHASSIS_COMM_AXES_VERIFIED
    assert(send(&a, &command, 20, 20));
    rm_output_t output;
    ChassisCommGetCommand(&a, 21, &output);
    assert(output.enabled && fabsf(output.vx - 0.1f) < 1e-6f);
    assert(!send(&a, &command, 22, 22));
    command.session = 100; command.sequence++;
    assert(!send(&a, &command, 23, 23));
    command.session = 99;
    stopped(&a, 120);
    assert(!send(&a, &command, 121, 121));
#else
    assert(!send(&a, &command, 20, 20));
    stopped(&a, 21);
    assert((feedback(&a, 21).fault_mask & (NC_FAULT_LOCK | NC_FAULT_UNITS)) ==
           (NC_FAULT_LOCK | NC_FAULT_UNITS));
#endif
    command = (rm_command_t){.route = 2, .session = 99, .sequence = 2};
    nc_group_t broken;
    assert(nc_command_group(&command, &broken));
    broken.data[1][1] ^= 1;
    for (unsigned i = 0; i < 3; i++) {
        assert(!ChassisCommReceive(&a, NC_COMMAND_BASE+i, 8, broken.data[i], 130, 130));
    }
    assert(feedback(&a, 130).sequence != 2);
    assert(!send(&a, &command, 140, 151));
    assert(feedback(&a, 151).fault_mask & NC_FAULT_RX);
    assert(send(&a, &command, 152, 152));
    assert(!(feedback(&a, 152).fault_mask & NC_FAULT_RX));
    ChassisCommSetSafety(&a, (ChassisCommSafety){false, true, true});
    stopped(&a, 153);
    assert(feedback(&a, 153).fault_mask & NC_FAULT_RC);
    assert(feedback(&a, 153).status & RM_STATUS_ESTOP);
    ChassisCommSetSafety(&a, ready);
    ChassisCommInit(&b);
    ChassisCommSetSafety(&b, ready);
    command.sequence=1;
    assert(send(&b, &command, UINT32_MAX-30, UINT32_MAX-30));
    command.flags=RM_FLAG_ENABLE; command.ttl_ms=100; command.vx=0.1f; command.sequence++;
    const bool allowed = CHASSIS_COMM_ALLOW_MOTION && CHASSIS_COMM_AXES_VERIFIED;
    assert(send(&b, &command, UINT32_MAX-20, UINT32_MAX-20) == allowed);
    stopped(&b, 80);
    puts("PASS module core: independent instances, STOP/session/sequence wrap, replay, CRC, backlog fault, RC loss, TTL and tick wrap");
}
