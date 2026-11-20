/* HOST NEGATIVE TEST ONLY: intentionally restores old defect. */
#include "chassis_comm_port.h"
#include "chassis_comm.h"
#include "chassis_comm_config.h"
#include "nav_can_hal.h"
#include "remote_can.h"
#include <string.h>

typedef struct {
    uint8_t bytes[8];
    uint8_t dlc;
    uint16_t id;
    uint32_t received_ms;
} QueuedCanFrame;

static QueuedCanFrame inbox[CHASSIS_COMM_RX_QUEUE_SIZE];
static volatile uint8_t head, tail, overrun;
static ChassisComm endpoint;
static RemoteCanReceiver remote;
volatile uint32_t chassis_rc_online, chassis_rc_navigation, chassis_rc_estop;
volatile uint32_t chassis_rc_accepted, chassis_rc_rejected;
volatile int16_t chassis_rc_channels[10];
static nch_tx_t sender;
static uint32_t feedback_ms;
volatile uint32_t chassis_comm_rx_frames;
volatile uint32_t chassis_comm_accepted_commands;
volatile uint32_t chassis_comm_rx_overruns;
volatile uint32_t chassis_comm_decode_errors;
volatile uint32_t chassis_comm_can_esr, chassis_comm_can_tsr;
volatile uint32_t chassis_comm_task_ticks, chassis_comm_feedback_groups;
volatile uint32_t chassis_nav_rx_frames, chassis_remote_rx_frames;
volatile uint32_t chassis_feedback_tx_mailbox_loaded_frames;
volatile uint32_t chassis_feedback_tx_addtx_errors;
volatile uint32_t chassis_feedback_tx_timeout_groups;
volatile uint32_t chassis_feedback_tx_aborted_groups;
/* Self-healing guard: nch_pump() aborts the software group but a frame already on the
 * wire keeps its request bit set, which latches nch_busy() forever and stops the
 * feedback stream. Release the mask after NC_TX_STALL_MS. */
#define NC_TX_STALL_MS 500u
volatile uint32_t chassis_feedback_tx_stall_recoveries;
static uint32_t feedback_stall_since;
static void recover_stalled_feedback_tx(uint32_t now)
{
    if (!nch_busy(&sender)) { feedback_stall_since = 0; return; }
    if (!feedback_stall_since) { feedback_stall_since = now ? now : 1u; return; }
    if ((uint32_t)(now - feedback_stall_since) > NC_TX_STALL_MS) {
        chassis_feedback_tx_stall_recoveries++;
        nch_reset(&sender);
        feedback_stall_since = 0;
    }
}

static void update_feedback_tx_debug(void)
{
    chassis_feedback_tx_mailbox_loaded_frames=sender.tx_mailbox_loaded_frames;
    chassis_feedback_tx_addtx_errors=sender.tx_addtx_errors;
    chassis_feedback_tx_timeout_groups=sender.tx_timeout_groups;
    chassis_feedback_tx_aborted_groups=sender.tx_aborted_groups;
}

static ChassisCommSafety read_safety(int motors_online)
{
    const uint32_t now=HAL_GetTick();
    const bool online=rc_can_fresh(&remote,now);
    const bool navigation=rc_can_nav(&remote,now);
    chassis_rc_online=online;chassis_rc_navigation=navigation;
    chassis_rc_estop=!online||(remote.sample.flags&RC_CAN_ESTOP)!=0;
    chassis_rc_accepted=remote.accepted;chassis_rc_rejected=remote.rejected;
    for(unsigned i=0;i<6;i++)chassis_rc_channels[i]=remote.sample.axes[i];
    for(unsigned i=0;i<4;i++)chassis_rc_channels[6+i]=remote.sample.switches[i];
    ChassisCommSafety safety = {
        .remote_online = online,
        .navigation_selected = navigation,
        .motors_online = motors_online != 0,
    };
    return safety;
}

/* ISR only copies a frame. Decode/safety/control are owned by the chassis task. */
static void receive_can(CANInstance *can)
{
    if(can->rx_id>=NC_COMMAND_BASE&&can->rx_id<=NC_COMMAND_BASE+2u)chassis_nav_rx_frames++;
    else if(can->rx_id>=RC_CAN_BASE&&can->rx_id<=RC_CAN_BASE+2u)chassis_remote_rx_frames++;
    const uint8_t next = (head + 1u) % CHASSIS_COMM_RX_QUEUE_SIZE;
    if (next == tail) {
        overrun = 1;
        chassis_comm_rx_overruns++;
        return;
    }
    memcpy(inbox[head].bytes, can->rx_buff, 8);
    inbox[head].dlc = can->rx_len;
    inbox[head].id = (uint16_t)can->rx_id;
    inbox[head].received_ms = HAL_GetTick();
    __DMB();
    head = next;
    chassis_comm_rx_frames++;
}

void ChassisCommPortInit(void)
{
    ChassisCommInit(&endpoint);
    sender.can = &CHASSIS_COMM_CAN_HANDLE;
    memset(&remote,0,sizeof(remote));
    for (unsigned i = 0; i < 6; i++) {
        CAN_Init_Config_s config = {
            .can_handle = &CHASSIS_COMM_CAN_HANDLE,
            .rx_fifo_policy = CAN_RX_FIFO_POLICY_AUTO,
            .tx_id = NC_FEEDBACK_BASE + (i%3),
            .rx_id = i<3 ? NC_COMMAND_BASE+i : RC_CAN_BASE+i-3,
            .can_module_callback = receive_can,
        };
        CANRegister(&config);
    }
}

void ChassisCommPortCommand(Chassis_Ctrl_Cmd_s *command, int motors_online)
{
    const uint32_t now = HAL_GetTick();
    chassis_comm_task_ticks = now;
#ifndef CHASSIS_COMM_HOST_TEST
    chassis_comm_can_esr = CHASSIS_COMM_CAN_HANDLE.Instance->ESR;
    chassis_comm_can_tsr = CHASSIS_COMM_CAN_HANDLE.Instance->TSR;
#endif
    if (overrun) {
        const uint32_t irq = __get_PRIMASK();
        __disable_irq();
        tail = head;
        overrun = 0;
        __set_PRIMASK(irq);
        rc_can_fault(&remote);
        ChassisCommRxFault(&endpoint);
    }
    ChassisCommSetSafety(&endpoint, read_safety(motors_online));
    while (tail != head) {
        const QueuedCanFrame frame = inbox[tail];
        __DMB();
        tail = (tail + 1u) % CHASSIS_COMM_RX_QUEUE_SIZE;
        if(frame.id>=RC_CAN_BASE&&frame.id<=RC_CAN_BASE+2){
            rc_can_push(&remote,frame.id,frame.dlc,frame.bytes,frame.received_ms,now);
        }else{
            ChassisCommSetSafety(&endpoint,read_safety(motors_online));
            ChassisCommReceive(&endpoint, frame.id, frame.dlc, frame.bytes, frame.received_ms, now);
        }
        /* Apply each transition, including a brief failsafe followed by recovery. */
        ChassisCommSetSafety(&endpoint,read_safety(motors_online));
    }
    chassis_comm_accepted_commands = endpoint.accepted_commands;
    chassis_comm_decode_errors = endpoint.decode_errors;
    rm_output_t output;
    ChassisCommGetCommand(&endpoint, now, &output);
    memset(command, 0, sizeof(*command));
    command->chassis_mode = CHASSIS_ZERO_FORCE;
    if (!output.enabled) {
        return;
    }
    /* Original driver's linear reference is rotor degrees/second (aps). */
    const float aps_to_mps = 6.28318530718f * (RADIUS_WHEEL / 1000.0f) /
        (360.0f * REDUCTION_RATIO_WHEEL);
    command->vx = output.vx * CHASSIS_COMM_VX_SIGN / aps_to_mps;
    command->vy = output.vy * CHASSIS_COMM_VY_SIGN / aps_to_mps;
    command->wz = output.wz * CHASSIS_COMM_WZ_SIGN /
        (1000.0f * 0.01745329252f * aps_to_mps);
    command->offset_angle = 0;
    command->chassis_speed_buff = 100;
    command->chassis_mode = CHASSIS_NO_FOLLOW;
}

void ChassisCommPortFeedback(const Chassis_Upload_Data_s *measured, int motors_online)
{
    const uint32_t now = HAL_GetTick();
    recover_stalled_feedback_tx(now);
    nch_pump(&sender, now);
    update_feedback_tx_debug();
    if ((uint32_t)(now - feedback_ms) < CHASSIS_COMM_FEEDBACK_PERIOD_MS) {
        return;
    }
    feedback_ms = now;
    ChassisCommSetSafety(&endpoint, read_safety(motors_online));
    nc_group_t group;
    if (ChassisCommMakeFeedback(&endpoint, now,
        measured->real_vx * CHASSIS_COMM_VX_SIGN,
        measured->real_vy * CHASSIS_COMM_VY_SIGN,
        measured->real_wz * 0.01745329252f * CHASSIS_COMM_WZ_SIGN, &group) &&
        nch_start(&sender, &group, now))
    {
        chassis_comm_feedback_groups++;
        nch_pump(&sender, now);
    }
    update_feedback_tx_debug();
}
