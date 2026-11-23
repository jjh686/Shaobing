/* 云台板间通信模块：USB导航队列、CAN1收发、SBUS快照转发及诊断。
 * USB/CAN中断只投递数据；解码和发送由独立普通优先级任务完成。
 * 调度实现见chassis_link_task.c，线协议继续复用nav_conversion。 */
#include "chassis_link.h"
#include "chassis_link_config.h"
#include "chassis_link_debug.h"
#include "nav_joint_config.h"
#include "nav_gateway.h"
#include "rm_chassis_comm/gimbal_protocol.h"
#include "bsp_can.h"
#include "../nav_conversion/nav_can_hal.h"
#include "bsp_usb.h"
#include "remote_control.h"
#include "remote_can.h"
#include <string.h>

/* 模块私有队列和会话状态。 */
typedef struct
{
    uint8_t bytes[RM_FRAME_SIZE];
    uint32_t tick;
} command_packet_t;
typedef struct
{
    uint8_t bytes[8];
    uint16_t id;
    uint8_t dlc;
    uint32_t tick;
} can_packet_t;
static command_packet_t commands[CHASSIS_LINK_COMMAND_QUEUE_SIZE];
static can_packet_t incoming[CHASSIS_LINK_FEEDBACK_QUEUE_SIZE];
static volatile uint8_t qh, qt, rh, rt, reset_requested;
static nav_gateway_t gateway;
static nc_rx_t receiver;
static nch_tx_t sender = {.can = &CHASSIS_LINK_CAN_HANDLE}; /* Gimbal CAN1 <-> chassis CAN2. */
static uint8_t pending[RM_FRAME_SIZE], pending_valid;
static uint32_t pending_ms, joint_ms;
static nch_tx_t remote_sender = {.can = &CHASSIS_LINK_CAN_HANDLE};
static uint32_t remote_sent_ms;
static uint16_t remote_sequence;
static uint8_t remote_started;
volatile uint32_t nav_rc_groups, nav_rc_online, nav_rc_estop, nav_rc_age_ms;
volatile uint32_t nav_command_expired, nav_command_busy, nav_command_invalid;
volatile uint32_t nav_feedback_expired, nav_feedback_replaced, nav_usb_busy;
volatile uint32_t nav_command_queue_full, nav_feedback_queue_full;
volatile uint32_t nav_task_max_gap_ms, nav_command_max_age_ms, nav_feedback_max_age_ms;
volatile nc_rx_stats_t nav_feedback_rx_stats;
static uint32_t task_last_ms;
static uint8_t task_seen;
/* Retry a stalled abort without forgetting hardware ownership. CAN1 bus-off
 * recovery is enabled; a reused motor mailbox is released by the BSP hook. */
volatile uint32_t nav_tx_stall_recoveries;
static uint32_t tx_stall_since;
volatile uint32_t nav_usb_received, nav_can_groups, nav_feedback_received, nav_gateway_drops;
/* Read-only SRAM mirrors let SWD distinguish bus errors from decode errors.
 * HAL ErrorCode alone stays zero unless CAN error notifications are enabled. */
volatile uint32_t nav_can_esr, nav_can_tsr, nav_can_rx_frames, nav_can_decode_errors;
/* Diagnose "frames arrive but no valid group": last STDID seen and how many frames
 * matched something outside NC_FEEDBACK_BASE..+2. */
volatile uint32_t nav_rx_last_id, nav_rx_foreign_frames;
/* nc_rx_push() returns false without any counter when a group never assembles
 * (wrong order / stale) or when its CRC seal fails. Split those out. */
volatile uint32_t nav_rx_groups_formed, nav_rx_push_fail, nav_rx_prev_id;
volatile uint32_t nav_can_task_ticks;
volatile uint32_t nav_command_tx_mailbox_loaded_frames, nav_command_tx_addtx_errors;
volatile uint32_t nav_command_tx_timeout_groups, nav_command_tx_aborted_groups;
volatile uint32_t nav_remote_tx_mailbox_loaded_frames, nav_remote_tx_addtx_errors;
volatile uint32_t nav_remote_tx_timeout_groups, nav_remote_tx_aborted_groups;
/* 邮箱所有权、发送恢复和遥控转发，由本模块统一管理。 */
void CANOnTxMailboxAssigned(CAN_HandleTypeDef *can, uint32_t mailbox)
{
    /* A completed navigation slot may already contain a queued yaw command.
     * Never retain ownership of that slot or abort the motor's new request. */
    if (sender.can == can)
        sender.mailboxes &= ~mailbox;
    if (remote_sender.can == can)
        remote_sender.mailboxes &= ~mailbox;
}
static void recover_stalled_tx(uint32_t now)
{
    if (!nch_busy(&sender) && !nch_busy(&remote_sender))
    {
        tx_stall_since = 0;
        return;
    }
    if (!tx_stall_since)
    {
        tx_stall_since = now ? now : 1u;
        return;
    }
    if ((uint32_t)(now - tx_stall_since) > CHASSIS_LINK_TX_STALL_MS)
    {
        nav_tx_stall_recoveries++;
        nch_reset(&sender);
        nch_reset(&remote_sender);
        tx_stall_since = 0;
    }
}
static void forward_remote(uint32_t now)
{
    nch_pump(&remote_sender, now);
    if (nch_busy(&remote_sender) ||
        (remote_started && (uint32_t)(now - remote_sent_ms) < CHASSIS_LINK_REMOTE_PERIOD_MS))
        return;
    FS_RC_ctrl_t rc = {0};
    RemoteCanSample sample = {0};
    nc_group_t group;
    bool online = FS_RemoteControlGetSnapshot(&rc) != 0;
    /* An IRQ may publish the snapshot after the task sampled its entry tick. */
    now = HAL_GetTick();
    sample.sequence = ++remote_sequence;
    sample.generation = (uint16_t)rc.fault_generation;
    uint32_t age = now - rc.received_ms;
    sample.age_ms = age > 65535u ? 65535u : (uint16_t)age;
    sample.axes[0] = rc.rc.rocker_r_;
    sample.axes[1] = rc.rc.rocker_r1;
    sample.axes[2] = rc.rc.rocker_l1;
    sample.axes[3] = rc.rc.rocker_l_;
    int16_t knob_l_raw = (int16_t)(rc.rc.knob_l + 240);
    int16_t knob_r_raw = (int16_t)(rc.rc.knob_r + 240);
    if (knob_l_raw < 0) knob_l_raw = 0;
    if (knob_l_raw > 2047) knob_l_raw = 2047;
    if (knob_r_raw < 0) knob_r_raw = 0;
    if (knob_r_raw > 2047) knob_r_raw = 2047;
    sample.axes[4] = knob_l_raw;
    sample.axes[5] = knob_r_raw;
    for (unsigned i = 0; i < 2; i++)
    {
        int value = rc.channel[6 + i];
        if (value < 0 || value > 2)
        {
            online = false;
            value = 0;
        }
        sample.switches[i] = (uint8_t)value;
    }
    sample.switches[2] = 1u; /* ET08A 为 8 通道, CH9/CH10 固定填中位 */
    sample.switches[3] = 1u;
    online = online && age <= RC_CAN_TIMEOUT_MS;
    /* ET08A: CH8(SC = switches[1]) 上档(0) = 急停 */
    sample.flags =
        (online ? RC_CAN_ONLINE : 0) | (!online || sample.switches[1] == 0 ? RC_CAN_ESTOP : 0);
    if (!rc_sample_ok(&sample))
    {
        memset(sample.axes, 0, sizeof(sample.axes));
        sample.flags = RC_CAN_ESTOP;
    }
    nav_rc_online = (sample.flags & RC_CAN_ONLINE) != 0;
    nav_rc_estop = (sample.flags & RC_CAN_ESTOP) != 0;
    nav_rc_age_ms = sample.age_ms;
    if (rc_can_encode(&sample, &group) && nch_start(&remote_sender, &group, now))
    {
        remote_started = 1;
        remote_sent_ms = now;
        nav_rc_groups++;
        nch_pump(&remote_sender, now);
    }
}

static void update_tx_debug(void)
{
    nav_feedback_rx_stats = receiver.stats;
    nav_command_tx_mailbox_loaded_frames = sender.tx_mailbox_loaded_frames;
    nav_command_tx_addtx_errors = sender.tx_addtx_errors;
    nav_command_tx_timeout_groups = sender.tx_timeout_groups;
    nav_command_tx_aborted_groups = sender.tx_aborted_groups;
    nav_remote_tx_mailbox_loaded_frames = remote_sender.tx_mailbox_loaded_frames;
    nav_remote_tx_addtx_errors = remote_sender.tx_addtx_errors;
    nav_remote_tx_timeout_groups = remote_sender.tx_timeout_groups;
    nav_remote_tx_aborted_groups = remote_sender.tx_aborted_groups;
}
/* CAN接收回调：保持短小，不在中断中解码整组。 */
static void can_receive(CANInstance *can)
{
    nav_can_rx_frames++;
    {
        uint16_t rid = (uint16_t)can->rx_id;
        nav_rx_prev_id = nav_rx_last_id;
        nav_rx_last_id = rid;
        if (rid < NC_FEEDBACK_BASE || rid > NC_FEEDBACK_BASE + 2u)
            nav_rx_foreign_frames++;
    }
    uint8_t next = (rh + 1) % CHASSIS_LINK_FEEDBACK_QUEUE_SIZE;
    if (next == rt)
    {
        nav_feedback_queue_full++;
        nav_gateway_drops++;
        reset_requested = 1;
        return;
    }
    memcpy(incoming[rh].bytes, can->rx_buff, 8);
    incoming[rh].id = (uint16_t)can->rx_id;
    incoming[rh].dlc = can->rx_len;
    incoming[rh].tick = HAL_GetTick();
    __DMB();
    rh = next;
}
/* 对外接入：初始化及USB投递。 */
void ChassisLinkResetUsbSession(void) { USBTransmitReset(); reset_requested = 1; }
void ChassisLinkReceiveUsb(const uint8_t *bytes, unsigned n, uint32_t received)
{
    uint8_t next = (qh + 1) % CHASSIS_LINK_COMMAND_QUEUE_SIZE;
    if (n != RM_FRAME_SIZE || next == qt)
    {
        if (next == qt) nav_command_queue_full++;
        else nav_command_invalid++;
        nav_gateway_drops++;
        reset_requested = 1;
        return;
    }
    memcpy(commands[qh].bytes, bytes, n);
    commands[qh].tick = received;
    __DMB();
    qh = next;
    nav_usb_received++;
}
void ChassisLinkInit(void)
{
    /* Autonomous aiming skips RC in RobotCMDInit, but chassis permission still
     * requires the real SBUS source. The remote service is initialized once. */
    FS_RemoteControlInit(&huart3);
    ng_init(&gateway);
    remote_started = 0;
    remote_sequence = 0;
    for (unsigned i = 0; i < 3; i++)
    {
        CAN_Init_Config_s c = {.can_handle = &CHASSIS_LINK_CAN_HANDLE,
                               .tx_id = NC_COMMAND_BASE + i,
                               .rx_fifo_policy = CAN_RX_FIFO_POLICY_FIFO0,
                               .rx_id = NC_FEEDBACK_BASE + i,
                               .can_module_callback = can_receive};
        CANRegister(&c);
    }
    CANEnableDiagnostics(&CHASSIS_LINK_CAN_HANDLE);
}
static void pump(uint32_t now) { nch_pump(&sender, now); }
/* 周期处理：保持原顺序及反馈/遥控/自瞄之间的调度关系。 */
void ChassisLinkUpdate(void)
{
    uint32_t now = HAL_GetTick();
    if (task_seen && (uint32_t)(now - task_last_ms) > nav_task_max_gap_ms)
        nav_task_max_gap_ms = now - task_last_ms;
    task_seen = 1;
    task_last_ms = now;
    nav_can_task_ticks = now;
    CANUpdateDiagnostics(&CHASSIS_LINK_CAN_HANDLE);
    USBTransmitPump();
    recover_stalled_tx(now);
    /* RC transport runs independently of USB connection / navigation sessions. */
    forward_remote(now);
    now = HAL_GetTick();
    update_tx_debug();
#ifndef CHASSIS_COMM_HOST_TEST
    nav_can_esr = CHASSIS_LINK_CAN_HANDLE.Instance->ESR;
    nav_can_tsr = CHASSIS_LINK_CAN_HANDLE.Instance->TSR;
#endif
    const bool usb_ready = USBIsConfigured() != 0;
    if (reset_requested)
    {
        uint32_t irq = __get_PRIMASK();
        __disable_irq();
        qh = qt = 0;
        reset_requested = 0;
        __set_PRIMASK(irq);
        ng_init(&gateway);
        nch_reset(&sender);
        pending_valid = 0;
    }
    pump(now);
    /* The chassis link must work over SWD-only benches too: draining the CAN ring is
     * independent of whether the PC is attached on USB. Only USB-bound state is
     * dropped when the PC is absent. */
    while (rt != rh)
    {
        can_packet_t p = incoming[rt];
        __DMB();
        rt = (rt + 1) % CHASSIS_LINK_FEEDBACK_QUEUE_SIZE;
        now = HAL_GetTick();
        uint32_t age = now - p.tick;
        if (age > nav_feedback_max_age_ms) nav_feedback_max_age_ms = age;
        nc_group_t group;
        rm_feedback_t f;
        if ((uint32_t)(now - p.tick) > NC_GROUP_MS)
        {
            nav_feedback_expired++;
            receiver.active = 0;
            continue;
        }
        if (nc_rx_push(&receiver, NC_FEEDBACK_BASE, p.id, false, false, p.dlc, p.bytes, p.tick,
                       &group))
        {
            nav_rx_groups_formed++;
            if (nc_feedback_decode(&group, &f) && rm_encode_feedback(&f, pending))
            {
                if (pending_valid) nav_feedback_replaced++;
                pending_ms = p.tick;
                pending_valid = 1;
                nav_feedback_received++;
            }
            else
                nav_can_decode_errors++;
        }
        else
            nav_rx_push_fail++;
    }
    if (!usb_ready)
    {
        uint32_t irq = __get_PRIMASK();
        __disable_irq();
        qh = qt = 0;
        __set_PRIMASK(irq);
        pending_valid = 0;
        update_tx_debug();
        return;
    }
    while (qt != qh)
    {
        command_packet_t p = commands[qt];
        now = HAL_GetTick();
        uint32_t age = now - p.tick;
        if (age > nav_command_max_age_ms) nav_command_max_age_ms = age;
        if (age > NC_GROUP_MS)
        {
            nav_command_expired++;
            nav_gateway_drops++;
        }
        else if (nch_busy(&sender))
        {
            nav_command_busy++;
            break; /* Keep the fresh packet until the preceding group completes. */
        }
        else
        {
            nc_group_t group;
            if (!ng_command(&gateway, p.bytes, p.tick, now, &group) ||
                !nch_start(&sender, &group, now))
            {
                nav_command_invalid++;
                nav_gateway_drops++;
            }
            else
            {
                nav_can_groups++;
                pump(now);
            }
        }
        __DMB();
        qt = (qt + 1) % CHASSIS_LINK_COMMAND_QUEUE_SIZE;
    }
    /* Prioritize chassis ACK over optional joint telemetry. CDC owns accepted
     * buffers and arbitrates with the original high-priority SP status sender. */
    if (pending_valid)
    {
        now = HAL_GetTick();
        if ((uint32_t)(now - pending_ms) > NC_GROUP_MS)
        {
            nav_feedback_expired++;
            pending_valid = 0;
        }
        else if (USBTransmitStamped(pending, sizeof(pending), pending_ms) == USBD_OK)
            pending_valid = 0;
        else nav_usb_busy++;
        USBTransmitPump();
        update_tx_debug();
        return;
    }
    now = HAL_GetTick();
    if (gateway.gate.ack_valid && (uint32_t)(now - joint_ms) >= CHASSIS_LINK_JOINT_PERIOD_MS)
    {
        joint_ms = now;
        rm_gimbal_t g = {0};
        g.route = 2;
        g.uptime_ms = now;
        g.sample_ms = now;
        g.command_received_ms = gateway.command_received_ms;
        g.session = gateway.gate.command.session;
        g.sequence = gateway.gate.command.sequence;
        /* No chassis-relative calibration exists in the supplied sources.
         * Zero values carry NO VALID/CALIBRATED bits, so the PC rejects them. */
        g.status = 0;
        uint8_t frame[RM_GIMBAL_FRAME_SIZE];
        if (rm_encode_gimbal(&g, frame))
            USBTransmit(frame, sizeof(frame));
    }
    update_tx_debug();
}
