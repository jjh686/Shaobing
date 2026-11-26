#include "chassis_comm_port.h"
#include "chassis_comm.h"
#include "chassis_comm_config.h"
#include "robot_def.h"
#include "nav_can_hal.h"
#include "remote_can.h"
#include <string.h>

/* 旁听 CAN2 上云台 Yaw GM6020(ID=1 -> 反馈帧 0x205)编码器, 计算底盘相对云台偏角 offset_angle */
#define CHASSIS_YAW_GM6020_RX_ID   0x205u
#define CHASSIS_YAW_ECD_TIMEOUT_MS 100u
volatile uint16_t chassis_yaw_ecd_raw;
volatile uint32_t chassis_yaw_ecd_frames;
volatile float    chassis_yaw_offset_deg;
static volatile uint32_t chassis_yaw_ecd_tick;
static volatile uint8_t  chassis_yaw_ecd_seen;

static void receive_yaw_ecd(CANInstance *can)
{
    if (can->rx_len < 2u) return;
    chassis_yaw_ecd_raw = (uint16_t)(((uint16_t)can->rx_buff[0] << 8) | can->rx_buff[1]);
    chassis_yaw_ecd_tick = HAL_GetTick();
    chassis_yaw_ecd_seen = 1u;
    chassis_yaw_ecd_frames++;
}

static float read_yaw_offset_deg(uint32_t now)
{
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    uint8_t  seen = chassis_yaw_ecd_seen;
    uint16_t ecd  = chassis_yaw_ecd_raw;
    uint32_t tick = chassis_yaw_ecd_tick;
    __set_PRIMASK(irq);
    if (!seen || (uint32_t)(now - tick) > CHASSIS_YAW_ECD_TIMEOUT_MS)
    {
        chassis_yaw_offset_deg = 0.0f;
        return 0.0f;
    }
    int32_t diff = (int32_t)ecd - (int32_t)YAW_CHASSIS_ALIGN_ECD;
    if (diff >  4096) diff -= 8192;
    if (diff < -4096) diff += 8192;
    float deg = (float)diff * (360.0f / 8192.0f);
    chassis_yaw_offset_deg = deg;
    return deg;
}

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
volatile uint32_t chassis_task_max_gap_ms,chassis_rx_max_age_ms,chassis_rx_queue_high_water;
volatile uint32_t chassis_command_expired,chassis_remote_expired;
static uint32_t task_last_ms;static uint8_t task_seen;
volatile nc_rx_stats_t chassis_command_rx_stats;
volatile RemoteCanDiagnostics chassis_rc_diagnostics;
volatile uint32_t chassis_rc_generation,chassis_rc_source_age_ms;
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
    chassis_rc_estop=!online||(remote.sample.flags&RC_CAN_ESTOP)!=0||remote.sample.switches[1]==0;
    chassis_rc_accepted=remote.accepted;chassis_rc_rejected=remote.rejected;
    chassis_rc_diagnostics=remote.diagnostics;
    chassis_rc_generation=remote.generation;chassis_rc_source_age_ms=remote.sample.age_ms;
    chassis_command_rx_stats=endpoint.receiver.stats;
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
    uint32_t used=(head+CHASSIS_COMM_RX_QUEUE_SIZE-tail)%CHASSIS_COMM_RX_QUEUE_SIZE;
    if(used>chassis_rx_queue_high_water)chassis_rx_queue_high_water=used;
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
            .rx_fifo_policy = CAN_RX_FIFO_POLICY_FIFO0,
            .tx_id = NC_FEEDBACK_BASE + (i%3),
            .rx_id = i<3 ? NC_COMMAND_BASE+i : RC_CAN_BASE+i-3,
            .can_module_callback = receive_can,
        };
        CANRegister(&config);
    }
    /* 只读旁听云台 Yaw GM6020(0x205), 独立走 FIFO1 与板间多帧协议 FIFO0 隔离 */
    {
        CAN_Init_Config_s yaw_snoop = {
            .can_handle = &CHASSIS_COMM_CAN_HANDLE,
            .rx_fifo_policy = CAN_RX_FIFO_POLICY_FIFO1,
            .tx_id = 0,
            .rx_id = CHASSIS_YAW_GM6020_RX_ID,
            .can_module_callback = receive_yaw_ecd,
        };
        CANRegister(&yaw_snoop);
    }
    CANEnableDiagnostics(&CHASSIS_COMM_CAN_HANDLE);
}

void ChassisCommPortCommand(Chassis_Ctrl_Cmd_s *command, int motors_online)
{
    uint32_t now = HAL_GetTick();
    if(task_seen&&(uint32_t)(now-task_last_ms)>chassis_task_max_gap_ms)chassis_task_max_gap_ms=now-task_last_ms;
    task_last_ms=now;task_seen=1;
    chassis_comm_task_ticks = now;
    CANUpdateDiagnostics(&CHASSIS_COMM_CAN_HANDLE);
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
        /* Sample after copying: IRQ producers can advance the queue's time. */
        now=HAL_GetTick();uint32_t age=now-frame.received_ms;
        if(age>chassis_rx_max_age_ms)chassis_rx_max_age_ms=age;
        if(frame.id>=RC_CAN_BASE&&frame.id<=RC_CAN_BASE+2){
            if(age>NC_GROUP_MS)chassis_remote_expired++;
            rc_can_push(&remote,frame.id,frame.dlc,frame.bytes,frame.received_ms,now);
        }else{
            ChassisCommSetSafety(&endpoint,read_safety(motors_online));
            now=HAL_GetTick();
            if((uint32_t)(now-frame.received_ms)>NC_GROUP_MS)chassis_command_expired++;
            ChassisCommReceive(&endpoint, frame.id, frame.dlc, frame.bytes, frame.received_ms, now);
        }
        /* Apply each transition, including a brief failsafe followed by recovery. */
        ChassisCommSetSafety(&endpoint,read_safety(motors_online));
    }
    chassis_comm_accepted_commands = endpoint.accepted_commands;
    chassis_comm_decode_errors = endpoint.decode_errors;

    static uint8_t prev_sc_valid = 0, prev_sc = 0;
    static int8_t spin_dir = 1;
    const uint32_t cmd_now = HAL_GetTick();
    const bool rc_online = rc_can_fresh(&remote, cmd_now);
    const uint8_t sb = remote.sample.switches[0]; /* CH7: 2=遥控 1=视觉导航 0=键鼠(无效) */
    const uint8_t sc = remote.sample.switches[1]; /* CH8: 0=急停 1=NO_FOLLOW 2=小陀螺 */

    /* SC(CH8) 每次从 中(1) 拨到 下(2): 翻转一次小陀螺旋转方向 */
    if (rc_online) {
        if (prev_sc_valid && prev_sc == 1u && sc == 2u) {
            spin_dir = (spin_dir > 0) ? -1 : 1;
        }
        prev_sc = sc;
        prev_sc_valid = 1u;
    } else {
        prev_sc_valid = 0u;
    }

    /* 保持导航链路 TTL 定时推进 */
    rm_output_t output;
    ChassisCommGetCommand(&endpoint, cmd_now, &output);
    memset(command, 0, sizeof(*command));
    command->chassis_mode = CHASSIS_ZERO_FORCE;

    /* 电机离线 / 遥控离线 / SC 上档(0)急停 -> 零力 */
    if (!motors_online || !rc_online || sc == 0u) {
        return;
    }

    /* CH5(LD = axes[4], 原始 SBUS 0..2047) -> 调速倍率 0.0~1.0 */
    float ld = (float)remote.sample.axes[4] - 1024.0f;
    if (ld < -660.0f) ld = -660.0f;
    if (ld >  660.0f) ld =  660.0f;
    const float speed_scale = (ld + 660.0f) / 1320.0f;
    const int speed_buff = (int)(speed_scale * 100.0f);

    if (sb == 2u) {
        /* CH7(SB) 下档: 遥控器手动控制 (CH2->vx, CH1->vy, 0x205->offset_angle) */
        int16_t ch_vx = remote.sample.axes[1]; /* CH2 rocker_r1 */
        int16_t ch_vy = remote.sample.axes[0]; /* CH1 rocker_r_ */
        if (ch_vx > -40 && ch_vx < 40) ch_vx = 0;
        if (ch_vy > -40 && ch_vy < 40) ch_vy = 0;
        command->vx = 35.0f * speed_scale * (float)ch_vx;
        command->vy = 35.0f * speed_scale * (float)ch_vy;
        command->offset_angle = read_yaw_offset_deg(cmd_now);
        command->chassis_speed_buff = speed_buff;
        if (sc == 2u) {
            /* SC 下档: 底盘小陀螺 + 云台世界系自稳 (带 offset_angle 平移解耦) */
            command->chassis_mode = CHASSIS_ROTATE;
            command->wz = (float)spin_dir;
        } else {
            /* SC 中档: 底盘跟随云台旋转 */
            command->chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            command->wz = 0.0f;
        }
        return;
    }

    if (sb == 1u) {
        /* CH7(SB) 中档: 视觉/导航模式 */
        const float aps_to_mps = 6.28318530718f * (RADIUS_WHEEL / 1000.0f) /
            (360.0f * REDUCTION_RATIO_WHEEL);
        if (output.enabled) {
            command->vx = output.vx * CHASSIS_COMM_VX_SIGN / aps_to_mps;
            command->vy = output.vy * CHASSIS_COMM_VY_SIGN / aps_to_mps;
            command->wz = output.wz * CHASSIS_COMM_WZ_SIGN /
                (1000.0f * 0.01745329252f * aps_to_mps);
        }
        command->offset_angle = 0.0f;
        if (sc == 2u) {
            /* 边导航边小陀螺 (即使导航未下发也维持原地自旋, 与 basic_framework SentryAutoSet 一致) */
            command->chassis_mode = CHASSIS_ROTATE;
            command->wz = (float)spin_dir;
            command->chassis_speed_buff = 80;
        } else if (output.enabled) {
            /* 纯全向导航 */
            command->chassis_mode = CHASSIS_NO_FOLLOW;
            command->chassis_speed_buff = 100;
        }
        return;
    }
    /* CH7(SB) 上档(0): 键鼠模式占位(ET08A 无键鼠, 保持 CHASSIS_ZERO_FORCE) */
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
