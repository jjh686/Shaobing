/* [USB_SP 修改 01 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
/* [USB_SP 修改说明]
 * 本文件适配新工程的 FS/SBUS 遥控与 USB SP；原始对应文件另存于交付包“原始改动文件”。
 * 遥控仍使用 USART3，左水平/左竖直分别控制 yaw/pitch，通道 CH7/CH9/CH10 不变。
 * 原文件的顶部说明写“左1中档视觉”，实际 RemoteControlSet 使用 up；以实际分支为默认。
 * GIMBAL_VISION_ENABLE=0：所有左1档位均为遥控，USB仍可双向收发，不接管电机。
 * GIMBAL_VISION_ENABLE=1：左1指定档位允许视觉；新鲜 mode1/2 使用视觉，mode0回遥控。
 * 遥控与视觉每周期只选一个目标来源；换来源时先对齐当前姿态，避免旧目标突跳。
 * 本适配使用统一 rad/rad/s 的IMU双环；原混合单位/Pitch轴已改变，不保证原手感或PID有效。
 * 安装门控为0时两种来源均不使能电机；仅云台联调，底盘/发射不运行。
 * [USB_SP 上电自瞄] 上述遥控逻辑仅在GIMBAL_AUTO_AIM_ON_BOOT=0时使用；
 * 默认=1时进入auto_aim_boot模块，无需遥控器，保留本文件原共存分支供切回。
 */
#include "robot_cmd.h"
/* [USB_SP 修改 01 结束] */
#include "robot_def.h"
/* [USB_SP 修改 02 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
#include "gimbal.h"
#include "control_guard.h"
/* [USB_SP 上电自瞄 新增 开始] 自主流程集中在独立模块。 */
#include "auto_aim_boot.h"
/* [USB_SP 上电自瞄 新增 结束] */
/* [USB_SP 修改 02 结束] */
#include "remote_control.h"
/* [USB_SP 修改 03 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
#include "robot_comm.h" /* [USB_SP 通信整理] 通过总入口使用通信服务。 */
/* [USB_SP 修改 03 结束] */
#include "message_center.h"
/* [USB_SP 修改 04 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
#include <math.h>
/* [USB_SP 修改 04 结束] */

/* [USB_SP 修改 05 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
static Publisher_t *gimbal_pub, *shoot_pub;
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;
#endif
static Gimbal_Ctrl_Cmd_s command;
/* [USB_SP 上电自瞄 新增 开始] 原遥控分支的私有状态保留。 */
#if !GIMBAL_AUTO_AIM_ON_BOOT
static ControlGuard guard;
/* 0=零力，1=摇杆，2=视觉；此状态仅用于来源切换时清掉上一来源的目标。 */
static uint8_t previous_source;
#endif
/* [USB_SP 上电自瞄 新增 结束] */
/* [USB_SP 修改 05 结束] */

/* [USB_SP 修改 06 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
void RobotCMDInit(void)
{
    /* [USB_SP 通信整理] SBUS仍在USART3；USB初始化改走统一通信入口。 */
    /* Always receive SBUS, including the USB/CAN commissioning build.
     * Reception alone does not forward RC safety or enable any actuator. */
    FS_RemoteControlInit(&huart3);
#if GIMBAL_AUTO_AIM_ON_BOOT
    AutoAimBootInit();
#endif
    /* [USB_SP 上电自瞄 新增 结束] */
    RobotCommInit();
    gimbal_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    shoot_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
/* [USB_SP 修改 06 结束] */
#ifdef ONE_BOARD
/* [USB_SP 修改 07 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
    chassis_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
/* [USB_SP 修改 07 结束] */
#endif
}

/* [USB_SP 上电自瞄 新增 开始] 下列目标函数只用于原遥控共存模式。 */
#if !GIMBAL_AUTO_AIM_ON_BOOT
/* [USB_SP 修改 08 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
/* [新增] 零力或输入来源切换时同步目标，不积累停止期间的摇杆位移。 */
static void HoldCurrent(const GimbalState *s)
/* [USB_SP 修改 08 结束] */
{
/* [USB_SP 修改 09 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
    command.yaw = s->yaw_continuous;
    command.pitch = s->pose.pitch;
/* [USB_SP 修改 09 结束] */
}

/* [USB_SP 修改 10 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
/* [保留意图/单位转换] 原每5ms累加0.001度*摇杆，转换成rad后再进入同一角度环。
 * 本函数只在当前来源为遥控时执行；USB来的角度绝不叠加到摇杆目标。
 */
static void ApplyRemoteTarget(const FS_RC_ctrl_t *rc, const GimbalState *s, float dt_scale)
/* [USB_SP 修改 10 结束] */
{
/* [USB_SP 修改 11 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
    command.yaw += GIMBAL_REMOTE_DEG_PER_COUNT * GIMBAL_RAD_PER_DEG * rc->rc.rocker_l_ * dt_scale;
    command.pitch += GIMBAL_REMOTE_DEG_PER_COUNT * GIMBAL_RAD_PER_DEG * rc->rc.rocker_l1 * dt_scale;
    float yaw_encoder, pitch_encoder;
    /* [新增限位] 只用编码器限制机械包络，角度/速度闭环仍取IMU，不切换到编码器环。 */
    if (GimbalEncoderTarget(command.yaw, s->pose.yaw, s->yaw_mechanical_deg,
            GIMBAL_YAW_POSE_TO_ENCODER_SIGN, GIMBAL_YAW_MIN_DEG, GIMBAL_YAW_MAX_DEG,
            GIMBAL_VISION_MAX_ERROR_DEG, &yaw_encoder) &&
        GimbalEncoderTarget(command.pitch, s->pose.pitch, s->pitch_mechanical_deg,
            GIMBAL_PITCH_POSE_TO_ENCODER_SIGN, GIMBAL_PITCH_MIN_DEG, GIMBAL_PITCH_MAX_DEG,
            GIMBAL_VISION_MAX_ERROR_DEG, &pitch_encoder)) {
        command.yaw = s->yaw_continuous + (yaw_encoder - s->yaw_mechanical_deg) *
            GIMBAL_YAW_POSE_TO_ENCODER_SIGN * GIMBAL_RAD_PER_DEG;
        command.pitch = s->pose.pitch + (pitch_encoder - s->pitch_mechanical_deg) *
            GIMBAL_PITCH_POSE_TO_ENCODER_SIGN * GIMBAL_RAD_PER_DEG;
        /* [保留原档位] 右1下=GYRO，右1中=FREE；本工程两者原本都使用IMU双环。 */
        command.gimbal_mode = fs_switch_is_down(rc->rc.switch_right_1) ?
            GIMBAL_GYRO_MODE : GIMBAL_FREE_MODE;
/* [USB_SP 修改 11 结束] */
    }
}

#endif
/* [USB_SP 上电自瞄 新增 结束] */
/* [USB_SP 修改 12 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
void RobotCMDTask(void)
{
    /* [USB_SP 上电自瞄 新增 开始] 原遥控共存分支不改算法，仅增加编译选择。 */
#if GIMBAL_AUTO_AIM_ON_BOOT
    AutoAimBootUpdate(&command);
#else
    /* [新增] USB进入会话与故障代数，避免两次5ms轮询间断线重连后自动续动。 */
    static uint8_t was_visual, received_since_entry;
    static uint32_t entry_tick, entry_sequence, entry_session, last_tick, last_rc_fault;
    FS_RC_ctrl_t rc = {0};
    int online = FS_RemoteControlGetSnapshot(&rc);
    online = online && rc.channel[6] >= 0 && rc.channel[6] <= 2 &&
        rc.channel[8] >= 0 && rc.channel[8] <= 2 && rc.channel[9] >= 0 && rc.channel[9] <= 2;
/* [USB_SP 修改 12 结束] */

/* [USB_SP 修改 13 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
    /* [保留通道] CH7=左1，CH9=右1，CH10=右2。右1上仍为零力，右2下仍急停。 */
    int recovery_up = online && fs_switch_is_up(rc.rc.switch_right_1);
    int estop = online && fs_switch_is_down(rc.rc.switch_right_2);
    int selected_visual = GIMBAL_VISION_ENABLE && online &&
        rc.rc.switch_left_1 == GIMBAL_VISION_SWITCH_POSITION;
    GimbalState state;
    GimbalGetState(&state);
    Vision_Recv_s rx = {0};
    int have = RobotCommGetVisionCommand(&rx);
    uint32_t now = HAL_GetTick();
    float dt_scale = last_tick ? fminf((uint32_t)(now-last_tick)/5.0f, 2.0f) : 1.0f;
    last_tick = now;

    /* [新增] 只有主动选择且已解锁的视觉档，才把USB超时作为运动故障。
     * 纯手动不依赖USB，电脑未连接/程序未运行不会导致手动超时停机。
     */
    int requested = selected_visual && !recovery_up && guard.armed && GIMBAL_INSTALLATION_CONFIRMED;
    if (requested && !was_visual) {
        entry_tick = now; entry_sequence = rx.sequence; entry_session = rx.session;
        received_since_entry = 0;
    }
    if (requested && have && rx.sequence != entry_sequence) received_since_entry = 1;
    int fresh = have && RobotCommUsbIsConfigured() && (uint32_t)(now-rx.received_tick) <= VISION_RX_TIMEOUT_MS;
    int visual_fault = requested && (rx.session != entry_session ||
        (received_since_entry ? !fresh : (uint32_t)(now-entry_tick) > VISION_STARTUP_GRACE_MS));
    int rc_fault_event = rc.fault_generation != last_rc_fault;
    last_rc_fault = rc.fault_generation;
    was_visual = requested;

    /* [新增保护] 超时/急停/失联后先撤销使能；恢复须健康状态下右1“非上→上→离开上”。 */
    if (state.driver_fault && guard.armed) { guard.armed = 0; guard.saw_non_up = 0; }
    int armed = ControlGuardUpdate(&guard, online, estop ? 1 : 0, recovery_up,
        !state.pose_valid || !state.motors_ready || visual_fault || rc_fault_event, 1, 0);
    if (armed && state.driver_fault && recovery_up && !GimbalClearMotorFaults()) {
        guard.armed = 0; guard.saw_non_up = 0; armed = 0;
    }

    /* [新增回传] mode1只表示请求视觉计算，不等于电机已使能；门控0也能进行协议联调。 */
    RobotCommSetVisionMode(selected_visual && !estop && !recovery_up && state.pose_valid ? 1 : 0);
    uint8_t source = 0;
    command.gimbal_mode = GIMBAL_ZERO_FORCE;
    if (armed && !recovery_up && GIMBAL_INSTALLATION_CONFIRMED) {
        int use_visual = selected_visual && received_since_entry && fresh && rx.mode != 0;
        source = use_visual ? 2 : 1;
        if (previous_source != source) HoldCurrent(&state);
        if (use_visual) {
            /* [新增自瞄来源] SP给绝对目标rad；只使用yaw/pitch，不发射，不再叠加摇杆。 */
            command.yaw = rx.yaw; command.pitch = rx.pitch;
            command.gimbal_mode = GIMBAL_VISION_MODE;
        } else {
            /* [保留遥控/补完原丢目标接管意图] 手动档、等待首帧、mode0均由左摇杆控制。
             * 已建立视觉会话后若数据过期则先锁存零力，不能悄悄把断线当成“没目标”。 */
            ApplyRemoteTarget(&rc, &state, dt_scale);
        }
    }
    if (!source) HoldCurrent(&state);
    previous_source = source;
#endif
    /* [USB_SP 上电自瞄 新增 结束] */

    /* [联调隔离] 原项目发射任务本来未启用；继续保持关闭。此交付也不运行底盘/UI。 */
    Shoot_Ctrl_Cmd_s shoot = {.shoot_mode=SHOOT_OFF, .friction_mode=FRICTION_OFF,
        .load_mode=LOAD_STOP, .lid_mode=LID_CLOSE};
    PubPushMessage(shoot_pub, &shoot);
/* [USB_SP 修改 13 结束] */
#ifdef ONE_BOARD
/* [USB_SP 修改 14 开始] 遥控与可选自瞄独占目标来源，默认只遥控，保留SBUS通道和右侧停止档，补完USB故障处理。 */
    Chassis_Ctrl_Cmd_s chassis = {.chassis_mode=CHASSIS_ZERO_FORCE};
    PubPushMessage(chassis_pub, &chassis);
#endif
    PubPushMessage(gimbal_pub, &command);
/* [USB_SP 修改 14 结束] */
}
