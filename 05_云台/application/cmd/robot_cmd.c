/* [ET08A 8通道适配] 对齐 basic_framework 实车验证方案, 保留 05_云台 rad 单位与 SP 视觉接口:
 *   CH1(rocker_r_)  -> 底盘 vy (右水平, 底盘板处理)
 *   CH2(rocker_r1)  -> 底盘 vx (右竖直, 底盘板处理)
 *   CH3(rocker_l1)  -> 云台 pitch (左竖直)
 *   CH4(rocker_l_)  -> 云台 yaw (左水平)
 *   CH5(knob_l)     -> 旋钮 LD (底盘调速, 底盘板处理)
 *   CH6(knob_r)     -> 旋钮 RD 发射油门 (最左 5% 死区停火, 往右旋开启摩擦轮分档转速 + 拨盘射频 0~9 发/秒)
 *   CH7(channel[6]) -> 三段开关 SB: 下(2)=遥控 / 中(1)=视觉 / 上(0)=键鼠(无效)
 *   CH8(channel[7]) -> 三段开关 SC: 上(0)=急停(离上档直接动) / 中(1)=NO_FOLLOW / 下(2)=小陀螺
 */
#include "robot_cmd.h"
#include "robot_def.h"
#include "gimbal.h"
#include "auto_aim_boot.h"
#include "remote_control.h"
#include "robot_comm.h"
#include "message_center.h"
#include <math.h>

static Publisher_t *gimbal_pub, *shoot_pub;
#ifdef ONE_BOARD
static Publisher_t *chassis_pub;
#endif
static Gimbal_Ctrl_Cmd_s command;

#if !GIMBAL_AUTO_AIM_ON_BOOT
/* 0=零力, 1=摇杆遥控, 2=视觉自瞄, 3=视觉无目标保持 */
static uint8_t previous_source;
/* CH6(RD 旋钮) 射频上限: 满旋钮时的发射速率(发/秒), 对齐 basic_framework 实车验证值 */
#define ET08A_SHOOT_RATE_MAX         9.0f
/* 离线容忍: 40 × 5ms = 200ms, 过滤偶发单帧 SBUS 丢包 */
#define ET08A_RC_OFFLINE_TOL_FRAMES  40u

/**
 * @brief RD 旋钮比例(0~1) → 摩擦轮转速档(承载在 bullet_speed 枚举上, 带 4% 滞回)
 * @note  档位→转速映射见 shoot.c:
 *          BULLET_SPEED_NONE → 2000 / SMALL_AMU_15 → 3000 / SMALL_AMU_18 → 6000 / SMALL_AMU_30 → 10000
 */
static Bullet_Speed_e FrictionLevelFromRatio(float r)
{
    static const Bullet_Speed_e map[4] = {
        BULLET_SPEED_NONE, SMALL_AMU_15, SMALL_AMU_18, SMALL_AMU_30};
    static const float up[3]   = {0.05f, 0.40f, 0.75f}; /* 升档阈值 */
    static const float down[3] = {0.03f, 0.36f, 0.70f}; /* 降档阈值(滞回) */
    static uint8_t lv = 0;

    if (lv < 3 && r >= up[lv])
        lv++;
    else if (lv > 0 && r < down[lv - 1])
        lv--;

    return map[lv];
}
#endif

void RobotCMDInit(void)
{
#if !GIMBAL_AUTO_AIM_ON_BOOT
    FS_RemoteControlInit(&huart3);
#else
    AutoAimBootInit();
#endif
    RobotCommInit();
    gimbal_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    shoot_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
#ifdef ONE_BOARD
    chassis_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
#endif
}

#if !GIMBAL_AUTO_AIM_ON_BOOT
static void HoldCurrent(const GimbalState *s)
{
    command.yaw = s->yaw_continuous;
    command.pitch = s->pose.pitch;
}

static void ApplyRemoteTarget(const FS_RC_ctrl_t *rc, const GimbalState *s, float dt_scale, uint8_t sc)
{
    command.yaw += GIMBAL_REMOTE_YAW_DEG_PER_COUNT * GIMBAL_RAD_PER_DEG * (float)rc->rc.rocker_l_ * dt_scale;
    command.pitch += GIMBAL_REMOTE_PITCH_DEG_PER_COUNT * GIMBAL_RAD_PER_DEG * (float)rc->rc.rocker_l1 * dt_scale;
    float yaw_encoder, pitch_imu_limited;
    float pitch_imu_deg = s->pose.pitch / GIMBAL_RAD_PER_DEG;
    if (GimbalEncoderTarget(command.yaw, s->pose.yaw, s->yaw_mechanical_deg,
            GIMBAL_YAW_POSE_TO_ENCODER_SIGN, GIMBAL_YAW_MIN_DEG, GIMBAL_YAW_MAX_DEG,
            GIMBAL_VISION_MAX_ERROR_DEG, &yaw_encoder) &&
        GimbalEncoderTarget(command.pitch, s->pose.pitch, pitch_imu_deg,
            1.0f, GIMBAL_PITCH_MIN_DEG, GIMBAL_PITCH_MAX_DEG,
            GIMBAL_VISION_MAX_ERROR_DEG, &pitch_imu_limited))
    {
        command.yaw = s->yaw_continuous + (yaw_encoder - s->yaw_mechanical_deg) *
            GIMBAL_YAW_POSE_TO_ENCODER_SIGN * GIMBAL_RAD_PER_DEG;
        command.pitch = pitch_imu_limited * GIMBAL_RAD_PER_DEG;
        command.gimbal_mode = (sc == FS_RC_SW_DOWN) ? GIMBAL_GYRO_MODE : GIMBAL_FREE_MODE;
    }
}
#endif

void RobotCMDTask(void)
{
    Shoot_Ctrl_Cmd_s shoot = {
        .shoot_mode = SHOOT_OFF,
        .friction_mode = FRICTION_OFF,
        .load_mode = LOAD_STOP,
        .lid_mode = LID_CLOSE,
        .bullet_speed = BULLET_SPEED_NONE,
        .shoot_rate = 0.0f,
    };

#if GIMBAL_AUTO_AIM_ON_BOOT
    AutoAimBootUpdate(&command);
#else
    static FS_RC_ctrl_t rc_last;
    static uint16_t off_cnt = ET08A_RC_OFFLINE_TOL_FRAMES; /* 上电初值给满, 未收到首帧前保持急停 */
    static uint8_t sc_up_cnt = 2;                          /* SC 上档 2 帧(10ms) 消抖 */
    static uint32_t last_tick;

    FS_RC_ctrl_t rc_now = {0};
    int online_raw = FS_RemoteControlGetSnapshot(&rc_now);
    online_raw = online_raw &&
                 rc_now.channel[6] >= 0 && rc_now.channel[6] <= 2 &&
                 rc_now.channel[7] >= 0 && rc_now.channel[7] <= 2;

    if (online_raw)
    {
        rc_last = rc_now;
        off_cnt = 0;
    }
    else if (off_cnt < ET08A_RC_OFFLINE_TOL_FRAMES)
    {
        off_cnt++;
    }

    int online = (off_cnt < ET08A_RC_OFFLINE_TOL_FRAMES);
    const FS_RC_ctrl_t *rc = &rc_last;
    uint8_t sb = online ? (uint8_t)rc->channel[6] : (uint8_t)FS_RC_SW_UP; /* CH7: 下=遥控 中=视觉 上=键鼠 */
    uint8_t sc = online ? (uint8_t)rc->channel[7] : (uint8_t)FS_RC_SW_UP; /* CH8: 上=急停 中=普通 下=陀螺 */

    /* SC 上档消抖: 连续 2 帧(10ms) 上档才判急停, 防止单帧误码引发抖动 */
    if (!online || sc == FS_RC_SW_UP)
    {
        if (sc_up_cnt < 2) sc_up_cnt++;
    }
    else
    {
        sc_up_cnt = 0;
    }
    int sc_up = (sc_up_cnt >= 2);

    GimbalState state;
    GimbalGetState(&state);
    Vision_Recv_s rx = {0};
    int have = RobotCommGetVisionCommand(&rx);
    uint32_t now = HAL_GetTick();
    float dt_scale = last_tick ? fminf((float)(uint32_t)(now - last_tick) / 5.0f, 2.0f) : 1.0f;
    last_tick = now;

    /* SC 离上档直接动: 电机健康后自动尝试清除瞬态锁存故障 */
    if (online && !sc_up && state.driver_fault && state.pose_valid && state.motors_ready)
    {
        (void)GimbalClearMotorFaults();
        GimbalGetState(&state);
    }

    int ready = online && !sc_up && GIMBAL_INSTALLATION_CONFIRMED &&
                state.pose_valid && state.motors_ready && !state.driver_fault;
    int selected_visual = ready && GIMBAL_VISION_ENABLE && (sb == GIMBAL_VISION_SWITCH_POSITION);
    RobotCommSetVisionMode(selected_visual ? 1 : 0);

    uint8_t source = 0;
    command.gimbal_mode = GIMBAL_ZERO_FORCE;

    if (!online || sc_up)
    {
        /* 急停或遥控离线: 云台零力 + 发射紧急停止 */
        HoldCurrent(&state);
        shoot.shoot_mode = SHOOT_OFF;
        shoot.friction_mode = FRICTION_OFF;
        shoot.load_mode = LOAD_STOP;
        shoot.bullet_speed = BULLET_SPEED_NONE;
        shoot.shoot_rate = 0.0f;
    }
    else if (sb == FS_RC_SW_DOWN)
    {
        /* CH7(SB) 下档: 遥控模式 */
        if (ready)
        {
            source = 1;
            if (previous_source != 1)
                HoldCurrent(&state);
            ApplyRemoteTarget(rc, &state, dt_scale, sc);
        }

        /* CH6(RD 旋钮) 当发射总油门(对齐 basic_framework 实测):
         * remote_control.c 存的是 raw-240, 还原到 raw-1024 域: 最左 +670, 最右 -671
         * fire_ratio: 0(最左 5% 死区停火) ~ 1(最右满油门 9 发/秒 + 最高档摩擦轮) */
        float kr = (float)(rc->rc.knob_r + 240) - 1024.0f;
        if (kr >  670.0f) kr =  670.0f;
        if (kr < -671.0f) kr = -671.0f;

        float fire_ratio = (670.0f - kr) / 1341.0f;
        if (fire_ratio < 0.05f)
            fire_ratio = 0.0f;

        shoot.shoot_mode   = SHOOT_ON;
        shoot.bullet_speed = FrictionLevelFromRatio(fire_ratio);
        if (fire_ratio > 0.0f)
        {
            shoot.friction_mode = FRICTION_ON;
            shoot.load_mode     = LOAD_BURSTFIRE;
            shoot.shoot_rate    = ET08A_SHOOT_RATE_MAX * fire_ratio;
        }
        else
        {
            shoot.friction_mode = FRICTION_OFF;
            shoot.load_mode     = LOAD_STOP;
            shoot.shoot_rate    = 0.0f;
        }
    }
    else if (sb == FS_RC_SW_MID)
    {
        /* CH7(SB) 中档: 视觉/导航模式 */
        if (ready)
        {
            int fresh = have && RobotCommUsbIsConfigured() &&
                        (uint32_t)(now - rx.received_tick) <= VISION_RX_TIMEOUT_MS;
            if (GIMBAL_VISION_ENABLE && fresh && rx.mode != 0)
            {
                source = 2;
                command.yaw = rx.yaw;
                command.pitch = rx.pitch;
                command.gimbal_mode = GIMBAL_VISION_MODE;
            }
            else
            {
                /* 视觉无目标或离线: 锁在当前姿态保持, 不掉落 */
                source = 3;
                if (previous_source != 3)
                    HoldCurrent(&state);
                command.gimbal_mode = GIMBAL_GYRO_MODE;
            }
        }
        shoot.shoot_mode = SHOOT_ON;
        shoot.friction_mode = FRICTION_OFF;
        shoot.load_mode = LOAD_STOP;
        shoot.bullet_speed = BULLET_SPEED_NONE;
        shoot.shoot_rate = 0.0f;
    }
    else
    {
        /* CH7(SB) 上档: 键鼠模式(ET08A 无键鼠输入, 保持安全停止) */
        HoldCurrent(&state);
        shoot.shoot_mode = SHOOT_OFF;
        shoot.friction_mode = FRICTION_OFF;
        shoot.load_mode = LOAD_STOP;
        shoot.bullet_speed = BULLET_SPEED_NONE;
        shoot.shoot_rate = 0.0f;
    }

    if (!source)
        HoldCurrent(&state);
    previous_source = source;
#endif

    PubPushMessage(shoot_pub, &shoot);
#ifdef ONE_BOARD
    Chassis_Ctrl_Cmd_s chassis = {.chassis_mode = CHASSIS_ZERO_FORCE};
    PubPushMessage(chassis_pub, &chassis);
#endif
    PubPushMessage(gimbal_pub, &command);
}
