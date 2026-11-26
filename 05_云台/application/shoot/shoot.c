#include "shoot.h"
#include "robot_def.h"

#include "dji_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "general_def.h"
#include <math.h>

/* 对于双发射机构的机器人,将下面的数据封装成结构体即可,生成两份shoot应用实例 */
static DJIMotorInstance *friction_l, *friction_r, *loader; // 摩擦轮与拨盘电机

static Publisher_t *shoot_pub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;         // 来自cmd的发射控制信息
static Subscriber_t *shoot_sub;
static Shoot_Upload_Data_s shoot_feedback_data; // 回传给cmd的发射状态

// dwt定时,计算冷却用
static float hibernate_time = 0, dead_time = 0;

/* ---------------- 拨盘卡弹检测 (移植自 basic_framework 实测验证版) ----------------
 * 判定用「speed 低 + Output 高 + 持续 250ms」, 过滤瞬时挤压
 */
#define LOADER_STALL_SPEED      1500.0f     /* aps, 低于此视为没在转 */
#define LOADER_STALL_OUT        6000.0f     /* 电流指令, 高于此视为硬顶 */
#define LOADER_STALL_MS         250.0f      /* 持续多久判定卡弹 */
#define LOADER_REVERSE_MS       300.0f      /* 反转退弹时长 */
#define LOADER_REVERSE_SPEED  (-6000.0f)    /* 反转速度 aps */
#define LOADER_RETRY_MAX        3           /* 连续退弹次数上限 */
#define LOADER_OK_RESET_MS      600.0f      /* 正常转满 600ms 归还配额 */
#define LOADER_COOLDOWN_MS      1000.0f     /* 初始冷却时长 */
#define LOADER_COOLDOWN_MAX     4000.0f     /* 冷却时长上限(指数退避) */
#define LOADER_STALL_TOTAL_MAX  3000.0f     /* 累计堵转 3s 强制停机(C610 无温度反馈) */

typedef enum
{
    LOADER_NORMAL = 0,      /* 正常 */
    LOADER_REVERSING,       /* 反转退弹中 */
    LOADER_COOLDOWN,        /* 冷却中, 期满自动重试 */
} loader_state_e;

static loader_state_e loader_state       = LOADER_NORMAL;
static float loader_stall_ms             = 0.0f;
static float loader_reverse_end          = 0.0f;
static float loader_cooldown_end         = 0.0f;
static float loader_ok_ms                = 0.0f;
static float loader_stall_total_ms       = 0.0f;
static uint8_t loader_retry_cnt          = 0;
static float loader_cooldown_ms          = LOADER_COOLDOWN_MS;
static uint16_t loader_jam_count         = 0;

/* 调试开关: 1=忽略遥控让拨盘恒速转, 0=正常遥控控制 */
#define LOADER_DEBUG_RUN     0
float loader_debug_speed = 3240.0f;

/* 适配 05_云台 的 dji_motor 故障锁存机制:
 * 仅在收到真实 CAN 反馈后使能, 断电复电后自动停机清故障再使能, 避免未接电机时永久锁死 */
static void ShootMotorEnable(DJIMotorInstance *m)
{
    if (!m) return;
    if (m->fault_latched)
    {
        DJIMotorStop(m);
        (void)DJIMotorClearFault(m);
    }
    if (!m->fault_latched && m->feedback_seen)
        DJIMotorEnable(m);
}

static void ShootUpdateFeedback(void)
{
    shoot_feedback_data.loader_state     = (uint8_t)loader_state;
    shoot_feedback_data.loader_retry_cnt = loader_retry_cnt;
    shoot_feedback_data.loader_jam_count = loader_jam_count;
    if (loader)
    {
        shoot_feedback_data.loader_output = (int16_t)loader->motor_controller.speed_PID.Output;
        shoot_feedback_data.loader_speed  = (int16_t)loader->measure.speed_aps;
    }
    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}

void ShootInit()
{
    // 左摩擦轮 (云台 CAN2, ID 1, 对齐 basic_framework 实测参数)
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = &hcan2,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 5.0f,
                .Ki = 0.3f,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 4000,
                .MaxOut = 10000,
            },
            .current_PID = {
                .Kp = 0.7f,
                .Ki = 0.1f,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 10000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M3508};
    friction_config.can_init_config.tx_id = 2;
    friction_l = DJIMotorInit(&friction_config);

    // 右摩擦轮 (云台 CAN2, ID 2, 反向)
    friction_config.can_init_config.tx_id = 1;
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    // 拨盘电机 (物理接底盘 CAN2, 经滑环与云台 CAN1 共总线, ID 3 -> 0x200发 / 0x203收)
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 3,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 3,                // 3, 正常工作落在线性区
                .Ki = 0.3,              // 0.3, 补稳态误差
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 2000,
                .MaxOut = 7000,         // 7A (C610 满量程 ±10000 = ±10A)
                .Output_LPF_RC = 0.05f, // 50ms 滤波, 平滑输出
            },
            .current_PID = {
                .Kp = 0.7,
                .Ki = 0.1,
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 7000,         // 与 speed_PID.MaxOut 同步为 7A
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = M2006
    };
    loader = DJIMotorInit(&loader_config);

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
}

/**
 * @brief 拨盘卡弹检测与退弹处理
 * @return 1 = 正在退弹/冷却/强制停机(调用方跳过本轮发射), 0 = 正常
 */
static uint8_t LoaderJamHandler(void)
{
    if (!loader || !loader->feedback_seen)
        return 0;

    const float now = DWT_GetTimeline_ms();

    /* ========== 冷却中: 期满自动恢复 ========== */
    if (loader_state == LOADER_COOLDOWN)
    {
        DJIMotorSetRef(loader, 0.0f);
        if (now < loader_cooldown_end)
            return 1;

        loader_state       = LOADER_NORMAL;
        loader_retry_cnt   = 0;
        loader_ok_ms       = 0.0f;
        loader_cooldown_ms = LOADER_COOLDOWN_MS;
        LOGINFO("[SHOOT] loader cooldown done, resume");
        return 0;
    }

    /* ========== 退弹中 ========== */
    if (loader_state == LOADER_REVERSING)
    {
        if (now < loader_reverse_end)
        {
            ShootMotorEnable(loader);
            DJIMotorOuterLoop(loader, SPEED_LOOP);
            DJIMotorSetRef(loader, LOADER_REVERSE_SPEED);
            return 1;
        }
        loader_state    = LOADER_NORMAL;
        loader_stall_ms = 0.0f;
        return 0;
    }

    /* ========== 正常转动判定 ========== */
    const uint8_t rotating =
        (fabsf(loader->measure.speed_aps) >= LOADER_STALL_SPEED) ||
        (fabsf(loader->motor_controller.speed_PID.Output) < LOADER_STALL_OUT);

    if (rotating)
    {
        loader_stall_ms = 0.0f;

        if (loader_stall_total_ms > 0.0f)
        {
            loader_stall_total_ms -= 5.0f; /* 与 ShootTask 200Hz 对应 */
            if (loader_stall_total_ms < 0.0f)
                loader_stall_total_ms = 0.0f;
        }

        loader_ok_ms += 5.0f;
        if (loader_ok_ms >= LOADER_OK_RESET_MS)
        {
            loader_retry_cnt   = 0;
            loader_ok_ms       = 0.0f;
            loader_cooldown_ms = LOADER_COOLDOWN_MS;
        }
        return 0;
    }

    /* 疑似卡住, 但还没到判定时间 */
    loader_ok_ms = 0.0f;
    if (loader_stall_ms == 0.0f)
    {
        loader_stall_ms = now;
        return 0;
    }
    if (now - loader_stall_ms < LOADER_STALL_MS)
        return 0;

    /* ========== 确认卡弹 ========== */
    const float stall_duration = now - loader_stall_ms;
    loader_stall_ms = 0.0f;
    loader_retry_cnt++;
    loader_jam_count++;

    loader_stall_total_ms += stall_duration;
    if (loader_stall_total_ms > LOADER_STALL_TOTAL_MAX)
    {
        LOGERROR("[SHOOT] loader stall total %.0fms, FORCE STOP", loader_stall_total_ms);
        DJIMotorSetRef(loader, 0.0f);
        DJIMotorStop(loader);
        loader_state          = LOADER_COOLDOWN;
        loader_cooldown_end   = now + 5000.0f;
        loader_stall_total_ms = 0.0f;
        return 1;
    }

    if (loader_retry_cnt > LOADER_RETRY_MAX)
    {
        loader_state        = LOADER_COOLDOWN;
        loader_cooldown_end = now + loader_cooldown_ms;
        loader_retry_cnt    = 0;

        loader_cooldown_ms *= 2.0f;
        if (loader_cooldown_ms > LOADER_COOLDOWN_MAX)
            loader_cooldown_ms = LOADER_COOLDOWN_MAX;

        DJIMotorSetRef(loader, 0.0f);
        LOGWARNING("[SHOOT] loader cooldown %.0fms", loader_cooldown_ms);
        return 1;
    }

    /* ========== 反转退弹 ========== */
    loader_state       = LOADER_REVERSING;
    loader_reverse_end = now + LOADER_REVERSE_MS;
    ShootMotorEnable(loader);
    DJIMotorOuterLoop(loader, SPEED_LOOP);
    DJIMotorSetRef(loader, LOADER_REVERSE_SPEED);
    LOGWARNING("[SHOOT] loader jam #%d, reversing (%d/%d)",
               loader_jam_count, loader_retry_cnt, LOADER_RETRY_MAX);
    return 1;
}

/* 机器人发射机构控制核心任务 */
void ShootTask()
{
    static float   fric_ref_ramp = 0.0f;
    const float    FRIC_RAMP_STEP = 100.0f; /* 5ms 每周期步进 100 aps (= 20000 aps/s, 10000档 0.5s 起满) */

    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    /* 紧急停止优先于卡弹退弹, 确保急停档绝对不转并重置斜坡 */
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DJIMotorStop(loader);
        DJIMotorSetRef(friction_l, 0.0f);
        DJIMotorSetRef(friction_r, 0.0f);
        DJIMotorSetRef(loader, 0.0f);
        loader_state    = LOADER_NORMAL;
        loader_stall_ms = 0.0f;
        fric_ref_ramp   = 0.0f;
        ShootUpdateFeedback();
        return;
    }

    if (LoaderJamHandler())
    {
        ShootUpdateFeedback();
        return;
    }

#if LOADER_DEBUG_RUN
    ShootMotorEnable(loader);
    DJIMotorOuterLoop(loader, SPEED_LOOP);
    DJIMotorSetRef(loader, loader_debug_speed);
    ShootUpdateFeedback();
    return;
#endif

    ShootMotorEnable(friction_l);
    ShootMotorEnable(friction_r);
    ShootMotorEnable(loader);

    /* ===== 1. 摩擦轮分档设速 + 软启动/减速斜坡 (对齐 basic_framework 实测值) ===== */
    float fric_target = 0.0f;
    if (shoot_cmd_recv.friction_mode == FRICTION_ON)
    {
        switch (shoot_cmd_recv.bullet_speed)
        {
        case SMALL_AMU_15:
            fric_target = 3000.0f;
            break;
        case SMALL_AMU_18:
            fric_target = 6000.0f;
            break;
        case SMALL_AMU_30:
            fric_target = 10000.0f;
            break;
        default:
            fric_target = 2000.0f;
            break;
        }
    }

    if (fric_ref_ramp < fric_target)
    {
        fric_ref_ramp += FRIC_RAMP_STEP;
        if (fric_ref_ramp > fric_target)
            fric_ref_ramp = fric_target;
    }
    else if (fric_ref_ramp > fric_target)
    {
        fric_ref_ramp -= FRIC_RAMP_STEP;
        if (fric_ref_ramp < fric_target)
            fric_ref_ramp = fric_target;
    }

    DJIMotorSetRef(friction_l, fric_ref_ramp);
    DJIMotorSetRef(friction_r, fric_ref_ramp);

    /* ===== 2. 拨盘控制 ===== */
    switch (shoot_cmd_recv.load_mode)
    {
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, 0.0f);
        break;
    case LOAD_1_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);
        DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE);
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 150;
        break;
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE);
        hibernate_time = DWT_GetTimeline_ms();
        dead_time = 300;
        break;
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, shoot_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
        break;
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, LOADER_REVERSE_SPEED);
        loader_retry_cnt = 0;
        break;
    default:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, 0.0f);
        break;
    }

    (void)hibernate_time;
    (void)dead_time;
    ShootUpdateFeedback();
}
