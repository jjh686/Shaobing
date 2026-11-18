/* [USB_SP 新增 01 开始] 集中配置视觉接管开关、安装门控、选择档位、方向、限位、双环参数及超时。 */
#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H
/* Gimbal-only commissioning: retain chassis/shooter sources, do not run them. */
#define GIMBAL_ONLY_BUILD 1
#if GIMBAL_ONLY_BUILD != 1
#error This adapter is gimbal-only; restore a chassis CMD before enabling chassis.
#endif
/* MUST match vision R_gimbal2imubody. Candidate, NOT physical calibration.
 * This QEKF calls the conventional Y pitch "Roll", and X roll "Pitch". */
#define GIMBAL_TO_IMU_ROTATION {1,0,0,0,1,0,0,0,1}
#ifndef GIMBAL_INSTALLATION_CONFIRMED
#define GIMBAL_INSTALLATION_CONFIRMED 0 /* Communication commissioning image: actuator gate closed. */
#endif
/* [新增：与原遥控共存] 0=只用遥控，USB照常收发；1=允许指定档位的视觉接管。
 * 默认保留原项目“自瞄未启用”的状态。电控确认后只改本宏，不用删遥控代码。
 */
#ifndef GIMBAL_VISION_ENABLE
#define GIMBAL_VISION_ENABLE 1
#endif
/* [USB_SP 上电自瞄 新增 开始]
 * 1=上电自主请求视觉，不依赖遥控器；0=使用原遥控与视觉共存流程。
 * 两种模式均初始化SBUS接收；自主模式不把遥控状态用于控制或CAN转发。
 * 自主模式仍受 GIMBAL_VISION_ENABLE、安装确认、姿态/电机健康和USB超时约束。
 * 电脑可稍后手动启动视觉；连续收到有效命令300ms后才允许电机工作。
 */
#ifndef GIMBAL_AUTO_AIM_ON_BOOT
#define GIMBAL_AUTO_AIM_ON_BOOT 1
#endif
#ifndef GIMBAL_AUTO_AIM_SETTLE_MS
#define GIMBAL_AUTO_AIM_SETTLE_MS 300u
#endif
#if (GIMBAL_AUTO_AIM_ON_BOOT != 0 && GIMBAL_AUTO_AIM_ON_BOOT != 1) || \
    GIMBAL_AUTO_AIM_SETTLE_MS < 100u || GIMBAL_AUTO_AIM_SETTLE_MS > 5000u
#error Invalid autonomous aim startup configuration.
#endif
/* [USB_SP 上电自瞄 新增 结束] */
/* 原文件头说MID但实际分支是UP；默认按实际UP。0=上、1=中、2=下，可集中修改。 */
#ifndef GIMBAL_VISION_SWITCH_POSITION
#define GIMBAL_VISION_SWITCH_POSITION 0
#endif
#if (GIMBAL_VISION_ENABLE != 0 && GIMBAL_VISION_ENABLE != 1) || \
    GIMBAL_VISION_SWITCH_POSITION < 0 || GIMBAL_VISION_SWITCH_POSITION > 2
#error Invalid vision enable or SBUS switch position.
#endif
/* 保留原遥控每5ms的0.001度/摇杆计数，在CMD统一转换为rad。 */
#define GIMBAL_REMOTE_DEG_PER_COUNT 0.001f
/* Final actuator output signs, NOT target-only motor_reverse_flag. */
#define GIMBAL_YAW_OUTPUT_SIGN 1
#define GIMBAL_PITCH_OUTPUT_SIGN 1
#define GIMBAL_YAW_POSE_TO_ENCODER_SIGN 1.0f
#define GIMBAL_PITCH_POSE_TO_ENCODER_SIGN 1.0f
/* Temporary envelope relative to FIRST real CAN angle after power-on.
 * Start physically centred. NOT calibrated stops. No old 927/6825 assumed. */
/* 相对上电位置，仅在上电姿态为行程中心时成立。
 * 这两个值同时是 GimbalGetState 的健康判定阈值([min-1, max+1])，
 * 设超过机械止挡会撞限位，设小于实际行程会中途失能。
 * Yaw放宽到±120用于覆盖目标大范围移动，前提是有滑环或线束余量。
 * Pitch仍为±20：上下是硬机械止挡，不能按Yaw放大。 */
#define GIMBAL_YAW_MIN_DEG (-120.0f)
#define GIMBAL_YAW_MAX_DEG 120.0f
#define GIMBAL_PITCH_MIN_DEG (-20.0f)
#define GIMBAL_PITCH_MAX_DEG 20.0f
#define GIMBAL_VISION_MAX_ERROR_DEG 5.0f
#define GIMBAL_RAD_PER_DEG 0.01745329251994329577f
/* tracking_v1: explicit units; numerical gains remain the original baseline.
 * The reference robot uses different time/output conventions: do NOT copy its
 * PID numbers here. These gains have NOT been tuned for this mechanism. */
#define GIMBAL_YAW_ANGLE_KP 8.0f
#define GIMBAL_PITCH_ANGLE_KP 10.0f
#define GIMBAL_YAW_ANGLE_DEADBAND_RAD (0.1f * GIMBAL_RAD_PER_DEG)
#define GIMBAL_YAW_SPEED_KP 50.0f /* CAN command / (rad/s) */
#define GIMBAL_PITCH_SPEED_KP 50.0f
/* 420 = 3000/2500*350，使Yaw与Pitch达到各自的I上限所需时间相同。 */
#define GIMBAL_YAW_SPEED_KI 420.0f
#define GIMBAL_PITCH_SPEED_KI 350.0f
/* limit on I output, NOT error sum. 稳态巡航时P项接近0，全靠I出力，
 * 原3000/2500只能维持约200°/s；提高后可覆盖外环500°/s。
 * 只影响最终可用力矩，不改变积分爬升速率，因此不增加起步延迟。 */
#define GIMBAL_YAW_SPEED_I_LIMIT 6000.0f
#define GIMBAL_PITCH_SPEED_I_LIMIT 5000.0f
#define GIMBAL_ANGLE_SPEED_LIMIT (500.0f * GIMBAL_RAD_PER_DEG)
#define GIMBAL_OUTPUT_LIMIT 20000.0f
#define GIMBAL_MOTOR_TIMEOUT_MS 50u
#define GIMBAL_INS_TIMEOUT_MS 20u
#define GIMBAL_CMD_TIMEOUT_MS 30u
/* First version targets the existing autonomous commissioning path. Setting
 * AUTO_AIM_ON_BOOT=0 keeps the remote/manual coexistence path on the original
 * PID implementation. Set this switch to 0 for a same-gain algorithm rollback. */
#ifndef GIMBAL_TRACKING_SPEED_PI_ENABLE
#define GIMBAL_TRACKING_SPEED_PI_ENABLE (GIMBAL_AUTO_AIM_ON_BOOT && GIMBAL_VISION_ENABLE)
#endif
#if (GIMBAL_TRACKING_SPEED_PI_ENABLE != 0 && GIMBAL_TRACKING_SPEED_PI_ENABLE != 1) || \
    (GIMBAL_TRACKING_SPEED_PI_ENABLE && !GIMBAL_AUTO_AIM_ON_BOOT)
#error tracking_v1 speed PI requires the autonomous commissioning path.
#endif
/* Use the command watchdog's existing time budget; never integrate a long
 * scheduling/debugging pause as if it were an ordinary motor-control step. */
#define GIMBAL_SPEED_PI_MAX_DT_S (0.001f * GIMBAL_CMD_TIMEOUT_MS)
/* Direction memory survives zero reference. A new direction must exceed the
 * speed corresponding to the existing yaw angle deadband (0.1deg * 8/s).
 * Pitch keeps its load-support integral; yaw-only reset is not a gravity model. */
#define GIMBAL_YAW_REVERSAL_REF_RAD_S (GIMBAL_YAW_ANGLE_KP * GIMBAL_YAW_ANGLE_DEADBAND_RAD)
#define GIMBAL_PITCH_REVERSAL_REF_RAD_S (GIMBAL_PITCH_ANGLE_KP * GIMBAL_YAW_ANGLE_DEADBAND_RAD)
#define RC_TIMEOUT_MS 100u
#define VISION_TX_PERIOD_MS 5u
#define VISION_RX_TIMEOUT_MS 100u
#define VISION_STARTUP_GRACE_MS 1500u
/* Inherited planning placeholder, not measured muzzle speed. */
#define VISION_NOMINAL_SPEED_MPS 30.0f
#endif
/* [USB_SP 新增 01 结束] */
