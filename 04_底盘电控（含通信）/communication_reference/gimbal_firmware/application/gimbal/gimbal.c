#include "gimbal.h"
/* [USB_SP 修改 01 开始] IMU统一rad/rad/s双环、连续yaw、安装转换、启动相对机械包络、运动门控和驱动故障恢复。 */
#include "gimbal_config.h"
#include "gimbal_speed_pi.h"
/* [USB_SP 修改 01 结束] */
#include "robot_def.h"
#include "dji_motor.h"
#include "ins_task.h"
#include "message_center.h"
/* [USB_SP 修改 02 开始] IMU统一rad/rad/s双环、连续yaw、安装转换、启动相对机械包络、运动门控和驱动故障恢复。 */
#include <math.h>
#include <string.h>
/* [USB_SP 修改 02 结束] */
static DJIMotorInstance *yaw_motor, *pitch_motor;
/* [USB_SP 修改 03 开始] IMU统一rad/rad/s双环、连续yaw、安装转换、启动相对机械包络、运动门控和驱动故障恢复。 */
static Publisher_t *gimbal_pub;
static Subscriber_t *gimbal_sub;
static GimbalState state;
static uint8_t pose_seen;
static float previous_yaw, continuous_yaw;
/* Angle and speed feedback use rad and rad/s in the two PID cascades. */
static float yaw_feedback, pitch_feedback, yaw_speed, pitch_speed;
static volatile uint32_t pose_tick;
static const float mount[9] = GIMBAL_TO_IMU_ROTATION;
#if GIMBAL_TRACKING_SPEED_PI_ENABLE
/* Motor task owns both PI states. The 5ms application task only queues reset
 * requests; it must never edit these states while the 1ms task is updating I. */
typedef struct {
    GimbalSpeedPi pi;
    DJIMotorInstance *motor;
    float pose_to_encoder_sign, min_deg, max_deg;
    uint32_t envelope_block_count;
} GimbalSpeedAxis;
static GimbalSpeedAxis yaw_speed_axis, pitch_speed_axis;

static float GimbalSpeedUpdate(void *context, PIDInstance *pid,
                               float measure_rad_s, float ref_rad_s)
{
    GimbalSpeedAxis *axis = context;
    GimbalSpeedPi *pi = &axis->pi;
    float dt_s = DWT_GetDeltaT(&pid->DWT_CNT);
    /* One authoritative gain set: existing PID watch fields remain tunable.
     * The dedicated controller is PI only; do not silently ignore a new Kd or
     * speed deadband entered in the debugger. Invalid configuration faults. */
    pi->config.kp = pid->Kp;
    pi->config.ki = pid->Ki;
    pi->config.i_limit = pid->IntegralLimit;
    pi->config.output_limit = pid->MaxOut;
    float output = (pid->Kd == 0.0f && pid->DeadBand == 0.0f)
        ? GimbalSpeedPiStep(pi, measure_rad_s, ref_rad_s, dt_s) : NAN;

    /* Limiting the angle target alone cannot prevent a retained I term from
     * pushing outward at a zero speed reference. Enforce the SAME startup
     * encoder envelope at the motor rate. Positive controller output means
     * positive pose; the existing pose->encoder sign maps that to mechanics.
     * The final configured actuator sign is still applied only by the driver. */
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    float mechanical_deg = axis->motor->measure.total_angle - axis->motor->startup_angle;
    __set_PRIMASK(mask);
    if (!isfinite(mechanical_deg)) {
        output = NAN;
    } else if (isfinite(output)) {
        float encoder_effort = output * axis->pose_to_encoder_sign;
        if ((mechanical_deg >= axis->max_deg && encoder_effort > 0.0f) ||
            (mechanical_deg <= axis->min_deg && encoder_effort < 0.0f)) {
            GimbalSpeedPiReset(pi, 0);
            output = 0.0f;
            if (axis->envelope_block_count < UINT32_MAX) ++axis->envelope_block_count;
        }
    }

    pid->dt = dt_s;
    pid->Measure = measure_rad_s;
    pid->Ref = ref_rad_s;
    pid->Err = ref_rad_s - measure_rad_s;
    pid->Pout = isfinite(output) ? pi->p : NAN;
    pid->Iout = pi->state.i;
    pid->Dout = 0.0f;
    pid->ITerm = isfinite(output) ? pi->integral_delta : 0.0f;
    pid->Output = output; /* NAN is deliberately passed to the driver fault gate. */
    pid->Last_Measure = pid->Measure;
    pid->Last_Err = pid->Err;
    pid->Last_ITerm = pid->ITerm;
    pid->Last_Output = pid->Output;
    pid->Last_Dout = 0.0f;
    return output;
}

static void GimbalSpeedReset(void *context, PIDInstance *pid,
                            uint8_t preserve_integral)
{
    GimbalSpeedAxis *axis = context;
    GimbalSpeedPi *pi = &axis->pi;
    GimbalSpeedPiReset(pi, preserve_integral);
    /* Driver has already cleared PID bookkeeping and refreshed DWT. Restore
     * only the explicitly retained load-support I; clear output stays zero. */
    pid->Iout = pi->state.i;
}

static void GimbalBindSpeedPi(DJIMotorInstance *motor, GimbalSpeedAxis *axis,
                             uint8_t clear_opposing_i, float reversal_ref_rad_s,
                             float pose_to_encoder_sign, float min_deg, float max_deg)
{
    axis->motor = motor;
    axis->pose_to_encoder_sign = pose_to_encoder_sign;
    axis->min_deg = min_deg;
    axis->max_deg = max_deg;
    const PIDInstance *pid = &motor->motor_controller.speed_PID;
    GimbalSpeedPiConfig config = {
        .kp = pid->Kp, .ki = pid->Ki, .i_limit = pid->IntegralLimit,
        .output_limit = pid->MaxOut, .max_dt_s = GIMBAL_SPEED_PI_MAX_DT_S,
        .reset_i_on_ref_reversal = clear_opposing_i,
        .reversal_ref_min_rad_s = reversal_ref_rad_s,
    };
    if ((pose_to_encoder_sign != 1.0f && pose_to_encoder_sign != -1.0f) ||
        !isfinite(min_deg) || !isfinite(max_deg) || min_deg >= max_deg ||
        !GimbalSpeedPiInit(&axis->pi, &config)) {
        motor->fault_latched = 1;
        return;
    }
    DJIMotorBindSpeedController(motor, GimbalSpeedUpdate, GimbalSpeedReset, axis);
}
#endif

static int MotorFresh(const DJIMotorInstance *m, uint32_t now)
{
    return m && m->feedback_seen && (uint32_t)(now-m->last_feedback_tick)<=GIMBAL_MOTOR_TIMEOUT_MS;
}
void GimbalUpdateFeedback(void)
{
    INS_Snapshot_t ins;
    GimbalPose pose;
    uint8_t valid=INS_GetSnapshot(&ins) &&
        (uint32_t)(HAL_GetTick()-ins.tick)<=GIMBAL_INS_TIMEOUT_MS &&
        GimbalPoseFromImu(ins.q,ins.gyro,mount,&pose);
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    state.pose_valid=valid;
    if (valid) {
        if (!pose_seen) {continuous_yaw=pose.yaw;pose_seen=1;}
        else continuous_yaw+=GimbalWrapRad(pose.yaw-previous_yaw);
        previous_yaw=pose.yaw;
        state.pose=pose;state.yaw_continuous=continuous_yaw;
        state.tick=pose_tick=ins.tick;
        yaw_feedback=continuous_yaw;pitch_feedback=pose.pitch;
        yaw_speed=pose.yaw_vel;pitch_speed=pose.pitch_vel;
    }
    __set_PRIMASK(mask);
}
void GimbalGetState(GimbalState *out)
{
    if (!out) return;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    *out=state;
    uint32_t now=HAL_GetTick();
    out->pose_valid=out->pose_valid && (uint32_t)(now-out->tick)<=GIMBAL_INS_TIMEOUT_MS;
    out->driver_fault=(yaw_motor && yaw_motor->fault_latched) || (pitch_motor && pitch_motor->fault_latched);
    out->motors_ready=MotorFresh(yaw_motor,now) && MotorFresh(pitch_motor,now);
    if (yaw_motor && yaw_motor->feedback_seen)
        out->yaw_mechanical_deg=yaw_motor->measure.total_angle-yaw_motor->startup_angle;
    if (pitch_motor && pitch_motor->feedback_seen)
        out->pitch_mechanical_deg=pitch_motor->measure.total_angle-pitch_motor->startup_angle;
    __set_PRIMASK(mask);
    if (!isfinite(out->yaw_mechanical_deg) || !isfinite(out->pitch_mechanical_deg) ||
        out->yaw_mechanical_deg<GIMBAL_YAW_MIN_DEG-1 || out->yaw_mechanical_deg>GIMBAL_YAW_MAX_DEG+1 ||
        out->pitch_mechanical_deg<GIMBAL_PITCH_MIN_DEG-1 || out->pitch_mechanical_deg>GIMBAL_PITCH_MAX_DEG+1)
        out->motors_ready=0;
}
void GimbalInit(void)
{
    /* INS initialization runs in its own task. A missing IMU must not block
     * scheduler startup or the USB/CAN navigation gateway. */
    Motor_Init_Config_s c={
        /* CAN1 is reserved for the navigation gateway. Actuators stay gated. */
        .can_init_config={.can_handle=&hcan2,.tx_id=1},
        .controller_param_init_config={
            .angle_PID={.Kp=GIMBAL_YAW_ANGLE_KP,.DeadBand=GIMBAL_YAW_ANGLE_DEADBAND_RAD,
                .Improve=PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement,
                .IntegralLimit=100*GIMBAL_RAD_PER_DEG,.MaxOut=GIMBAL_ANGLE_SPEED_LIMIT},
            .speed_PID={.Kp=GIMBAL_YAW_SPEED_KP,.Ki=GIMBAL_YAW_SPEED_KI,
                .Improve=PID_Trapezoid_Intergral|PID_Integral_Limit|PID_Derivative_On_Measurement,
                .IntegralLimit=GIMBAL_YAW_SPEED_I_LIMIT,.MaxOut=GIMBAL_OUTPUT_LIMIT},
            .other_angle_feedback_ptr=&yaw_feedback,.other_speed_feedback_ptr=&yaw_speed,
/* [USB_SP 修改 03 结束] */
        },
/* [USB_SP 修改 04 开始] IMU统一rad/rad/s双环、连续yaw、安装转换、启动相对机械包络、运动门控和驱动故障恢复。 */
        .controller_setting_init_config={
            .angle_feedback_source=OTHER_FEED,.speed_feedback_source=OTHER_FEED,
            .outer_loop_type=ANGLE_LOOP,.close_loop_type=ANGLE_LOOP|SPEED_LOOP,
            .motor_reverse_flag=MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag=GIMBAL_YAW_OUTPUT_SIGN<0 ? FEEDBACK_DIRECTION_REVERSE:FEEDBACK_DIRECTION_NORMAL,
        },.motor_type=GM6020,
/* [USB_SP 修改 04 结束] */
    };
/* [USB_SP 修改 05 开始] IMU统一rad/rad/s双环、连续yaw、安装转换、启动相对机械包络、运动门控和驱动故障恢复。 */
    yaw_motor=DJIMotorInit(&c);
    c.can_init_config.tx_id=2;
    c.controller_param_init_config.angle_PID.Kp=GIMBAL_PITCH_ANGLE_KP;
    c.controller_param_init_config.angle_PID.DeadBand=0;
    c.controller_param_init_config.speed_PID.Kp=GIMBAL_PITCH_SPEED_KP;
    c.controller_param_init_config.speed_PID.Ki=GIMBAL_PITCH_SPEED_KI;
    c.controller_param_init_config.speed_PID.IntegralLimit=GIMBAL_PITCH_SPEED_I_LIMIT;
    c.controller_param_init_config.other_angle_feedback_ptr=&pitch_feedback;
    c.controller_param_init_config.other_speed_feedback_ptr=&pitch_speed;
    c.controller_setting_init_config.feedback_reverse_flag=GIMBAL_PITCH_OUTPUT_SIGN<0 ? FEEDBACK_DIRECTION_REVERSE:FEEDBACK_DIRECTION_NORMAL;
    pitch_motor=DJIMotorInit(&c);
/* 临时实验覆盖：外环 Kp 抬到 200，使 5° 误差给出 17.46rad/s 的参考，
 * 由 angle_PID.MaxOut 截断为 8.727rad/s(500°/s)，用于测量内环起动输出。
 * 实验结束必须删除本块；宏 GIMBAL_*_ANGLE_KP 未被修改，
 * 因此 GIMBAL_YAW_REVERSAL_REF_RAD_S 仍按 8 计算，不随本覆盖联动。 */
    yaw_motor->motor_controller.angle_PID.Kp = 200.0f;
    pitch_motor->motor_controller.angle_PID.Kp = 200.0f;
#if GIMBAL_TRACKING_SPEED_PI_ENABLE
    GimbalBindSpeedPi(yaw_motor, &yaw_speed_axis, 1, GIMBAL_YAW_REVERSAL_REF_RAD_S,
        GIMBAL_YAW_POSE_TO_ENCODER_SIGN, GIMBAL_YAW_MIN_DEG, GIMBAL_YAW_MAX_DEG);
    GimbalBindSpeedPi(pitch_motor, &pitch_speed_axis, 0, GIMBAL_PITCH_REVERSAL_REF_RAD_S,
        GIMBAL_PITCH_POSE_TO_ENCODER_SIGN, GIMBAL_PITCH_MIN_DEG, GIMBAL_PITCH_MAX_DEG);
#endif
    yaw_motor->external_feedback_tick=pitch_motor->external_feedback_tick=&pose_tick;
    yaw_motor->external_feedback_timeout_ms=pitch_motor->external_feedback_timeout_ms=GIMBAL_INS_TIMEOUT_MS;
    yaw_motor->command_timeout_ms=pitch_motor->command_timeout_ms=GIMBAL_CMD_TIMEOUT_MS;
    DJIMotorStop(yaw_motor);DJIMotorStop(pitch_motor);
    gimbal_pub=PubRegister("gimbal_feed",sizeof(Gimbal_Upload_Data_s));
    gimbal_sub=SubRegister("gimbal_cmd",sizeof(Gimbal_Ctrl_Cmd_s));
/* [USB_SP 修改 05 结束] */
}
/* [USB_SP 修改 06 开始] IMU统一rad/rad/s双环、连续yaw、安装转换、启动相对机械包络、运动门控和驱动故障恢复。 */
uint8_t GimbalClearMotorFaults(void)
/* [USB_SP 修改 06 结束] */
{
/* [USB_SP 修改 07 开始] IMU统一rad/rad/s双环、连续yaw、安装转换、启动相对机械包络、运动门控和驱动故障恢复。 */
    GimbalState s;GimbalGetState(&s);
    if (!s.pose_valid || !s.motors_ready) return 0;
    uint32_t mask=__get_PRIMASK();__disable_irq();
    uint8_t ok=DJIMotorClearFault(yaw_motor) && DJIMotorClearFault(pitch_motor);
    __set_PRIMASK(mask);return ok;
}
void GimbalTask(void)
{
#if GIMBAL_TRACKING_SPEED_PI_ENABLE
    static uint8_t was_enabled;
    static gimbal_mode_e previous_mode = GIMBAL_ZERO_FORCE;
#endif
    Gimbal_Ctrl_Cmd_s command={0};SubGetMessage(gimbal_sub,&command);
    GimbalState s;GimbalGetState(&s);
    float yaw_ref=0,pitch_ref=0,yaw_limited=0,pitch_limited=0;
    int enabled=GIMBAL_INSTALLATION_CONFIRMED && s.pose_valid && s.motors_ready && !s.driver_fault &&
        (command.gimbal_mode==GIMBAL_FREE_MODE || command.gimbal_mode==GIMBAL_GYRO_MODE || command.gimbal_mode==GIMBAL_VISION_MODE);
    if (enabled) {
        /* Encoder is only a mechanical-envelope monitor. Inner loop stays IMU. */
        enabled=GimbalEncoderTarget(command.yaw,s.pose.yaw,s.yaw_mechanical_deg,
            GIMBAL_YAW_POSE_TO_ENCODER_SIGN,GIMBAL_YAW_MIN_DEG,GIMBAL_YAW_MAX_DEG,
            GIMBAL_VISION_MAX_ERROR_DEG,&yaw_limited) &&
            GimbalEncoderTarget(command.pitch,s.pose.pitch,s.pitch_mechanical_deg,
            GIMBAL_PITCH_POSE_TO_ENCODER_SIGN,GIMBAL_PITCH_MIN_DEG,GIMBAL_PITCH_MAX_DEG,
            GIMBAL_VISION_MAX_ERROR_DEG,&pitch_limited);
        yaw_ref=s.yaw_continuous+(yaw_limited-s.yaw_mechanical_deg)*GIMBAL_YAW_POSE_TO_ENCODER_SIGN*GIMBAL_RAD_PER_DEG;
        pitch_ref=s.pose.pitch+(pitch_limited-s.pitch_mechanical_deg)*GIMBAL_PITCH_POSE_TO_ENCODER_SIGN*GIMBAL_RAD_PER_DEG;
/* [USB_SP 修改 07 结束] */
    }
/* [USB_SP 修改 08 开始] IMU统一rad/rad/s双环、连续yaw、安装转换、启动相对机械包络、运动门控和驱动故障恢复。 */
    uint32_t mask=__get_PRIMASK();__disable_irq();
    if (enabled) {
        DJIMotorSetRef(yaw_motor,yaw_ref);DJIMotorSetRef(pitch_motor,pitch_ref);
#if GIMBAL_TRACKING_SPEED_PI_ENABLE
        if (!was_enabled) {
            /* A STOP may have lasted less than a motor-task period. Always
             * queue fresh state on re-enable; do not depend on observing STOP. */
            DJIMotorRequestSpeedReset(yaw_motor, 0);
            DJIMotorRequestSpeedReset(pitch_motor, 0);
        } else if (command.gimbal_mode != previous_mode) {
            /* TRACKING <-> HOLD: yaw drops old bias; pitch retains load support.
             * Both discard error history. No reset on ordinary target updates. */
            DJIMotorRequestSpeedReset(yaw_motor, 0);
            DJIMotorRequestSpeedReset(pitch_motor, 1);
        }
#endif
        DJIMotorEnable(yaw_motor);DJIMotorEnable(pitch_motor);
    } else {DJIMotorStop(yaw_motor);DJIMotorStop(pitch_motor);}
#if GIMBAL_TRACKING_SPEED_PI_ENABLE
    was_enabled = enabled != 0;
    previous_mode = command.gimbal_mode;
#endif
    __set_PRIMASK(mask);
    Gimbal_Upload_Data_s feedback={0};INS_Snapshot_t ins;
    if (INS_GetSnapshot(&ins)) feedback.gimbal_imu_data=ins.attitude;
    mask=__get_PRIMASK();__disable_irq();
    feedback.yaw_motor_single_round_angle=yaw_motor->measure.angle_single_round;
    __set_PRIMASK(mask);
    PubPushMessage(gimbal_pub,&feedback);
}
/* [USB_SP 修改 08 结束] */
