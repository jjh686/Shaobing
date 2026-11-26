#include "gimbal.h"
/* [USB_SP 修改 01 开始] IMU统一rad/rad/s双环、连续yaw、安装转换、启动相对机械包络、运动门控和驱动故障恢复。 */
#include "gimbal_config.h"
#include "gimbal_speed_pi.h"
#include "gimbal_pitch_control.h"
/* [USB_SP 修改 01 结束] */
#include "robot_def.h"
#include "dji_motor.h"
#include "dm4310_pitch.h"
#include "ins_task.h"
#include "message_center.h"
/* [USB_SP 修改 02 开始] IMU统一rad/rad/s双环、连续yaw、安装转换、启动相对机械包络、运动门控和驱动故障恢复。 */
#include <math.h>
#include <string.h>
/* [USB_SP 修改 02 结束] */
static DJIMotorInstance *yaw_motor;
static DM4310PitchInstance *pitch_motor;
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
static GimbalPitchAssist pitch_assist;
volatile GimbalPitchTrace gimbal_pitch_trace[GIMBAL_PITCH_TRACE_CAPACITY];
volatile uint32_t gimbal_pitch_trace_total, gimbal_pitch_trace_count, gimbal_pitch_trace_next;
static uint32_t trace_last_tick, trace_activity_tick, trace_last_gates;
static uint8_t trace_has_tracked;

static void GimbalRecordPitch(const Gimbal_Ctrl_Cmd_s *command, int enabled)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now-trace_last_tick) < GIMBAL_PITCH_TRACE_PERIOD_MS) return;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    uint32_t gates = pitch_motor->last_gate_flags;
    if (enabled && command->gimbal_mode == GIMBAL_VISION_MODE) {
        trace_has_tracked = 1;
        trace_activity_tick = now;
    }
    if (gates != trace_last_gates) trace_activity_tick = now;
    trace_last_gates = gates;
    if (trace_has_tracked && (uint32_t)(now-trace_activity_tick) > GIMBAL_PITCH_TRACE_TAIL_MS) {
        __set_PRIMASK(mask);
        return;
    }
    const Motor_Controller_s *c = &pitch_motor->motor_controller;
    GimbalPitchTrace record = {
        .tick=now, .motor_tick=pitch_motor->last_feedback_tick, .gate_flags=gates,
        .fault_reason_flags=pitch_motor->fault_reason_flags,
        .source_target_rad=command->pitch, .driver_target_rad=c->pid_ref,
        .measured_rad=pitch_feedback, .speed_rad_s=pitch_speed, .roll_rad=state.pose.roll,
        .mechanical_deg=pitch_motor->measure.total_angle-pitch_motor->startup_angle,
        .raw_speed_ref_rad_s=c->angle_PID.Output,
        .speed_ref_rad_s=c->speed_PID.Ref,
        .p_nm=c->speed_PID.Pout*GIMBAL_PITCH_DM_NM_PER_COMMAND,
        .i_nm=c->speed_PID.Iout*GIMBAL_PITCH_DM_NM_PER_COMMAND,
        .gravity_nm=pitch_assist.gravity_nm, .friction_nm=pitch_assist.friction_nm,
        .requested_nm=pitch_motor->requested_torque_nm,
        .applied_nm=pitch_motor->last_torque_nm, .reported_nm=pitch_motor->measure.torque_nm,
        .mode=(uint8_t)command->gimbal_mode, .enabled=(uint8_t)(enabled != 0),
        .motor_state=pitch_motor->measure.state, .fault=pitch_motor->fault_latched,
    };
    uint32_t slot = gimbal_pitch_trace_next;
    gimbal_pitch_trace[slot] = record;
    gimbal_pitch_trace[slot].sequence = ++gimbal_pitch_trace_total;
    gimbal_pitch_trace_next = (slot+1u)%GIMBAL_PITCH_TRACE_CAPACITY;
    if (gimbal_pitch_trace_count < GIMBAL_PITCH_TRACE_CAPACITY) ++gimbal_pitch_trace_count;
    trace_last_tick = now;
    __set_PRIMASK(mask);
}
#if GIMBAL_TRACKING_SPEED_PI_ENABLE
/* Motor task owns both PI states. The 5ms application task only queues reset
 * requests; it must never edit these states while the 1ms task is updating I. */
typedef struct {
    GimbalSpeedPi pi;
    const float *mechanical_angle, *startup_angle;
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
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    float mechanical_deg = *axis->mechanical_angle - *axis->startup_angle;
    float pitch_imu_deg = pitch_feedback / GIMBAL_RAD_PER_DEG;
    __set_PRIMASK(mask);
#if GIMBAL_PITCH_MOTION_LIMIT_ENABLE
    if (axis == &pitch_speed_axis) {
        ref_rad_s = GimbalPitchSpeedReference(&pitch_assist, ref_rad_s,
            pitch_motor->motor_controller.angle_PID.Err, pitch_imu_deg, dt_s);
    }
#endif
    float output = (pid->Kd == 0.0f && pid->DeadBand == 0.0f)
        ? GimbalSpeedPiStep(pi, measure_rad_s, ref_rad_s, dt_s) : NAN;

    /* Yaw uses a slip ring with no mechanical stop; keep only NaN safety here.
     * Pitch limits are enforced via IMU absolute angle in GimbalPitchSpeedReference
     * and dm4310_pitch.c. */
    if (!isfinite(mechanical_deg)) {
        output = NAN;
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
    /* [pitch力矩总装] 两项都是开环补偿，统一叠加在PI输出之后：
     *   速度前馈：按"指令速度"，抵消与运动方向有关的干摩擦/阻尼。不读反馈，
     *             不会放大陀螺上的结构振动；参考回零即归零，不留到位残余。
     *   重力前馈：按角度，见GimbalPitchCompensation。其内部的摩擦起动辅助
     *             当前系数为0，已由上面的速度前馈取代。
     * 注意：pid->Output只含PI部分，不含这两项；trace里p_nm/i_nm同样不含。 */
    if (axis == &pitch_speed_axis && isfinite(output)) {
        float ff_nm = GIMBAL_PITCH_SPEED_FF_NM * ref_rad_s / GIMBAL_PITCH_SPEED_LIMIT_RAD_S;
        ff_nm = fminf(fmaxf(ff_nm, -GIMBAL_PITCH_SPEED_FF_NM), GIMBAL_PITCH_SPEED_FF_NM);
        float compensation = GimbalPitchCompensation(&pitch_assist,
            pitch_motor->motor_controller.angle_PID.Measure,
            pitch_motor->motor_controller.angle_PID.Err, measure_rad_s, dt_s);
        output += (ff_nm+compensation)/GIMBAL_PITCH_DM_NM_PER_COMMAND;
    } else if (axis == &yaw_speed_axis && isfinite(output)) {
        float bemf_ff = GIMBAL_YAW_BEMF_FF_PER_APS * yaw_motor->measure.speed_aps;
        if (isfinite(bemf_ff)) {
            bemf_ff = fminf(fmaxf(bemf_ff, -GIMBAL_YAW_BEMF_FF_MAX), GIMBAL_YAW_BEMF_FF_MAX);
            output = fminf(fmaxf(output + bemf_ff, -GIMBAL_OUTPUT_LIMIT), GIMBAL_OUTPUT_LIMIT);
        }
    }
    return output;
}

static void GimbalPitchApplied(void *context, PIDInstance *pid,
                               float requested, float applied, uint32_t flags)
{
    GimbalSpeedAxis *axis = context;
    if (flags & DM_PITCH_GATE_ENVELOPE) {
        GimbalSpeedPiReset(&axis->pi, 0);
        if (axis->envelope_block_count < UINT32_MAX) ++axis->envelope_block_count;
    } else {
        GimbalSpeedPiApplyOutputLimit(&axis->pi, requested, applied);
    }
    pid->Iout = axis->pi.state.i;
    pid->Pout = axis->pi.p;
    pid->ITerm = axis->pi.integral_delta;
    pid->Output = axis->pi.output;
    pid->Last_Output = pid->Output;
    pid->Last_ITerm = pid->ITerm;
}

static void GimbalSpeedReset(void *context, PIDInstance *pid,
                            uint8_t preserve_integral)
{
    GimbalSpeedAxis *axis = context;
    GimbalSpeedPi *pi = &axis->pi;
    GimbalSpeedPiReset(pi, preserve_integral);
    if (axis == &pitch_speed_axis) GimbalPitchAssistReset(&pitch_assist, preserve_integral);
    /* Driver has already cleared PID bookkeeping and refreshed DWT. Restore
     * only the explicitly retained load-support I; clear output stays zero. */
    pid->Iout = pi->state.i;
}

static int GimbalConfigureSpeedPi(Motor_Controller_s *controller, GimbalSpeedAxis *axis,
                             const float *mechanical_angle, const float *startup_angle,
                             uint8_t clear_opposing_i, float reversal_ref_rad_s,
                             float pose_to_encoder_sign, float min_deg, float max_deg)
{
    axis->mechanical_angle = mechanical_angle;
    axis->startup_angle = startup_angle;
    axis->pose_to_encoder_sign = pose_to_encoder_sign;
    axis->min_deg = min_deg;
    axis->max_deg = max_deg;
    const PIDInstance *pid = &controller->speed_PID;
    GimbalSpeedPiConfig config = {
        .kp = pid->Kp, .ki = pid->Ki, .i_limit = pid->IntegralLimit,
        .output_limit = pid->MaxOut, .max_dt_s = GIMBAL_SPEED_PI_MAX_DT_S,
        .reset_i_on_ref_reversal = clear_opposing_i,
        .reversal_ref_min_rad_s = reversal_ref_rad_s,
    };
    if ((pose_to_encoder_sign != 1.0f && pose_to_encoder_sign != -1.0f) ||
        !isfinite(min_deg) || !isfinite(max_deg) || min_deg >= max_deg ||
        !GimbalSpeedPiInit(&axis->pi, &config)) {
        return 0;
    }
    return 1;
}
#endif

static int MotorFresh(const DJIMotorInstance *m, uint32_t now)
{
    return m && m->feedback_seen && (uint32_t)(now-m->last_feedback_tick)<=GIMBAL_MOTOR_TIMEOUT_MS;
}
static int PitchMotorFresh(const DM4310PitchInstance *m, uint32_t now)
{
    return m && m->feedback_seen && m->measure.state <= 1u &&
        (uint32_t)(now-m->last_feedback_tick)<=GIMBAL_MOTOR_TIMEOUT_MS;
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
    out->motors_ready=MotorFresh(yaw_motor,now) && PitchMotorFresh(pitch_motor,now);
    if (yaw_motor && yaw_motor->feedback_seen)
        out->yaw_mechanical_deg=yaw_motor->measure.total_angle-yaw_motor->startup_angle;
    if (pitch_motor && pitch_motor->feedback_seen)
        out->pitch_mechanical_deg=pitch_motor->measure.total_angle-pitch_motor->startup_angle;
    __set_PRIMASK(mask);
    float pitch_imu_deg = out->pose.pitch / GIMBAL_RAD_PER_DEG;
    if (!isfinite(out->yaw_mechanical_deg) || !isfinite(out->pitch_mechanical_deg) ||
        !isfinite(pitch_imu_deg) ||
        (out->pose_valid && (pitch_imu_deg<GIMBAL_PITCH_HARD_MIN_DEG || pitch_imu_deg>GIMBAL_PITCH_HARD_MAX_DEG)))
        out->motors_ready=0;
}
void GimbalInit(void)
{
    INS_Init();
    gimbal_pub=PubRegister("gimbal_feed",sizeof(Gimbal_Upload_Data_s));
    gimbal_sub=SubRegister("gimbal_cmd",sizeof(Gimbal_Ctrl_Cmd_s));
    Motor_Init_Config_s c={
        .can_init_config={.can_handle=&hcan1,.tx_id=1},
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
    /* DM MIT commands use the motor CAN ID itself; feedback uses Master ID.
     * IMU feedback and all reference signs remain identical to the baseline. */
    c.can_init_config.can_handle=GIMBAL_PITCH_DM_CAN_HANDLE;
    c.can_init_config.tx_id=GIMBAL_PITCH_DM_CAN_ID;
    c.can_init_config.rx_id=GIMBAL_PITCH_DM_MASTER_ID;
    c.motor_type=MOTOR_TYPE_NONE; /* Dedicated DM transport, never DJI grouping. */
    c.controller_param_init_config.angle_PID.Kp=GIMBAL_PITCH_ANGLE_KP;
    c.controller_param_init_config.angle_PID.DeadBand=0;
    c.controller_param_init_config.speed_PID.Kp=GIMBAL_PITCH_SPEED_KP;
    c.controller_param_init_config.speed_PID.Ki=GIMBAL_PITCH_SPEED_KI;
    c.controller_param_init_config.speed_PID.IntegralLimit=GIMBAL_PITCH_SPEED_I_LIMIT;
    c.controller_param_init_config.other_angle_feedback_ptr=&pitch_feedback;
    c.controller_param_init_config.other_speed_feedback_ptr=&pitch_speed;
    c.controller_setting_init_config.feedback_reverse_flag=GIMBAL_PITCH_OUTPUT_SIGN<0 ? FEEDBACK_DIRECTION_REVERSE:FEEDBACK_DIRECTION_NORMAL;
    pitch_motor=DM4310PitchInit(&c);
    if (!pitch_motor) {
        yaw_motor->fault_latched=1;
        DJIMotorStop(yaw_motor);
        return;
    }
/* 临时实验覆盖：外环 Kp 抬到 200，使 5° 误差给出 17.46rad/s 的参考，
 * 由 angle_PID.MaxOut 截断为 8.727rad/s(500°/s)。
 * 历史：120 试过，接近目标速度指令小了(1°→120°/s)，但远处跟随变慢，已回退。
 * 宏 GIMBAL_YAW_ANGLE_KP(8) 未被修改，
 * 因此 GIMBAL_YAW_REVERSAL_REF_RAD_S 仍按 8 计算，不随本覆盖联动。 */
    /* Yaw/Pitch outer gains come directly from GIMBAL_YAW_ANGLE_KP / GIMBAL_PITCH_ANGLE_KP. */
    /* Pitch gain comes only from GIMBAL_PITCH_ANGLE_KP (baseline 25). */
#if GIMBAL_TRACKING_SPEED_PI_ENABLE
    const GimbalPitchAssistConfig assist_config = {
        .max_speed_rad_s=GIMBAL_PITCH_SPEED_LIMIT_RAD_S,
        .accel_rad_s2=GIMBAL_PITCH_ACCEL_RAD_S2, .brake_rad_s2=GIMBAL_PITCH_BRAKE_RAD_S2,
        .min_deg=GIMBAL_PITCH_MIN_DEG, .max_deg=GIMBAL_PITCH_MAX_DEG,
        .pose_to_encoder_sign=1.0f, /* IMU horizontal zero */
        .max_dt_s=GIMBAL_SPEED_PI_MAX_DT_S,
        .gravity_cos_nm=GIMBAL_PITCH_GRAVITY_COS_NM, .gravity_sin_nm=GIMBAL_PITCH_GRAVITY_SIN_NM,
        .gravity_zero_rad=GIMBAL_PITCH_GRAVITY_ZERO_RAD, .gravity_ramp_nm_s=GIMBAL_PITCH_GRAVITY_RAMP_NM_S,
        .friction_positive_nm=GIMBAL_PITCH_FRICTION_POSITIVE_NM,
        .friction_negative_nm=GIMBAL_PITCH_FRICTION_NEGATIVE_NM,
        .friction_ramp_nm_s=GIMBAL_PITCH_FRICTION_RAMP_NM_S,
        .friction_start_rad=GIMBAL_PITCH_FRICTION_START_RAD,
        .friction_release_rad=GIMBAL_PITCH_FRICTION_RELEASE_RAD,
        .stopped_rad_s=GIMBAL_PITCH_FRICTION_STOPPED_RAD_S,
        .friction_timeout_s=GIMBAL_PITCH_FRICTION_TIMEOUT_S,
    };
    if (!GimbalPitchAssistInit(&pitch_assist, &assist_config)) pitch_motor->fault_latched=1;
    if (!GimbalConfigureSpeedPi(&yaw_motor->motor_controller, &yaw_speed_axis,
        &yaw_motor->measure.total_angle, &yaw_motor->startup_angle,
        0, GIMBAL_YAW_REVERSAL_REF_RAD_S, GIMBAL_YAW_POSE_TO_ENCODER_SIGN,
        GIMBAL_YAW_MIN_DEG, GIMBAL_YAW_MAX_DEG)) yaw_motor->fault_latched=1;
    if (!GimbalConfigureSpeedPi(&pitch_motor->motor_controller, &pitch_speed_axis,
        &pitch_motor->measure.total_angle, &pitch_motor->startup_angle,
        0, GIMBAL_PITCH_REVERSAL_REF_RAD_S, GIMBAL_PITCH_POSE_TO_ENCODER_SIGN,
        GIMBAL_PITCH_MIN_DEG, GIMBAL_PITCH_MAX_DEG)) pitch_motor->fault_latched=1;
    DJIMotorBindSpeedController(yaw_motor, GimbalSpeedUpdate, GimbalSpeedReset, &yaw_speed_axis);
    DM4310PitchBindSpeedController(pitch_motor, GimbalSpeedUpdate, GimbalSpeedReset, &pitch_speed_axis);
    DM4310PitchBindAppliedCallback(pitch_motor, GimbalPitchApplied, &pitch_speed_axis);
#endif
    yaw_motor->external_feedback_tick=pitch_motor->external_feedback_tick=&pose_tick;
    yaw_motor->external_feedback_timeout_ms=pitch_motor->external_feedback_timeout_ms=GIMBAL_INS_TIMEOUT_MS;
    yaw_motor->command_timeout_ms=pitch_motor->command_timeout_ms=GIMBAL_CMD_TIMEOUT_MS;
    DJIMotorStop(yaw_motor);DM4310PitchStop(pitch_motor);
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
    uint8_t yaw_ok=DJIMotorClearFault(yaw_motor);
    uint8_t pitch_ok=DM4310PitchClearFault(pitch_motor);
    uint8_t ok=yaw_ok && pitch_ok;
    __set_PRIMASK(mask);return ok;
}
void GimbalTask(void)
{
    if (!yaw_motor || !pitch_motor) return;
#if GIMBAL_TRACKING_SPEED_PI_ENABLE
    static uint8_t was_enabled;
    static gimbal_mode_e previous_mode = GIMBAL_ZERO_FORCE;
#endif
    Gimbal_Ctrl_Cmd_s command={0};SubGetMessage(gimbal_sub,&command);
    GimbalState s;GimbalGetState(&s);
    float yaw_ref=0,pitch_ref=0,pitch_limited=0;
    int enabled=GIMBAL_INSTALLATION_CONFIRMED && s.pose_valid && s.motors_ready && !s.driver_fault &&
        (command.gimbal_mode==GIMBAL_FREE_MODE || command.gimbal_mode==GIMBAL_GYRO_MODE || command.gimbal_mode==GIMBAL_VISION_MODE);
    if (enabled) {
        /* Yaw uses slip-ring continuous IMU world heading (no ±360° encoder cap);
         * Pitch uses IMU absolute horizontal zero [-30.5°, +30.5°]. */
        float yaw_err=GimbalWrapRad(command.yaw-s.pose.yaw);
        const float max_yaw_err=((command.gimbal_mode==GIMBAL_VISION_MODE)
            ? GIMBAL_VISION_MAX_ERROR_DEG
            : GIMBAL_YAW_REMOTE_MAX_ERROR_DEG)*GIMBAL_RAD_PER_DEG;
        yaw_err=fminf(fmaxf(yaw_err,-max_yaw_err),max_yaw_err);
        float pitch_imu_deg=s.pose.pitch/GIMBAL_RAD_PER_DEG;
        enabled=isfinite(yaw_err) &&
            GimbalEncoderTarget(command.pitch,s.pose.pitch,pitch_imu_deg,
            1.0f,GIMBAL_PITCH_MIN_DEG,GIMBAL_PITCH_MAX_DEG,
            GIMBAL_VISION_MAX_ERROR_DEG,&pitch_limited);
        yaw_ref=s.yaw_continuous+yaw_err;
        pitch_ref=pitch_limited*GIMBAL_RAD_PER_DEG;
/* [USB_SP 修改 07 结束] */
    }
/* [USB_SP 修改 08 开始] IMU统一rad/rad/s双环、连续yaw、安装转换、启动相对机械包络、运动门控和驱动故障恢复。 */
    uint32_t mask=__get_PRIMASK();__disable_irq();
    if (enabled) {
        DJIMotorSetRef(yaw_motor,yaw_ref);DM4310PitchSetRef(pitch_motor,pitch_ref);
#if GIMBAL_TRACKING_SPEED_PI_ENABLE
        if (!was_enabled) {
            /* A STOP may have lasted less than a motor-task period. Always
             * queue fresh state on re-enable; do not depend on observing STOP. */
            DJIMotorRequestSpeedReset(yaw_motor, 0);
            DM4310PitchRequestSpeedReset(pitch_motor, 0);
        } else if (command.gimbal_mode != previous_mode) {
            /* TRACKING <-> HOLD: yaw drops old bias; pitch retains load support.
             * Both discard error history. No reset on ordinary target updates. */
            DJIMotorRequestSpeedReset(yaw_motor, 0);
            DM4310PitchRequestSpeedReset(pitch_motor, 1);
        }
#endif
        DJIMotorEnable(yaw_motor);DM4310PitchEnable(pitch_motor);
    } else {DJIMotorStop(yaw_motor);DM4310PitchStop(pitch_motor);}
#if GIMBAL_TRACKING_SPEED_PI_ENABLE
    was_enabled = enabled != 0;
    previous_mode = command.gimbal_mode;
#endif
    __set_PRIMASK(mask);
    GimbalRecordPitch(&command, enabled);
    Gimbal_Upload_Data_s feedback={0};INS_Snapshot_t ins;
    if (INS_GetSnapshot(&ins)) feedback.gimbal_imu_data=ins.attitude;
    mask=__get_PRIMASK();__disable_irq();
    feedback.yaw_motor_single_round_angle=yaw_motor->measure.angle_single_round;
    __set_PRIMASK(mask);
    PubPushMessage(gimbal_pub,&feedback);
}
/* [USB_SP 修改 08 结束] */
