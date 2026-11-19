/* Test the real adapter and math/controller sources. MCU clocks, snapshots,
 * message transport and motor service calls are replaced by observable stubs.
 * Include gimbal.c to inspect its private binding and call the real callbacks. */
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gimbal.c"

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); \
    exit(EXIT_FAILURE); } } while (0)

CAN_HandleTypeDef hcan1, hcan2;
static uint32_t clock_ms = 100, primask, dwt_calls;
static float next_dt_s = 0.001f;
static uint32_t *last_dwt_counter;
static INS_Snapshot_t snapshot;
static uint8_t snapshot_available = 1;
static Gimbal_Ctrl_Cmd_s incoming_command;
static Gimbal_Upload_Data_s published;
static Publisher_t publisher;
static Subscriber_t subscriber;
static DJIMotorInstance yaw_stub;
static DM4310PitchInstance pitch_stub;
static uint8_t fail_pitch_init;
static Motor_Init_Config_s configs[2];
static unsigned init_count, stop_calls[2], enable_calls[2], reset_calls[2];
static uint8_t last_preserve[2];
static unsigned publications;

static unsigned MotorIndex(const void *m)
{
    CHECK(m == &yaw_stub || m == &pitch_stub);
    return m == &yaw_stub ? 0u : 1u;
}
static void Near(float actual, float expected, float tolerance)
{
    CHECK(isfinite(actual));
    if (fabsf(actual - expected) > tolerance) {
        fprintf(stderr, "actual=%g expected=%g tolerance=%g\n", actual, expected, tolerance);
        CHECK(0);
    }
}
uint32_t HAL_GetTick(void) { return clock_ms; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1; }
void __set_PRIMASK(uint32_t value) { primask = value; }
float DWT_GetDeltaT(uint32_t *counter)
{
    last_dwt_counter = counter;
    *counter = ++dwt_calls;
    return next_dt_s;
}
attitude_t *INS_Init(void) { return &snapshot.attitude; }
uint8_t INS_GetSnapshot(INS_Snapshot_t *out)
{
    *out = snapshot;
    return snapshot_available;
}
Publisher_t *PubRegister(char *name, uint8_t size)
{
    CHECK(strcmp(name, "gimbal_feed") == 0);
    CHECK(size == sizeof(Gimbal_Upload_Data_s));
    return &publisher;
}
Subscriber_t *SubRegister(char *name, uint8_t size)
{
    CHECK(strcmp(name, "gimbal_cmd") == 0);
    CHECK(size == sizeof(Gimbal_Ctrl_Cmd_s));
    return &subscriber;
}
uint8_t SubGetMessage(Subscriber_t *sub, void *data)
{
    CHECK(sub == &subscriber);
    memcpy(data, &incoming_command, sizeof(incoming_command));
    return 1;
}
uint8_t PubPushMessage(Publisher_t *pub, void *data)
{
    CHECK(pub == &publisher);
    memcpy(&published, data, sizeof(published));
    ++publications;
    return 1;
}
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *c)
{
    CHECK(init_count == 0);
    ++init_count;
    configs[0] = *c;
    DJIMotorInstance *m = &yaw_stub;
    memset(m, 0, sizeof(*m));
    m->motor_type = c->motor_type;
    m->motor_settings = c->controller_setting_init_config;
    PIDInit(&m->motor_controller.angle_PID, &c->controller_param_init_config.angle_PID);
    PIDInit(&m->motor_controller.speed_PID, &c->controller_param_init_config.speed_PID);
    m->motor_controller.other_angle_feedback_ptr = c->controller_param_init_config.other_angle_feedback_ptr;
    m->motor_controller.other_speed_feedback_ptr = c->controller_param_init_config.other_speed_feedback_ptr;
    return m;
}
void DJIMotorBindSpeedController(DJIMotorInstance *m, DJIMotorSpeedUpdateFn update,
                                DJIMotorSpeedResetFn reset, void *context)
{
    m->speed_controller.update = update;
    m->speed_controller.reset = reset;
    m->speed_controller.context = context;
}
void DJIMotorRequestSpeedReset(DJIMotorInstance *m, uint8_t preserve)
{
    unsigned i = MotorIndex(m);
    ++reset_calls[i];
    last_preserve[i] = preserve;
}
void DJIMotorSetRef(DJIMotorInstance *m, float ref)
{
    m->motor_controller.pid_ref = ref;
    m->last_command_tick = clock_ms;
}
void DJIMotorStop(DJIMotorInstance *m)
{
    ++stop_calls[MotorIndex(m)];
    m->stop_flag = MOTOR_STOP;
    /* Deliberately do not run a motor cycle or clear the PI here. */
}
void DJIMotorEnable(DJIMotorInstance *m)
{
    ++enable_calls[MotorIndex(m)];
    m->stop_flag = MOTOR_ENABLED;
}
uint8_t DJIMotorClearFault(DJIMotorInstance *m)
{
    m->fault_latched = 0;
    return 1;
}
DM4310PitchInstance *DM4310PitchInit(Motor_Init_Config_s *c)
{
    CHECK(init_count == 1);
    ++init_count;
    configs[1] = *c;
    if (fail_pitch_init) return NULL;
    DM4310PitchInstance *m = &pitch_stub;
    memset(m, 0, sizeof(*m));
    m->motor_settings = c->controller_setting_init_config;
    PIDInit(&m->motor_controller.angle_PID, &c->controller_param_init_config.angle_PID);
    PIDInit(&m->motor_controller.speed_PID, &c->controller_param_init_config.speed_PID);
    m->motor_controller.other_angle_feedback_ptr = c->controller_param_init_config.other_angle_feedback_ptr;
    m->motor_controller.other_speed_feedback_ptr = c->controller_param_init_config.other_speed_feedback_ptr;
    return m;
}
void DM4310PitchBindSpeedController(DM4310PitchInstance *m,
                                   DJIMotorSpeedUpdateFn update,
                                   DJIMotorSpeedResetFn reset, void *context)
{
    m->speed_controller.update = update;
    m->speed_controller.reset = reset;
    m->speed_controller.context = context;
}
void DM4310PitchRequestSpeedReset(DM4310PitchInstance *m, uint8_t preserve)
{
    unsigned i = MotorIndex(m);
    ++reset_calls[i];
    last_preserve[i] = preserve;
}
void DM4310PitchBindAppliedCallback(DM4310PitchInstance *m,
    DM4310PitchAppliedFn callback, void *context)
{
    m->applied_callback = callback;
    m->applied_context = context;
}
void DM4310PitchSetRef(DM4310PitchInstance *m, float ref)
{
    m->motor_controller.pid_ref = ref;
    m->last_command_tick = clock_ms;
}
void DM4310PitchStop(DM4310PitchInstance *m)
{
    ++stop_calls[MotorIndex(m)];
    m->stop_flag = MOTOR_STOP;
}
void DM4310PitchEnable(DM4310PitchInstance *m)
{
    ++enable_calls[MotorIndex(m)];
    m->stop_flag = MOTOR_ENABLED;
}
uint8_t DM4310PitchClearFault(DM4310PitchInstance *m)
{
    m->fault_latched = 0;
    return 1;
}
static void ApplyReset(void *m, uint8_t preserve)
{
    if (MotorIndex(m) == 0u) {
        PIDClear(&yaw_stub.motor_controller.speed_PID);
        yaw_stub.speed_controller.reset(yaw_stub.speed_controller.context,
                                         &yaw_stub.motor_controller.speed_PID, preserve);
    } else {
        PIDClear(&pitch_stub.motor_controller.speed_PID);
        pitch_stub.speed_controller.reset(pitch_stub.speed_controller.context,
                                           &pitch_stub.motor_controller.speed_PID, preserve);
    }
}
static float Step(void *m, float measure, float ref)
{
    if (MotorIndex(m) == 0u)
        return yaw_stub.speed_controller.update(yaw_stub.speed_controller.context,
                                                &yaw_stub.motor_controller.speed_PID, measure, ref);
    return pitch_stub.speed_controller.update(pitch_stub.speed_controller.context,
                                              &pitch_stub.motor_controller.speed_PID, measure, ref);
}
static void Prepare(void)
{
    clock_ms += 10;
    next_dt_s = 0.001f;
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.q[0] = 1.0f;
    snapshot.tick = clock_ms;
    snapshot.valid = 1;
    snapshot_available = 1;
    yaw_stub.feedback_seen = pitch_stub.feedback_seen = 1;
    yaw_stub.last_feedback_tick = pitch_stub.last_feedback_tick = clock_ms;
    yaw_stub.measure.total_angle = yaw_stub.startup_angle = 100.0f;
    pitch_stub.measure.total_angle = pitch_stub.startup_angle = 100.0f;
    pitch_stub.measure.state = 1;
    yaw_stub.fault_latched = pitch_stub.fault_latched = 0;
    GimbalUpdateFeedback();
    memset(&incoming_command, 0, sizeof(incoming_command));
    incoming_command.gimbal_mode = GIMBAL_ZERO_FORCE;
    GimbalTask(); /* Establish the application's disabled state. */
    ApplyReset(&yaw_stub, 0);
    ApplyReset(&pitch_stub, 0);
    for (unsigned i = 0; i < 2; ++i) {
        reset_calls[i] = 0;
        stop_calls[i] = enable_calls[i] = 0;
    }
    yaw_speed_axis.pose_to_encoder_sign = pitch_speed_axis.pose_to_encoder_sign = 1.0f;
}
static void BindingsAndOriginalGains(void)
{
    CHECK(init_count == 2);
    CHECK(configs[0].can_init_config.can_handle == &hcan1);
    /* 实机接线：pitch DM4310 在 CAN2，CAN1 留给与底盘的板间链路。 */
    CHECK(configs[1].can_init_config.can_handle == &hcan2);
    CHECK(configs[0].can_init_config.tx_id == 1 && configs[1].can_init_config.tx_id == 2);
    CHECK(yaw_motor == &yaw_stub && pitch_motor == &pitch_stub);
    CHECK(yaw_stub.motor_type == GM6020);
    CHECK(configs[1].motor_type == MOTOR_TYPE_NONE);
    CHECK(configs[1].can_init_config.rx_id == 0);
    CHECK(configs[1].can_init_config.tx_id == 2);
    CHECK(yaw_stub.speed_controller.context == &yaw_speed_axis);
    CHECK(pitch_stub.speed_controller.context == &pitch_speed_axis);
    CHECK(yaw_speed_axis.pi.config.reset_i_on_ref_reversal == 1);
    CHECK(pitch_speed_axis.pi.config.reset_i_on_ref_reversal == 0);
    Near(yaw_speed_axis.pi.config.reversal_ref_min_rad_s, GIMBAL_YAW_ANGLE_KP * GIMBAL_YAW_ANGLE_DEADBAND_RAD, 1e-7f);
    Near(yaw_speed_axis.pi.config.max_dt_s, 0.03f, 1e-7f);
    /* Keep yaw's commissioned override; pitch has one authoritative gain. */
    Near(yaw_stub.motor_controller.angle_PID.Kp, 200.0f, 1e-7f);
    Near(pitch_stub.motor_controller.angle_PID.Kp, 25.0f, 1e-7f);
    Near(yaw_speed_axis.pi.config.kp, 200.0f, 1e-7f);
    Near(pitch_speed_axis.pi.config.kp, 900.0f, 1e-7f);
    Near(yaw_speed_axis.pi.config.ki, 420.0f, 1e-7f);
    Near(pitch_speed_axis.pi.config.ki, 500.0f, 1e-7f);
    Near(yaw_speed_axis.pi.config.i_limit, 6000.0f, 1e-7f);
    Near(pitch_speed_axis.pi.config.i_limit, 1200.0f, 1e-7f);
    CHECK(yaw_stub.motor_settings.angle_feedback_source == OTHER_FEED);
    CHECK(yaw_stub.motor_settings.speed_feedback_source == OTHER_FEED);
    CHECK(pitch_stub.motor_settings.angle_feedback_source == OTHER_FEED);
    CHECK(pitch_stub.motor_settings.speed_feedback_source == OTHER_FEED);
    CHECK(pitch_stub.motor_controller.other_angle_feedback_ptr == &pitch_feedback);
    CHECK(pitch_stub.motor_controller.other_speed_feedback_ptr == &pitch_speed);
    CHECK(yaw_speed_axis.mechanical_angle == &yaw_stub.measure.total_angle);
    CHECK(yaw_speed_axis.startup_angle == &yaw_stub.startup_angle);
    CHECK(pitch_speed_axis.mechanical_angle == &pitch_stub.measure.total_angle);
    CHECK(pitch_speed_axis.startup_angle == &pitch_stub.startup_angle);
    CHECK(yaw_stub.external_feedback_tick == pitch_stub.external_feedback_tick);
    CHECK(yaw_stub.command_timeout_ms == 30);
}
static void RadianFeedbackAndAbsoluteTarget(void)
{
    Prepare();
    const float yaw = 0.05f;
    snapshot.q[0] = cosf(yaw / 2.0f);
    snapshot.q[3] = sinf(yaw / 2.0f);
    snapshot.gyro[0] = 0.2f; /* Installed gimbal +Y is IMU +X. */
    snapshot.gyro[2] = -0.3f;
    GimbalUpdateFeedback();
    Near(*yaw_stub.motor_controller.other_angle_feedback_ptr, yaw, 1e-6f);
    Near(*pitch_stub.motor_controller.other_angle_feedback_ptr, 0.0f, 1e-6f);
    Near(*yaw_stub.motor_controller.other_speed_feedback_ptr, -0.3f, 1e-6f);
    Near(*pitch_stub.motor_controller.other_speed_feedback_ptr, 0.2f, 1e-6f);
    incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
    incoming_command.yaw = 0.08f;
    incoming_command.pitch = -0.04f;
    GimbalTask();
    Near(yaw_stub.motor_controller.pid_ref, 0.08f, 1e-6f);
    Near(pitch_stub.motor_controller.pid_ref, -0.04f, 1e-6f);
    GimbalTask();
    Near(yaw_stub.motor_controller.pid_ref, 0.08f, 1e-6f);
    CHECK(reset_calls[0] == 1 && reset_calls[1] == 1);
}
static void PitchFeedbackKeepsRadianConvention(void)
{
    Prepare();
    const float pitch = 0.12f;
    snapshot.q[0] = cosf(pitch / 2.0f);
    snapshot.q[1] = sinf(pitch / 2.0f);
    snapshot.gyro[0] = 0.25f;
    GimbalUpdateFeedback();
    Near(*pitch_stub.motor_controller.other_angle_feedback_ptr, pitch, 1e-6f);
    Near(*pitch_stub.motor_controller.other_speed_feedback_ptr, 0.25f, 1e-6f);
    incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
    incoming_command.pitch = 0.15f;
    GimbalTask();
    Near(pitch_stub.motor_controller.pid_ref, 0.15f, 1e-6f);
    CHECK(pitch_stub.stop_flag == MOTOR_ENABLED);
    /* The mechanical envelope is in startup-relative degrees, independent of
     * the IMU's absolute radian feedback and of DM's wire encoding. */
    pitch_stub.measure.total_angle = pitch_stub.startup_angle + 3.0f;
    GimbalState result;
    GimbalGetState(&result);
    Near(result.pitch_mechanical_deg, 3.0f, 1e-6f);
    Near(result.pose.pitch, pitch, 1e-6f);
}
/* Physical installation regression: yaw remains about IMU +Z, while the
 * pitch motor rotates IMU +X. Check mixed poses/rates, not just Euler names.
 * Identity mounting would incorrectly give pitch=0 and pitch_vel~0 here. */
static void InstalledPitchAndYawUseConsistentPoseAndRate(void)
{
    const float pitches[] = {-0.30f, -0.10f, 0.10f, 0.30f};
    for (int turn = -1; turn <= 1; turn += 2) {
        const float yaw = 0.6f*(float)turn;
        const float yaw_rate = -0.3f*(float)turn;
        for (unsigned i = 0; i < sizeof(pitches)/sizeof(pitches[0]); ++i) {
            Prepare();
            float pitch = pitches[i], pitch_rate = 0.25f*(float)turn;
            float cy=cosf(yaw/2), sy=sinf(yaw/2);
            float cp=cosf(pitch/2), sp=sinf(pitch/2);
            /* Raw IMU q = Rz(yaw) Rx(physical_pitch). */
            snapshot.q[0]=cy*cp; snapshot.q[1]=cy*sp;
            snapshot.q[2]=sy*sp; snapshot.q[3]=sy*cp;
            snapshot.gyro[0]=pitch_rate;
            snapshot.gyro[1]=yaw_rate*sinf(pitch);
            snapshot.gyro[2]=yaw_rate*cosf(pitch);
            GimbalUpdateFeedback();
            Near(state.pose.yaw,yaw,1e-6f);
            Near(state.pose.pitch,pitch,1e-6f);
            Near(state.pose.roll,0,1e-6f);
            Near(yaw_speed,yaw_rate,1e-6f);
            Near(pitch_speed,pitch_rate,1e-6f);
            pitch_stub.measure.total_angle=pitch_stub.startup_angle+
                pitch/GIMBAL_RAD_PER_DEG;
            incoming_command.gimbal_mode=GIMBAL_VISION_MODE;
            incoming_command.yaw=yaw;
            incoming_command.pitch=pitch+0.01f;
            GimbalTask();
            CHECK(pitch_stub.stop_flag==MOTOR_ENABLED);
            Near(pitch_stub.motor_controller.pid_ref,pitch+0.01f,1e-6f);
            PIDInstance *angle=&pitch_stub.motor_controller.angle_PID;
            PIDCalculate(angle,pitch_feedback,pitch_stub.motor_controller.pid_ref);
            Near(angle->Err,0.01f,1e-6f);
        }
    }
}
static void ModeHandoverAndBriefStop(void)
{
    Prepare();
    incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
    yaw_speed_axis.pi.state.i = 12.0f;
    pitch_speed_axis.pi.state.i = 24.0f;
    GimbalTask();
    CHECK(reset_calls[0] == 1 && reset_calls[1] == 1);
    CHECK(last_preserve[0] == 0 && last_preserve[1] == 0);
    /* Requests must not modify motor-owned states in the application task. */
    Near(yaw_speed_axis.pi.state.i, 12.0f, 1e-6f);
    Near(pitch_speed_axis.pi.state.i, 24.0f, 1e-6f);
    ApplyReset(&yaw_stub, 0); ApplyReset(&pitch_stub, 0);
    incoming_command.yaw = 0.04f;
    GimbalTask();
    CHECK(reset_calls[0] == 1 && reset_calls[1] == 1);
    yaw_speed_axis.pi.state.i = 10.0f;
    pitch_speed_axis.pi.state.i = 20.0f;
    incoming_command.gimbal_mode = GIMBAL_GYRO_MODE;
    GimbalTask();
    CHECK(reset_calls[0] == 2 && reset_calls[1] == 2);
    CHECK(last_preserve[0] == 0 && last_preserve[1] == 1);
    ApplyReset(&yaw_stub, last_preserve[0]);
    ApplyReset(&pitch_stub, last_preserve[1]);
    Near(yaw_speed_axis.pi.state.i, 0.0f, 1e-6f);
    Near(pitch_speed_axis.pi.state.i, 20.0f, 1e-6f);
    incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
    GimbalTask();
    CHECK(reset_calls[0] == 3 && reset_calls[1] == 3);
    CHECK(last_preserve[0] == 0 && last_preserve[1] == 1);
    yaw_speed_axis.pi.state.i = 50.0f;
    pitch_speed_axis.pi.state.i = 60.0f;
    incoming_command.gimbal_mode = GIMBAL_ZERO_FORCE;
    GimbalTask();
    CHECK(yaw_stub.stop_flag == MOTOR_STOP && pitch_stub.stop_flag == MOTOR_STOP);
    /* No motor cycle runs between disable and re-enable. */
    incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
    GimbalTask();
    CHECK(reset_calls[0] == 4 && reset_calls[1] == 4);
    CHECK(last_preserve[0] == 0 && last_preserve[1] == 0);
    Near(pitch_speed_axis.pi.state.i, 60.0f, 1e-6f);
}
static void DwtAndDiagnosticMirror(void)
{
    Prepare();
    PIDInstance *pid = &yaw_stub.motor_controller.speed_PID;
    next_dt_s = 0.007f;
    Near(Step(&yaw_stub, 0.1f, 0.4f), 60.882f, 1e-4f);
    CHECK(last_dwt_counter == &pid->DWT_CNT);
    Near(pid->dt, 0.007f, 1e-7f);
    Near(pid->Measure, 0.1f, 1e-6f); Near(pid->Ref, 0.4f, 1e-6f);
    Near(pid->Err, 0.3f, 1e-6f); Near(pid->Pout, 60.0f, 1e-4f);
    Near(pid->Iout, 0.882f, 1e-4f); Near(pid->ITerm, 0.882f, 1e-4f);
    Near(pid->Output, 60.882f, 1e-4f);
    Near(pid->Last_Measure, pid->Measure, 1e-7f);
    Near(pid->Last_Err, pid->Err, 1e-7f);
    Near(pid->Last_Output, pid->Output, 1e-7f);
    Near(pid->Dout, 0.0f, 1e-7f);
    pid->Kp = 60.0f; pid->Ki = 100.0f; pid->IntegralLimit = 200.0f; pid->MaxOut = 1000.0f;
    Near(Step(&yaw_stub, 0.1f, 0.4f), 19.092f, 1e-4f);
    Near(yaw_speed_axis.pi.config.kp, 60.0f, 1e-6f);
    Near(yaw_speed_axis.pi.config.i_limit, 200.0f, 1e-6f);
    Near(pitch_speed_axis.pi.config.kp, 900.0f, 1e-6f);
    Near(pitch_speed_axis.pi.state.i, 0.0f, 1e-6f);
    /* Restore the real binding's baseline tunables for following groups. */
    pid->Kp = 200.0f; pid->Ki = 420.0f; pid->IntegralLimit = 6000.0f; pid->MaxOut = 20000.0f;
}
static void InvalidDataRemainsNan(void)
{
    Prepare();
    PIDInstance *pid = &yaw_stub.motor_controller.speed_PID;
    Near(Step(&yaw_stub, 0.0f, 0.5f), 100.21f, 1e-4f);
    GimbalSpeedPiState before = yaw_speed_axis.pi.state;
    next_dt_s = 0.031f;
    CHECK(isnan(Step(&yaw_stub, 0.0f, 0.5f)));
    CHECK(isnan(pid->Output) && isnan(pid->Pout));
    Near(pid->dt, 0.031f, 1e-7f);
    CHECK(memcmp(&before, &yaw_speed_axis.pi.state, sizeof(before)) == 0);
    next_dt_s = 0.001f;
    CHECK(isnan(Step(&yaw_stub, NAN, 0.5f)));
    CHECK(isnan(pid->Output));
    pid->Kd = 1.0f;
    CHECK(isnan(Step(&yaw_stub, 0.0f, 0.5f)));
    pid->Kd = 0.0f; pid->DeadBand = 0.1f;
    CHECK(isnan(Step(&yaw_stub, 0.0f, 0.5f)));
    pid->DeadBand = 0.0f;
    yaw_stub.measure.total_angle = NAN;
    CHECK(isnan(Step(&yaw_stub, 0.0f, 0.5f)));
    CHECK(isnan(pid->Output));
}
static void MechanicalEffortGate(void)
{
    Prepare();
    /* Pitch TOTAL-output gating now lives in the dedicated driver, after all
     * compensation. This test retains yaw's original application-side gate. */
    DJIMotorInstance *motor = &yaw_stub;
    PIDInstance *pid = &motor->motor_controller.speed_PID;
    GimbalSpeedAxis *axis = &yaw_speed_axis;
    const float test_load = 100.0f;
    for (int sign = -1; sign <= 1; sign += 2) {
        axis->pose_to_encoder_sign = (float)sign;
        for (int edge = -1; edge <= 1; edge += 2) {
            ApplyReset(motor, 0);
            motor->measure.total_angle = motor->startup_angle +
                (float)(edge > 0 ? GIMBAL_YAW_MAX_DEG : GIMBAL_YAW_MIN_DEG);
            axis->pi.state.i = test_load * (float)(sign * edge);
            uint32_t before = axis->envelope_block_count;
            Near(Step(motor, 0.0f, 0.0f), 0.0f, 1e-6f);
            Near(axis->pi.state.i, 0.0f, 1e-6f);
            Near(pid->Iout, 0.0f, 1e-6f);
            CHECK(axis->envelope_block_count == before + 1);
            axis->pi.state.i = -test_load * (float)(sign * edge);
            Near(Step(motor, 0.0f, 0.0f), -test_load * (float)(sign * edge), 1e-6f);
            CHECK(axis->envelope_block_count == before + 1);
        }
    }
    ApplyReset(motor, 0);
    axis->pose_to_encoder_sign = 1.0f;
    motor->measure.total_angle = motor->startup_angle + GIMBAL_YAW_MAX_DEG - 1.0f;
    axis->pi.state.i = test_load;
    Near(Step(motor, 0.0f, 0.0f), test_load, 1e-6f);
    motor->measure.total_angle = motor->startup_angle + GIMBAL_YAW_MAX_DEG;
    CHECK(isnan(Step(motor, NAN, 0.0f)));
    CHECK(isnan(pid->Output)); /* The limit must not mask a fault as valid zero. */
}
static void HealthStillDisables(void)
{
    Prepare();
    incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
    GimbalTask();
    yaw_stub.fault_latched = 1;
    GimbalTask();
    CHECK(yaw_stub.stop_flag == MOTOR_STOP && pitch_stub.stop_flag == MOTOR_STOP);
    yaw_stub.fault_latched = 0;
    pitch_stub.last_feedback_tick = clock_ms - 51;
    GimbalTask();
    CHECK(yaw_stub.stop_flag == MOTOR_STOP && pitch_stub.stop_flag == MOTOR_STOP);
    pitch_stub.last_feedback_tick = clock_ms;
    snapshot.tick = clock_ms - 21;
    GimbalUpdateFeedback();
    GimbalTask();
    CHECK(yaw_stub.stop_flag == MOTOR_STOP && pitch_stub.stop_flag == MOTOR_STOP);
    Prepare();
    incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
    pitch_stub.measure.state = 3; /* Fresh CAN data with a DM driver error. */
    GimbalTask();
    CHECK(yaw_stub.stop_flag == MOTOR_STOP && pitch_stub.stop_flag == MOTOR_STOP);
    pitch_stub.measure.state = 1;
    pitch_stub.fault_latched = 1;
    GimbalTask();
    CHECK(yaw_stub.stop_flag == MOTOR_STOP && pitch_stub.stop_flag == MOTOR_STOP);
    CHECK(publications > 0 && primask == 0);
}
static void FailedPitchInitKeepsYawStopped(void)
{
    /* Run last: model a fresh boot whose dedicated DM registration fails. */
    init_count = 0;
    fail_pitch_init = 1;
    GimbalInit();
    CHECK(yaw_motor == &yaw_stub && pitch_motor == NULL);
    CHECK(yaw_stub.fault_latched == 1 && yaw_stub.stop_flag == MOTOR_STOP);
    GimbalState result;
    GimbalGetState(&result);
    CHECK(result.driver_fault && !result.motors_ready);
    unsigned before = publications;
    GimbalTask();
    CHECK(publications == before);
    CHECK(!GimbalClearMotorFaults());
    CHECK(primask == 0);
}
static void TraceKeepsTrackingTail(void)
{
    Prepare();
    trace_last_tick = 0; trace_has_tracked = 0; trace_last_gates = 0;
    gimbal_pitch_trace_total = gimbal_pitch_trace_count = gimbal_pitch_trace_next = 0;
    incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
    incoming_command.pitch = 0.02f;
    pitch_stub.requested_torque_nm = 0.4f;
    pitch_stub.last_torque_nm = 0.3f;
    GimbalRecordPitch(&incoming_command, 1);
    CHECK(gimbal_pitch_trace_count == 1);
    Near(gimbal_pitch_trace[0].source_target_rad, 0.02f, 1e-7f);
    Near(gimbal_pitch_trace[0].applied_nm, 0.3f, 1e-7f);
    incoming_command.gimbal_mode = GIMBAL_GYRO_MODE;
    clock_ms += GIMBAL_PITCH_TRACE_TAIL_MS + 1u;
    GimbalRecordPitch(&incoming_command, 1);
    CHECK(gimbal_pitch_trace_count == 1); /* Idle must not erase the event. */
    incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
    GimbalRecordPitch(&incoming_command, 1);
    CHECK(gimbal_pitch_trace_count == 2);
    for (unsigned i=0; i<GIMBAL_PITCH_TRACE_CAPACITY+3u; ++i) {
        clock_ms += GIMBAL_PITCH_TRACE_PERIOD_MS;
        GimbalRecordPitch(&incoming_command, 1);
    }
    CHECK(gimbal_pitch_trace_count == GIMBAL_PITCH_TRACE_CAPACITY);
    CHECK(gimbal_pitch_trace_next < GIMBAL_PITCH_TRACE_CAPACITY);
}
/* Exercise the application's target mapping, real angle PID, pitch speed
 * governor and PI together. Fixed feedback is deliberate: this checks that
 * small commands keep building output, not that a real load will move. */
static void PitchSmallTargetsKeepBuildingOutput(void)
{
    /* 本组验证PI路由与积分单调建立。实机重力/摩擦前馈已非0，常量力矩会在
     * 小误差下把"输出方向与目标一致"的断言推翻，因此临时置零，末尾恢复。 */
    GimbalPitchAssistConfig original=pitch_assist.config;
    pitch_assist.config.gravity_cos_nm=0; pitch_assist.config.gravity_sin_nm=0;
    pitch_assist.config.friction_positive_nm=0; pitch_assist.config.friction_negative_nm=0;
    const float targets_deg[] = {-1.0f, -0.1f, 0.1f, 1.0f};
    for (unsigned j = 0; j < sizeof(targets_deg)/sizeof(targets_deg[0]); ++j) {
        Prepare();
        incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
        incoming_command.pitch = targets_deg[j]*GIMBAL_RAD_PER_DEG;
        GimbalTask();
        CHECK(pitch_stub.stop_flag == MOTOR_ENABLED);
        ApplyReset(&pitch_stub, 0);
        PIDInstance *angle = &pitch_stub.motor_controller.angle_PID;
        PIDInstance *speed = &pitch_stub.motor_controller.speed_PID;
        float direction = targets_deg[j] > 0 ? 1.0f : -1.0f;
        uint32_t blocks_before = pitch_speed_axis.envelope_block_count;
        float previous_i = 0;
        for (unsigned step = 1; step <= 1000; ++step) {
            ++clock_ms;
            if (step % 5 == 0) {
                snapshot.tick = clock_ms;
                yaw_stub.last_feedback_tick = pitch_stub.last_feedback_tick = clock_ms;
                GimbalUpdateFeedback();
                GimbalTask();
            }
            float desired = PIDCalculate(angle, pitch_feedback,
                                         pitch_stub.motor_controller.pid_ref);
            float output = Step(&pitch_stub, pitch_speed, desired);
            CHECK(isfinite(output) && output*direction > 0);
            CHECK(speed->Ref*direction > 0);
            CHECK(fabsf(speed->Ref) <= GIMBAL_PITCH_SPEED_LIMIT_RAD_S+1e-6f);
            CHECK(pitch_speed_axis.pi.state.i*direction >= previous_i);
            previous_i = pitch_speed_axis.pi.state.i*direction;
        }
        CHECK(previous_i > 0);
        CHECK(reset_calls[1] == 1); /* Ordinary command refreshes retain I. */
        CHECK(pitch_speed_axis.envelope_block_count == blocks_before);
        CHECK(primask == 0);
    }
    CHECK(GimbalPitchAssistInit(&pitch_assist,&original));
}
static void PitchAssistRoutingAndLimits(void)
{
    Prepare();
    GimbalPitchAssistConfig original=pitch_assist.config;
    /* 本组只验证重力路由。实机摩擦辅助系数已非0，静止且误差大于门限时会叠加，
     * 因此先置0；分组末尾用original恢复真实配置。 */
    pitch_assist.config.friction_positive_nm=0;
    pitch_assist.config.friction_negative_nm=0;
    ApplyReset(&pitch_stub,0);
    PIDInstance *angle=&pitch_stub.motor_controller.angle_PID;
    PIDInstance *speed=&pitch_stub.motor_controller.speed_PID;
    angle->Err=0.02f; angle->Measure=0;
    Step(&pitch_stub,0,0.5f);
    Near(speed->Ref,GIMBAL_PITCH_ACCEL_RAD_S2*next_dt_s,1e-6f);
    CHECK(speed->Ref < 0.5f);
    pitch_assist.config.gravity_cos_nm=0.5f;
    for (int i=0;i<150;++i) Step(&pitch_stub,0,0.5f);
    Near(pitch_assist.gravity_nm,0.5f,1e-5f);
    float combined=Step(&pitch_stub,0,0.5f);
    /* 总装里除重力项外还叠加了按指令速度的速度前馈，本组只验证重力路由，
     * 因此把前馈按实际Ref一并计入期望。 */
    float ff_nm=GIMBAL_PITCH_SPEED_FF_NM*speed->Ref/GIMBAL_PITCH_SPEED_LIMIT_RAD_S;
    Near(combined-speed->Output,(0.5f+ff_nm)/GIMBAL_PITCH_DM_NM_PER_COMMAND,0.001f);
    CHECK(pitch_stub.applied_callback == GimbalPitchApplied);
    pitch_stub.applied_callback(pitch_stub.applied_context,speed,combined,0,DM_PITCH_GATE_ENVELOPE);
    Near(pitch_speed_axis.pi.state.i,0,1e-6f);
    ApplyReset(&pitch_stub,1); Near(pitch_assist.gravity_nm,0.5f,1e-5f);
    Near(pitch_assist.speed_ref_rad_s,0,1e-6f);
    ApplyReset(&pitch_stub,0); Near(pitch_assist.gravity_nm,0,1e-6f);
    CHECK(GimbalPitchAssistInit(&pitch_assist,&original));
}
int main(void)
{
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.q[0] = 1.0f; snapshot.tick = clock_ms; snapshot.valid = 1;
    GimbalInit();
    void (*tests[])(void) = {
        BindingsAndOriginalGains, RadianFeedbackAndAbsoluteTarget,
        PitchFeedbackKeepsRadianConvention, InstalledPitchAndYawUseConsistentPoseAndRate,
        ModeHandoverAndBriefStop, DwtAndDiagnosticMirror,
        InvalidDataRemainsNan, MechanicalEffortGate, HealthStillDisables,
        PitchAssistRoutingAndLimits, PitchSmallTargetsKeepBuildingOutput,
        TraceKeepsTrackingTail,
        FailedPitchInitKeepsYawStopped
    };
    unsigned count = (unsigned)(sizeof(tests) / sizeof(tests[0]));
    for (unsigned i = 0; i < count; ++i) tests[i]();
    printf("gimbal adapter: %u integration test groups passed\n", count);
    return EXIT_SUCCESS;
}
