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
static DJIMotorInstance motors[2];
static Motor_Init_Config_s configs[2];
static unsigned init_count, stop_calls[2], enable_calls[2], reset_calls[2];
static uint8_t last_preserve[2];
static unsigned publications;

static unsigned MotorIndex(DJIMotorInstance *m)
{
    CHECK(m == &motors[0] || m == &motors[1]);
    return m == &motors[0] ? 0u : 1u;
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
    CHECK(init_count < 2);
    unsigned i = init_count++;
    configs[i] = *c;
    DJIMotorInstance *m = &motors[i];
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
static void ApplyReset(DJIMotorInstance *m, uint8_t preserve)
{
    PIDClear(&m->motor_controller.speed_PID);
    m->speed_controller.reset(m->speed_controller.context,
                             &m->motor_controller.speed_PID, preserve);
}
static float Step(DJIMotorInstance *m, float measure, float ref)
{
    return m->speed_controller.update(m->speed_controller.context,
                                     &m->motor_controller.speed_PID, measure, ref);
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
    for (unsigned i = 0; i < 2; ++i) {
        motors[i].feedback_seen = 1;
        motors[i].last_feedback_tick = clock_ms;
        motors[i].measure.total_angle = motors[i].startup_angle = 100.0f;
        motors[i].fault_latched = 0;
    }
    GimbalUpdateFeedback();
    memset(&incoming_command, 0, sizeof(incoming_command));
    incoming_command.gimbal_mode = GIMBAL_ZERO_FORCE;
    GimbalTask(); /* Establish the application's disabled state. */
    for (unsigned i = 0; i < 2; ++i) {
        ApplyReset(&motors[i], 0);
        reset_calls[i] = 0;
        stop_calls[i] = enable_calls[i] = 0;
    }
    yaw_speed_axis.pose_to_encoder_sign = pitch_speed_axis.pose_to_encoder_sign = 1.0f;
}
static void BindingsAndOriginalGains(void)
{
    CHECK(init_count == 2);
    CHECK(configs[0].can_init_config.can_handle == &hcan2);
    CHECK(configs[1].can_init_config.can_handle == &hcan2);
    CHECK(configs[0].can_init_config.tx_id == 1 && configs[1].can_init_config.tx_id == 2);
    CHECK(yaw_motor == &motors[0] && pitch_motor == &motors[1]);
    CHECK(motors[0].motor_type == GM6020 && motors[1].motor_type == GM6020);
    CHECK(motors[0].speed_controller.context == &yaw_speed_axis);
    CHECK(motors[1].speed_controller.context == &pitch_speed_axis);
    CHECK(yaw_speed_axis.pi.config.reset_i_on_ref_reversal == 1);
    CHECK(pitch_speed_axis.pi.config.reset_i_on_ref_reversal == 0);
    Near(yaw_speed_axis.pi.config.reversal_ref_min_rad_s, 8.0f * 0.1f * GIMBAL_RAD_PER_DEG, 1e-7f);
    Near(yaw_speed_axis.pi.config.max_dt_s, 0.03f, 1e-7f);
    /* gimbal.c 的临时实验覆盖把两轴外环 Kp 都设为 200，此处随之更新。
     * 覆盖删除后应恢复为 8.0f / 10.0f，以继续验证两轴参数隔离。 */
    Near(motors[0].motor_controller.angle_PID.Kp, 200.0f, 1e-7f);
    Near(motors[1].motor_controller.angle_PID.Kp, 200.0f, 1e-7f);
    Near(yaw_speed_axis.pi.config.kp, 50.0f, 1e-7f);
    Near(pitch_speed_axis.pi.config.kp, 50.0f, 1e-7f);
    Near(yaw_speed_axis.pi.config.ki, 420.0f, 1e-7f);
    Near(pitch_speed_axis.pi.config.ki, 350.0f, 1e-7f);
    Near(yaw_speed_axis.pi.config.i_limit, 6000.0f, 1e-7f);
    Near(pitch_speed_axis.pi.config.i_limit, 5000.0f, 1e-7f);
    CHECK(motors[0].motor_settings.angle_feedback_source == OTHER_FEED);
    CHECK(motors[1].motor_settings.speed_feedback_source == OTHER_FEED);
    CHECK(motors[0].external_feedback_tick == motors[1].external_feedback_tick);
    CHECK(motors[0].command_timeout_ms == 30);
}
static void RadianFeedbackAndAbsoluteTarget(void)
{
    Prepare();
    const float yaw = 0.05f;
    snapshot.q[0] = cosf(yaw / 2.0f);
    snapshot.q[3] = sinf(yaw / 2.0f);
    snapshot.gyro[1] = 0.2f;
    snapshot.gyro[2] = -0.3f;
    GimbalUpdateFeedback();
    Near(*motors[0].motor_controller.other_angle_feedback_ptr, yaw, 1e-6f);
    Near(*motors[1].motor_controller.other_angle_feedback_ptr, 0.0f, 1e-6f);
    Near(*motors[0].motor_controller.other_speed_feedback_ptr, -0.3f, 1e-6f);
    Near(*motors[1].motor_controller.other_speed_feedback_ptr, 0.2f, 1e-6f);
    incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
    incoming_command.yaw = 0.08f;
    incoming_command.pitch = -0.04f;
    GimbalTask();
    Near(motors[0].motor_controller.pid_ref, 0.08f, 1e-6f);
    Near(motors[1].motor_controller.pid_ref, -0.04f, 1e-6f);
    GimbalTask();
    Near(motors[0].motor_controller.pid_ref, 0.08f, 1e-6f);
    CHECK(reset_calls[0] == 1 && reset_calls[1] == 1);
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
    ApplyReset(&motors[0], 0); ApplyReset(&motors[1], 0);
    incoming_command.yaw = 0.04f;
    GimbalTask();
    CHECK(reset_calls[0] == 1 && reset_calls[1] == 1);
    yaw_speed_axis.pi.state.i = 10.0f;
    pitch_speed_axis.pi.state.i = 20.0f;
    incoming_command.gimbal_mode = GIMBAL_GYRO_MODE;
    GimbalTask();
    CHECK(reset_calls[0] == 2 && reset_calls[1] == 2);
    CHECK(last_preserve[0] == 0 && last_preserve[1] == 1);
    ApplyReset(&motors[0], last_preserve[0]);
    ApplyReset(&motors[1], last_preserve[1]);
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
    CHECK(motors[0].stop_flag == MOTOR_STOP && motors[1].stop_flag == MOTOR_STOP);
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
    PIDInstance *pid = &motors[0].motor_controller.speed_PID;
    next_dt_s = 0.007f;
    Near(Step(&motors[0], 0.1f, 0.4f), 15.882f, 1e-4f);
    CHECK(last_dwt_counter == &pid->DWT_CNT);
    Near(pid->dt, 0.007f, 1e-7f);
    Near(pid->Measure, 0.1f, 1e-6f); Near(pid->Ref, 0.4f, 1e-6f);
    Near(pid->Err, 0.3f, 1e-6f); Near(pid->Pout, 15.0f, 1e-4f);
    Near(pid->Iout, 0.882f, 1e-4f); Near(pid->ITerm, 0.882f, 1e-4f);
    Near(pid->Output, 15.882f, 1e-4f);
    Near(pid->Last_Measure, pid->Measure, 1e-7f);
    Near(pid->Last_Err, pid->Err, 1e-7f);
    Near(pid->Last_Output, pid->Output, 1e-7f);
    Near(pid->Dout, 0.0f, 1e-7f);
    pid->Kp = 60.0f; pid->Ki = 100.0f; pid->IntegralLimit = 200.0f; pid->MaxOut = 1000.0f;
    Near(Step(&motors[0], 0.1f, 0.4f), 19.092f, 1e-4f);
    Near(yaw_speed_axis.pi.config.kp, 60.0f, 1e-6f);
    Near(yaw_speed_axis.pi.config.i_limit, 200.0f, 1e-6f);
    Near(pitch_speed_axis.pi.config.kp, 50.0f, 1e-6f);
    Near(pitch_speed_axis.pi.state.i, 0.0f, 1e-6f);
    /* Restore the real binding's baseline tunables for following groups. */
    pid->Kp = 50.0f; pid->Ki = 420.0f; pid->IntegralLimit = 6000.0f; pid->MaxOut = 20000.0f;
}
static void InvalidDataRemainsNan(void)
{
    Prepare();
    PIDInstance *pid = &motors[0].motor_controller.speed_PID;
    Near(Step(&motors[0], 0.0f, 0.5f), 25.21f, 1e-4f);
    GimbalSpeedPiState before = yaw_speed_axis.pi.state;
    next_dt_s = 0.031f;
    CHECK(isnan(Step(&motors[0], 0.0f, 0.5f)));
    CHECK(isnan(pid->Output) && isnan(pid->Pout));
    Near(pid->dt, 0.031f, 1e-7f);
    CHECK(memcmp(&before, &yaw_speed_axis.pi.state, sizeof(before)) == 0);
    next_dt_s = 0.001f;
    CHECK(isnan(Step(&motors[0], NAN, 0.5f)));
    CHECK(isnan(pid->Output));
    pid->Kd = 1.0f;
    CHECK(isnan(Step(&motors[0], 0.0f, 0.5f)));
    pid->Kd = 0.0f; pid->DeadBand = 0.1f;
    CHECK(isnan(Step(&motors[0], 0.0f, 0.5f)));
    pid->DeadBand = 0.0f;
    motors[0].measure.total_angle = NAN;
    CHECK(isnan(Step(&motors[0], 0.0f, 0.5f)));
    CHECK(isnan(pid->Output));
}
static void MechanicalEffortGate(void)
{
    Prepare();
    DJIMotorInstance *motor = &motors[1];
    PIDInstance *pid = &motor->motor_controller.speed_PID;
    GimbalSpeedAxis *axis = &pitch_speed_axis;
    const float test_load = 100.0f;
    for (int sign = -1; sign <= 1; sign += 2) {
        axis->pose_to_encoder_sign = (float)sign;
        for (int edge = -1; edge <= 1; edge += 2) {
            ApplyReset(motor, 0);
            motor->measure.total_angle = motor->startup_angle +
                (float)(edge > 0 ? GIMBAL_PITCH_MAX_DEG : GIMBAL_PITCH_MIN_DEG);
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
    motor->measure.total_angle = motor->startup_angle + GIMBAL_PITCH_MAX_DEG - 1.0f;
    axis->pi.state.i = test_load;
    Near(Step(motor, 0.0f, 0.0f), test_load, 1e-6f);
    motor->measure.total_angle = motor->startup_angle + GIMBAL_PITCH_MAX_DEG;
    CHECK(isnan(Step(motor, NAN, 0.0f)));
    CHECK(isnan(pid->Output)); /* The limit must not mask a fault as valid zero. */
}
static void HealthStillDisables(void)
{
    Prepare();
    incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
    GimbalTask();
    motors[0].fault_latched = 1;
    GimbalTask();
    CHECK(motors[0].stop_flag == MOTOR_STOP && motors[1].stop_flag == MOTOR_STOP);
    motors[0].fault_latched = 0;
    motors[1].last_feedback_tick = clock_ms - 51;
    GimbalTask();
    CHECK(motors[0].stop_flag == MOTOR_STOP && motors[1].stop_flag == MOTOR_STOP);
    motors[1].last_feedback_tick = clock_ms;
    snapshot.tick = clock_ms - 21;
    GimbalUpdateFeedback();
    GimbalTask();
    CHECK(motors[0].stop_flag == MOTOR_STOP && motors[1].stop_flag == MOTOR_STOP);
    CHECK(publications > 0 && primask == 0);
}
int main(void)
{
    memset(&snapshot, 0, sizeof(snapshot));
    snapshot.q[0] = 1.0f; snapshot.tick = clock_ms; snapshot.valid = 1;
    GimbalInit();
    if (!GIMBAL_INSTALLATION_CONFIRMED) {
        Prepare();
        incoming_command.gimbal_mode = GIMBAL_VISION_MODE;
        incoming_command.yaw = 0.08f; incoming_command.pitch = -0.04f;
        GimbalTask();
        CHECK(motors[0].stop_flag == MOTOR_STOP && motors[1].stop_flag == MOTOR_STOP);
        CHECK(enable_calls[0] == 0 && enable_calls[1] == 0);
        puts("gimbal adapter: commissioning lock keeps both motors stopped");
        return EXIT_SUCCESS;
    }
    void (*tests[])(void) = {
        BindingsAndOriginalGains, RadianFeedbackAndAbsoluteTarget,
        ModeHandoverAndBriefStop, DwtAndDiagnosticMirror,
        InvalidDataRemainsNan, MechanicalEffortGate, HealthStillDisables
    };
    unsigned count = (unsigned)(sizeof(tests) / sizeof(tests[0]));
    for (unsigned i = 0; i < count; ++i) tests[i]();
    printf("gimbal adapter: %u integration test groups passed\n", count);
    return EXIT_SUCCESS;
}
