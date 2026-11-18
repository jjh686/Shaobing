/* Compile this translation unit with the real dji_motor.c included, and link
 * the real controller.c. Only MCU/CAN/clock/daemon services are stubbed.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dji_motor.c"

CAN_HandleTypeDef hcan1, hcan2;
static uint32_t now = 100, primask, dwt;
static unsigned transmit_count;
static void (*on_unlock)(void);
uint32_t HAL_GetTick(void) { return now; }
uint32_t __get_PRIMASK(void) { return primask; }
void __disable_irq(void) { primask = 1; }
void __set_PRIMASK(uint32_t value)
{
    primask = value;
    if (!value && on_unlock) {
        void (*callback)(void) = on_unlock;
        on_unlock = NULL;
        callback();
    }
}
float DWT_GetDeltaT(uint32_t *count) { *count = ++dwt; return 0.001f; }
void tracking_test_log(const char *format, ...) { (void)format; }
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    CANInstance *instance = calloc(1, sizeof(*instance));
    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    return instance;
}
void CANTransmit(CANInstance *instance, uint8_t length)
{
    (void)instance;
    assert(length == 8);
    ++transmit_count;
}
DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    (void)config;
    return calloc(1, sizeof(DaemonInstance));
}
void DaemonReload(DaemonInstance *instance) { (void)instance; }

typedef struct {
    unsigned updates, resets;
    uint8_t last_preserve;
    float load_i, output, last_ref;
    uint8_t request_during_update;
    DJIMotorInstance *motor;
} Hook;
static float update(void *context, PIDInstance *pid, float measure, float ref)
{
    Hook *hook = context;
    ++hook->updates;
    hook->last_ref = ref;
    pid->Measure = measure;
    pid->Ref = ref;
    pid->Output = hook->output;
    if (hook->request_during_update) {
        hook->request_during_update = 0;
        DJIMotorRequestSpeedReset(hook->motor, 1);
    }
    return hook->output;
}
static void reset(void *context, PIDInstance *pid, uint8_t preserve)
{
    Hook *hook = context;
    ++hook->resets;
    hook->last_preserve = preserve;
    assert(pid->Output == 0 && pid->Iout == 0 && pid->Last_Err == 0);
    if (!preserve) hook->load_i = 0;
    pid->Iout = hook->load_i;
}
static void setup(DJIMotorInstance *motor)
{
    memset(motor, 0, sizeof(*motor));
    motor->feedback_seen = 1;
    motor->last_feedback_tick = now;
    motor->last_command_tick = now;
    motor->command_timeout_ms = 30;
    motor->stop_flag = MOTOR_ENABLED;
    motor->motor_settings.close_loop_type = SPEED_LOOP;
    motor->motor_settings.outer_loop_type = SPEED_LOOP;
    PID_Init_Config_s config = {.Kp = 2, .MaxOut = 20000, .IntegralLimit = 100};
    PIDInit(&motor->motor_controller.speed_PID, &config);
    motor->motor_controller.pid_ref = 3;
    idx = 1;
    dji_motor_instance[0] = motor;
    memset(sender_enable_flag, 0, sizeof(sender_enable_flag));
    sender_enable_flag[0] = 1;
    memset(sender_assignment[0].tx_buff, 0, 8);
}
static int16_t sent(void)
{
    return (int16_t)((uint16_t)sender_assignment[0].tx_buff[0] << 8 |
                     sender_assignment[0].tx_buff[1]);
}
static void test_fallback(void)
{
    DJIMotorInstance motor;
    setup(&motor);
    DJIMotorControl();
    assert(sent() == 6);
    motor.motor_controller.speed_PID.Iout = 123;
    DJIMotorRequestSpeedReset(&motor, 1);
    DJIMotorControl();
    assert(motor.motor_controller.speed_PID.Iout == 100);
    assert(sent() == 106);
    uint32_t stamp = motor.motor_controller.speed_PID.DWT_CNT;
    DJIMotorRequestSpeedReset(&motor, 0);
    DJIMotorControl();
    assert(sent() == 6 && motor.motor_controller.speed_PID.Iout == 0);
    assert(motor.motor_controller.speed_PID.DWT_CNT > stamp);
    DJIMotorBindSpeedController(&motor, NULL, reset, (void *)1);
    assert(!motor.speed_controller.update && !motor.speed_controller.reset);
}
static void test_once_and_stop(void)
{
    DJIMotorInstance motor;
    setup(&motor);
    Hook hook = {.load_i = 20, .output = 42, .motor = &motor};
    DJIMotorBindSpeedController(&motor, update, reset, &hook);
    DJIMotorRequestSpeedReset(&motor, 1);
    uint32_t request = motor.speed_reset_request;
    DJIMotorControl();
    DJIMotorControl();
    assert(hook.updates == 2 && hook.resets == 1 && hook.last_preserve == 1);
    assert(motor.speed_reset_consumed == request && sent() == 42);
    assert(motor.motor_controller.speed_PID.Iout == 20);
    hook.request_during_update = 1;
    DJIMotorControl();
    assert(hook.resets == 1 && motor.speed_reset_consumed != motor.speed_reset_request);
    DJIMotorControl();
    assert(hook.resets == 2 && motor.speed_reset_consumed == motor.speed_reset_request);
    DJIMotorRequestSpeedReset(&motor, 1);
    DJIMotorStop(&motor);
    DJIMotorControl();
    assert(hook.last_preserve == 0 && hook.load_i == 0 && sent() == 0);
    uint32_t stamp = motor.motor_controller.speed_PID.DWT_CNT;
    DJIMotorControl();
    assert(motor.motor_controller.speed_PID.DWT_CNT > stamp);
    assert(hook.updates == 4 && hook.resets == 4);
}
static DJIMotorInstance *injected_motor;
static void commit_new_target(void)
{
    DJIMotorSetRef(injected_motor, 7);
    DJIMotorRequestSpeedReset(injected_motor, 1);
}
static void test_target_request_snapshot(void)
{
    DJIMotorInstance motor;
    setup(&motor);
    Hook hook = {.output = 1};
    DJIMotorBindSpeedController(&motor, update, reset, &hook);
    injected_motor = &motor;
    on_unlock = commit_new_target;
    DJIMotorControl();
    assert(hook.last_ref == 3 && hook.resets == 0);
    assert(motor.speed_reset_consumed != motor.speed_reset_request);
    DJIMotorControl();
    assert(hook.last_ref == 7 && hook.resets == 1);
    primask = 1;
    DJIMotorRequestSpeedReset(&motor, 0);
    assert(primask == 1); /* Nested request preserves the caller's IRQ state. */
    primask = 0;
    motor.speed_reset_request = 0xffffffffu;
    DJIMotorRequestSpeedReset(&motor, 1);
    assert(motor.speed_reset_request == 1);
}
static void commit_enable(void)
{
    DJIMotorSetRef(injected_motor, 11);
    DJIMotorRequestSpeedReset(injected_motor, 0);
    DJIMotorEnable(injected_motor);
}
static void test_stopped_snapshot_enable_race(void)
{
    DJIMotorInstance motor;
    setup(&motor);
    motor.stop_flag = MOTOR_STOP;
    motor.last_command_tick = now - 100;
    motor.motor_controller.speed_PID.Iout = 37;
    Hook hook = {.load_i = 37, .output = 42};
    DJIMotorBindSpeedController(&motor, update, reset, &hook);
    injected_motor = &motor;
    on_unlock = commit_enable;
    DJIMotorControl();
    assert(hook.updates == 0 && hook.resets == 1 && sent() == 0);
    assert(hook.load_i == 0 && motor.motor_controller.speed_PID.Iout == 0);
    assert(motor.stop_flag == MOTOR_ENABLED && !motor.fault_latched);
    assert(motor.speed_reset_consumed != motor.speed_reset_request);
    DJIMotorControl();
    assert(hook.updates == 1 && hook.resets == 2 && hook.last_ref == 11);
    assert(motor.speed_reset_consumed == motor.speed_reset_request && sent() == 42);
}
static void test_snapshot_command_validation(void)
{
    for (unsigned which = 0; which < 2; ++which) {
        DJIMotorInstance motor;
        setup(&motor);
        Hook hook = {.output = 42};
        DJIMotorBindSpeedController(&motor, update, reset, &hook);
        if (which == 0) motor.last_command_tick = now - 31;
        else motor.motor_controller.pid_ref = NAN;
        injected_motor = &motor;
        on_unlock = commit_new_target;
        DJIMotorControl();
        /* A newly published target must not mask the invalid command that
         * was captured for this iteration, nor run one unsafe speed update.
         */
        assert(hook.updates == 0 && motor.fault_latched && sent() == 0);
        assert(motor.stop_flag == MOTOR_STOP);
    }
}
static void test_fault_paths(void)
{
    for (unsigned which = 0; which < 4; ++which) {
        DJIMotorInstance motor;
        setup(&motor);
        Hook hook = {.load_i = 9, .output = which ? INFINITY : NAN};
        DJIMotorBindSpeedController(&motor, update, reset, &hook);
        motor.motor_settings.close_loop_type |= CURRENT_LOOP;
        motor.motor_controller.current_PID.MaxOut = 1;
        if (which == 2) motor.last_feedback_tick = now - 51;
        if (which == 3) motor.last_command_tick = now - 31;
        DJIMotorControl();
        assert(motor.fault_latched && motor.stop_flag == MOTOR_STOP && sent() == 0);
        assert(hook.last_preserve == 0 && hook.load_i == 0 && hook.resets == 1);
        assert(hook.updates == (which < 2 ? 1u : 0u));
    }
}
static void test_pending_reset_policy(void)
{
    DJIMotorInstance motor;
    setup(&motor);
    Hook hook = {.load_i = 25, .output = 1};
    DJIMotorBindSpeedController(&motor, update, reset, &hook);

    DJIMotorRequestSpeedReset(&motor, 0);
    assert(motor.speed_reset_request == 2);
    DJIMotorRequestSpeedReset(&motor, 1);
    assert(motor.speed_reset_request == 4); /* Pending full dominates. */
    DJIMotorControl();
    assert(hook.resets == 1 && hook.last_preserve == 0 && hook.load_i == 0);

    hook.load_i = 17;
    DJIMotorRequestSpeedReset(&motor, 1);
    assert(motor.speed_reset_request == 7); /* Full was already consumed. */
    DJIMotorControl();
    assert(hook.resets == 2 && hook.last_preserve == 1 && hook.load_i == 17);

    DJIMotorRequestSpeedReset(&motor, 1);
    assert(motor.speed_reset_request == 9);
    DJIMotorRequestSpeedReset(&motor, 0);
    assert(motor.speed_reset_request == 10); /* New full overrides preserve. */
    DJIMotorControl();
    assert(hook.resets == 3 && hook.last_preserve == 0 && hook.load_i == 0);

    motor.speed_reset_consumed = 0xfffffffcu;
    motor.speed_reset_request = 0xfffffffeu;
    DJIMotorRequestSpeedReset(&motor, 1);
    assert(motor.speed_reset_request == 0); /* Full survives generation wrap. */
    DJIMotorControl();
    assert(hook.resets == 4 && hook.last_preserve == 0);
    DJIMotorRequestSpeedReset(&motor, 1);
    assert(motor.speed_reset_request == 3);
    DJIMotorControl();
    assert(hook.resets == 5 && hook.last_preserve == 1);
}
int main(void)
{
    test_fallback();
    test_once_and_stop();
    test_target_request_snapshot();
    test_stopped_snapshot_enable_race();
    test_snapshot_command_validation();
    test_fault_paths();
    test_pending_reset_policy();
    assert(transmit_count > 0);
    puts("DJI speed-controller routing tests passed");
    return 0;
}
