/* Real driver + real PID, MCU/CAN/time only stubbed. No hardware operations. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "dm4310_pitch.c"

CAN_HandleTypeDef hcan1, hcan2;
static CANInstance can;
static uint32_t now, mask_value, dwt;
static volatile uint32_t pose_tick;
static float pose_angle, pose_speed;
static uint8_t frames[128][8];
static unsigned frame_count;
static void (*on_unlock)(void);

uint32_t HAL_GetTick(void) { return now; }
uint32_t __get_PRIMASK(void) { return mask_value; }
void __disable_irq(void) { mask_value = 1; }
void __set_PRIMASK(uint32_t value)
{
    mask_value = value;
    if (!value && on_unlock) {
        void (*callback)(void) = on_unlock;
        on_unlock = NULL;
        callback();
    }
}
float DWT_GetDeltaT(uint32_t *count) { *count = ++dwt; return 0.001f; }
CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    memset(&can, 0, sizeof(can));
    can.can_handle = config->can_handle; can.tx_id = config->tx_id;
    can.rx_id = config->rx_id; can.id = config->id;
    can.can_module_callback = config->can_module_callback;
    return &can;
}
void CANTransmit(CANInstance *instance, uint8_t length)
{
    assert(mask_value && length == 8 && instance->tx_id == 2);
    assert(frame_count < 128);
    memcpy(frames[frame_count++], instance->tx_buff, 8);
}

static uint16_t Code(float value, float max, unsigned top)
{
    return (uint16_t)((value + max) * (float)top / (2.0f * max));
}
static void Feedback(uint8_t state, float position)
{
    uint16_t p = Code(position, GIMBAL_PITCH_DM_P_MAX, 65535);
    uint16_t v = Code(0, GIMBAL_PITCH_DM_V_MAX, 4095);
    uint16_t t = Code(0, GIMBAL_PITCH_DM_T_MAX, 4095);
    can.rx_len = 8;
    can.rx_buff[0] = (uint8_t)((state << 4) | 2);
    can.rx_buff[1] = (uint8_t)(p >> 8); can.rx_buff[2] = (uint8_t)p;
    can.rx_buff[3] = (uint8_t)(v >> 4);
    can.rx_buff[4] = (uint8_t)(((v & 15u) << 4) | (t >> 8));
    can.rx_buff[5] = (uint8_t)t; can.rx_buff[6] = 35; can.rx_buff[7] = 36;
    can.can_module_callback(&can);
    pose_tick = now;
}
static float Torque(const uint8_t *frame)
{
    assert(frame[4] == 0 && frame[5] == 0 && !(frame[3] & 15u) && !(frame[6] & 0xf0u));
    return ((float)(((uint16_t)frame[6] << 8) | frame[7]) / 4095.0f * 2 - 1) * GIMBAL_PITCH_DM_T_MAX;
}
static float Hook(void *context, PIDInstance *pid, float measure, float ref)
{
    (void)pid; (void)measure; (void)ref;
    return *(float *)context;
}
static DM4310PitchInstance *Fixture(void)
{
    registered = 0; now = 100; pose_tick = now; mask_value = 0;
    pose_angle = pose_speed = 0; frame_count = 0; on_unlock = NULL;
    Motor_Init_Config_s config = {
        .can_init_config = {.can_handle=&hcan1, .tx_id=2, .rx_id=0},
        .controller_setting_init_config = {
            .angle_feedback_source=OTHER_FEED, .speed_feedback_source=OTHER_FEED,
            .close_loop_type=ANGLE_LOOP|SPEED_LOOP, .outer_loop_type=ANGLE_LOOP,
        },
        .controller_param_init_config = {
            .other_angle_feedback_ptr=&pose_angle, .other_speed_feedback_ptr=&pose_speed,
            .angle_PID={.Kp=10, .MaxOut=10}, .speed_PID={.Kp=50, .MaxOut=20000},
        },
    };
    DM4310PitchInstance *m = DM4310PitchInit(&config);
    assert(m && m->stop_flag == MOTOR_STOP && frame_count == 0);
    assert(can.rx_id == 0 && can.tx_id == 2);
    m->external_feedback_tick = &pose_tick;
    m->external_feedback_timeout_ms = GIMBAL_INS_TIMEOUT_MS;
    m->command_timeout_ms = GIMBAL_CMD_TIMEOUT_MS;
    return m;
}

static void TestWireAndFeedback(void)
{
    DM4310PitchInstance *m = Fixture();
    uint8_t frame[8];
    PackTorque(10, frame); assert((frame[6]&15u) == 15u && frame[7] == 255u);
    PackTorque(-10, frame); assert(frame[6] == 0 && frame[7] == 0);
    PackTorque(100, frame); assert(fabsf(Torque(frame)-10) < 0.001f);
    PackTorque(0, frame); assert(fabsf(Torque(frame)) < 0.005f);
    Feedback(0, 1.0f);
    assert(m->feedback_seen && m->measure.state == 0 && !m->fault_latched);
    assert(fabsf(m->measure.position_rad-1) < 0.0002f);
    assert(fabsf(m->startup_angle - m->measure.total_angle) < 0.001f);
    assert(m->measure.temperature == 36 && m->measure.mos_temperature == 35);
    uint32_t last = m->last_feedback_tick;
    now += 1; can.rx_len=7; can.can_module_callback(&can);
    assert(m->last_feedback_tick == last && m->rejected_feedback == 1);
    can.rx_len=8; can.rx_buff[0]=3; can.can_module_callback(&can);
    assert(m->last_feedback_tick == last && m->rejected_feedback == 2);
    can.rx_buff[0]=2; can.rx_buff[3]=255; can.rx_buff[4]=255; can.rx_buff[5]=255;
    can.can_module_callback(&can);
    assert(fabsf(m->measure.velocity_rad_s-GIMBAL_PITCH_DM_V_MAX) < 0.0001f);
    assert(fabsf(m->measure.torque_nm-GIMBAL_PITCH_DM_T_MAX) < 0.0001f);
    m = Fixture();
    Feedback(0, GIMBAL_PITCH_DM_P_MAX-0.05f);
    float first = m->measure.total_angle;
    Feedback(0, -GIMBAL_PITCH_DM_P_MAX+0.05f);
    assert(fabsf((m->measure.total_angle-first) * GIMBAL_RAD_PER_DEG - 0.1f) < 0.0003f);
    assert(m->startup_angle == first);
}

static void TestEnableAndStop(void)
{
    DM4310PitchInstance *m=Fixture();
    Feedback(0, 0);
    for (unsigned i=0;i<5;++i) { DM4310PitchControl(); ++now; pose_tick=now; }
    assert(frame_count == 5);
    for (unsigned i=0;i<frame_count;++i) assert(fabsf(Torque(frames[i])) < 0.005f);
    frame_count=0;
    DM4310PitchSetRef(m, 0.1f); DM4310PitchEnable(m); DM4310PitchControl();
    assert(frame_count == 2 && fabsf(Torque(frames[0])) < 0.005f);
    assert(frames[1][0] == 255 && frames[1][7] == 0xfc);
    DM4310PitchControl(); assert(frame_count == 3); /* throttled, zero only */
    float effort=5000;
    DM4310PitchBindSpeedController(m, Hook, NULL, &effort);
    Feedback(1, 0); DM4310PitchControl();
    assert(fabsf(Torque(frames[3])-5000*GIMBAL_PITCH_DM_NM_PER_COMMAND) < 0.006f);
    m->motor_settings.feedback_reverse_flag=FEEDBACK_DIRECTION_REVERSE;
    DM4310PitchControl(); assert(Torque(frames[4]) < -0.7f);
    effort=1e8f;
    DM4310PitchControl(); assert(fabsf(Torque(frames[5])+GIMBAL_PITCH_DM_TORQUE_LIMIT_NM) < 0.006f);
    DM4310PitchStop(m); DM4310PitchControl();
    assert(fabsf(Torque(frames[6])) < 0.005f && !m->fault_latched);
    Feedback(0,0); /* A running device disappeared/disabled: no automatic FC. */
    assert(m->fault_latched && m->stop_flag==MOTOR_STOP);
    frame_count=0; DM4310PitchControl(); assert(frame_count==1);
    assert(fabsf(Torque(frames[0])) < 0.005f);
    assert(DM4310PitchClearFault(m)); Feedback(0,0);
    assert(!m->fault_latched && m->stop_flag==MOTOR_STOP);
}

static void TestFaults(void)
{
    DM4310PitchInstance *m=Fixture(); Feedback(1, 0);
    DM4310PitchSetRef(m, 0.1f); DM4310PitchEnable(m);
    now += GIMBAL_CMD_TIMEOUT_MS + 1; pose_tick=now; Feedback(1, 0);
    DM4310PitchControl(); assert(m->fault_latched && m->stop_flag==MOTOR_STOP);
    assert(m->last_gate_flags & DM_PITCH_GATE_COMMAND_STALE);
    assert(fabsf(Torque(frames[0])) < 0.005f);
    DM4310PitchEnable(m); assert(m->stop_flag == MOTOR_STOP);
    assert(DM4310PitchClearFault(m)); assert(!m->fault_latched && m->stop_flag==MOTOR_STOP);
    Feedback(8, 0); assert(m->fault_latched && !DM4310PitchClearFault(m));
    unsigned before=frame_count; DM4310PitchControl();
    assert(frame_count==before+1 && fabsf(Torque(frames[before])) < 0.005f); /* no auto FB */
    Feedback(0, 0); assert(m->fault_latched); assert(DM4310PitchClearFault(m));
    m=Fixture(); Feedback(1, 0); DM4310PitchSetRef(m, 0.1f); DM4310PitchEnable(m);
    pose_angle=NAN; DM4310PitchControl(); assert(m->fault_latched);
    m=Fixture(); Feedback(1, 0); DM4310PitchSetRef(m, 0.1f); DM4310PitchEnable(m);
    now += GIMBAL_INS_TIMEOUT_MS+1; m->last_feedback_tick=now;
    DM4310PitchControl(); assert(m->fault_latched);
    m=Fixture(); Feedback(1, 0); DM4310PitchSetRef(m, 0.1f); DM4310PitchEnable(m);
    now += GIMBAL_MOTOR_TIMEOUT_MS+1; pose_tick=now; m->last_command_tick=now;
    DM4310PitchControl(); assert(m->fault_latched);
    assert(m->last_gate_flags & DM_PITCH_GATE_CAN_STALE);
    m=Fixture(); Feedback(1, 0); DM4310PitchSetRef(m, 0.1f); DM4310PitchEnable(m);
    float effort=NAN; DM4310PitchBindSpeedController(m, Hook, NULL, &effort);
    DM4310PitchControl(); assert(m->fault_latched && fabsf(Torque(frames[0])) < 0.005f);
}

static void LateStop(void) { DM4310PitchStop(&pitch_instance); }
static float LateReset(void *context, PIDInstance *pid, float measure, float ref)
{
    (void)context; (void)pid; (void)measure; (void)ref;
    DM4310PitchRequestSpeedReset(&pitch_instance, 1);
    return 5000;
}
static void TestConcurrentCommitAndReset(void)
{
    DM4310PitchInstance *m=Fixture(); Feedback(1, 0);
    DM4310PitchSetRef(m, 0.1f); DM4310PitchEnable(m);
    float effort=5000; DM4310PitchBindSpeedController(m, Hook, NULL, &effort);
    on_unlock=LateStop; DM4310PitchControl();
    assert(m->stop_flag==MOTOR_STOP && fabsf(Torque(frames[0])) < 0.005f);
    DM4310PitchRequestSpeedReset(m, 0); DM4310PitchRequestSpeedReset(m, 1);
    assert(!(m->speed_reset_request & 1u)); /* full reset dominates */
    DM4310PitchSetRef(m,0.1f); DM4310PitchEnable(m);
    DM4310PitchBindSpeedController(m, LateReset, NULL, NULL);
    DM4310PitchControl();
    assert(m->speed_reset_request != m->speed_reset_consumed);
    assert(fabsf(Torque(frames[1])) < 0.005f);
}

static void TestTotalTorqueEnvelope(void)
{
    for (int edge=-1; edge<=1; edge+=2) {
        for (int reverse=0; reverse<=1; ++reverse) {
            DM4310PitchInstance *m=Fixture(); Feedback(1,0);
            m->measure.total_angle=m->startup_angle+(edge > 0 ? GIMBAL_PITCH_MAX_DEG : GIMBAL_PITCH_MIN_DEG);
            float effort=100.0f*(float)edge, ff=1200.0f*(float)edge;
            DM4310PitchBindSpeedController(m,Hook,NULL,&effort);
            m->motor_controller.current_feedforward_ptr=&ff;
            m->motor_settings.feedforward_flag=CURRENT_FEEDFORWARD;
            m->motor_settings.feedback_reverse_flag=reverse ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL;
            DM4310PitchSetRef(m,0.1f); DM4310PitchEnable(m); DM4310PitchControl();
            assert(m->last_gate_flags & DM_PITCH_GATE_ENVELOPE);
            assert(fabsf(m->last_torque_nm) < 1e-7f && !m->fault_latched);
            effort=-2000.0f*(float)edge; /* Inward net effort, with FF still enabled. */
            DM4310PitchControl();
            assert(!(m->last_gate_flags & DM_PITCH_GATE_ENVELOPE));
            float expected=(effort+ff)*GIMBAL_PITCH_DM_NM_PER_COMMAND*(reverse ? -1 : 1);
            assert(fabsf(m->last_torque_nm-expected) < 1e-5f);
        }
    }
    DM4310PitchInstance *m=Fixture(); Feedback(1,0);
    m->measure.total_angle=m->startup_angle+GIMBAL_PITCH_HARD_MAX_DEG+0.1f;
    DM4310PitchSetRef(m,-0.1f); DM4310PitchEnable(m); DM4310PitchControl();
    assert(m->fault_latched && m->last_torque_nm == 0);
    assert(m->fault_reason_flags & DM_PITCH_GATE_HARD_LIMIT);
    assert(!DM4310PitchClearFault(m));
}

static void AppliedLateFault(void *context, PIDInstance *pid,
    float requested, float applied, uint32_t flags)
{
    (void)pid; (void)requested; (void)applied; (void)flags;
    ((DM4310PitchInstance *)context)->fault_latched=1;
}
static void TestAppliedCallbackCannotLeakFaultTorque(void)
{
    DM4310PitchInstance *m=Fixture(); Feedback(1,0);
    float effort=1000;
    DM4310PitchBindSpeedController(m,Hook,NULL,&effort);
    DM4310PitchBindAppliedCallback(m,AppliedLateFault,m);
    DM4310PitchSetRef(m,0.1f); DM4310PitchEnable(m); DM4310PitchControl();
    assert(m->last_torque_nm==0 && (m->last_gate_flags & DM_PITCH_GATE_FAULT));
    assert(fabsf(Torque(frames[0])) < 0.005f);
}

int main(void)
{
    TestWireAndFeedback(); TestEnableAndStop(); TestFaults();
    TestConcurrentCommitAndReset();
    TestTotalTorqueEnvelope();
    TestAppliedCallbackCannotLeakFaultTorque();
    puts("DM4310 pitch driver: wire, feedback, handshake, torque/sign, watchdogs, fault latch, reset/STOP commit passed");
    return 0;
}
