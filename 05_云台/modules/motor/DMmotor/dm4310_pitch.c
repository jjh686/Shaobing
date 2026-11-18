#include "dm4310_pitch.h"
#include "gimbal_config.h"
#include <math.h>
#include <string.h>

static DM4310PitchInstance pitch_instance;
static uint8_t registered;

static float DecodeRange(uint32_t value, float maximum, uint32_t top)
{
    return (float)value * (2.0f * maximum) / (float)top - maximum;
}

static uint16_t EncodeRange(float value, float maximum, uint32_t top)
{
    value = fminf(fmaxf(value, -maximum), maximum);
    return (uint16_t)((value + maximum) * (float)top / (2.0f * maximum));
}

/* This is deliberately a pure-torque frame: position/velocity are neutral,
 * Kp=Kd=0; all mechanical feedback and reference signs remain in application. */
static void PackTorque(float torque, uint8_t out[8])
{
    uint16_t p = EncodeRange(0, GIMBAL_PITCH_DM_P_MAX, 65535u);
    uint16_t v = EncodeRange(0, GIMBAL_PITCH_DM_V_MAX, 4095u);
    uint16_t t = EncodeRange(torque, GIMBAL_PITCH_DM_T_MAX, 4095u);
    out[0] = (uint8_t)(p >> 8); out[1] = (uint8_t)p;
    out[2] = (uint8_t)(v >> 4); out[3] = (uint8_t)((v & 15u) << 4);
    out[4] = 0; out[5] = 0;
    out[6] = (uint8_t)(t >> 8); out[7] = (uint8_t)t;
}

static void DecodeFeedback(CANInstance *can)
{
    DM4310PitchInstance *m = can->id;
    const uint8_t *d = can->rx_buff;
    if (can->rx_len != 8 || (d[0] & 15u) != (can->tx_id & 15u)) {
        ++m->rejected_feedback;
        return;
    }
    DM4310PitchMeasure next = {0};
    next.state = d[0] >> 4;
    next.position_rad = DecodeRange(((uint16_t)d[1] << 8) | d[2], GIMBAL_PITCH_DM_P_MAX, 65535u);
    next.velocity_rad_s = DecodeRange(((uint16_t)d[3] << 4) | (d[4] >> 4), GIMBAL_PITCH_DM_V_MAX, 4095u);
    next.torque_nm = DecodeRange(((uint16_t)(d[4] & 15u) << 8) | d[5], GIMBAL_PITCH_DM_T_MAX, 4095u);
    next.temperature = d[7]; next.mos_temperature = d[6];
    if (!m->feedback_seen) {
        next.total_angle = next.position_rad / GIMBAL_RAD_PER_DEG;
        m->startup_angle = next.total_angle;
    } else {
        float delta = next.position_rad - m->measure.position_rad;
        /* Wrap by the configured wire span, never a hidden gear ratio. */
        if (delta > GIMBAL_PITCH_DM_P_MAX) delta -= 2.0f * GIMBAL_PITCH_DM_P_MAX;
        if (delta < -GIMBAL_PITCH_DM_P_MAX) delta += 2.0f * GIMBAL_PITCH_DM_P_MAX;
        next.total_angle = m->measure.total_angle + delta / GIMBAL_RAD_PER_DEG;
    }
    m->measure = next;
    m->last_feedback_tick = HAL_GetTick();
    m->feedback_seen = 1;
    if (next.state > 1u || (next.state == 0u && m->hardware_running_seen)) {
        if (!m->fault_latched) m->fault_reason_flags = DM_PITCH_GATE_NOT_ENABLED;
        m->fault_latched = 1;
        m->stop_flag = MOTOR_STOP;
    }
}

DM4310PitchInstance *DM4310PitchInit(Motor_Init_Config_s *config)
{
    if (registered || !config || !config->can_init_config.can_handle ||
        config->can_init_config.tx_id == 0 || config->can_init_config.tx_id > 15u ||
        config->can_init_config.rx_id > 0x7ffu ||
        !isfinite(GIMBAL_PITCH_DM_P_MAX) || !(GIMBAL_PITCH_DM_P_MAX > 0) ||
        !isfinite(GIMBAL_PITCH_DM_V_MAX) || !(GIMBAL_PITCH_DM_V_MAX > 0) ||
        !isfinite(GIMBAL_PITCH_DM_T_MAX) || !(GIMBAL_PITCH_DM_T_MAX > 0) ||
        !isfinite(GIMBAL_PITCH_DM_NM_PER_COMMAND) || GIMBAL_PITCH_DM_NM_PER_COMMAND <= 0 ||
        !isfinite(GIMBAL_PITCH_DM_TORQUE_LIMIT_NM) || GIMBAL_PITCH_DM_TORQUE_LIMIT_NM <= 0 ||
        GIMBAL_PITCH_DM_TORQUE_LIMIT_NM > GIMBAL_PITCH_DM_T_MAX)
        return NULL;
    if (!isfinite(GIMBAL_PITCH_MIN_DEG) || !isfinite(GIMBAL_PITCH_MAX_DEG) ||
        !isfinite(GIMBAL_PITCH_HARD_MIN_DEG) || !isfinite(GIMBAL_PITCH_HARD_MAX_DEG) ||
        GIMBAL_PITCH_HARD_MIN_DEG >= GIMBAL_PITCH_MIN_DEG ||
        GIMBAL_PITCH_HARD_MAX_DEG <= GIMBAL_PITCH_MAX_DEG ||
        GIMBAL_PITCH_MIN_DEG >= GIMBAL_PITCH_MAX_DEG ||
        (GIMBAL_PITCH_POSE_TO_ENCODER_SIGN != 1 && GIMBAL_PITCH_POSE_TO_ENCODER_SIGN != -1))
        return NULL;
    DM4310PitchInstance *m = &pitch_instance;
    memset(m, 0, sizeof(*m));
    m->motor_settings = config->controller_setting_init_config;
    Motor_Controller_s *c = &m->motor_controller;
    Motor_Controller_Init_s *p = &config->controller_param_init_config;
    c->other_angle_feedback_ptr = p->other_angle_feedback_ptr;
    c->other_speed_feedback_ptr = p->other_speed_feedback_ptr;
    c->speed_feedforward_ptr = p->speed_feedforward_ptr;
    c->current_feedforward_ptr = p->current_feedforward_ptr;
    PIDInit(&c->angle_PID, &p->angle_PID);
    PIDInit(&c->speed_PID, &p->speed_PID);
    PIDInit(&c->current_PID, &p->current_PID);
    CAN_Init_Config_s can_config = config->can_init_config;
    can_config.id = m;
    can_config.can_module_callback = DecodeFeedback;
    m->motor_can_instance = CANRegister(&can_config);
    if (!m->motor_can_instance) return NULL;
    m->stop_flag = MOTOR_STOP;
    registered = 1;
    return m;
}

void DM4310PitchSetRef(DM4310PitchInstance *m, float ref)
{
    if (!m) return;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    m->motor_controller.pid_ref = ref;
    m->last_command_tick = HAL_GetTick();
    __set_PRIMASK(mask);
}

void DM4310PitchStop(DM4310PitchInstance *m)
{
    if (m) m->stop_flag = MOTOR_STOP;
}

void DM4310PitchEnable(DM4310PitchInstance *m)
{
    if (!m) return;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    if (!m->fault_latched) m->stop_flag = MOTOR_ENABLED;
    __set_PRIMASK(mask);
}

void DM4310PitchBindSpeedController(DM4310PitchInstance *m,
    DJIMotorSpeedUpdateFn update, DJIMotorSpeedResetFn reset, void *context)
{
    if (!m) return;
    m->speed_controller.update = update;
    m->speed_controller.reset = update ? reset : NULL;
    m->speed_controller.context = update ? context : NULL;
}

void DM4310PitchBindAppliedCallback(DM4310PitchInstance *m,
    DM4310PitchAppliedFn callback, void *context)
{
    if (!m) return;
    m->applied_callback = callback;
    m->applied_context = callback ? context : NULL;
}

void DM4310PitchRequestSpeedReset(DM4310PitchInstance *m, uint8_t preserve)
{
    if (!m) return;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    uint32_t pending = m->speed_reset_request;
    if (pending != m->speed_reset_consumed && !(pending & 1u)) preserve = 0;
    m->speed_reset_request = ((pending + 2u) & ~1u) | (preserve ? 1u : 0u);
    __set_PRIMASK(mask);
}

static void ResetSpeed(DM4310PitchInstance *m, uint32_t request, uint8_t preserve)
{
    PIDInstance *pid = &m->motor_controller.speed_PID;
    float previous_i = pid->Iout;
    PIDClear(pid);
    if (m->speed_controller.reset)
        m->speed_controller.reset(m->speed_controller.context, pid, preserve);
    else if (preserve && isfinite(previous_i) && isfinite(pid->IntegralLimit) && pid->IntegralLimit >= 0)
        pid->Iout = fminf(fmaxf(previous_i, -pid->IntegralLimit), pid->IntegralLimit);
    m->speed_reset_consumed = request;
}

static void ResetControllers(DM4310PitchInstance *m, uint32_t request)
{
    PIDClear(&m->motor_controller.angle_PID);
    ResetSpeed(m, request, 0);
    PIDClear(&m->motor_controller.current_PID);
}

/* Caller holds the IRQ mask while sampling ISR-owned feedback and timestamps. */
static int InputsHealthy(const DM4310PitchInstance *m, uint32_t now)
{
    const Motor_Controller_s *c = &m->motor_controller;
    const Motor_Control_Setting_s *s = &m->motor_settings;
    float mechanical_deg = m->measure.total_angle-m->startup_angle;
    return m->feedback_seen && m->measure.state <= 1u && isfinite(mechanical_deg) &&
        mechanical_deg >= GIMBAL_PITCH_HARD_MIN_DEG && mechanical_deg <= GIMBAL_PITCH_HARD_MAX_DEG &&
        (uint32_t)(now - m->last_feedback_tick) <= GIMBAL_MOTOR_TIMEOUT_MS &&
        (!m->external_feedback_tick ||
         (uint32_t)(now - *m->external_feedback_tick) <= m->external_feedback_timeout_ms) &&
        isfinite(c->pid_ref) &&
        s->angle_feedback_source == OTHER_FEED && c->other_angle_feedback_ptr &&
        isfinite(*c->other_angle_feedback_ptr) &&
        s->speed_feedback_source == OTHER_FEED && c->other_speed_feedback_ptr &&
        isfinite(*c->other_speed_feedback_ptr) &&
        !(s->close_loop_type & CURRENT_LOOP) &&
        (!(s->feedforward_flag & SPEED_FEEDFORWARD) ||
         (c->speed_feedforward_ptr && isfinite(*c->speed_feedforward_ptr))) &&
        (!(s->feedforward_flag & CURRENT_FEEDFORWARD) ||
         (c->current_feedforward_ptr && isfinite(*c->current_feedforward_ptr)));
}

uint8_t DM4310PitchClearFault(DM4310PitchInstance *m)
{
    if (!m) return 0;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    int ok = m->stop_flag == MOTOR_STOP && InputsHealthy(m, HAL_GetTick());
    if (ok) {
        m->fault_latched = 0;
        m->fault_reason_flags = 0;
        m->hardware_running_seen = 0;
    }
    __set_PRIMASK(mask);
    /* No 0xFB automatic hardware reset: a real overtemperature/voltage error
     * must first be corrected and cleared explicitly outside this loop. */
    return (uint8_t)ok;
}

void DM4310PitchControl(void)
{
    if (!registered) return;
    DM4310PitchInstance *m = &pitch_instance;
    Motor_Controller_s *c = &m->motor_controller;
    Motor_Control_Setting_s *s = &m->motor_settings;
    uint32_t mask = __get_PRIMASK(); __disable_irq();
    uint32_t now = HAL_GetTick(), command_tick = m->last_command_tick;
    uint32_t request = m->speed_reset_request;
    Motor_Working_Type_e stop = m->stop_flag;
    float ref = c->pid_ref;
    float angle = c->other_angle_feedback_ptr ? *c->other_angle_feedback_ptr : NAN;
    float speed = c->other_speed_feedback_ptr ? *c->other_speed_feedback_ptr : NAN;
    float speed_ff = c->speed_feedforward_ptr ? *c->speed_feedforward_ptr : 0;
    float current_ff = c->current_feedforward_ptr ? *c->current_feedforward_ptr : 0;
    int healthy = InputsHealthy(m, now) && (stop != MOTOR_ENABLED ||
        !m->command_timeout_ms || (uint32_t)(now-command_tick) <= m->command_timeout_ms);
    uint8_t hardware_enabled = m->measure.state == 1u;
    if (stop == MOTOR_ENABLED && !healthy) {
        if (!m->fault_latched) {
            uint32_t reason = DM_PITCH_GATE_INVALID;
            if (!m->feedback_seen || (uint32_t)(now-m->last_feedback_tick) > GIMBAL_MOTOR_TIMEOUT_MS)
                reason |= DM_PITCH_GATE_CAN_STALE;
            if (m->external_feedback_tick && (uint32_t)(now-*m->external_feedback_tick) > m->external_feedback_timeout_ms)
                reason |= DM_PITCH_GATE_IMU_STALE;
            if (m->command_timeout_ms && (uint32_t)(now-command_tick) > m->command_timeout_ms)
                reason |= DM_PITCH_GATE_COMMAND_STALE;
            float mechanical = m->measure.total_angle-m->startup_angle;
            if (!isfinite(mechanical) || mechanical < GIMBAL_PITCH_HARD_MIN_DEG || mechanical > GIMBAL_PITCH_HARD_MAX_DEG)
                reason |= DM_PITCH_GATE_HARD_LIMIT;
            m->fault_reason_flags = reason;
        }
        m->fault_latched = 1; m->stop_flag = MOTOR_STOP;
    }
    int run = stop == MOTOR_ENABLED && !m->fault_latched && healthy && hardware_enabled;
    __set_PRIMASK(mask);

    float torque = 0;
    m->requested_torque_nm = 0;
    if (!run) {
        ResetControllers(m, request);
    } else {
        if (s->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) ref = -ref;
        if ((s->close_loop_type & ANGLE_LOOP) && s->outer_loop_type == ANGLE_LOOP)
            ref = PIDCalculate(&c->angle_PID, angle, ref);
        if ((s->close_loop_type & SPEED_LOOP) && (s->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP))) {
            if (s->feedforward_flag & SPEED_FEEDFORWARD) ref += speed_ff;
            if (request != m->speed_reset_consumed) ResetSpeed(m, request, (uint8_t)(request & 1u));
            ref = m->speed_controller.update
                ? m->speed_controller.update(m->speed_controller.context, &c->speed_PID, speed, ref)
                : PIDCalculate(&c->speed_PID, speed, ref);
        }
        if (s->feedforward_flag & CURRENT_FEEDFORWARD) ref += current_ff;
        if (s->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE) ref = -ref;
        torque = ref * GIMBAL_PITCH_DM_NM_PER_COMMAND;
        m->requested_torque_nm = torque;
        if (!isfinite(torque)) {
            mask = __get_PRIMASK(); __disable_irq();
            if (!m->fault_latched) m->fault_reason_flags = DM_PITCH_GATE_INVALID;
            m->fault_latched = 1; m->stop_flag = MOTOR_STOP;
            __set_PRIMASK(mask);
            torque = 0;
            ResetControllers(m, request);
        }
        torque = fminf(fmaxf(torque, -GIMBAL_PITCH_DM_TORQUE_LIMIT_NM), GIMBAL_PITCH_DM_TORQUE_LIMIT_NM);
    }

    /* Recheck AFTER callbacks and immediately before enqueue. A concurrent
     * STOP, stale IMU, new hardware fault, or reset request cannot leak torque.
     * CANTransmit copies the frame synchronously into its queue under this mask. */
    mask = __get_PRIMASK(); __disable_irq();
    now = HAL_GetTick();
    int final_healthy = InputsHealthy(m, now) && (m->stop_flag != MOTOR_ENABLED ||
        !m->command_timeout_ms || (uint32_t)(now-m->last_command_tick) <= m->command_timeout_ms);
    uint32_t gates = 0;
    if (stop != MOTOR_ENABLED || m->stop_flag != MOTOR_ENABLED) gates |= DM_PITCH_GATE_STOP;
    if (m->fault_latched) gates |= DM_PITCH_GATE_FAULT;
    if (!m->feedback_seen || (uint32_t)(now-m->last_feedback_tick) > GIMBAL_MOTOR_TIMEOUT_MS)
        gates |= DM_PITCH_GATE_CAN_STALE;
    if (m->external_feedback_tick &&
        (uint32_t)(now-*m->external_feedback_tick) > m->external_feedback_timeout_ms)
        gates |= DM_PITCH_GATE_IMU_STALE;
    if ((stop == MOTOR_ENABLED || m->stop_flag == MOTOR_ENABLED) && m->command_timeout_ms &&
        (uint32_t)(now-m->last_command_tick) > m->command_timeout_ms)
        gates |= DM_PITCH_GATE_COMMAND_STALE;
    if (m->measure.state != 1u) gates |= DM_PITCH_GATE_NOT_ENABLED;
    float mechanical_now = m->measure.total_angle-m->startup_angle;
    if (!isfinite(mechanical_now) || mechanical_now < GIMBAL_PITCH_HARD_MIN_DEG ||
        mechanical_now > GIMBAL_PITCH_HARD_MAX_DEG) gates |= DM_PITCH_GATE_HARD_LIMIT;
    if (request != m->speed_reset_request) gates |= DM_PITCH_GATE_RESET_PENDING;
    if (!healthy || !final_healthy || !isfinite(m->requested_torque_nm)) gates |= DM_PITCH_GATE_INVALID;
    if (isfinite(m->requested_torque_nm) &&
        fabsf(m->requested_torque_nm) > GIMBAL_PITCH_DM_TORQUE_LIMIT_NM)
        gates |= DM_PITCH_GATE_TORQUE_LIMIT;
    if (m->stop_flag == MOTOR_ENABLED && !final_healthy) {
        if (!m->fault_latched) m->fault_reason_flags = gates | DM_PITCH_GATE_INVALID;
        m->fault_latched = 1; m->stop_flag = MOTOR_STOP;
    }
    if (stop != MOTOR_ENABLED || m->stop_flag != MOTOR_ENABLED || m->fault_latched ||
        !healthy || !final_healthy || m->measure.state != 1u || request != m->speed_reset_request)
        torque = 0;
    else {
        m->hardware_running_seen = 1;
        float mechanical = m->measure.total_angle-m->startup_angle;
        float output_sign = s->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE ? -1.0f : 1.0f;
        float encoder_effort = torque*output_sign*GIMBAL_PITCH_POSE_TO_ENCODER_SIGN;
        /* Last check applies to TOTAL torque, including CURRENT_FEEDFORWARD.
         * At a soft edge, permit inward torque; outside the hard envelope the
         * existing unhealthy gate above stops the motor. */
        if ((mechanical >= GIMBAL_PITCH_MAX_DEG && encoder_effort > 0) ||
            (mechanical <= GIMBAL_PITCH_MIN_DEG && encoder_effort < 0)) {
            torque = 0;
            gates |= DM_PITCH_GATE_ENVELOPE;
        }
        if (m->applied_callback)
            m->applied_callback(m->applied_context, &c->speed_PID,
                m->requested_torque_nm*output_sign/GIMBAL_PITCH_DM_NM_PER_COMMAND,
                torque*output_sign/GIMBAL_PITCH_DM_NM_PER_COMMAND, gates);
    }
    if (m->stop_flag != MOTOR_ENABLED) { torque = 0; gates |= DM_PITCH_GATE_STOP; }
    if (request != m->speed_reset_request) { torque = 0; gates |= DM_PITCH_GATE_RESET_PENDING; }
    if (m->fault_latched) { torque = 0; gates |= DM_PITCH_GATE_FAULT; }
    m->last_torque_nm = torque;
    m->last_gate_flags = gates;
    PackTorque(torque, m->motor_can_instance->tx_buff);
    CANTransmit(m->motor_can_instance, 8);
    if (stop == MOTOR_ENABLED && m->stop_flag == MOTOR_ENABLED && !m->fault_latched &&
        healthy && final_healthy && m->measure.state == 0u &&
        (!m->enable_sent || (uint32_t)(now-m->last_enable_tick) >= GIMBAL_PITCH_DM_ENABLE_RETRY_MS)) {
        /* Enable only after a zero command and an explicit application enable.
         * Output stays zero until a later feedback confirms state=1. */
        memset(m->motor_can_instance->tx_buff, 0xff, 7);
        m->motor_can_instance->tx_buff[7] = 0xfc;
        CANTransmit(m->motor_can_instance, 8);
        m->last_enable_tick = now; m->enable_sent = 1;
    }
    __set_PRIMASK(mask);
}
