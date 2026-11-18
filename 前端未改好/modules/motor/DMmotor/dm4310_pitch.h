#ifndef DM4310_PITCH_H
#define DM4310_PITCH_H

#include "dji_motor.h"

/* Dedicated MIT torque transport. The application still closes both loops
 * using its existing IMU rad / rad/s feedback. Mechanical angles alone use
 * degrees to preserve the existing startup-relative envelope contract. */
typedef struct {
    float total_angle, position_rad, velocity_rad_s, torque_nm;
    uint8_t state, temperature, mos_temperature;
} DM4310PitchMeasure;

enum {
    DM_PITCH_GATE_STOP = 1u << 0,
    DM_PITCH_GATE_FAULT = 1u << 1,
    DM_PITCH_GATE_CAN_STALE = 1u << 2,
    DM_PITCH_GATE_IMU_STALE = 1u << 3,
    DM_PITCH_GATE_COMMAND_STALE = 1u << 4,
    DM_PITCH_GATE_NOT_ENABLED = 1u << 5,
    DM_PITCH_GATE_RESET_PENDING = 1u << 6,
    DM_PITCH_GATE_INVALID = 1u << 7,
    DM_PITCH_GATE_ENVELOPE = 1u << 8,
    DM_PITCH_GATE_TORQUE_LIMIT = 1u << 9,
    DM_PITCH_GATE_HARD_LIMIT = 1u << 10,
};

/* Motor-task callback after final limiting, before CAN enqueue. Raw efforts
 * use canonical-pitch sign. Must be short, nonblocking, and must not send CAN. */
typedef void (*DM4310PitchAppliedFn)(void *context, PIDInstance *pid,
    float requested_raw, float applied_raw, uint32_t gate_flags);

typedef struct {
    DM4310PitchMeasure measure;
    Motor_Controller_s motor_controller;
    Motor_Control_Setting_s motor_settings;
    CANInstance *motor_can_instance;
    float startup_angle;
    volatile uint32_t last_feedback_tick;
    volatile uint8_t feedback_seen, fault_latched;
    volatile Motor_Working_Type_e stop_flag;
    volatile uint32_t *external_feedback_tick;
    uint32_t external_feedback_timeout_ms;
    uint32_t last_command_tick, command_timeout_ms;
    DJIMotorSpeedController speed_controller;
    volatile uint32_t speed_reset_request, speed_reset_consumed;
    uint32_t last_enable_tick, rejected_feedback;
    uint8_t enable_sent, hardware_running_seen;
    float last_torque_nm;
    float requested_torque_nm; /* Signed motor torque before final limiting. */
    uint32_t last_gate_flags;  /* DM_PITCH_GATE_*; bit mask, not a fault latch. */
    uint32_t fault_reason_flags; /* First latched reason, survives later STOPs. */
    DM4310PitchAppliedFn applied_callback;
    void *applied_context;
} DM4310PitchInstance;

/* Init never enables, zeroes the encoder, or clears a hardware error.
 * Exactly one dedicated pitch instance is supported. No DMMotorTask register. */
DM4310PitchInstance *DM4310PitchInit(Motor_Init_Config_s *config);
void DM4310PitchSetRef(DM4310PitchInstance *motor, float ref_rad);
void DM4310PitchStop(DM4310PitchInstance *motor);
void DM4310PitchEnable(DM4310PitchInstance *motor);
uint8_t DM4310PitchClearFault(DM4310PitchInstance *motor);
void DM4310PitchBindSpeedController(DM4310PitchInstance *motor,
    DJIMotorSpeedUpdateFn update, DJIMotorSpeedResetFn reset, void *context);
void DM4310PitchRequestSpeedReset(DM4310PitchInstance *motor, uint8_t preserve_integral);
void DM4310PitchBindAppliedCallback(DM4310PitchInstance *motor,
    DM4310PitchAppliedFn callback, void *context);
/* Call every 1ms from the existing motor task, beside DJIMotorControl(). */
void DM4310PitchControl(void);

#endif
