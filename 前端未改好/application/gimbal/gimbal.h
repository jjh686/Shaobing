#ifndef GIMBAL_H
#define GIMBAL_H
/* [USB_SP 修改 01 开始] 新增一致姿态/机械角/健康快照及应用接口。 */
#include <stdint.h>
#include "gimbal_math.h"
typedef struct {
    GimbalPose pose;
    float yaw_continuous; /* rad, unwrapped since INS startup */
    float yaw_mechanical_deg, pitch_mechanical_deg; /* power-on encoder anchor */
    uint32_t tick;
    uint8_t pose_valid, motors_ready, driver_fault;
} GimbalState;
void GimbalInit(void);
void GimbalUpdateFeedback(void); /* INS task, 1 kHz, sole pose writer */
void GimbalGetState(GimbalState *out);
void GimbalTask(void);
uint8_t GimbalClearMotorFaults(void);

/* Recorded in RAM by the 5 ms gimbal task, at TRACE_PERIOD_MS. Values have
 * units in their names; motor_tick is the timestamp of the driver feedback.
 * sequence is committed last under the IRQ mask. This is diagnostic history,
 * never a source of control commands. Read only with the matching ELF. */
typedef struct {
    uint32_t sequence, tick, motor_tick, gate_flags, fault_reason_flags;
    float source_target_rad, driver_target_rad, measured_rad, speed_rad_s;
    float roll_rad, mechanical_deg, raw_speed_ref_rad_s, speed_ref_rad_s;
    float p_nm, i_nm, gravity_nm, friction_nm;
    float requested_nm, applied_nm, reported_nm;
    uint8_t mode, enabled, motor_state, fault;
} GimbalPitchTrace;
#endif
/* [USB_SP 修改 01 结束] */
