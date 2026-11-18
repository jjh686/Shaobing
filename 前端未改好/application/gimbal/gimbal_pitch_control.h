#ifndef GIMBAL_PITCH_CONTROL_H
#define GIMBAL_PITCH_CONTROL_H
#include <stdint.h>

/* All torques below are in the canonical positive-pitch direction, BEFORE
 * the motor output sign. Angles/rates use rad and rad/s, limits use degrees. */
typedef struct {
    float max_speed_rad_s, accel_rad_s2, brake_rad_s2;
    float min_deg, max_deg, pose_to_encoder_sign, max_dt_s;
    float gravity_cos_nm, gravity_sin_nm, gravity_zero_rad, gravity_ramp_nm_s;
    float friction_positive_nm, friction_negative_nm, friction_ramp_nm_s;
    float friction_start_rad, friction_release_rad, stopped_rad_s, friction_timeout_s;
} GimbalPitchAssistConfig;

typedef struct {
    GimbalPitchAssistConfig config;
    float speed_ref_rad_s, gravity_nm, friction_nm, friction_elapsed_s;
    int8_t friction_direction;
    uint8_t friction_finished;
} GimbalPitchAssist;

int GimbalPitchAssistInit(GimbalPitchAssist *assist, const GimbalPitchAssistConfig *config);
void GimbalPitchAssistReset(GimbalPitchAssist *assist, uint8_t preserve_support);
float GimbalPitchSpeedReference(GimbalPitchAssist *assist, float desired_rad_s,
    float error_rad, float mechanical_deg, float dt_s);
/* Finite, bounded breakaway assistance. Re-arms only after error changes
 * direction or returns inside release_rad; motion ends the current attempt. */
float GimbalPitchCompensation(GimbalPitchAssist *assist, float angle_rad,
    float error_rad, float speed_rad_s, float dt_s);
#endif
