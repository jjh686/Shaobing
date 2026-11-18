#ifndef GIMBAL_SPEED_PI_H
#define GIMBAL_SPEED_PI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Speed inputs are rad/s, dt is seconds, outputs are raw motor commands.
 * kp: command/(rad/s), ki: command/rad, i_limit: command, max_dt_s: seconds.
 * This controller owns its state; callers serialize init/reset/step. */
typedef struct {
    float kp;
    float ki;
    float i_limit;
    float output_limit;
    float max_dt_s;
    uint8_t reset_i_on_ref_reversal;
    float reversal_ref_min_rad_s;
} GimbalSpeedPiConfig;

typedef struct {
    float i;
    float prev_error;
    uint8_t prev_error_valid;
    int8_t ref_direction;
} GimbalSpeedPiState;

typedef struct {
    GimbalSpeedPiConfig config;
    GimbalSpeedPiState state;
    float p;
    float error;
    float output;
    float integral_delta; /* Actual integration only; excludes I reset/reclamp. */
    uint32_t reversal_reset_count; /* Lifetime count; reset() preserves it. */
} GimbalSpeedPi;

/* Returns 1 on success, 0 for invalid config. Does not modify pi on failure. */
int GimbalSpeedPiInit(GimbalSpeedPi *pi, const GimbalSpeedPiConfig *config);

/* Clears history and diagnostics, preserving the reversal counter.
 * Retains bounded finite I only when requested. Init starts the counter at 0. */
void GimbalSpeedPiReset(GimbalSpeedPi *pi, uint8_t preserve_integral);

/* Returns NAN on invalid/overflowing data or dt outside (0,max_dt_s].
 * Failures do not modify state/diagnostics; the motor driver must fault.
 * Config can change between calls. A reduced i_limit clamps existing I.
 * A significant yaw reference reversal may clear only opposing old I.
 * Candidate antiwindup rejects further saturation, always permitting I unload;
 * an unloading step crossing zero may stop at zero instead of accumulating. */
float GimbalSpeedPiStep(GimbalSpeedPi *pi, float measure_rad_s,
                       float ref_rad_s, float dt_s);
/* Downstream saturation (including feedforward) rejects only this step's I
 * increment that pushes further into the limit. Does not erase load support. */
void GimbalSpeedPiApplyOutputLimit(GimbalSpeedPi *pi, float requested, float applied);

#ifdef __cplusplus
}
#endif
#endif
