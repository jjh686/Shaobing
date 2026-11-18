#include "gimbal_speed_pi.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static float Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

static int ConfigValid(const GimbalSpeedPiConfig *c)
{
    return c != NULL && isfinite(c->kp) && c->kp >= 0.0f &&
           isfinite(c->ki) && c->ki >= 0.0f &&
           isfinite(c->i_limit) && c->i_limit >= 0.0f &&
           isfinite(c->output_limit) && c->output_limit > 0.0f &&
           isfinite(c->max_dt_s) && c->max_dt_s > 0.0f &&
           isfinite(c->reversal_ref_min_rad_s) &&
           c->reversal_ref_min_rad_s >= 0.0f;
}

int GimbalSpeedPiInit(GimbalSpeedPi *pi, const GimbalSpeedPiConfig *config)
{
    if (pi == NULL || !ConfigValid(config)) return 0;
    /* Copy before clearing so initialization also accepts &pi->config. */
    GimbalSpeedPiConfig c = *config;
    memset(pi, 0, sizeof(*pi));
    pi->config = c;
    return 1;
}

void GimbalSpeedPiReset(GimbalSpeedPi *pi, uint8_t preserve_integral)
{
    if (pi == NULL) return;
    float i = 0.0f;
    if (preserve_integral && ConfigValid(&pi->config) && isfinite(pi->state.i))
        i = Clamp(pi->state.i, pi->config.i_limit);
    memset(&pi->state, 0, sizeof(pi->state));
    pi->state.i = i;
    pi->p = 0.0f;
    pi->error = 0.0f;
    pi->output = 0.0f;
    pi->integral_delta = 0.0f;
}

float GimbalSpeedPiStep(GimbalSpeedPi *pi, float measure_rad_s,
                       float ref_rad_s, float dt_s)
{
    if (pi == NULL || !ConfigValid(&pi->config) ||
        !isfinite(measure_rad_s) || !isfinite(ref_rad_s) ||
        !isfinite(dt_s) || dt_s <= 0.0f || dt_s > pi->config.max_dt_s ||
        !isfinite(pi->state.i) ||
        (pi->state.prev_error_valid && !isfinite(pi->state.prev_error)))
        return NAN;

    const GimbalSpeedPiConfig *c = &pi->config;
    GimbalSpeedPiState next = pi->state;
    next.i = Clamp(next.i, c->i_limit);
    uint32_t reset_count = pi->reversal_reset_count;
    float error = ref_rad_s - measure_rad_s;
    float p = c->kp * error;
    if (!isfinite(error) || !isfinite(p)) return NAN;

    int8_t direction = 0;
    if (ref_rad_s != 0.0f && fabsf(ref_rad_s) >= c->reversal_ref_min_rad_s)
        direction = ref_rad_s > 0.0f ? 1 : -1;
    if (direction != 0) {
        if (c->reset_i_on_ref_reversal && next.ref_direction != 0 &&
            direction != next.ref_direction &&
            ((direction > 0 && next.i < 0.0f) ||
             (direction < 0 && next.i > 0.0f))) {
            next.i = 0.0f;
            next.prev_error_valid = 0;
            if (reset_count < UINT32_MAX) ++reset_count;
        }
        next.ref_direction = direction;
    }

    float previous = next.prev_error_valid ? next.prev_error : error;
    float delta_i = c->ki * (0.5f * error + 0.5f * previous) * dt_s;
    float unbounded_i = next.i + delta_i;
    if (!isfinite(delta_i) || !isfinite(unbounded_i)) return NAN;
    float candidate_i = Clamp(unbounded_i, c->i_limit);
    float candidate_output = p + candidate_i;
    if (!isfinite(candidate_output)) return NAN;

    /* Reject only the part that worsens saturation. Existing I may always
     * unload, including the portion up to zero in a crossing step. */
    if (candidate_output > c->output_limit && candidate_i > next.i) {
        if (next.i < 0.0f)
            candidate_i = candidate_i < 0.0f ? candidate_i : 0.0f;
        else
            candidate_i = next.i;
    } else if (candidate_output < -c->output_limit && candidate_i < next.i) {
        if (next.i > 0.0f)
            candidate_i = candidate_i > 0.0f ? candidate_i : 0.0f;
        else
            candidate_i = next.i;
    }

    float output = Clamp(p + candidate_i, c->output_limit);
    if (!isfinite(output)) return NAN;
    float applied_delta = candidate_i - next.i;
    if (!isfinite(applied_delta)) return NAN;
    next.i = candidate_i;
    next.prev_error = error;
    next.prev_error_valid = 1;
    pi->state = next;
    pi->p = p;
    pi->error = error;
    pi->output = output;
    pi->integral_delta = applied_delta;
    pi->reversal_reset_count = reset_count;
    return output;
}

void GimbalSpeedPiApplyOutputLimit(GimbalSpeedPi *pi, float requested, float applied)
{
    if (!pi || !ConfigValid(&pi->config) || !isfinite(requested) || !isfinite(applied) ||
        !isfinite(pi->state.i) || !isfinite(pi->integral_delta)) return;
    float excess = requested-applied;
    if ((excess > 0 && pi->integral_delta > 0) || (excess < 0 && pi->integral_delta < 0)) {
        pi->state.i = Clamp(pi->state.i-pi->integral_delta, pi->config.i_limit);
        pi->integral_delta = 0;
        pi->output = Clamp(pi->p+pi->state.i, pi->config.output_limit);
    }
}
