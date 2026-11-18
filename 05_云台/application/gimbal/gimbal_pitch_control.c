#include "gimbal_pitch_control.h"
#include <math.h>
#include <stddef.h>
#include <string.h>

#define RAD_PER_DEG 0.01745329251994329577f
static float Bound(float value, float lo, float hi)
{
    return fminf(fmaxf(value, lo), hi);
}
static float Approach(float value, float target, float step)
{
    return value + Bound(target-value, -step, step);
}
static int Valid(const GimbalPitchAssistConfig *c)
{
    return c && isfinite(c->max_speed_rad_s) && c->max_speed_rad_s > 0 &&
        isfinite(c->accel_rad_s2) && c->accel_rad_s2 > 0 &&
        isfinite(c->brake_rad_s2) && c->brake_rad_s2 > 0 &&
        isfinite(c->min_deg) && isfinite(c->max_deg) && c->min_deg < c->max_deg &&
        (c->pose_to_encoder_sign == 1 || c->pose_to_encoder_sign == -1) &&
        isfinite(c->max_dt_s) && c->max_dt_s > 0 &&
        isfinite(c->gravity_cos_nm) && isfinite(c->gravity_sin_nm) &&
        isfinite(c->gravity_zero_rad) && isfinite(c->gravity_ramp_nm_s) && c->gravity_ramp_nm_s > 0 &&
        isfinite(c->friction_positive_nm) && c->friction_positive_nm >= 0 &&
        isfinite(c->friction_negative_nm) && c->friction_negative_nm >= 0 &&
        isfinite(c->friction_ramp_nm_s) && c->friction_ramp_nm_s > 0 &&
        isfinite(c->friction_start_rad) && isfinite(c->friction_release_rad) &&
        c->friction_release_rad >= 0 && c->friction_start_rad > c->friction_release_rad &&
        isfinite(c->stopped_rad_s) && c->stopped_rad_s > 0 &&
        isfinite(c->friction_timeout_s) && c->friction_timeout_s > 0;
}
static int Ready(const GimbalPitchAssist *a, float dt)
{
    return a && Valid(&a->config) && isfinite(dt) && dt > 0 && dt <= a->config.max_dt_s;
}
int GimbalPitchAssistInit(GimbalPitchAssist *a, const GimbalPitchAssistConfig *config)
{
    if (!a || !Valid(config)) return 0;
    GimbalPitchAssistConfig copy = *config;
    memset(a, 0, sizeof(*a));
    a->config = copy;
    return 1;
}
void GimbalPitchAssistReset(GimbalPitchAssist *a, uint8_t preserve_support)
{
    if (!a) return;
    float gravity = preserve_support && isfinite(a->gravity_nm) ? a->gravity_nm : 0;
    a->speed_ref_rad_s = a->friction_nm = a->friction_elapsed_s = 0;
    a->friction_direction = 0;
    a->friction_finished = 0;
    a->gravity_nm = gravity;
}
float GimbalPitchSpeedReference(GimbalPitchAssist *a, float desired, float error,
                               float mechanical_deg, float dt)
{
    if (!Ready(a, dt) || !isfinite(desired) || !isfinite(error) ||
        !isfinite(mechanical_deg) || !isfinite(a->speed_ref_rad_s)) return NAN;
    const GimbalPitchAssistConfig *c = &a->config;
    float positive_distance = (c->pose_to_encoder_sign > 0 ? c->max_deg-mechanical_deg
                                                        : mechanical_deg-c->min_deg)*RAD_PER_DEG;
    float negative_distance = (c->pose_to_encoder_sign > 0 ? mechanical_deg-c->min_deg
                                                        : c->max_deg-mechanical_deg)*RAD_PER_DEG;
    float positive_cap = fminf(c->max_speed_rad_s,
        sqrtf(2*c->brake_rad_s2*fmaxf(0, fminf(error, positive_distance))));
    float negative_cap = fminf(c->max_speed_rad_s,
        sqrtf(2*c->brake_rad_s2*fmaxf(0, fminf(-error, negative_distance))));
    if (!isfinite(positive_cap) || !isfinite(negative_cap)) return NAN;
    float target = Bound(desired, -negative_cap, positive_cap);
    float next = Approach(a->speed_ref_rad_s, target, c->accel_rad_s2*dt);
    /* Braking/changed target may reduce the cap immediately. Safety caps take
     * precedence over acceleration smoothing; this is not a physical stop guarantee. */
    a->speed_ref_rad_s = Bound(next, -negative_cap, positive_cap);
    return a->speed_ref_rad_s;
}
float GimbalPitchCompensation(GimbalPitchAssist *a, float angle, float error,
                             float speed, float dt)
{
    if (!Ready(a, dt) || !isfinite(angle) || !isfinite(error) || !isfinite(speed) ||
        !isfinite(a->gravity_nm) || !isfinite(a->friction_nm) || !isfinite(a->friction_elapsed_s)) return NAN;
    const GimbalPitchAssistConfig *c = &a->config;
    float gravity_target = 0;
    if (c->gravity_cos_nm != 0 || c->gravity_sin_nm != 0) {
        float phase = angle-c->gravity_zero_rad;
        gravity_target = c->gravity_cos_nm*cosf(phase) + c->gravity_sin_nm*sinf(phase);
    }
    if (!isfinite(gravity_target)) return NAN;
    a->gravity_nm = Approach(a->gravity_nm, gravity_target, c->gravity_ramp_nm_s*dt);
    /* ---- 以下为"起动摩擦辅助"，【当前停用】：两个FRICTION_*_NM系数为0 ----
     * 它按"位置误差符号"给一次有界脉冲(0.25s窗口、一动就撤、同一方向不重复)，
     * 只适合破静摩擦的"第一下"。实机中真正拖慢的是运动中持续存在的摩擦，
     * 已由GimbalSpeedUpdate末尾的"速度前馈"GIMBAL_PITCH_SPEED_FF_NM取代。
     * 代码与单元测试保留，方便以后确认起动力矩偏大时重新启用。 */
    int8_t direction = error > 0 ? 1 : error < 0 ? -1 : 0;
    if (fabsf(error) <= c->friction_release_rad ||
        (a->friction_direction != 0 && a->friction_direction != direction)) {
        a->friction_direction = 0;
        a->friction_elapsed_s = 0;
        a->friction_finished = 0;
    }
    if (a->friction_direction == 0 && fabsf(error) >= c->friction_start_rad) {
        a->friction_direction = direction;
        a->friction_elapsed_s = 0;
        a->friction_finished = 0;
    }
    float friction_target = 0;
    if (a->friction_direction != 0) {
        a->friction_elapsed_s = fminf(a->friction_elapsed_s+dt, c->friction_timeout_s);
        if (fabsf(speed) > c->stopped_rad_s || a->friction_elapsed_s >= c->friction_timeout_s)
            a->friction_finished = 1;
        if (!a->friction_finished && fabsf(error) > c->friction_release_rad)
            friction_target = a->friction_direction > 0 ? c->friction_positive_nm
                                                       : -c->friction_negative_nm;
    }
    a->friction_nm = Approach(a->friction_nm, friction_target, c->friction_ramp_nm_s*dt);
    if (a->friction_nm*error < 0 || fabsf(error) <= c->friction_release_rad)
        a->friction_nm = 0; /* Never retain a kick pointing away from the target. */
    return a->gravity_nm+a->friction_nm;
}
