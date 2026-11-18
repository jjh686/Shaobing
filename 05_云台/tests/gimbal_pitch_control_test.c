#include "gimbal_pitch_control.h"
#include "gimbal_speed_pi.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void Near(float actual, float expected)
{
    assert(isfinite(actual) && fabsf(actual-expected) < 0.0001f);
}
static GimbalPitchAssist Fixture(void)
{
    GimbalPitchAssistConfig c = {
        .max_speed_rad_s=1.5f, .accel_rad_s2=6, .brake_rad_s2=3,
        .min_deg=-20, .max_deg=20, .pose_to_encoder_sign=1, .max_dt_s=0.03f,
        .gravity_ramp_nm_s=5, .friction_ramp_nm_s=2,
        .friction_start_rad=0.003f, .friction_release_rad=0.001f,
        .stopped_rad_s=0.02f, .friction_timeout_s=0.25f,
    };
    GimbalPitchAssist a;
    assert(GimbalPitchAssistInit(&a,&c));
    return a;
}
static void GravityAndSupportHandover(void)
{
    GimbalPitchAssist a=Fixture();
    for (int i=0;i<100;++i) Near(GimbalPitchCompensation(&a,0,0.1f,0,0.01f),0);
    a.config.gravity_cos_nm=-0.5f;
    Near(GimbalPitchCompensation(&a,0,0.1f,0,0.01f),-0.05f);
    for (int i=0;i<20;++i) GimbalPitchCompensation(&a,0,0.1f,0,0.01f);
    Near(a.gravity_nm,-0.5f);
    Near(GimbalPitchCompensation(&a,0,-0.1f,0,0.01f),-0.5f); /* Reversing target does not flip gravity. */
    GimbalPitchAssistReset(&a,1); Near(a.gravity_nm,-0.5f);
    GimbalPitchAssistReset(&a,0); Near(a.gravity_nm,0);
    a.config.gravity_cos_nm=0; a.config.gravity_sin_nm=0.4f;
    for (int i=0;i<20;++i) GimbalPitchCompensation(&a,1.5707963268f,0,0,0.01f);
    Near(a.gravity_nm,0.4f);
    assert(isnan(GimbalPitchCompensation(&a,NAN,0,0,0.01f)));
    assert(isnan(GimbalPitchCompensation(&a,0,0,0,0.04f)));
}
static void FiniteDirectionalBreakaway(void)
{
    GimbalPitchAssist a=Fixture();
    a.config.friction_positive_nm=0.2f; a.config.friction_negative_nm=0.1f;
    Near(GimbalPitchCompensation(&a,0,0.01f,0,0.01f),0.02f);
    for (int i=0;i<15;++i) GimbalPitchCompensation(&a,0,0.01f,0,0.01f);
    Near(a.friction_nm,0.2f);
    for (int i=0;i<200;++i) GimbalPitchCompensation(&a,0,0.01f,0,0.01f);
    Near(a.friction_nm,0); assert(a.friction_finished); /* No periodic kicks into a jam. */
    GimbalPitchCompensation(&a,0,-0.01f,0,0.01f);
    assert(a.friction_nm<0);
    for (int i=0;i<10;++i) GimbalPitchCompensation(&a,0,-0.01f,0,0.01f);
    Near(a.friction_nm,-0.1f);
    GimbalPitchCompensation(&a,0,-0.01f,0.1f,0.01f);
    for (int i=0;i<10;++i) GimbalPitchCompensation(&a,0,-0.01f,0.1f,0.01f);
    Near(a.friction_nm,0); /* Moving ends this attempt. */
    GimbalPitchCompensation(&a,0,0,0,0.01f);
    assert(!a.friction_finished && a.friction_direction==0);
    GimbalPitchCompensation(&a,0,0.01f,0,0.01f);
    assert(a.friction_nm>0);
    GimbalPitchCompensation(&a,0,-0.01f,0,0.01f);
    assert(a.friction_nm<=0); /* A reversal never retains the old outward kick. */
}
static void DownstreamAntiWindup(void)
{
    GimbalSpeedPiConfig c={.kp=2,.ki=10,.i_limit=100,.output_limit=200,.max_dt_s=0.03f};
    GimbalSpeedPi p;
    for (int sign=-1;sign<=1;sign+=2) {
        assert(GimbalSpeedPiInit(&p,&c));
        p.state.i=5*(float)sign;
        GimbalSpeedPiStep(&p,0,(float)sign,0.01f);
        Near(p.state.i,5.1f*(float)sign);
        GimbalSpeedPiApplyOutputLimit(&p,12*(float)sign,10*(float)sign);
        Near(p.state.i,5*(float)sign); /* Existing support survives. */
        GimbalSpeedPiStep(&p,0,-(float)sign,0.01f); /* Trapezoid first crosses zero error. */
        GimbalSpeedPiStep(&p,0,-(float)sign,0.01f);
        float unloaded=p.state.i;
        GimbalSpeedPiApplyOutputLimit(&p,12*(float)sign,10*(float)sign);
        Near(p.state.i,unloaded); /* Always permit unloading I. */
    }
}
static void SpeedAndStoppingCaps(void)
{
    GimbalPitchAssist a=Fixture();
    Near(GimbalPitchSpeedReference(&a,10,0.5f,0,0.01f),0.06f);
    for (int i=0;i<100;++i) assert(GimbalPitchSpeedReference(&a,10,0.5f,0,0.01f)<=1.5001f);
    float close=GimbalPitchSpeedReference(&a,10,0.001f,0,0.01f);
    assert(close<=sqrtf(0.006f)+0.0001f);
    Near(GimbalPitchSpeedReference(&a,10,0,0,0.01f),0);
    for (int sign=-1;sign<=1;sign+=2) {
        a.config.pose_to_encoder_sign=(float)sign;
        for (int direction=-1;direction<=1;direction+=2) {
            GimbalPitchAssistReset(&a,0);
            float edge=20*(float)(sign*direction);
            Near(GimbalPitchSpeedReference(&a,10*(float)direction,0.5f*(float)direction,edge,0.01f),0);
            assert(GimbalPitchSpeedReference(&a,-10*(float)direction,-0.5f*(float)direction,edge,0.01f)*(float)direction<0);
        }
    }
    assert(isnan(GimbalPitchSpeedReference(&a,NAN,0,0,0.01f)));
    assert(isnan(GimbalPitchSpeedReference(&a,0,0,0,0.04f)));
}
int main(void)
{
    GravityAndSupportHandover(); FiniteDirectionalBreakaway();
    DownstreamAntiWindup(); SpeedAndStoppingCaps();
    puts("pitch assistance: gravity/sign, support handover, bounded breakaway, downstream antiwindup, speed/braking/edge caps passed");
    return 0;
}
