#include "gimbal_speed_pi.h"

#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); \
    exit(EXIT_FAILURE); } } while (0)

static void Near(float value, float expected, float tolerance)
{
    CHECK(isfinite(value));
    CHECK(fabsf(value - expected) <= tolerance);
}

static GimbalSpeedPiConfig Config(void)
{
    const GimbalSpeedPiConfig c = {
        .kp = 50.0f, .ki = 200.0f, .i_limit = 3000.0f,
        .output_limit = 20000.0f, .max_dt_s = 0.03f,
        .reset_i_on_ref_reversal = 0,
        .reversal_ref_min_rad_s = 0.013962634f
    };
    return c;
}

static GimbalSpeedPi NewPi(GimbalSpeedPiConfig c)
{
    GimbalSpeedPi p;
    CHECK(GimbalSpeedPiInit(&p, &c));
    return p;
}

static void ExpectRejected(GimbalSpeedPi *p, float measure, float ref, float dt)
{
    GimbalSpeedPi before;
    memcpy(&before, p, sizeof(before));
    CHECK(isnan(GimbalSpeedPiStep(p, measure, ref, dt)));
    CHECK(memcmp(&before, p, sizeof(before)) == 0);
}

static void SecondsAndTrapezoid(void)
{
    GimbalSpeedPi p = NewPi(Config());
    Near(GimbalSpeedPiStep(&p, 0.0f, 0.5f, 0.01f), 26.0f, 0.0001f);
    Near(GimbalSpeedPiStep(&p, 0.0f, 1.5f, 0.01f), 78.0f, 0.0001f);
    Near(p.state.i, 3.0f, 0.0001f);
    Near(p.integral_delta, 2.0f, 0.0001f);
    GimbalSpeedPiReset(&p, 0);
    for (int i = 0; i < 1000; ++i)
        CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 0.5f, 0.001f)));
    Near(p.state.i, 100.0f, 0.002f);
    Near(p.output, 125.0f, 0.002f);
}

static void IrregularTime(void)
{
    GimbalSpeedPi p = NewPi(Config());
    const float times[] = {0.001f, 0.009f, 0.02f};
    for (unsigned i = 0; i < sizeof(times) / sizeof(times[0]); ++i)
        CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 0.5f, times[i])));
    Near(p.state.i, 3.0f, 0.0001f);
}

static void CandidateSaturation(void)
{
    GimbalSpeedPiConfig c = Config();
    c.kp = 20.0f; c.ki = 10.0f; c.i_limit = 50.0f; c.output_limit = 10.0f;
    GimbalSpeedPi p = NewPi(c);
    for (int i = 0; i < 50; ++i)
        Near(GimbalSpeedPiStep(&p, 0.0f, 1.0f, 0.01f), 10.0f, 0.0001f);
    Near(p.state.i, 0.0f, 0.0001f);
    Near(p.integral_delta, 0.0f, 0.0001f);
    GimbalSpeedPiReset(&p, 0);
    for (int i = 0; i < 50; ++i)
        Near(GimbalSpeedPiStep(&p, 0.0f, -1.0f, 0.01f), -10.0f, 0.0001f);
    Near(p.state.i, 0.0f, 0.0001f);
    Near(GimbalSpeedPiStep(&p, 0.0f, 0.1f, 0.01f), 1.955f, 0.0001f);
    Near(GimbalSpeedPiStep(&p, 0.0f, 0.1f, 0.01f), 1.965f, 0.0001f);
}

static void UnloadWhileSaturated(void)
{
    GimbalSpeedPiConfig c = Config();
    c.kp = 20.0f; c.ki = 10.0f; c.i_limit = 50.0f; c.output_limit = 10.0f;
    GimbalSpeedPi p = NewPi(c);
    p.state.i = -5.0f;
    Near(GimbalSpeedPiStep(&p, 0.0f, 1.0f, 0.01f), 10.0f, 0.0001f);
    Near(p.state.i, -4.9f, 0.0001f);
    GimbalSpeedPiReset(&p, 0);
    p.state.i = 5.0f;
    Near(GimbalSpeedPiStep(&p, 0.0f, -1.0f, 0.01f), -10.0f, 0.0001f);
    Near(p.state.i, 4.9f, 0.0001f);
}

static void UnloadCrossingZero(void)
{
    GimbalSpeedPiConfig c = Config();
    c.kp = 20.0f; c.ki = 10.0f; c.i_limit = 50.0f; c.output_limit = 10.0f;
    GimbalSpeedPi p = NewPi(c);
    p.state.i = -0.05f;
    Near(GimbalSpeedPiStep(&p, 0.0f, 1.0f, 0.01f), 10.0f, 0.0001f);
    Near(p.state.i, 0.0f, 0.0001f);
    Near(p.integral_delta, 0.05f, 0.0001f);
    GimbalSpeedPiReset(&p, 0);
    p.state.i = 0.05f;
    Near(GimbalSpeedPiStep(&p, 0.0f, -1.0f, 0.01f), -10.0f, 0.0001f);
    Near(p.state.i, 0.0f, 0.0001f);
    Near(p.integral_delta, -0.05f, 0.0001f);
}

static void IntegralLimitAndLiveReclamp(void)
{
    GimbalSpeedPiConfig c = Config();
    c.kp = 0.0f; c.ki = 100.0f; c.i_limit = 1.0f;
    GimbalSpeedPi p = NewPi(c);
    for (int i = 0; i < 100; ++i)
        CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 1.0f, 0.03f)));
    Near(p.state.i, 1.0f, 0.0001f);
    p.config.i_limit = 0.25f;
    p.config.ki = 0.0f;
    Near(GimbalSpeedPiStep(&p, 0.0f, 0.0f, 0.01f), 0.25f, 0.0001f);
    Near(p.integral_delta, 0.0f, 0.0001f);
}

static void YawReversal(void)
{
    GimbalSpeedPiConfig c = Config();
    c.reset_i_on_ref_reversal = 1;
    GimbalSpeedPi p = NewPi(c);
    CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 1.0f, 0.01f)));
    p.state.i = 100.0f;
    Near(GimbalSpeedPiStep(&p, 0.0f, -1.0f, 0.01f), -52.0f, 0.0001f);
    Near(p.state.i, -2.0f, 0.0001f);
    Near(p.integral_delta, -2.0f, 0.0001f);
    CHECK(p.reversal_reset_count == 1);
    CHECK(p.state.ref_direction == -1);
    p.state.i = -20.0f;
    CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 1.0f, 0.01f)));
    CHECK(p.reversal_reset_count == 2);
}

static void YawKeepsAlignedIntegral(void)
{
    GimbalSpeedPiConfig c = Config();
    c.reset_i_on_ref_reversal = 1; c.ki = 0.0f;
    GimbalSpeedPi p = NewPi(c);
    CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 1.0f, 0.01f)));
    p.state.i = -20.0f;
    Near(GimbalSpeedPiStep(&p, 0.0f, -1.0f, 0.01f), -70.0f, 0.0001f);
    Near(p.state.i, -20.0f, 0.0001f);
    CHECK(p.reversal_reset_count == 0);
}

static void SignificantReferenceOnly(void)
{
    GimbalSpeedPiConfig c = Config();
    c.reset_i_on_ref_reversal = 1; c.ki = 0.0f; c.reversal_ref_min_rad_s = 0.1f;
    GimbalSpeedPi p = NewPi(c);
    CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 0.5f, 0.01f)));
    p.state.i = 100.0f;
    CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, -0.05f, 0.01f)));
    CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 0.0f, 0.01f)));
    Near(p.state.i, 100.0f, 0.0001f);
    CHECK(p.reversal_reset_count == 0 && p.state.ref_direction == 1);
    CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, -0.5f, 0.01f)));
    Near(p.state.i, 0.0f, 0.0001f);
    CHECK(p.reversal_reset_count == 1);
}

static void PitchKeepsLoadBias(void)
{
    GimbalSpeedPiConfig c = Config();
    GimbalSpeedPi p = NewPi(c);
    CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 1.0f, 0.01f)));
    p.state.i = 100.0f;
    Near(GimbalSpeedPiStep(&p, 0.0f, -1.0f, 0.01f), 50.0f, 0.0001f);
    Near(p.state.i, 100.0f, 0.0001f);
    CHECK(p.reversal_reset_count == 0);
    GimbalSpeedPiReset(&p, 1);
    Near(p.state.i, 100.0f, 0.0001f);
    CHECK(!p.state.prev_error_valid && p.state.ref_direction == 0);
    Near(GimbalSpeedPiStep(&p, 0.0f, 0.0f, 0.01f), 100.0f, 0.0001f);
}

static void ResetAndCounter(void)
{
    GimbalSpeedPi p = NewPi(Config());
    p.state.i = 10.0f; p.state.prev_error = 1.0f; p.state.prev_error_valid = 1;
    p.state.ref_direction = 1; p.reversal_reset_count = 7;
    GimbalSpeedPiReset(&p, 1);
    Near(p.state.i, 10.0f, 0.0001f);
    Near(p.integral_delta, 0.0f, 0.0001f);
    CHECK(!p.state.prev_error_valid && p.state.ref_direction == 0);
    CHECK(p.reversal_reset_count == 7);
    GimbalSpeedPiReset(&p, 0);
    Near(p.state.i, 0.0f, 0.0001f);
    CHECK(p.reversal_reset_count == 7);
    CHECK(GimbalSpeedPiInit(&p, &p.config));
    CHECK(p.reversal_reset_count == 0);
}

static void InvalidTimingAndData(void)
{
    GimbalSpeedPi p = NewPi(Config());
    CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 1.0f, 0.01f)));
    ExpectRejected(&p, 0.0f, 1.0f, 0.0f);
    ExpectRejected(&p, 0.0f, 1.0f, -0.001f);
    ExpectRejected(&p, 0.0f, 1.0f, 0.031f);
    ExpectRejected(&p, 0.0f, 1.0f, NAN);
    ExpectRejected(&p, 0.0f, 1.0f, INFINITY);
    ExpectRejected(&p, NAN, 1.0f, 0.001f);
    ExpectRejected(&p, 0.0f, INFINITY, 0.001f);
    CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 1.0f, p.config.max_dt_s)));
    p.state.i = NAN;
    ExpectRejected(&p, 0.0f, 1.0f, 0.001f);
}

static void InvalidConfiguration(void)
{
    GimbalSpeedPi p = NewPi(Config());
    GimbalSpeedPi before;
    memcpy(&before, &p, sizeof(before));
    GimbalSpeedPiConfig invalid = Config();
    invalid.kp = -1.0f;
    CHECK(!GimbalSpeedPiInit(&p, &invalid));
    CHECK(memcmp(&before, &p, sizeof(p)) == 0);
    p.config.ki = NAN;
    ExpectRejected(&p, 0.0f, 1.0f, 0.001f);
    p.config = Config(); p.config.output_limit = 0.0f;
    ExpectRejected(&p, 0.0f, 1.0f, 0.001f);
    CHECK(!GimbalSpeedPiInit(NULL, &invalid));
    CHECK(!GimbalSpeedPiInit(&p, NULL));
    CHECK(isnan(GimbalSpeedPiStep(NULL, 0.0f, 0.0f, 0.001f)));
    GimbalSpeedPiReset(NULL, 0);
}

static void OverflowDoesNotCommitReversal(void)
{
    GimbalSpeedPiConfig c = Config();
    c.reset_i_on_ref_reversal = 1;
    GimbalSpeedPi p = NewPi(c);
    CHECK(isfinite(GimbalSpeedPiStep(&p, 0.0f, 1.0f, 0.01f)));
    p.state.i = 100.0f;
    ExpectRejected(&p, FLT_MAX, -FLT_MAX, 0.001f);
    p.config.kp = 0.0f; p.config.ki = FLT_MAX;
    ExpectRejected(&p, 0.0f, -FLT_MAX, 0.01f);
    CHECK(p.reversal_reset_count == 0);
    Near(p.state.i, 100.0f, 0.0001f);
}

int main(void)
{
    void (*tests[])(void) = {
        SecondsAndTrapezoid, IrregularTime, CandidateSaturation,
        UnloadWhileSaturated, UnloadCrossingZero, IntegralLimitAndLiveReclamp,
        YawReversal, YawKeepsAlignedIntegral, SignificantReferenceOnly,
        PitchKeepsLoadBias, ResetAndCounter, InvalidTimingAndData,
        InvalidConfiguration, OverflowDoesNotCommitReversal
    };
    const unsigned count = (unsigned)(sizeof(tests) / sizeof(tests[0]));
    for (unsigned i = 0; i < count; ++i) tests[i]();
    printf("gimbal speed PI: %u test groups passed\n", count);
    return EXIT_SUCCESS;
}
