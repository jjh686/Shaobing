/* [USB_SP 新增 01 开始] 新增与视觉M转置*R(q)*M一致的姿态、欧拉角速度及机械包络计算。 */
#include "gimbal_math.h"
#include <math.h>

#define PI_F 3.14159265358979323846f
static float Clamp(float x, float lo, float hi) { return fminf(fmaxf(x, lo), hi); }

float GimbalWrapRad(float angle)
{
    float a = remainderf(angle, 2.0f * PI_F);
    return a;
}

float GimbalWrapDeg(float angle) { return remainderf(angle, 360.0f); }

int GimbalRotationValid(const float m[9])
{
    for (int i = 0; i < 9; ++i) if (!isfinite(m[i])) return 0;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) {
            float dot = 0;
            for (int k = 0; k < 3; ++k) dot += m[k * 3 + i] * m[k * 3 + j];
            if (fabsf(dot - (i == j ? 1.0f : 0.0f)) > 0.002f) return 0;
        }
    float det = m[0]*(m[4]*m[8]-m[5]*m[7]) - m[1]*(m[3]*m[8]-m[5]*m[6]) +
                m[2]*(m[3]*m[7]-m[4]*m[6]);
    return fabsf(det - 1.0f) < 0.002f;
}

int GimbalPoseFromImu(const float q[4], const float gyro[3], const float m[9], GimbalPose *pose)
{
    float n = 0;
    for (int i = 0; i < 4; ++i) { if (!isfinite(q[i])) return 0; n += q[i]*q[i]; }
    for (int i = 0; i < 3; ++i) if (!isfinite(gyro[i])) return 0;
    if (n < 0.25f || n > 2.25f || !GimbalRotationValid(m)) return 0;
    float scale = 1.0f / sqrtf(n);
    float w=q[0]*scale, x=q[1]*scale, y=q[2]*scale, z=q[3]*scale;
    float r[9] = {1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                  2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                  2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)};
    float temp[9] = {0}, g[9] = {0}, omega[3] = {0};
    /* Match Solver::set_R_gimbal2world exactly: M^T R(q) M. */
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) for (int k=0;k<3;++k)
        temp[i*3+j] += r[i*3+k] * m[k*3+j];
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) for (int k=0;k<3;++k)
        g[i*3+j] += m[k*3+i] * temp[k*3+j];
    for (int i=0;i<3;++i) for (int k=0;k<3;++k) omega[i] += m[k*3+i] * gyro[k];
    pose->yaw = atan2f(g[3], g[0]);
    pose->pitch = asinf(Clamp(-g[6], -1, 1));
    pose->roll = atan2f(g[7], g[8]);
    float cp = cosf(pose->pitch), sr = sinf(pose->roll), cr = cosf(pose->roll);
    if (fabsf(cp) < 0.1f) return 0; /* Euler singularity is outside this gimbal's limits. */
    pose->pitch_vel = cr * omega[1] - sr * omega[2];
    pose->yaw_vel = (sr * omega[1] + cr * omega[2]) / cp;
    return isfinite(pose->yaw_vel) && isfinite(pose->pitch_vel);
}

int GimbalEncoderTarget(float target, float current, float encoder, float direction,
                        float lo, float hi, float max_error, float *result)
{
    if (!isfinite(target) || !isfinite(current) || !isfinite(encoder) ||
        !isfinite(lo) || !isfinite(hi) || !isfinite(max_error) || lo >= hi ||
        max_error <= 0 || (direction != 1 && direction != -1)) return 0;
    float error = GimbalWrapRad(target - current) * (180.0f / PI_F);
    error = Clamp(error, -max_error, max_error);
    /* encoder and result both use degrees relative to the SAME mechanical zero. */
    *result = Clamp(encoder + direction * error, lo, hi);
    return isfinite(*result);
}
/* [USB_SP 新增 01 结束] */
