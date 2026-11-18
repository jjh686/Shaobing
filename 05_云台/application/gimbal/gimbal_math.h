/* [USB_SP 新增 01 开始] 新增独立姿态数学/角度包装/限位接口，便于主机测试。 */
#ifndef GIMBAL_MATH_H
#define GIMBAL_MATH_H

typedef struct {
    float yaw, pitch, roll; /* rad, ZYX Euler angles in vision's gimbal frame */
    float yaw_vel, pitch_vel; /* rad/s, Euler rates, not raw body gyro axes */
} GimbalPose;

float GimbalWrapRad(float angle);
float GimbalWrapDeg(float angle);
int GimbalRotationValid(const float rotation[9]);
int GimbalPoseFromImu(const float q[4], const float gyro[3],
                      const float gimbal_to_imu[9], GimbalPose *pose);
int GimbalEncoderTarget(float target_rad, float current_rad, float encoder_deg,
                        float direction, float min_deg, float max_deg,
                        float max_error_deg, float *result_deg);

#endif
/* [USB_SP 新增 01 结束] */
