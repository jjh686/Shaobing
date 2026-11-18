/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "spi.h"
#include "tim.h"
#include "user_lib.h"
#include "general_def.h"
/* [USB_SP 修改 01 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
#include <math.h>
#include <string.h>
/* [USB_SP 修改 01 结束] */

static INS_t INS;
/* [USB_SP 修改 02 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
static attitude_t attitude_data;
static INS_Snapshot_t published_sample;
/* [USB_SP 修改 02 结束] */
static IMU_Param_t IMU_Param;
static PIDInstance TempCtrl = {0};

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static float RefTemp = 40; // 恒温设定温度

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

static void IMUPWMSet(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

/**
 * @brief 温度控制
 *
 */
static void IMU_Temperature_Ctrl(void)
{
    PIDCalculate(&TempCtrl, BMI088.Temperature, RefTemp);
    IMUPWMSet(float_constrain(float_rounding(TempCtrl.Output), 0, UINT32_MAX));
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
/* [USB_SP 修改 03 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
static uint8_t InitQuaternion(float *init_q4)
/* [USB_SP 修改 03 结束] */
{
    float acc_init[3] = {0};
/* [USB_SP 修改 04 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
    uint8_t samples = 0;
/* [USB_SP 修改 04 结束] */
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
/* [USB_SP 修改 05 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
        if (!BMI088_ReadChecked(&BMI088)) { DWT_Delay(0.001f); continue; }
        ++samples;
/* [USB_SP 修改 05 结束] */
        acc_init[X] += BMI088.Accel[X];
        acc_init[Y] += BMI088.Accel[Y];
        acc_init[Z] += BMI088.Accel[Z];
        DWT_Delay(0.001);
    }
/* [USB_SP 修改 06 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
    if (samples < 90) return 0;
/* [USB_SP 修改 06 结束] */
    for (uint8_t i = 0; i < 3; ++i)
/* [USB_SP 修改 07 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
        acc_init[i] /= samples;
    float norm = sqrtf(acc_init[0]*acc_init[0] + acc_init[1]*acc_init[1] + acc_init[2]*acc_init[2]);
    init_q4[0] = 1; init_q4[1] = init_q4[2] = init_q4[3] = 0;
    if (!isfinite(norm) || norm < 1.0e-6f) return 0;
    for (int i=0; i<3; ++i) acc_init[i] /= norm;
    /* Shortest rotation a -> +Z. Explicit antiparallel case avoids normalizing
     * a zero cross product when the board is level or upside down. */
    if (acc_init[2] < -0.999999f) {
        init_q4[0] = 0; init_q4[1] = 1;
        return 1;
    }
    float w = 1.0f + acc_init[2], x = acc_init[1], y = -acc_init[0];
    norm = sqrtf(w*w + x*x + y*y);
    init_q4[0] = w/norm; init_q4[1] = x/norm; init_q4[2] = y/norm;
    return 1;
/* [USB_SP 修改 07 结束] */
}

attitude_t *INS_Init(void)
{
    if (!INS.init)
        INS.init = 1;
    else
/* [USB_SP 修改 08 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
        return &attitude_data;
/* [USB_SP 修改 08 结束] */

    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);

    while (BMI088Init(&hspi1, 1) != BMI088_NO_ERROR)
        ;
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;

    float init_quaternion[4] = {0};
/* [USB_SP 修改 09 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
    while (!InitQuaternion(init_quaternion)) DWT_Delay(0.01f);
/* [USB_SP 修改 09 结束] */
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
    // imu heat init
    PID_Init_Config_s config = {.MaxOut = 2000,
                                .IntegralLimit = 300,
                                .DeadBand = 0,
                                .Kp = 1000,
                                .Ki = 20,
                                .Kd = 0,
                                .Improve = 0x01}; // enable integratiaon limit
    PIDInit(&TempCtrl, &config);

    // noise of accel is relatively big and of high freq,thus lpf is used
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);
/* [USB_SP 修改 10 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
    return &attitude_data;
}

uint8_t INS_GetSnapshot(INS_Snapshot_t *snapshot)
{
    if (!snapshot) return 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    *snapshot = published_sample;
    __set_PRIMASK(primask);
    return snapshot->valid;
}

static void PublishSample(void)
{
    INS_Snapshot_t sample = {0};
    float norm = 0;
    sample.valid = 1;
    for (int i=0; i<4; ++i) {
        if (!isfinite(INS.q[i])) sample.valid = 0;
        norm += INS.q[i]*INS.q[i];
    }
    if (!isfinite(norm) || norm < 0.25f || norm > 2.25f) sample.valid = 0;
    if (sample.valid) {
        float scale = 1.0f / sqrtf(norm);
        for (int i=0; i<4; ++i) sample.q[i] = INS.q[i]*scale;
    }
    for (int i=0; i<3; ++i) {
        sample.gyro[i] = INS.Gyro[i];
        sample.attitude.Gyro[i] = INS.Gyro[i];
        sample.attitude.Accel[i] = INS.Accel[i];
        if (!isfinite(INS.Gyro[i]) || !isfinite(INS.Accel[i])) sample.valid = 0;
    }
    sample.attitude.Yaw = INS.Yaw;
    sample.attitude.Pitch = INS.Pitch;
    sample.attitude.Roll = INS.Roll;
    sample.attitude.YawTotalAngle = INS.YawTotalAngle;
    sample.tick = HAL_GetTick();
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    published_sample = sample;
    attitude_data = sample.attitude;
    __set_PRIMASK(primask);
/* [USB_SP 修改 10 结束] */
}

/* 注意以1kHz的频率运行此任务 */
void INS_Task(void)
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;

    // ins update
    if ((count % 1) == 0)
    {
/* [USB_SP 修改 11 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
        if (!BMI088_ReadChecked(&BMI088)) {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            published_sample.valid = 0; /* Never refresh the last good sample tick. */
            __set_PRIMASK(primask);
            IMUPWMSet(0);
            ++count;
            return; /* Do not integrate stale gyro or heat from a stale temperature. */
        }
/* [USB_SP 修改 11 结束] */

        INS.Accel[X] = BMI088.Accel[X];
        INS.Accel[Y] = BMI088.Accel[Y];
        INS.Accel[Z] = BMI088.Accel[Z];
        INS.Gyro[X] = BMI088.Gyro[X];
        INS.Gyro[Y] = BMI088.Gyro[Y];
        INS.Gyro[Z] = BMI088.Gyro[Z];

        // demo function,用于修正安装误差,可以不管,本demo暂时没用
        IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展,本demo暂时没用
        // INS.atanxz = -atan2f(INS.Accel[X], INS.Accel[Z]) * 180 / PI;
        // INS.atanyz = atan2f(INS.Accel[Y], INS.Accel[Z]) * 180 / PI;

        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.Gyro[X], INS.Gyro[Y], INS.Gyro[Z], INS.Accel[X], INS.Accel[Y], INS.Accel[Z], dt);

        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);

        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n

        INS.Yaw = QEKF_INS.Yaw;
        INS.Pitch = QEKF_INS.Pitch;
        INS.Roll = QEKF_INS.Roll;
        INS.YawTotalAngle = QEKF_INS.YawTotalAngle;

/* [USB_SP 修改 12 开始] 修复初始姿态退化、发布一致快照、拒绝读取失败样本，失败不刷新时间戳、不跑EKF并关加热。 */
        PublishSample();
/* [USB_SP 修改 12 结束] */
    }

    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl();
    }

    if ((count++ % 1000) == 0)
    {
        // 1Hz 可以加入monitor函数,检查IMU是否正常运行/离线
    }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
