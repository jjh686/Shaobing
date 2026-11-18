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
#endif
/* [USB_SP 修改 01 结束] */
