/* [USB_SP 通信整理] 旧Vision接口兼容层，实际收发逻辑集中在robot_comm.c。 */
#include "master_process.h"
#include "robot_comm.h"

Vision_Recv_s *VisionInit(UART_HandleTypeDef *unused)
{
    (void)unused;
    return RobotCommInit();
}

void VisionSend(void)
{
    RobotCommUpdate();
}

uint8_t VisionGetCommand(Vision_Recv_s *out)
{
    return RobotCommGetVisionCommand(out);
}

uint8_t VisionIsOnline(void)
{
    return RobotCommVisionIsOnline();
}

void VisionSetMode(uint8_t mode)
{
    RobotCommSetVisionMode(mode);
}

void VisionSetFlag(Enemy_Color_e color, Work_Mode_e mode, Bullet_Speed_e speed)
{
    (void)color;
    (void)mode;
    RobotCommSetVisionBulletSpeed(speed);
}

void VisionSetAltitude(float yaw, float pitch, float roll)
{
    /* 保留原来的兼容空操作，实际回传使用完整INS快照。 */
    (void)yaw;
    (void)pitch;
    (void)roll;
}
