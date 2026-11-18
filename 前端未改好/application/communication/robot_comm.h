/* [USB_SP 通信整理] 云台C板应用层通信总入口。 */
#ifndef ROBOT_COMM_H
#define ROBOT_COMM_H

/* 复用原命令类型，保留旧模块的数据布局和接口兼容性。 */
#include "master_process.h"

/* 由RobotCMDInit调用一次；返回值仅供旧VisionInit接口兼容使用。
 * 任务读取命令必须使用RobotCommGetVisionCommand的一致快照。 */
Vision_Recv_s *RobotCommInit(void);

/* 保留在INS姿态更新后调用；内部仍按VISION_TX_PERIOD_MS节流发送。
 * 未来导航/底盘转发不能直接加到这个高优先级调用路径里。 */
void RobotCommUpdate(void);

uint8_t RobotCommGetVisionCommand(Vision_Recv_s *command);
uint8_t RobotCommVisionIsOnline(void);
uint8_t RobotCommUsbIsConfigured(void);
void RobotCommSetVisionMode(uint8_t mode);
void RobotCommSetVisionBulletSpeed(Bullet_Speed_e speed);

#endif
