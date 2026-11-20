#ifndef CHASSIS_LINK_H
#define CHASSIS_LINK_H

#include <stdint.h>

/* 云台CAN1 ↔ 底盘CAN2：导航转发、遥控转发、底盘反馈。
 * 本模块不注册USB回调；USB唯一入口仍由RobotComm管理。 */

/* RobotCommInit内调用一次，保持原先CAN注册和USB初始化顺序。 */
void ChassisLinkInit(void);

/* OSTaskInit内调用：创建模块自有的普通优先级1 ms任务，重复调用不重复建任务。 */
void ChassisLinkStartTask(void);

/* USB中断入口：仅复制完整导航帧、记录接收时间，不执行CAN发送。 */
void ChassisLinkReceiveUsb(const uint8_t *bytes, unsigned length, uint32_t received_ms);

/* USB初始化/反初始化时请求清除导航会话，不中断遥控转发。 */
void ChassisLinkResetUsbSession(void);

/* 仅由模块任务或主机测试周期调用；不要放入main循环或INS高优先级路径。 */
void ChassisLinkUpdate(void);

#endif
