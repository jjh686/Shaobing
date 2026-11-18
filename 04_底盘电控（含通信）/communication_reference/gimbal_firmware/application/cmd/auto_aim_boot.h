/* [USB_SP 上电自瞄 新增 开始] 独立自主入口，不改变SP帧、PID、反馈和限位。 */
#ifndef AUTO_AIM_BOOT_H
#define AUTO_AIM_BOOT_H

#include "robot_def.h"

typedef enum {
    AUTO_AIM_DISABLED = 0,
    AUTO_AIM_WAIT_HARDWARE = 1,
    AUTO_AIM_WAIT_USB = 2,
    AUTO_AIM_WAIT_COMMAND = 3,
    AUTO_AIM_SETTLING = 4,
    AUTO_AIM_HOLD = 5,
    AUTO_AIM_TRACKING = 6,
    AUTO_AIM_DRIVER_FAULT = 7,
    AUTO_AIM_INSTALLATION_BLOCKED = 8
} AutoAimBootState;

/* 可在调试监视中设为0停止自主输出；设回1后重新等待新鲜命令稳定。
 * 不是新增USB指令，不会把SP mode0误用为急停或回零指令。
 */
extern volatile uint8_t gimbal_auto_aim_enabled;
extern volatile AutoAimBootState gimbal_auto_aim_state;

void AutoAimBootInit(void);
void AutoAimBootUpdate(Gimbal_Ctrl_Cmd_s *command);

#endif
/* [USB_SP 上电自瞄 新增 结束] */
