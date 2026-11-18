/* [USB_SP 新增 01 开始] 新增故障/急停/人工恢复状态机接口。 */
#ifndef CONTROL_GUARD_H
#define CONTROL_GUARD_H
#include <stdint.h>

typedef struct {
    uint8_t armed, estop, saw_non_up;
} ControlGuard;

/* Fault/estop always wins. Recovery needs a real non-UP -> UP operation on
 * valid remote frames; boot/reconnection with UP held never arms the robot. */
int ControlGuardUpdate(ControlGuard *guard, int remote_online, int dial,
                       int recovery_up, int fault, int assert_level, int release_level);
#endif
/* [USB_SP 新增 01 结束] */
