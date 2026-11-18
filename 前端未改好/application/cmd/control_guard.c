/* [USB_SP 新增 01 开始] 新增人工恢复状态机：故障撤销使能，健康后非上档→上档恢复。 */
#include "control_guard.h"

int ControlGuardUpdate(ControlGuard *g, int online, int dial, int up, int fault,
                       int assert_level, int release_level)
{
    if (!online) { g->armed = 0; g->saw_non_up = 0; return 0; }
    if (dial >= assert_level) g->estop = 1;
    else if (dial <= release_level) g->estop = 0;
    if (fault || g->estop) {
        g->armed = 0;
        g->saw_non_up = 0;
        return 0;
    }
    if (!up) g->saw_non_up = 1;
    else if (g->saw_non_up) { g->armed = 1; g->saw_non_up = 0; }
    return g->armed;
}
/* [USB_SP 新增 01 结束] */
