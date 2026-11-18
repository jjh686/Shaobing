/* [USB_SP 上电自瞄 新增 开始]
 * CMD的可选自主目标来源：不读取遥控器，不修改电机环路或机械限位。
 * 上电先回传mode1请求视觉；电脑未启动时一直等待，不锁死在遥控解锁流程。
 */
#include "auto_aim_boot.h"
#include "gimbal.h"
#include "robot_comm.h" /* [USB_SP 通信整理] 不直接依赖USB驱动。 */

volatile uint8_t gimbal_auto_aim_enabled = 1;
volatile AutoAimBootState gimbal_auto_aim_state = AUTO_AIM_WAIT_HARDWARE;

static uint8_t stream_started, holding;
static uint32_t stable_since, stream_session;
static float hold_yaw, hold_pitch;

void AutoAimBootInit(void)
{
    gimbal_auto_aim_enabled = 1;
    gimbal_auto_aim_state = AUTO_AIM_WAIT_HARDWARE;
    stream_started = holding = 0;
    stable_since = stream_session = 0;
    hold_yaw = hold_pitch = 0;
}

/* 所有等待/停止路径都撤销本次使能；恢复时必须重新积累有效帧时间。 */
static void WaitFor(AutoAimBootState state)
{
    stream_started = holding = 0;
    gimbal_auto_aim_state = state;
}

void AutoAimBootUpdate(Gimbal_Ctrl_Cmd_s *command)
{
    GimbalState state;
    Vision_Recv_s rx = {0};
    GimbalGetState(&state);
    int have = RobotCommGetVisionCommand(&rx);
    uint32_t now = HAL_GetTick();
    int selected = GIMBAL_VISION_ENABLE && gimbal_auto_aim_enabled;

    /* 请求计算不等待电脑命令，避免双方互相等候；安装门控为0仍可联调收发。
     * 电机故障时停止请求控制，但姿态回传仍由RobotCommUpdate持续提供。
     */
    RobotCommSetVisionMode(selected && state.pose_valid && !state.driver_fault ? 1 : 0);
    *command = (Gimbal_Ctrl_Cmd_s){.gimbal_mode = GIMBAL_ZERO_FORCE};
    if (state.pose_valid) {
        command->yaw = state.yaw_continuous;
        command->pitch = state.pose.pitch;
    }

    if (!selected) { WaitFor(AUTO_AIM_DISABLED); return; }
    /* 不自动清除驱动器锁存故障；检修后复位C板即可重新走自主启动。 */
    if (state.driver_fault) { WaitFor(AUTO_AIM_DRIVER_FAULT); return; }
    if (!state.pose_valid || !state.motors_ready) {
        WaitFor(AUTO_AIM_WAIT_HARDWARE); return;
    }
    if (!GIMBAL_INSTALLATION_CONFIRMED) {
        WaitFor(AUTO_AIM_INSTALLATION_BLOCKED); return;
    }
    if (!RobotCommUsbIsConfigured()) { WaitFor(AUTO_AIM_WAIT_USB); return; }
    if (!have || (uint32_t)(now - rx.received_tick) > VISION_RX_TIMEOUT_MS) {
        WaitFor(AUTO_AIM_WAIT_COMMAND); return;
    }

    /* 超时、停用、硬件不健康或USB重新枚举后，重新等连续有效输入。
     * 会话代数能识别两个CMD周期之间发生的拔插，不沿用旧目标。
     * 时间差使用无符号减法，兼容HAL毫秒计时回绕。
     */
    if (!stream_started || stream_session != rx.session) {
        stream_started = 1;
        stream_session = rx.session;
        stable_since = now;
        holding = 0;
    }
    if ((uint32_t)(now - stable_since) < GIMBAL_AUTO_AIM_SETTLE_MS) {
        gimbal_auto_aim_state = AUTO_AIM_SETTLING;
        return;
    }

    if (rx.mode != 0) {
        /* mode1/2均只提供云台角度；发射始终由CMD公共尾部强制关闭。 */
        holding = 0;
        command->yaw = rx.yaw;
        command->pitch = rx.pitch;
        command->gimbal_mode = GIMBAL_VISION_MODE;
        gimbal_auto_aim_state = AUTO_AIM_TRACKING;
    } else {
        /* mode0表示没有有效目标。在进入等待时锁定实测姿态，不能跟随零角度。
         * 之后固定该目标做保持，而非每周期随动修改目标。
         */
        if (!holding) {
            hold_yaw = state.yaw_continuous;
            hold_pitch = state.pose.pitch;
            holding = 1;
        }
        command->yaw = hold_yaw;
        command->pitch = hold_pitch;
        command->gimbal_mode = GIMBAL_GYRO_MODE;
        gimbal_auto_aim_state = AUTO_AIM_HOLD;
    }
}
/* [USB_SP 上电自瞄 新增 结束] */
