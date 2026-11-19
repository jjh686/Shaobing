/* [USB_SP 新增 01 开始] 集中配置视觉接管开关、安装门控、选择档位、方向、限位、双环参数及超时。 */
#ifndef GIMBAL_CONFIG_H
#define GIMBAL_CONFIG_H
/* Gimbal-only commissioning: retain chassis/shooter sources, do not run them. */
#define GIMBAL_ONLY_BUILD 1
#if GIMBAL_ONLY_BUILD != 1
#error This adapter is gimbal-only; restore a chassis CMD before enabling chassis.
#endif
/* MUST match configs/live_usb_sp.yaml R_gimbal2imubody on the vision PC.
 * Retained hardware trace: encoder pitch follows IMU X rotation (old roll),
 * not IMU Y (old pitch). Map gimbal +Y to IMU +X and preserve vertical +Z.
 * M = Rz(-90deg), used for BOTH quaternion pose and gyro. This corrects the
 * dominant installation axis; small mounting errors still need calibration.
 * QEKF's legacy Euler field names are not the coordinate definition. */
#define GIMBAL_TO_IMU_ROTATION {0,1,0,-1,0,0,0,0,1}
#ifndef GIMBAL_INSTALLATION_CONFIRMED
#define GIMBAL_INSTALLATION_CONFIRMED 1
#endif
/* [新增：与原遥控共存] 0=只用遥控，USB照常收发；1=允许指定档位的视觉接管。
 * 默认保留原项目“自瞄未启用”的状态。电控确认后只改本宏，不用删遥控代码。
 */
#ifndef GIMBAL_VISION_ENABLE
#define GIMBAL_VISION_ENABLE 1
#endif
/* [USB_SP 上电自瞄 新增 开始]
 * 1=上电自主请求视觉，不初始化/依赖遥控器；0=使用原遥控与视觉共存流程。
 * 自主模式仍受 GIMBAL_VISION_ENABLE、安装确认、姿态/电机健康和USB超时约束。
 * 电脑可稍后手动启动视觉；连续收到有效命令300ms后才允许电机工作。
 */
#ifndef GIMBAL_AUTO_AIM_ON_BOOT
#define GIMBAL_AUTO_AIM_ON_BOOT 1
#endif
#ifndef GIMBAL_AUTO_AIM_SETTLE_MS
#define GIMBAL_AUTO_AIM_SETTLE_MS 300u
#endif
#if (GIMBAL_AUTO_AIM_ON_BOOT != 0 && GIMBAL_AUTO_AIM_ON_BOOT != 1) || \
    GIMBAL_AUTO_AIM_SETTLE_MS < 100u || GIMBAL_AUTO_AIM_SETTLE_MS > 5000u
#error Invalid autonomous aim startup configuration.
#endif
/* [USB_SP 上电自瞄 新增 结束] */
/* 原文件头说MID但实际分支是UP；默认按实际UP。0=上、1=中、2=下，可集中修改。 */
#ifndef GIMBAL_VISION_SWITCH_POSITION
#define GIMBAL_VISION_SWITCH_POSITION 0
#endif
#if (GIMBAL_VISION_ENABLE != 0 && GIMBAL_VISION_ENABLE != 1) || \
    GIMBAL_VISION_SWITCH_POSITION < 0 || GIMBAL_VISION_SWITCH_POSITION > 2
#error Invalid vision enable or SBUS switch position.
#endif
/* 保留原遥控每5ms的0.001度/摇杆计数，在CMD统一转换为rad。 */
#define GIMBAL_REMOTE_DEG_PER_COUNT 0.001f
/* Final actuator output signs, NOT target-only motor_reverse_flag. */
#define GIMBAL_YAW_OUTPUT_SIGN 1
#define GIMBAL_PITCH_OUTPUT_SIGN 1
#define GIMBAL_YAW_POSE_TO_ENCODER_SIGN 1.0f
#define GIMBAL_PITCH_POSE_TO_ENCODER_SIGN 1.0f
/* Temporary envelope relative to FIRST real CAN angle after power-on.
 * Start physically centred. NOT calibrated stops. No old 927/6825 assumed. */
/* 相对上电位置，仅在上电姿态为行程中心时成立。
 * 这两个值同时是 GimbalGetState 的健康判定阈值([min-1, max+1])，
 * 设超过机械止挡会撞限位，设小于实际行程会中途失能。
 * Yaw已放宽到±360：本机yaw为滑环、无机械止挡，允许整周旋转（每侧一圈）。
 * 多圈能力本身由 continuous_yaw 逐周期解包提供，这里只放开包络与健康阈值
 * ([min-1,max+1]=±361)，仍保留兜底防止目标跳变/姿态异常导致无限累加。
 * 注意视觉端若仍按±180发绝对角，跨±180时靠 GimbalWrapRad 走最短路，不会绕圈。
 * Pitch仍为±20：上下是硬机械止挡，不能按Yaw放大。 */
#define GIMBAL_YAW_MIN_DEG (-360.0f)
#define GIMBAL_YAW_MAX_DEG 360.0f
#define GIMBAL_PITCH_MIN_DEG (-20.0f)
#define GIMBAL_PITCH_MAX_DEG 20.0f
/* Separate normal soft limits from fault thresholds. Defaults preserve the
 * former min-1/max+1 fault envelope; calibrate against the real assembly. */
#define GIMBAL_PITCH_HARD_MIN_DEG (-21.0f)
#define GIMBAL_PITCH_HARD_MAX_DEG 21.0f
#define GIMBAL_VISION_MAX_ERROR_DEG 5.0f
#define GIMBAL_RAD_PER_DEG 0.01745329251994329577f
/* tracking_v1: explicit units; numerical gains remain the original baseline.
 * The reference robot uses different time/output conventions: do NOT copy its
 * PID numbers here. These gains have NOT been tuned for this mechanism. */
#define GIMBAL_YAW_ANGLE_KP 8.0f
#define GIMBAL_PITCH_ANGLE_KP 25.0f /* User-confirmed commissioning baseline. */
/* 0.2°：死区内 PIDCalculate 输出0(controller.c:202)，用于止住"目标停住后
 * 位置环继续给60~100°/s修正"造成的到点回摆。0.1°时零点附近仍会反推一次。
 * 代价是稳态最多偏0.2°；若死区边缘出现小幅来回则缩回0.15°。
 * 注意该宏也被 GIMBAL_PITCH_REVERSAL_REF_RAD_S 借用，但pitch的
 * reset_i_on_ref_reversal=0，该阈值不生效，pitch不受影响。 */
#define GIMBAL_YAW_ANGLE_DEADBAND_RAD (0.2f * GIMBAL_RAD_PER_DEG)
/* 200*1 = 200 command/(rad/s)。原50太小：满误差(约8.7rad/s)只给435 counts(1.5%)，
 * 推不动GM6020且起步要靠积分爬(约3654 counts/s)，实测有0.3秒反应延迟、两个方向
 * 都跟不上(实际速度只有指令的0.6~2%)。200时起步延迟消失、跟随正常，实测比120
 * 更跟手，故最终定在200。代价是目标停住时会冲过头回摆两下——那属于位置环
 * (gimbal.c:316的Kp=200)缺少阻尼，不要再回退速度环来压制。
 * 若出现高频嗡嗡则退160。 */
#define GIMBAL_YAW_SPEED_KP 200.0f /* CAN command / (rad/s) */
/* Dedicated controller currently implements PI (speed Kd must remain 0).
 * 900*0.0005 = 0.45 Nm/(rad/s)。历史：500时起步要等积分爬约5秒；2500消除了
 * 该延迟，但实机运动时出现高频抖动——速度反馈pitch_speed取IMU陀螺原始角速度，
 * 全链路无低通，P把减速箱/结构的振动一并放大成力矩，形成自激。
 * 1500 实测失败(抖动更重)，证实瓶颈是高频自激而非刹车力不足；1000→900 实测
 * 抖动减轻，故本轮停在900。800 试过，与900差别不明显，回900以保持与上一版
 * 板内固件一致，便于单独验证重力前馈这一项。提速不要再靠抬Kp。 */
#define GIMBAL_PITCH_SPEED_KP 900.0f
/* 420 = 3000/2500*350，使Yaw与Pitch达到各自的I上限所需时间相同。 */
#define GIMBAL_YAW_SPEED_KI 420.0f
/* Single-parameter follow-up: Ki=200 reduced oscillation but the user reports
 * frequent missed/late pitch motion. Use 500 to build support 2.5x faster at
 * the same speed error; retain the speed/braking limits and 1 Nm I capacity.
 * Physical tracking/stability and gravity remain uncalibrated. */
#define GIMBAL_PITCH_SPEED_KI 500.0f
/* limit on I output, NOT error sum. 稳态巡航时P项接近0，全靠I出力，
 * 原3000/2500只能维持约200°/s；提高后可覆盖外环500°/s。
 * 只影响最终可用力矩，不改变积分爬升速率，因此不增加起步延迟。 */
#define GIMBAL_YAW_SPEED_I_LIMIT 6000.0f
/* 1200*0.0005 = 0.60 Nm。重力由下面的前馈接管后，本值决定"抵消重力后还剩多少
 * 机动余量"：P在5°截断下最多贡献0.30Nm，所以总余量=0.30+本值。
 * 实测600(=0.30Nm)时：16°处重力0.57Nm，P+I仅0.61Nm，净力矩0.04Nm，
 * 轴只能以约1°/s爬行——余量不足。1200把余量提到0.90Nm。
 * 代价是最坏退饱和从约2秒变为约4秒(原3000时为约10秒)；但前馈接管重力后，
 * 正常跟踪时积分很小，只有被卡住/顶住才会攒满，实际远小于4秒。只改容量不改Ki。 */
#define GIMBAL_PITCH_SPEED_I_LIMIT 1200.0f
/* Pitch-only conservative oscillation test. Limits are reference planning
 * assumptions, not measured physical capability. Lower BRAKE means earlier
 * slowing, not less available braking torque. Kp remains 25. */
#define GIMBAL_PITCH_SPEED_LIMIT_RAD_S (45.0f * GIMBAL_RAD_PER_DEG)
/* 720°/s²只缩短参考速度从0爬到上限的时间(到34.6°/s约48ms)，使速度误差更早
 * 建立、P项更早出力，即改善"反应"而不提高巡航速度。参考仍被制动上限
 * Bound()截断，MAX_SPEED/BRAKE未改，终端sqrt(2*a*d)曲线与落点精度不变。 */
#define GIMBAL_PITCH_ACCEL_RAD_S2 (720.0f * GIMBAL_RAD_PER_DEG)
#define GIMBAL_PITCH_BRAKE_RAD_S2 (120.0f * GIMBAL_RAD_PER_DEG)
#define GIMBAL_PITCH_MOTION_LIMIT_ENABLE 1
/* Signed canonical-pitch Nm: A*cos(pitch-zero)+B*sin(pitch-zero)，θ用IMU俯仰角(rad)。
 * 本平台实测负载近似"与角度无关的常值"，不是随sinθ增大的重力矩，所以只填A、B=0。
 * 依据：零前馈基线下用"同角度上下各走一次"直接测(摩擦随方向反号、负载不反,
 * 故 (T_up+T_down)/2 即为真实负载)：-2~6°得-0.476(上-0.042/下-0.910)，
 * 6~14°得-0.420(上-0.109/下-0.730)，2~10°另一次测得-0.484，重复性约0.05。
 * 故取 A=-0.45。0~14°的样本充足；负角度与14°以上暂无可靠数据（14~22°只有3个
 * 上升样本且上下差仅0.13，不采用），若那些角度积分仍顶满，说明欠补，需单独补。
 * 旧值 A=0.10/B=-2.41 由"轴卡住时的积分读数"拟合，静摩擦下该读数反映卡滞而非
 * 负载：它在0°给+0.10(应为-0.45，方向相反)、5°给-0.11、14°约-0.49(碰巧接近)、
 * 20°给-0.73(过补)，即斜率错而非整体写反。
 * 顺带实测动摩擦约0.43Nm、静摩擦约1.5Nm，粘滑是快跟随抖动的根源。
 * 第二轮(常值-0.45生效后)按角度统计积分残余："还差多少"为 -8~-2°→-0.285、
 * -2~6°→-0.114、6~14°→-0.050，饱和占比均为0%（改前长期顶满±0.60），说明常值
 * 方向正确，只是低角度欠补约0.3。
 * 但按三点最小二乘补上角度项(A=-0.64,B=+0.88)后实机抖动反而更重，故已回退：
 * 角度项会随姿态连续变化，在粘滑系统里等于给每次挣脱叠加不同幅值，反而放大抖动。
 * 结论：本平台只保留常值 A=-0.45，B=0。低角度残余的0.3交由积分承担（积分上限
 * 0.6，实测未饱和），宁可欠补也不引入随角度变化的力矩。
 * 改完必须重新上电，让积分从0建立。 */
#define GIMBAL_PITCH_GRAVITY_COS_NM (-0.45f)
#define GIMBAL_PITCH_GRAVITY_SIN_NM 0.0f
#define GIMBAL_PITCH_GRAVITY_ZERO_RAD 0.0f
#define GIMBAL_PITCH_GRAVITY_RAMP_NM_S 5.0f
/* Optional finite breakaway assistance, independently measured per direction.
 * Both zero = off. These thresholds enable assistance, not an angle deadband.
 * 本轮已关闭：0.5Nm对这台5°附近近乎平衡的轴过大，一次脉冲等于一次"踹"，
 * 而它每个方向只踹一次(0.25s窗口、一动就撤)，表现正是"顿一下、停一下"。
 * 若以后实测起动力矩确实偏大，先在单方向填小值(0.15~0.25)再观察。 */
#define GIMBAL_PITCH_FRICTION_POSITIVE_NM 0.0f
#define GIMBAL_PITCH_FRICTION_NEGATIVE_NM 0.0f
#define GIMBAL_PITCH_FRICTION_START_RAD (0.15f * GIMBAL_RAD_PER_DEG)
#define GIMBAL_PITCH_FRICTION_RELEASE_RAD (0.05f * GIMBAL_RAD_PER_DEG)
#define GIMBAL_PITCH_FRICTION_STOPPED_RAD_S 0.02f
/* 2.0Nm/s建立0.5Nm需要250ms，比参考速度斜坡(48ms)慢得多，助推会迟到；
 * 提高到10Nm/s后约50ms到位，与参考斜坡同量级。本值同时用于退出。 */
#define GIMBAL_PITCH_FRICTION_RAMP_NM_S 10.0f
#define GIMBAL_PITCH_FRICTION_TIMEOUT_S 0.25f
/* 【摩擦前馈·速度型】与上面的重力前馈并列，同在GimbalSpeedUpdate末尾的总装里叠加。
 * 与"指令速度"成正比：抵消与运动方向有关、与角度无关的阻力(干摩擦/减速箱阻尼)。
 * 不读反馈，因此不在反馈回路内，不会像提高Kp那样放大陀螺上的结构振动；
 * 参考回零即归零，不留到位残余。
 * 上面那组FRICTION_*是"起动一脚"(有状态机、一动就撤、0.25s超时)，已停用(系数0)，
 * 由本项取代——它补不了运动中持续的摩擦，正是之前"一卡一卡"的来源。
 * 实测依据(27秒连续动作采样)：参考持续-34.6°/s时，下发力矩-1.2~-1.5Nm
 * 仍只以1~2°/s的平均速度间歇爬行(偶发8~11°/s的突跳)，而P+I+前馈在满速时
 * 合计只有0.30+0.60+0.46=1.36Nm，正好卡在起动门槛(约1.5Nm)之下，形成粘滑。
 * 本平台结构所需推动力矩大于普通云台，1.00(满速0.77Nm，合计1.67Nm)确实把平均速度
 * 从0.9°/s提到7.4°/s，但快跟随中出现粘滑式爆发：挣脱瞬间多出约0.9Nm(静1.5-动0.6)
 * 全部用于加速，0.1秒即冲到50~70°/s，过冲9~12°后参考反号再冲一次。
 * 故回退到0.85(满速0.65Nm，合计1.55Nm)：仍高于起动门槛，但爆发幅度减小。
 * 若又变慢就回1.0；若仍抖则问题在阻尼(速度环Kp)而非前馈。0=关闭。 */
#define GIMBAL_PITCH_SPEED_FF_NM 0.85f
#define GIMBAL_ANGLE_SPEED_LIMIT (500.0f * GIMBAL_RAD_PER_DEG)
#define GIMBAL_OUTPUT_LIMIT 20000.0f
/* DM-J4310 pitch, MIT torque-only transport. Match the motor's saved ranges.
 * These do not change the IMU/vision coordinate system or the PI gain units. */
/* Real sentry wiring: pitch DM4310 is on CAN2; CAN1 is reserved for the
 * inter-board link to the chassis (0x330~332 / 0x338~33A / 0x340~342). */
#define GIMBAL_PITCH_DM_CAN_HANDLE (&hcan2)
#define GIMBAL_PITCH_DM_CAN_ID 0x002u
#define GIMBAL_PITCH_DM_MASTER_ID 0x000u
#define GIMBAL_PITCH_DM_P_MAX 3.14159f
#define GIMBAL_PITCH_DM_V_MAX 30.0f
#define GIMBAL_PITCH_DM_T_MAX 10.0f
/* Existing commissioning conversion: 20000 raw commands = 10 Nm BEFORE
 * the final 4 Nm clamp. It is not a GM6020 voltage-to-torque equivalence. */
#ifndef GIMBAL_PITCH_DM_NM_PER_COMMAND
#define GIMBAL_PITCH_DM_NM_PER_COMMAND (10.0f / 20000.0f)
#endif
#ifndef GIMBAL_PITCH_DM_TORQUE_LIMIT_NM
#define GIMBAL_PITCH_DM_TORQUE_LIMIT_NM 4.0f
#endif
#define GIMBAL_PITCH_DM_ENABLE_RETRY_MS 20u
#define GIMBAL_MOTOR_TIMEOUT_MS 50u
#define GIMBAL_INS_TIMEOUT_MS 20u
#define GIMBAL_CMD_TIMEOUT_MS 30u
/* First version targets the existing autonomous commissioning path. Setting
 * AUTO_AIM_ON_BOOT=0 keeps the remote/manual coexistence path on the original
 * PID implementation. Set this switch to 0 for a same-gain algorithm rollback. */
#ifndef GIMBAL_TRACKING_SPEED_PI_ENABLE
#define GIMBAL_TRACKING_SPEED_PI_ENABLE (GIMBAL_AUTO_AIM_ON_BOOT && GIMBAL_VISION_ENABLE)
#endif
#if (GIMBAL_TRACKING_SPEED_PI_ENABLE != 0 && GIMBAL_TRACKING_SPEED_PI_ENABLE != 1) || \
    (GIMBAL_TRACKING_SPEED_PI_ENABLE && !GIMBAL_AUTO_AIM_ON_BOOT)
#error tracking_v1 speed PI requires the autonomous commissioning path.
#endif
/* Use the command watchdog's existing time budget; never integrate a long
 * scheduling/debugging pause as if it were an ordinary motor-control step. */
#define GIMBAL_SPEED_PI_MAX_DT_S (0.001f * GIMBAL_CMD_TIMEOUT_MS)
/* Direction memory survives zero reference. A new direction must exceed the
 * speed corresponding to the existing yaw angle deadband (0.1deg * 8/s).
 * Pitch keeps its load-support integral; yaw-only reset is not a gravity model. */
#define GIMBAL_YAW_REVERSAL_REF_RAD_S (GIMBAL_YAW_ANGLE_KP * GIMBAL_YAW_ANGLE_DEADBAND_RAD)
#define GIMBAL_PITCH_REVERSAL_REF_RAD_S (GIMBAL_PITCH_ANGLE_KP * GIMBAL_YAW_ANGLE_DEADBAND_RAD)
/* RAM trace: 6.4 seconds at 20 Hz. Freeze 2 seconds after the last tracking
 * command / gate transition; a new tracking command resumes recording. */
#define GIMBAL_PITCH_TRACE_CAPACITY 128u
#define GIMBAL_PITCH_TRACE_PERIOD_MS 50u
#define GIMBAL_PITCH_TRACE_TAIL_MS 2000u
#define RC_TIMEOUT_MS 100u
#define VISION_TX_PERIOD_MS 5u
#define VISION_RX_TIMEOUT_MS 100u
#define VISION_STARTUP_GRACE_MS 1500u
/* Inherited planning placeholder, not measured muzzle speed. */
#define VISION_NOMINAL_SPEED_MPS 30.0f
#endif
/* [USB_SP 新增 01 结束] */
