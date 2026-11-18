#ifndef CHASSIS_COMM_CONFIG_H
#define CHASSIS_COMM_CONFIG_H

/* Motor commissioning enabled at the user's request. RC, motor-online,
 * STOP/session and TTL gates still apply. Wheel axes need physical testing. */
#ifndef CHASSIS_COMM_ALLOW_MOTION
#define CHASSIS_COMM_ALLOW_MOTION 1
#endif
#ifndef CHASSIS_COMM_AXES_VERIFIED
#define CHASSIS_COMM_AXES_VERIFIED 1
#endif
#define CHASSIS_COMM_USE_INS 0
#define CHASSIS_COMM_ROUTE 2u
#define CHASSIS_COMM_FEEDBACK_PERIOD_MS 20u
#define CHASSIS_COMM_RX_QUEUE_SIZE 16u
#define CHASSIS_COMM_CAN_HANDLE hcan2
#define CHASSIS_COMM_REMOTE_SOURCE_CAN 1 /* Cloud UART3 SBUS -> CAN2; no local receiver */
/* Protocol: +X forward, +Y left, +yaw counterclockwise (viewed from above).
 * 2026-11-17: the user observed reversed lateral/yaw motion on this chassis.
 * Convert protocol -> existing driver here, and driver -> protocol in feedback.
 * These signs apply to this wheel/ID installation; verify all six directions. */
#define CHASSIS_COMM_VX_SIGN 1.0f
#define CHASSIS_COMM_VY_SIGN (-1.0f)
#define CHASSIS_COMM_WZ_SIGN (-1.0f)

#if defined(CHASSIS_TEST_FORWARD)
#error CHASSIS_TEST_FORWARD bypasses command arbitration; remove it for navigation
#endif
#endif
