#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

/* [USB_SP 通信整理] 本头保留原数据类型和Vision兼容接口；
 * 新应用统一使用application/communication/robot_comm.h。
 * 实际USB收发和会话管理集中在robot_comm.c。 */
#include "bsp_usart.h"
/* [USB_SP 修改 01 开始] SP帧长度、rad命令字段、序列/会话代数及一致快照接口。 */
#include "sp_protocol.h"
/* [USB_SP 修改 01 结束] */

/* [USB_SP 修改 02 开始] SP帧长度、rad命令字段、序列/会话代数及一致快照接口。 */
#define VISION_RECV_SIZE SP_COMMAND_SIZE
#define VISION_SEND_SIZE SP_STATUS_SIZE
/* [USB_SP 修改 02 结束] */

/* [USB_SP 修改 03 开始] SP帧长度、rad命令字段、序列/会话代数及一致快照接口。 */
/* [USB_SP 修改 03 结束] */
typedef enum
{
	NO_FIRE = 0,
	AUTO_FIRE = 1,
	AUTO_AIM = 2
} Fire_Mode_e;

typedef enum
{
	NO_TARGET = 0,
	TARGET_CONVERGING = 1,
	READY_TO_FIRE = 2
} Target_State_e;

typedef enum
{
	NO_TARGET_NUM = 0,
	HERO1 = 1,
	ENGINEER2 = 2,
	INFANTRY3 = 3,
	INFANTRY4 = 4,
	INFANTRY5 = 5,
	OUTPOST = 6,
	SENTRY = 7,
	BASE = 8
} Target_Type_e;

typedef struct
{
	Fire_Mode_e fire_mode;
	Target_State_e target_state;
	Target_Type_e target_type;

/* [USB_SP 修改 04 开始] SP帧长度、rad命令字段、序列/会话代数及一致快照接口。 */
	float pitch; /* rad, same reference as transmitted quaternion */
/* [USB_SP 修改 04 结束] */
	float yaw;
/* [USB_SP 修改 05 开始] SP帧长度、rad命令字段、序列/会话代数及一致快照接口。 */
	float yaw_vel, yaw_acc, pitch_vel, pitch_acc;
	uint8_t mode;
	uint32_t received_tick, sequence, session; /* USB init/deinit generation */
/* [USB_SP 修改 05 结束] */
} Vision_Recv_s;

typedef enum
{
	COLOR_NONE = 0,
	COLOR_BLUE = 1,
	COLOR_RED = 2,
} Enemy_Color_e;

typedef enum
{
	VISION_MODE_AIM = 0,
	VISION_MODE_SMALL_BUFF = 1,
	VISION_MODE_BIG_BUFF = 2
} Work_Mode_e;

typedef enum
{
	BULLET_SPEED_NONE = 0,
	BIG_AMU_10 = 10,
	SMALL_AMU_15 = 15,
	BIG_AMU_16 = 16,
	SMALL_AMU_18 = 18,
	SMALL_AMU_30 = 30,
} Bullet_Speed_e;

typedef struct
{
	Enemy_Color_e enemy_color;
	Work_Mode_e work_mode;
	Bullet_Speed_e bullet_speed;

	float yaw;
	float pitch;
	float roll;
} Vision_Send_s;
/* [USB_SP 修改 06 开始] SP帧长度、rad命令字段、序列/会话代数及一致快照接口。 */
/* [USB_SP 修改 06 结束] */

/* [USB_SP 修改 07 开始] SP帧长度、rad命令字段、序列/会话代数及一致快照接口。 */
/**
 * @brief Initialize USB CDC SP; the UART argument is retained for source compatibility.
 *
 * @param _handle Unused. Tasks read a coherent command via VisionGetCommand().
 */
/* [USB_SP 修改 07 结束] */
Vision_Recv_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
/* [USB_SP 修改 08 开始] SP帧长度、rad命令字段、序列/会话代数及一致快照接口。 */
void VisionSend(void);
void VisionSetMode(uint8_t mode);
uint8_t VisionGetCommand(Vision_Recv_s *command);
uint8_t VisionIsOnline(void);
/* [USB_SP 修改 08 结束] */

/**
 * @brief 设置视觉发送标志位
 *
 * @param enemy_color
 * @param work_mode
 * @param bullet_speed
 */
void VisionSetFlag(Enemy_Color_e enemy_color, Work_Mode_e work_mode, Bullet_Speed_e bullet_speed);

/* [USB_SP 修改 09 开始] SP帧长度、rad命令字段、序列/会话代数及一致快照接口。 */
/**
 * @brief Deprecated compatibility no-op. Telemetry reads a complete INS snapshot.
 *
 * @param yaw
 * @param pitch
 */
/* [USB_SP 修改 09 结束] */
void VisionSetAltitude(float yaw, float pitch, float roll);

/* [USB_SP 修改 10 开始] SP帧长度、rad命令字段、序列/会话代数及一致快照接口。 */
#endif // !MASTER_PROCESS_H
/* [USB_SP 修改 10 结束] */
