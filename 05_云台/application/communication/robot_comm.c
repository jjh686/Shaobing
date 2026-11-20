/* [USB_SP 通信整理 开始]
 * 云台C板应用层通信总文件：USB命令接收、命令快照、会话、在线状态、姿态回传。
 * 从master_process.c迁入，保留SP 29/43字节、rad单位、超时和发送节流行为。
 * SP编解码仍在sp_protocol.c，USB硬件收发仍由bsp_usb/usbd_cdc_if负责。
 * [NAV] USB 只有一个接收者：DispatchUsb 按帧长把 SP 命令与导航整帧分流。
 */
#include "robot_comm.h"
#include "nav_usb_mux.h"
#include "chassis_link.h"
#include "bsp_usb.h"
#include "ins_task.h"
#include "gimbal_config.h"
#include "gimbal_math.h"
#include <string.h>

/* 一、接收状态。USB接收回调是解析器和命令数据的唯一写入者。 */
static SP_Parser parser;
static nav_mux_t usb_mux;
static Vision_Recv_s recv_data;
static uint8_t *rx_buffer;
static volatile uint8_t command_valid, requested_mode;
static float nominal_speed = VISION_NOMINAL_SPEED_MPS;
static volatile uint32_t tx_busy_count, tx_fail_count;

static void OnCommand(const SP_Command *command, void *context)
{
    (void)context;
    recv_data.yaw = command->yaw;
    recv_data.pitch = command->pitch;
    recv_data.yaw_vel = command->yaw_vel;
    recv_data.yaw_acc = command->yaw_acc;
    recv_data.pitch_vel = command->pitch_vel;
    recv_data.pitch_acc = command->pitch_acc;
    recv_data.mode = command->mode;
    recv_data.target_state = command->mode ? TARGET_CONVERGING : NO_TARGET;
    /* 保留当前仅控制云台的行为，mode2不转为发射请求。 */
    recv_data.fire_mode = NO_FIRE;
    recv_data.received_tick = HAL_GetTick();
    ++recv_data.sequence;
    command_valid = 1;
}

static void DispatchUsb(const uint8_t *bytes,unsigned length,uint32_t received,void *context)
{
    (void)context;
    if(length==SP_COMMAND_SIZE) SP_ParserFeed(&parser,bytes,length,OnCommand,NULL);
    else ChassisLinkReceiveUsb(bytes,length,received);
}
static void DecodeVision(uint16_t length)
{
    /* USB初始化/反初始化通过零长度回调通知新会话，不沿用残帧或旧命令。 */
    if (!length) {
        ++recv_data.session;
        command_valid = 0;
        SP_ParserInit(&parser);
        memset(&usb_mux,0,sizeof(usb_mux));ChassisLinkResetUsbSession();
        return;
    }
    uint32_t now=HAL_GetTick();
    for(unsigned i=0;i<length;i++)nav_mux_push(&usb_mux,rx_buffer[i],now,DispatchUsb,NULL);
}

/* 二、通信初始化。保持原RobotCMDInit中的初始化顺序，不重复注册USB回调。 */
Vision_Recv_s *RobotCommInit(void)
{
    SP_ParserInit(&parser);
    memset(&usb_mux,0,sizeof(usb_mux));
    ChassisLinkInit();
    memset(&recv_data, 0, sizeof(recv_data));
    command_valid = requested_mode = 0;
    USB_Init_Config_s config = {.rx_cbk = DecodeVision, .tx_cbk = NULL};
    rx_buffer = USBInit(config);
    return &recv_data;
}

/* 三、控制层只读取一致快照，通过此处统一查询链路状态。 */
uint8_t RobotCommGetVisionCommand(Vision_Recv_s *out)
{
    if (!out) return 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t valid = command_valid;
    *out = recv_data;
    __set_PRIMASK(primask);
    return valid;
}

uint8_t RobotCommUsbIsConfigured(void)
{
    return USBIsConfigured();
}

uint8_t RobotCommVisionIsOnline(void)
{
    Vision_Recv_s command;
    return RobotCommUsbIsConfigured() && RobotCommGetVisionCommand(&command) &&
           (uint32_t)(HAL_GetTick() - command.received_tick) <= VISION_RX_TIMEOUT_MS;
}

void RobotCommSetVisionMode(uint8_t mode)
{
    requested_mode = mode <= 3 ? mode : 0;
}

void RobotCommSetVisionBulletSpeed(Bullet_Speed_e speed)
{
    if (speed > 0) nominal_speed = (float)speed;
}

/* 四、周期回传。调用位置和last_tx更新顺序均与原VisionSend相同。 */
void RobotCommUpdate(void)
{
    static uint32_t last_tx;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last_tx) < VISION_TX_PERIOD_MS) return;
    last_tx = now;
    if (!RobotCommUsbIsConfigured()) return;
    INS_Snapshot_t ins;
    GimbalPose pose;
    static const float mount[9] = GIMBAL_TO_IMU_ROTATION;
    if (!INS_GetSnapshot(&ins) || (uint32_t)(HAL_GetTick() - ins.tick) > GIMBAL_INS_TIMEOUT_MS ||
        !GimbalPoseFromImu(ins.q, ins.gyro, mount, &pose)) return;
    SP_Status status = {0};
    status.mode = requested_mode;
    memcpy(status.q, ins.q, sizeof(status.q));
    status.yaw = pose.yaw;
    status.yaw_vel = pose.yaw_vel;
    status.pitch = pose.pitch;
    status.pitch_vel = pose.pitch_vel;
    status.bullet_speed = nominal_speed;
    status.bullet_count = 0;
    uint8_t frame[SP_STATUS_SIZE];
    SP_PackStatus(&status, frame);
    /* CDC仍将已接受的帧拷贝到独立在途缓冲，BUSY时按原周期再尝试。 */
    uint8_t result = USBTransmit(frame, sizeof(frame));
    if (result == USBD_BUSY) ++tx_busy_count;
    else if (result != USBD_OK) ++tx_fail_count;
}

/* USB has one owner: DispatchUsb separates SP and navigation whole frames.
 * Navigation conversion/forwarding runs in its independent normal-priority task. */
/* [USB_SP 通信整理 结束] */
