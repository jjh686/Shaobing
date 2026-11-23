#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
/* [USB_SP 修改 01 开始] 保留992中位与原通道，拒绝无效SBUS、真实心跳、故障代数快照和迟到离线回调复核。 */
#include "gimbal_config.h"
/* [USB_SP 修改 01 结束] */

#define REMOTE_CONTROL_FRAME_SIZE 18u // 遥控器接收的buffer大小
/* [USB_SP 修改 02 开始] 适配 ET08A 8 通道: 中位 1024, 死区 40(与 basic_framework 一致)。 */
#define FS_RC_MID 1024
#define FS_RC_DEADBAND 40
#define FS_RC_RANGE 660
/* [USB_SP 修改 02 结束] */
#define SBUS_FRAME_SIZE 25u//fs遥控器的buffer

//新遥控器摇杆位置
#define SWITCH_HIGH 1807
#define SWITCH_MID  1024
#define SWITCH_LOW  240
#define SWITCH_TOLERANCE 200

/* [USB_SP 修改 03 开始] 保留992中位与原通道，拒绝无效SBUS、真实心跳、故障代数快照和迟到离线回调复核。 */
/* [USB_SP 修改 03 结束] */
// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static FS_RC_ctrl_t fs_rc_ctrl[2]; //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位
/* [USB_SP 修改 04 开始] 保留992中位与原通道，拒绝无效SBUS、真实心跳、故障代数快照和迟到离线回调复核。 */
static volatile uint8_t fs_seen, fs_fault = 1;
static volatile uint32_t fs_last_valid_tick, fs_fault_generation;
volatile FS_SbusDiagnostics fs_sbus_diag;
static uint8_t fs_partial[SBUS_FRAME_SIZE],fs_partial_len;
static uint32_t fs_partial_ms,fs_uart_errors_seen;
/* [USB_SP 修改 04 结束] */

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮
    RectifyRCjoystick();
    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void fs_to_rc(const uint8_t *sbus_buf)
{
    for(unsigned i=0;i<10;i++){
        unsigned bit=i*11u,index=1u+bit/8u;
        uint32_t word=sbus_buf[index]|((uint32_t)sbus_buf[index+1]<<8)|((uint32_t)sbus_buf[index+2]<<16);
        fs_sbus_diag.raw_channels[i]=(word>>(bit%8u))&0x07ffu;
    }
/* [USB_SP 修改 05 开始] 保留992中位与原通道，拒绝无效SBUS、真实心跳、故障代数快照和迟到离线回调复核。 */
    fs_rc_ctrl[LAST] = fs_rc_ctrl[TEMP];
    fs_rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[1]     | sbus_buf[2] << 8) & 0x07FF)-FS_RC_MID;
    fs_rc_ctrl[TEMP].rc.rocker_r1 = ((sbus_buf[2] >> 3 | sbus_buf[3] << 5) & 0x07FF) -FS_RC_MID;
/* [USB_SP 修改 05 结束] */
    fs_rc_ctrl[TEMP].rc.rocker_l1 = ((sbus_buf[3] >> 6 | sbus_buf[4] << 2 | sbus_buf[5] << 10) & 0x07FF) - FS_RC_MID;
    fs_rc_ctrl[TEMP].rc.rocker_l_ = ((sbus_buf[5] >> 1 | sbus_buf[6] << 7) & 0x07FF) - FS_RC_MID;

/* [USB_SP 修改 06 开始] 保留992中位与原通道，拒绝无效SBUS、真实心跳、故障代数快照和迟到离线回调复核。 */
    int16_t *channels[] = {&fs_rc_ctrl[TEMP].rc.rocker_l_, &fs_rc_ctrl[TEMP].rc.rocker_l1,
                           &fs_rc_ctrl[TEMP].rc.rocker_r_, &fs_rc_ctrl[TEMP].rc.rocker_r1};
    for (unsigned i=0; i<4; ++i) {
        int16_t v = *channels[i];
        if (v >= -FS_RC_DEADBAND && v <= FS_RC_DEADBAND) v = 0;
        if (v > FS_RC_RANGE) v = FS_RC_RANGE;
        if (v < -FS_RC_RANGE) v = -FS_RC_RANGE;
        *channels[i] = v;
/* [USB_SP 修改 06 结束] */
    }
/* [USB_SP 修改 07 开始] 保留992中位与原通道，拒绝无效SBUS、真实心跳、故障代数快照和迟到离线回调复核。 */
/* [USB_SP 修改 07 结束] */
    //旋钮
    fs_rc_ctrl[TEMP].rc.knob_l = ((sbus_buf[6] >> 4 | sbus_buf[7] << 4) & 0x07FF)-240u;
    fs_rc_ctrl[TEMP].rc.knob_r = ((sbus_buf[7] >> 7 | sbus_buf[8] << 1 | sbus_buf[9] << 9) & 0x07FF)-240u;
    //拨杆0上，1中，2下 (ET08A 8通道仅使用 CH7=SB/channel[6] 与 CH8=SC/channel[7])
    fs_rc_ctrl[TEMP].channel[6] = ((sbus_buf[9] >> 2 | sbus_buf[10] << 6) & 0x07FF);
    fs_rc_ctrl[TEMP].channel[7] = ((sbus_buf[10] >> 5 | sbus_buf[11] << 3) & 0x07FF);
    fs_rc_ctrl[TEMP].channel[8] = FS_RC_SW_MID;
    fs_rc_ctrl[TEMP].channel[9] = FS_RC_SW_MID;
    for (int i = 6; i <= 7; i++)
    {
        uint16_t val = fs_rc_ctrl[TEMP].channel[i];
        if (val > SWITCH_HIGH - SWITCH_TOLERANCE && val < SWITCH_HIGH + SWITCH_TOLERANCE)
        {
            fs_rc_ctrl[TEMP].channel[i] = 2; // 下挡
        }
        else if (val > SWITCH_MID - SWITCH_TOLERANCE && val < SWITCH_MID + SWITCH_TOLERANCE)
        {
            fs_rc_ctrl[TEMP].channel[i] = 1; // 中挡
        }
        else if (val > SWITCH_LOW - SWITCH_TOLERANCE && val < SWITCH_LOW + SWITCH_TOLERANCE)
        {
            fs_rc_ctrl[TEMP].channel[i] = 0; // 上挡
        }
        else
        {
/* [USB_SP 修改 08 开始] 保留有效区间校验，避免非法值被当成中挡。 */
            fs_rc_ctrl[TEMP].channel[i] = -1; // Invalid switch is not a valid MID command
/* [USB_SP 修改 08 结束] */
        }
    }

    fs_rc_ctrl[TEMP].rc.switch_left_1 = fs_rc_ctrl[TEMP].channel[6];
    fs_rc_ctrl[TEMP].rc.switch_left_2 = fs_rc_ctrl[TEMP].channel[7];
    fs_rc_ctrl[TEMP].rc.switch_right_1 = fs_rc_ctrl[TEMP].channel[8];
    fs_rc_ctrl[TEMP].rc.switch_right_2 = fs_rc_ctrl[TEMP].channel[9];
/* [USB_SP 修改 09 开始] 保留992中位与原通道，拒绝无效SBUS、真实心跳、故障代数快照和迟到离线回调复核。 */
/* [USB_SP 修改 09 结束] */

}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 对fs_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void fs_uart_fault(void)
{
    uint32_t irq=__get_PRIMASK();__disable_irq();
    uint32_t errors=rc_usart_instance->error_events;
    if(errors!=fs_uart_errors_seen){
        fs_sbus_diag.uart_errors+=errors-fs_uart_errors_seen;fs_uart_errors_seen=errors;
        fs_partial_len=0;fs_fault=1;++fs_fault_generation;
    }
    __set_PRIMASK(irq);
}

static uint8_t fs_accept_frame(const uint8_t *frame,uint32_t now)
{
    if(frame[23]&0xf0){fs_sbus_diag.bad_flags++;++fs_fault_generation;fs_fault=1;return 0;}
    if(frame[23]&0x08){
        fs_sbus_diag.failsafe++;
        ++fs_fault_generation;fs_fault=1;return 1;
    }
    if(frame[23]&0x04){
        /* 单帧射频丢包不立即置 fault(防抖), 仅不喂狗, 连续超时由 RC_TIMEOUT_MS 判离线 */
        fs_sbus_diag.frame_lost++;
        return 1;
    }
    uint32_t gap=now-fs_last_valid_tick;
    if(fs_seen&&gap>fs_sbus_diag.max_valid_gap_ms)fs_sbus_diag.max_valid_gap_ms=gap;
    if(fs_seen&&gap>RC_TIMEOUT_MS){++fs_fault_generation;fs_sbus_diag.timeouts++;}
    fs_to_rc(frame);
    for(unsigned i=6;i<8;i++)if(fs_rc_ctrl[TEMP].channel[i]<0){
        fs_sbus_diag.invalid_switches++;++fs_fault_generation;fs_fault=1;return 0;
    }
    if(fs_switch_is_up(fs_rc_ctrl[TEMP].rc.switch_left_2)){
        ++fs_fault_generation;fs_sbus_diag.estop_frames++;
    }
    fs_last_valid_tick=now;fs_seen=1;fs_fault=0;fs_sbus_diag.valid_frames++;
    DaemonReload(rc_daemon_instance);
    return 1;
}

/* DMA/IDLE boundaries are not SBUS frame boundaries. Keep partial frames and
 * slide to the next header after corruption. Real lost/failsafe bits stay fatal. */
static void FS_RemoteControlRxCallback(void)
{
    fs_uart_fault();
    uint32_t now=HAL_GetTick();
    if(fs_partial_len&&(uint32_t)(now-fs_partial_ms)>20u){
        fs_partial_len=0;fs_sbus_diag.partial_timeouts++;fs_fault=1;++fs_fault_generation;
    }
    for(unsigned i=0;i<rc_usart_instance->recv_len;i++){
        uint8_t byte=rc_usart_instance->recv_buff[i];
        if(!fs_partial_len&&byte!=0x0f){fs_sbus_diag.discarded_bytes++;continue;}
        fs_partial[fs_partial_len++]=byte;fs_partial_ms=now;
        if(fs_partial_len!=SBUS_FRAME_SIZE)continue;
        uint8_t tail_ok=fs_partial[24]==0||(fs_partial[24]&0x0f)==0x04;
        if(tail_ok&&fs_accept_frame(fs_partial,now)){
            fs_partial_len=0;
        }else{
            if(!tail_ok){fs_sbus_diag.bad_tail++;fs_fault=1;++fs_fault_generation;}
            unsigned skip=1;while(skip<SBUS_FRAME_SIZE&&fs_partial[skip]!=0x0f)skip++;
            fs_sbus_diag.discarded_bytes+=skip;fs_partial_len=SBUS_FRAME_SIZE-skip;
            memmove(fs_partial,fs_partial+skip,fs_partial_len);
        }
    }
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

static void FS_RCLostCallback(void *id)
{
/* [USB_SP 修改 11 开始] 保留992中位与原通道，拒绝无效SBUS、真实心跳、故障代数快照和迟到离线回调复核。 */
    (void)id;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    /* A valid IRQ may have reloaded the daemon after its task tested zero. */
    if (fs_seen && !fs_fault && (uint32_t)(HAL_GetTick() - fs_last_valid_tick) <= RC_TIMEOUT_MS) {
        __set_PRIMASK(primask);
        return;
    }
    uint8_t report = !fs_fault;
    if (report) {++fs_fault_generation;fs_sbus_diag.timeouts++;}
    fs_fault = 1;
/* [USB_SP 修改 11 结束] */
    memset(fs_rc_ctrl, 0, sizeof(fs_rc_ctrl)); // 清空遥控器数据
/* [USB_SP 修改 12 开始] 保留992中位与原通道，拒绝无效SBUS、真实心跳、故障代数快照和迟到离线回调复核。 */
    __set_PRIMASK(primask);
    if (report) LOGWARNING("[rc] remote control lost");
    /* DMA is already waiting. Repeatedly restarting it here can lose frames;
     * the USART error callback is responsible for error recovery. */
/* [USB_SP 修改 12 结束] */
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

FS_RC_ctrl_t *FS_RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    /* The gimbal controller and board link share one SBUS receiver. */
    if (rc_init_flag) return fs_rc_ctrl;
    USART_Init_Config_s conf;
    conf.module_callback = FS_RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = SBUS_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = FS_RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return fs_rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}

uint8_t FS_RemoteControlIsOnline(){
/* [USB_SP 修改 13 开始] 保留992中位与原通道，拒绝无效SBUS、真实心跳、故障代数快照和迟到离线回调复核。 */
    uint32_t irq=__get_PRIMASK();__disable_irq();
    if(rc_init_flag)fs_uart_fault();
    uint8_t online=rc_init_flag && fs_seen && !fs_fault &&
           (uint32_t)(HAL_GetTick() - fs_last_valid_tick) <= RC_TIMEOUT_MS;
    __set_PRIMASK(irq);return online;
/* [USB_SP 修改 13 结束] */
}
/* [USB_SP 修改 14 开始] 保留992中位与原通道，拒绝无效SBUS、真实心跳、故障代数快照和迟到离线回调复核。 */

uint8_t FS_RemoteControlGetSnapshot(FS_RC_ctrl_t *snapshot)
{
    if (!snapshot) return 0;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t online = FS_RemoteControlIsOnline();
    *snapshot = fs_rc_ctrl[TEMP];
    snapshot->fault_generation = fs_fault_generation;
    snapshot->received_ms = fs_last_valid_tick;
    __set_PRIMASK(primask);
    return online;
}
/* [USB_SP 修改 14 结束] */
