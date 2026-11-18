/**
 * @file bsp_usb.c
 * @author your name (you@domain.com)
 * @brief usb是单例bsp,因此不保存实例
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_usb.h"
#include "bsp_log.h"
#include "bsp_dwt.h"
/* [USB_SP 修改 01 开始] 长度化USB接收回调与配置状态查询、传播发送结果。 */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* [USB_SP 修改 01 结束] */

static uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里,buffer size为2048
/* [USB_SP 修改 02 开始] 长度化USB接收回调与配置状态查询、传播发送结果。 */
// Full-speed endpoint packets are <=64 bytes; application frames are a byte stream.
/* [USB_SP 修改 02 结束] */

uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    // usb的软件复位(模拟拔插)在usbd_conf.c中的HAL_PCD_MspInit()中
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(usb_conf.tx_cbk, usb_conf.rx_cbk); // 获取接收数据指针
    // usb的接收回调函数会在这里被设置,并将数据保存在bsp_usb_rx_buffer中
    LOGINFO("USB init success");
    return bsp_usb_rx_buffer;
}

/* [USB_SP 修改 03 开始] 长度化USB接收回调与配置状态查询、传播发送结果。 */
uint8_t USBTransmit(uint8_t *buffer, uint16_t len)
/* [USB_SP 修改 03 结束] */
{
/* [USB_SP 修改 04 开始] 长度化USB接收回调与配置状态查询、传播发送结果。 */
    return CDC_Transmit_FS(buffer, len);
/* [USB_SP 修改 04 结束] */
}
/* [USB_SP 修改 05 开始] 长度化USB接收回调与配置状态查询、传播发送结果。 */

uint8_t USBIsConfigured(void)
{
    return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != NULL;
}
/* [USB_SP 修改 05 结束] */
