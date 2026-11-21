/**
 * @file bsp_usb.h
 * @author your name (you@domain.com)
 * @brief 提供对usb vpc(virtal com port)的操作接口,hid和msf考虑后续添加
 * @attention 这一版usb修改了usbd_cdc_if.c中的CDC_Receive_FS函数,若使用cube生成后会被覆盖.后续需要由usbcdciftemplate创建一套自己的模板
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#pragma once
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"

typedef struct
{
    USBCallback tx_cbk;
    USBCallback rx_cbk;
} USB_Init_Config_s;

/* @note 虚拟串口的波特率/校验位/数据位等动态可变,取决于上位机的设定 */
/* 使用时不需要关心这些设置(作为从机) */

uint8_t *USBInit(USB_Init_Config_s usb_conf); // bsp初始化时调用会重新枚举设备

/* [USB_SP 修改 01 开始] USB回调/发送状态/配置状态接口适配。 */
uint8_t USBTransmit(uint8_t *buffer, uint16_t len); // USBD_OK / BUSY / FAIL
uint8_t USBTransmitStamped(uint8_t *buffer,uint16_t len,uint32_t received_ms);
/* OK means copied into a bounded telemetry queue. The task pumps CDC at 1 kHz. */
void USBTransmitPump(void);
void USBTransmitReset(void);
uint8_t USBIsConfigured(void);
/* [USB_SP 修改 01 结束] */
