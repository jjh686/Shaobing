/* [USB_SP 通信整理] 主机测试通过回调注入USB字节，不访问真实设备。 */
#ifndef ROBOT_COMM_TEST_USB_H
#define ROBOT_COMM_TEST_USB_H

#include "main.h"

#define USBD_OK 0u
#define USBD_BUSY 1u
#define USBD_FAIL 2u

typedef void (*USBCallback)(uint16_t length);
typedef struct {
    USBCallback tx_cbk;
    USBCallback rx_cbk;
} USB_Init_Config_s;

uint8_t *USBInit(USB_Init_Config_s config);
uint8_t USBTransmit(uint8_t *buffer, uint16_t length);
uint8_t USBIsConfigured(void);

#endif
