#ifndef TEST_USB_H
#define TEST_USB_H
#include <stdint.h>
#define USBD_OK 0u
#define USBD_BUSY 1u
uint8_t USBIsConfigured(void);
uint8_t USBTransmit(uint8_t *,uint16_t);
#endif
