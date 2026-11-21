#include "usb_device.h"
typedef void (*USBCallback)(uint16_t);
uint8_t *CDCInitRxbufferNcallback(USBCallback,USBCallback);
uint8_t CDC_Transmit_FS(uint8_t *,uint16_t);
