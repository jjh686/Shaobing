#ifndef TEST_USB_H
#define TEST_USB_H
#include <stdint.h>
#define USBD_OK 0u
#define USBD_BUSY 1u
uint8_t USBIsConfigured(void);
uint8_t USBTransmit(uint8_t *,uint16_t);
static inline uint8_t USBTransmitStamped(uint8_t *p,uint16_t n,uint32_t t){(void)t;return USBTransmit(p,n);}
static inline void USBTransmitPump(void){}
static inline void USBTransmitReset(void){}
#endif
