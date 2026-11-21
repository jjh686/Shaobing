#ifndef USB_TEST_DEVICE_H
#define USB_TEST_DEVICE_H
#include <stdint.h>
#include <stddef.h>
typedef struct {unsigned dev_state;void *pClassData;} USBD_HandleTypeDef;
#define USBD_STATE_CONFIGURED 3u
#define USBD_OK 0u
#define USBD_BUSY 1u
#define USBD_FAIL 2u
#define __get_PRIMASK() 0u
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
uint32_t HAL_GetTick(void);
#endif
