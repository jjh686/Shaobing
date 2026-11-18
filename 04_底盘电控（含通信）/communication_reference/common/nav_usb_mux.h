#ifndef NAV_USB_MUX_H
#define NAV_USB_MUX_H
#include <stdint.h>
#include <stddef.h>
#include "rm_chassis_comm/protocol.h"
typedef struct {uint8_t bytes[64];unsigned used;uint32_t last_ms,first_ms;} nav_mux_t;
typedef void (*nav_usb_frame_fn)(const uint8_t *,unsigned,uint32_t,void *);
/* Direction PC -> board only: SP=29, navigation=34. A valid whole frame is
 * consumed before scanning payload bytes for a new header. */
void nav_mux_push(nav_mux_t *,uint8_t,uint32_t,nav_usb_frame_fn,void *);
#endif
