#ifndef NAV_GATEWAY_H
#define NAV_GATEWAY_H
#include "nav_can.h"
#include "rm_chassis_comm/mcu_link.h"
typedef struct {rm_mcu_link_t gate;uint32_t command_received_ms;} nav_gateway_t;
void ng_init(nav_gateway_t *);
bool ng_command(nav_gateway_t *,const uint8_t usb[RM_FRAME_SIZE],uint32_t received,
                uint32_t now,nc_group_t *);
#endif
