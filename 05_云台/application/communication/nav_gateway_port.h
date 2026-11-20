#ifndef NAV_GATEWAY_PORT_H
#define NAV_GATEWAY_PORT_H
/* Compatibility for existing callers and the unchanged 04 host fixtures.
 * New application code uses chassis_link.h; this file owns no state or task. */
#include "chassis_link.h"
#include "chassis_link_debug.h"
static inline void NavGatewayInit(void) { ChassisLinkInit(); }
static inline void NavGatewayReset(void) { ChassisLinkResetUsbSession(); }
static inline void NavGatewayReceive(const uint8_t *bytes,unsigned length,uint32_t received)
{ ChassisLinkReceiveUsb(bytes,length,received); }
static inline void NavGatewayTask(void) { ChassisLinkUpdate(); }
#endif
