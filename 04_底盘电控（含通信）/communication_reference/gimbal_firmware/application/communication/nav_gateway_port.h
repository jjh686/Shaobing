#ifndef NAV_GATEWAY_PORT_H
#define NAV_GATEWAY_PORT_H
#include <stdint.h>
void NavGatewayInit(void);
void NavGatewayReset(void); /* USB IRQ, requests foreground reset. */
void NavGatewayReceive(const uint8_t *,unsigned,uint32_t); /* USB IRQ: copy only. */
void NavGatewayTask(void); /* Normal priority 1 ms communication task. */
#endif
