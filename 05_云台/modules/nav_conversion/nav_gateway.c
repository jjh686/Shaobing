#include "nav_gateway.h"
#include <string.h>
void ng_init(nav_gateway_t *g)
{
    memset(g,0,sizeof(*g));rm_mcu_init(&g->gate,2);
    /* This gate enforces transport/session freshness only. Actual RC/actuator
     * authorization is independently evaluated by the chassis board. */
    rm_mcu_set_safety(&g->gate,true,false,false,0);
}
bool ng_command(nav_gateway_t *g,const uint8_t usb[RM_FRAME_SIZE],uint32_t received,
                uint32_t now,nc_group_t *out)
{
    rm_command_t c;uint8_t normalized[RM_FRAME_SIZE];
    if(!g||!out||!rm_decode_command(usb,RM_FRAME_SIZE,&c)||c.route!=2)return false;
    uint32_t age=now-received;
    if(age>NC_GROUP_MS)return false; /* No stale queue replay, including STOP ACK. */
    if(c.flags&RM_FLAG_ENABLE){
        if(c.ttl_ms>NC_MAX_TTL_MS||c.ttl_ms<age+NC_GROUP_MS+20u)return false;
        c.ttl_ms-=(uint16_t)(age+NC_GROUP_MS); /* Reserve time for CAN delivery. */
    }
    if(!nc_command_group(&c,out)||!rm_encode_command(&c,normalized))return false;
    if(!rm_mcu_receive(&g->gate,normalized,sizeof(normalized),now))return false;
    g->command_received_ms=received;return true;
}
