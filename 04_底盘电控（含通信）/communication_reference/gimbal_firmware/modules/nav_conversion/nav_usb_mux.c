#include "nav_usb_mux.h"
#include "nav_can.h"
#include <string.h>
static void drop(nav_mux_t *m,unsigned n){m->used-=n;memmove(m->bytes,m->bytes+n,m->used);}
void nav_mux_push(nav_mux_t *m,uint8_t byte,uint32_t now,nav_usb_frame_fn callback,void *context)
{
    if((uint32_t)(now-m->last_ms)>50)m->used=0;
    m->last_ms=now;if(m->used==sizeof(m->bytes))drop(m,1);
    if(!m->used)m->first_ms=now;
    m->bytes[m->used++]=byte;
    while(m->used){
        unsigned n=0;
        if(m->bytes[0]=='S'){
            if(m->used<2)return;
            if(m->bytes[1]=='P')n=29;
        }else if(m->bytes[0]==0x5a){
            if(m->used<3)return;
            if(m->bytes[1]==28&&m->bytes[2]==RM_ID_COMMAND)n=34;
        }
        if(!n){drop(m,1);continue;}
        if(m->used<n)return;
        rm_command_t command;
        bool valid=n==29?nc_crc(m->bytes,27)==((uint16_t)m->bytes[27]|((uint16_t)m->bytes[28]<<8)):
            rm_decode_command(m->bytes,34,&command);
        if(!valid){drop(m,1);continue;}
        if(callback)callback(m->bytes,n,m->first_ms,context);
        drop(m,n);
    }
}
