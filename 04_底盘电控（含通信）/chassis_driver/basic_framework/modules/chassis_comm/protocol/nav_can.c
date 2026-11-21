#include "nav_can.h"
#include <math.h>
#include <string.h>
static void w16(uint8_t *p,uint16_t v) { p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8); }
static void w32(uint8_t *p,uint32_t v) { for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i)); }
static uint16_t r16(const uint8_t *p) { return (uint16_t)p[0]|((uint16_t)p[1]<<8); }
static int32_t s16(const uint8_t *p) { uint16_t v=r16(p);return v>=32768?(int32_t)v-65536:v; }
static uint32_t r32(const uint8_t *p) { return (uint32_t)r16(p)|((uint32_t)r16(p+2)<<16); }
uint16_t nc_crc(const uint8_t *p,unsigned n)
{
    uint16_t c=0xffff;
    while(n--) { c^=*p++;for(unsigned i=0;i<8;i++)c=(uint16_t)((c>>1)^((c&1)?0x8408:0)); }
    return c;
}
static void seal(nc_group_t *g,uint32_t uptime,uint16_t seq)
{
    uint8_t bytes[20];memcpy(bytes,g->data,16);w32(bytes+16,uptime);
    w32(g->data[2],uptime);w16(g->data[2]+4,nc_crc(bytes,sizeof(bytes)));w16(g->data[2]+6,seq);
}
static bool sealed(const nc_group_t *g)
{
    uint8_t bytes[20];memcpy(bytes,g->data,16);memcpy(bytes+16,g->data[2],4);
    return r16(g->data[2]+6)==r16(g->data[0]+4) && r16(g->data[2]+4)==nc_crc(bytes,20);
}
static bool command_ok(const rm_command_t *c)
{
    uint8_t test[RM_FRAME_SIZE];
    return c && c->route==2 && rm_encode_command(c,test) &&
        c->ttl_ms<=NC_MAX_TTL_MS && hypotf(c->vx,c->vy)<=0.300001f && fabsf(c->wz)<=0.500001f;
}
static void velocity(uint8_t *p,float vx,float vy,float wz,bool feedback)
{
    float v[3]={vx,vy,wz};
    for(unsigned i=0;i<3;i++) {
        /* Commands round toward zero so quantization cannot increase speed. */
        int32_t q=(int32_t)(feedback?roundf(v[i]*1000.0f):truncf(v[i]*1000.0f));
        w16(p+2*i,(uint16_t)q);
    }
}
bool nc_command_group(const rm_command_t *c,nc_group_t *g)
{
    if(!g||!command_ok(c))return false;
    memset(g,0,sizeof(*g));g->base=NC_COMMAND_BASE;
    w32(g->data[0],c->session);w16(g->data[0]+4,c->sequence);w16(g->data[0]+6,c->ttl_ms);
    velocity(g->data[1],c->vx,c->vy,c->wz,false);w16(g->data[1]+6,NC_VERSION|c->flags);
    seal(g,c->uptime_ms,c->sequence);return true;
}
bool nc_command_decode(const nc_group_t *g,rm_command_t *c)
{
    if(!g||!c||g->base!=NC_COMMAND_BASE||!sealed(g)||(r16(g->data[1]+6)&0xff00)!=NC_VERSION)return false;
    rm_command_t value={0};value.route=2;value.session=r32(g->data[0]);value.sequence=r16(g->data[0]+4);
    value.ttl_ms=r16(g->data[0]+6);value.flags=r16(g->data[1]+6)&255;
    value.vx=s16(g->data[1])/1000.0f;value.vy=s16(g->data[1]+2)/1000.0f;value.wz=s16(g->data[1]+4)/1000.0f;
    value.uptime_ms=r32(g->data[2]);if(!command_ok(&value))return false;*c=value;return true;
}
bool nc_feedback_group(const rm_feedback_t *f,nc_group_t *g)
{
    uint8_t bytes[RM_FRAME_SIZE];
    if(!g||!f||f->route!=2||!rm_encode_feedback(f,bytes)||fabsf(f->wz)>32.767f)return false;
    memset(g,0,sizeof(*g));g->base=NC_FEEDBACK_BASE;
    w32(g->data[0],f->session);w16(g->data[0]+4,f->sequence);w16(g->data[0]+6,NC_VERSION|f->status);
    velocity(g->data[1],f->vx,f->vy,f->wz,true);w16(g->data[1]+6,f->fault_mask);
    seal(g,f->uptime_ms,f->sequence);return true;
}
bool nc_feedback_decode(const nc_group_t *g,rm_feedback_t *f)
{
    if(!g||!f||g->base!=NC_FEEDBACK_BASE||!sealed(g)||(r16(g->data[0]+6)&0xff00)!=NC_VERSION)return false;
    rm_feedback_t value={0};uint8_t bytes[RM_FRAME_SIZE];value.route=2;
    value.session=r32(g->data[0]);value.sequence=r16(g->data[0]+4);value.status=r16(g->data[0]+6)&255;
    value.vx=s16(g->data[1])/1000.0f;value.vy=s16(g->data[1]+2)/1000.0f;value.wz=s16(g->data[1]+4)/1000.0f;
    value.fault_mask=r16(g->data[1]+6);value.uptime_ms=r32(g->data[2]);
    if(!rm_encode_feedback(&value,bytes))return false;
    *f=value;return true;
}
bool nc_rx_push(nc_rx_t *r,uint16_t base,uint32_t id,bool extended,bool rtr,unsigned dlc,
                const uint8_t *bytes,uint32_t now,nc_group_t *out)
{
    if(!r||!out||id<base||id>base+2u)return false;
    if(extended||rtr||dlc!=8||!bytes){r->stats.malformed++;r->active=0;return false;}
    if(id==base){
        if(r->active)r->stats.restarted_groups++;
        r->active=1;r->next=0;r->started_ms=now;r->group.base=base;
    }
    if(!r->active||id!=base+r->next){r->stats.order_errors++;r->active=0;return false;}
    if((uint32_t)(now-r->started_ms)>NC_GROUP_MS){r->stats.group_timeouts++;r->active=0;return false;}
    memcpy(r->group.data[r->next++],bytes,8);
    if(r->next!=3)return false;
    r->active=0;if(!sealed(&r->group)){r->stats.crc_errors++;return false;}*out=r->group;return true;
}

bool nc_tx_start(nc_tx_t *t,const nc_group_t *g,uint32_t now)
{
    if(!t||!g||t->active)return false;
    t->group=*g;t->next=0;t->active=1;t->started_ms=now;return true;
}
bool nc_tx_peek(nc_tx_t *t,uint32_t now,uint16_t *id,uint8_t bytes[8])
{
    if(!t||!id||!bytes)return false;
    if((uint32_t)(now-t->started_ms)>NC_TX_MS)t->active=0;
    if(!t->active)return false;
    *id=t->group.base+t->next;memcpy(bytes,t->group.data[t->next],8);return true;
}
void nc_tx_commit(nc_tx_t *t){if(t&&t->active&&++t->next==3)t->active=0;}
