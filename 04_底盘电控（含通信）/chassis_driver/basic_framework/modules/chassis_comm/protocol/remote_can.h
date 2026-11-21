#ifndef REMOTE_CAN_H
#define REMOTE_CAN_H
/* RC wire v1: 3 ordered standard data frames, 8 bytes each, LE, CRC16.
 * Navigation transport remains unchanged. No raw struct is sent on the wire. */
#include "nav_can.h"
#include <string.h>
#define RC_CAN_BASE 0x340u
#define RC_CAN_TIMEOUT_MS 100u
#define RC_CAN_ONLINE 1u
#define RC_CAN_ESTOP 2u
#define RC_CAN_VERSION 1u
typedef struct {
    int16_t axes[6]; /* CH1..4: centred/deadband +/-660; CH5..6: raw SBUS 0..2047 */
    uint8_t switches[4]; /* CH7..10: up=0, mid=1, down=2 */
    uint8_t flags;
    uint16_t sequence, generation, age_ms;
} RemoteCanSample;
typedef struct {
    uint32_t bad_dlc,queue_expired,order_errors,group_timeouts,decode_errors,sequence_errors;
    uint32_t source_faults,transport_timeouts;
} RemoteCanDiagnostics;
typedef struct {
    RemoteCanDiagnostics diagnostics;
    nc_group_t group;
    RemoteCanSample sample;
    uint32_t group_ms, received_ms;
    uint16_t sequence, generation;
    uint8_t next, assembling, seen, valid, off_seen, armed;
    uint32_t accepted, rejected;
} RemoteCanReceiver;
static inline void rc_put16(uint8_t *p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static inline uint16_t rc_get16(const uint8_t *p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static inline int16_t rc_signed(uint16_t v){return (int16_t)(v<32768u?(int32_t)v:(int32_t)v-65536);}
static inline bool rc_sample_ok(const RemoteCanSample *s){
    if(s->flags & ~(RC_CAN_ONLINE|RC_CAN_ESTOP))return false;
    for(unsigned i=0;i<4;i++)if(s->switches[i]>2||s->axes[i]<-660||s->axes[i]>660)return false;
    for(unsigned i=4;i<6;i++)if(s->axes[i]<0||s->axes[i]>2047)return false;
    return true;
}
static inline bool rc_can_encode(const RemoteCanSample *s,nc_group_t *g){
    if(!rc_sample_ok(s))return false;
    memset(g,0,sizeof(*g));g->base=RC_CAN_BASE;uint8_t b[24]={0};
    b[0]=RC_CAN_VERSION;b[1]=s->flags;rc_put16(b+2,s->sequence);
    rc_put16(b+4,s->generation);rc_put16(b+6,s->age_ms);
    for(unsigned i=0;i<6;i++)rc_put16(b+8+2*i,(uint16_t)s->axes[i]);
    for(unsigned i=0;i<4;i++)b[20]|=(uint8_t)(s->switches[i]<<(2*i));
    rc_put16(b+22,nc_crc(b,22));memcpy(g->data,b,24);return true;
}
static inline bool rc_can_decode(const nc_group_t *g,RemoteCanSample *s){
    uint8_t b[24];memcpy(b,g->data,24);
    if(g->base!=RC_CAN_BASE||b[0]!=RC_CAN_VERSION||b[21]||rc_get16(b+22)!=nc_crc(b,22))return false;
    memset(s,0,sizeof(*s));s->flags=b[1];s->sequence=rc_get16(b+2);
    s->generation=rc_get16(b+4);s->age_ms=rc_get16(b+6);
    for(unsigned i=0;i<6;i++)s->axes[i]=rc_signed(rc_get16(b+8+2*i));
    for(unsigned i=0;i<4;i++)s->switches[i]=(b[20]>>(2*i))&3;
    return rc_sample_ok(s);
}
static inline void rc_can_fault(RemoteCanReceiver *r){r->valid=r->armed=r->off_seen=r->assembling=0;}
static inline bool rc_can_fresh(RemoteCanReceiver *r,uint32_t now){
    if(!r->valid)return false;
    if((uint32_t)(now-r->received_ms)+r->sample.age_ms>RC_CAN_TIMEOUT_MS){
        r->diagnostics.transport_timeouts++;rc_can_fault(r);return false;
    }
    return true;
}
static inline bool rc_can_nav(RemoteCanReceiver *r,uint32_t now){return rc_can_fresh(r,now)&&r->armed;}
/* Faults require a valid non-navigation sample before entering CH7+CH8 MID.
 * Fault generation carries SBUS faults occurring between CAN polling cycles. */
static inline bool rc_can_push(RemoteCanReceiver *r,uint16_t id,unsigned dlc,const uint8_t *p,uint32_t received,uint32_t now){
    if(id<RC_CAN_BASE||id>RC_CAN_BASE+2)return false;
    bool timed_out=!r->seen||(uint32_t)(now-r->received_ms)>RC_CAN_TIMEOUT_MS;
    (void)rc_can_fresh(r,now);
    if(dlc!=8||!p){r->diagnostics.bad_dlc++;rc_can_fault(r);r->rejected++;return false;}
    if((uint32_t)(now-received)>NC_GROUP_MS){r->diagnostics.queue_expired++;rc_can_fault(r);r->rejected++;return false;}
    if(id==RC_CAN_BASE){r->assembling=1;r->next=0;r->group_ms=received;r->group.base=RC_CAN_BASE;}
    if(!r->assembling||id!=RC_CAN_BASE+r->next){r->diagnostics.order_errors++;rc_can_fault(r);r->rejected++;return false;}
    if((uint32_t)(received-r->group_ms)>NC_GROUP_MS){r->diagnostics.group_timeouts++;rc_can_fault(r);r->rejected++;return false;}
    memcpy(r->group.data[r->next++],p,8);if(r->next!=3)return false;r->assembling=0;
    RemoteCanSample s;
    if(!rc_can_decode(&r->group,&s)){r->diagnostics.decode_errors++;rc_can_fault(r);r->rejected++;return false;}
    uint16_t delta=(uint16_t)(s.sequence-r->sequence);
    if(r->seen&&!timed_out&&(delta==0||delta>=32768u)){r->diagnostics.sequence_errors++;rc_can_fault(r);r->rejected++;return false;}
    if(r->seen&&s.generation!=r->generation)r->diagnostics.source_faults++;
    if(timed_out||(r->seen&&s.generation!=r->generation))rc_can_fault(r);
    r->seen=1;r->sequence=s.sequence;r->generation=s.generation;r->sample=s;
    /* Include time spent assembling/queuing the group in source age. */
    uint32_t age=(uint32_t)s.age_ms+(uint32_t)(now-r->group_ms);
    r->received_ms=now;r->sample.age_ms=age>65535u?65535u:(uint16_t)age;r->accepted++;
    if(!(s.flags&RC_CAN_ONLINE)||(s.flags&RC_CAN_ESTOP)||s.switches[3]==2||age>RC_CAN_TIMEOUT_MS){rc_can_fault(r);return true;}
    r->valid=1;
    bool nav=s.switches[0]==1&&s.switches[1]==1;
    if(!nav){r->armed=0;r->off_seen=1;}else if(r->off_seen){r->armed=1;}
    return true;
}
#endif
