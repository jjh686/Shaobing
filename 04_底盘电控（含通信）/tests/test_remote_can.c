#include "remote_can.h"
#include <assert.h>
#include <stdio.h>
static uint32_t t=100;
static RemoteCanSample s={.flags=RC_CAN_ONLINE,.sequence=1,.axes={-660,660,-1,0,0,2047}};
static void send_sample(RemoteCanReceiver *r){nc_group_t g;assert(rc_can_encode(&s,&g));for(unsigned i=0;i<3;i++)rc_can_push(r,RC_CAN_BASE+i,8,g.data[i],t,t);t+=20;s.sequence++;}
int main(void){
 RemoteCanReceiver r={0};nc_group_t g;RemoteCanSample decoded;
 assert(rc_can_encode(&s,&g)&&rc_can_decode(&g,&decoded));assert(decoded.axes[0]==-660&&decoded.axes[5]==2047);
 s.switches[0]=s.switches[1]=1;send_sample(&r);assert(!rc_can_nav(&r,t)); /* cannot boot armed */
 s.switches[0]=0;send_sample(&r);s.switches[0]=1;send_sample(&r);assert(rc_can_nav(&r,t));
 assert(!rc_can_nav(&r,t+101)); /* CAN forwarding stops */
 send_sample(&r);assert(!rc_can_nav(&r,t)); /* reconnect remains disarmed */
 s.switches[0]=0;send_sample(&r);s.switches[0]=1;send_sample(&r);assert(rc_can_nav(&r,t));
 s.generation++;send_sample(&r);assert(!rc_can_nav(&r,t)); /* missed short SBUS fault */
 s.switches[0]=0;send_sample(&r);s.switches[0]=1;send_sample(&r);assert(rc_can_nav(&r,t));
 s.switches[3]=2;send_sample(&r);assert(!rc_can_fresh(&r,t));
 s.switches[3]=0;send_sample(&r);assert(!rc_can_nav(&r,t));
 s.switches[0]=0;send_sample(&r);s.switches[0]=1;send_sample(&r);assert(rc_can_nav(&r,t));
 s.age_ms=95;send_sample(&r);assert(!rc_can_fresh(&r,t));s.age_ms=0;
 s.flags=RC_CAN_ESTOP;send_sample(&r);assert(!rc_can_fresh(&r,t));s.flags=RC_CAN_ONLINE;
 s.switches[0]=0;send_sample(&r);s.switches[0]=1;send_sample(&r);assert(rc_can_nav(&r,t));
 s.sequence--;send_sample(&r);assert(!rc_can_nav(&r,t)); /* duplicate is not fresh */
 s.switches[0]=0;send_sample(&r);s.switches[0]=1;send_sample(&r);
 assert(rc_can_encode(&s,&g));g.data[1][0]^=1;
 for(unsigned i=0;i<3;i++)rc_can_push(&r,RC_CAN_BASE+i,8,g.data[i],t,t);
 assert(!rc_can_nav(&r,t));
 assert(rc_can_encode(&s,&g));
 rc_can_push(&r,RC_CAN_BASE,8,g.data[0],t,t);
 rc_can_push(&r,RC_CAN_BASE+2,8,g.data[2],t,t);assert(!rc_can_nav(&r,t));
 rc_can_push(&r,RC_CAN_BASE,8,g.data[0],t,t);
 rc_can_push(&r,RC_CAN_BASE+1,8,g.data[1],t+11,t+11);assert(!r.assembling);
 assert(!rc_can_push(&r,RC_CAN_BASE,7,g.data[0],t,t));
 assert(!rc_can_push(&r,RC_CAN_BASE,8,g.data[0],t,t+11));
 s.switches[0]=3;assert(!rc_can_encode(&s,&g));s.switches[0]=0;
 s.axes[0]=661;assert(!rc_can_encode(&s,&g));s.axes[0]=0;
 /* Sequence and millisecond wrap do not defeat freshness. */
 r=(RemoteCanReceiver){0};t=0xfffffff0u;s.sequence=65535;s.switches[1]=0;send_sample(&r);
 s.switches[0]=s.switches[1]=1;send_sample(&r);assert(rc_can_nav(&r,t));
 assert(!rc_can_nav(&r,t+101));
 puts("PASS remote CAN: channel encoding, CRC, order/drop/DLC/age, duplicate, sequence/tick wrap, source age, lost link, CH10 stop, fault generation, reconnect/rearm");
}
