#include "nav_gateway.h"
#include "nav_usb_mux.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static unsigned nav_seen,sp_seen;
static void got(const uint8_t *p,unsigned n,uint32_t t,void *ctx)
{(void)p;(void)t;(void)ctx;if(n==34)nav_seen++;else {assert(n==29);sp_seen++;}}
static void feed(nav_mux_t *m,const uint8_t *p,unsigned n,uint32_t t)
{for(unsigned i=0;i<n;i++)nav_mux_push(m,p[i],t,got,NULL);}
static void seal(nc_group_t *g)
{uint8_t p[20];memcpy(p,g->data,16);memcpy(p+16,g->data[2],4);uint16_t c=nc_crc(p,20);g->data[2][4]=c;g->data[2][5]=c>>8;}
int main(void)
{
 rm_command_t c={.route=2,.session=123,.sequence=65534},d;
 uint8_t usb[34];nc_group_t g,o;nav_gateway_t gate;ng_init(&gate);
 c.flags=1;c.ttl_ms=150;c.vx=.1239f;c.vy=-.0989f;c.wz=-.4999f;
 assert(rm_encode_command(&c,usb));assert(!ng_command(&gate,usb,0,0,&g));
 c.flags=0;c.ttl_ms=0;c.vx=c.vy=c.wz=0;assert(rm_encode_command(&c,usb));
 assert(ng_command(&gate,usb,0,0,&g));assert(nc_command_decode(&g,&d));
 assert(!ng_command(&gate,usb,1,1,&g)); /* duplicate STOP latches stop */
 c.sequence++;assert(rm_encode_command(&c,usb));assert(ng_command(&gate,usb,2,2,&g));
 c.sequence=0;c.flags=1;c.ttl_ms=150;c.vx=.1239f;c.vy=-.0989f;c.wz=-.4999f;
 assert(rm_encode_command(&c,usb));assert(ng_command(&gate,usb,10,13,&g));
 assert(nc_command_decode(&g,&d));assert(d.sequence==0&&d.ttl_ms==137);
 assert(fabsf(d.vx-.123f)<1e-6f&&fabsf(d.vy+.098f)<1e-6f&&fabsf(d.wz+.499f)<1e-6f);
 assert(!ng_command(&gate,usb,10,21,&o));
 nc_rx_t rx={0};assert(!nc_rx_push(&rx,0x330,0x330,0,0,8,g.data[0],100,&o));
 assert(!nc_rx_push(&rx,0x330,0x201,0,0,8,g.data[1],101,&o));
 assert(!nc_rx_push(&rx,0x330,0x331,0,0,8,g.data[1],101,&o));
 assert(nc_rx_push(&rx,0x330,0x332,0,0,8,g.data[2],102,&o));assert(nc_command_decode(&o,&d));
 for(unsigned fault=0;fault<7;fault++){
  rx=(nc_rx_t){0};assert(!nc_rx_push(&rx,0x330,0x330,0,0,8,g.data[0],100,&o));
  uint32_t id=fault==0?0x332:0x331, t=fault==1?111:101;
  assert(!nc_rx_push(&rx,0x330,id,fault==2,fault==3,fault==4?7:8,g.data[1],t,&o));
  if(fault==5)rx.group.data[0][0]^=1;
  if(fault==6)rx.group.data[0][4]^=1;
  assert(!nc_rx_push(&rx,0x330,0x332,0,0,8,g.data[2],t+1,&o));
 }
 o=g;o.data[1][7]=2;seal(&o);assert(!nc_command_decode(&o,&d));
 o=g;o.data[1][6]=0x80;seal(&o);assert(!nc_command_decode(&o,&d));
 c.vx=.301f;assert(!nc_command_group(&c,&o));c.vx=NAN;assert(!nc_command_group(&c,&o));
 nc_tx_t tx={0};uint16_t id;uint8_t bytes[8];
 assert(nc_tx_start(&tx,&g,UINT32_MAX-3));assert(!nc_tx_start(&tx,&g,1));
 assert(nc_tx_peek(&tx,0,&id,bytes)&&id==0x330);assert(nc_tx_peek(&tx,0,&id,bytes)&&id==0x330);
 nc_tx_commit(&tx);assert(nc_tx_peek(&tx,1,&id,bytes)&&id==0x331);assert(!nc_tx_peek(&tx,6,&id,bytes));
 rm_feedback_t f={.route=2,.session=123,.sequence=5,.status=73,.vx=.1234f,.vy=-.2346f,.wz=1.5678f},z;
 assert(nc_feedback_group(&f,&g));assert(nc_feedback_decode(&g,&z));
 assert(fabsf(z.vx-f.vx)<.000501f&&fabsf(z.vy-f.vy)<.000501f&&fabsf(z.wz-f.wz)<.000501f);
 f.wz=40;assert(!nc_feedback_group(&f,&g));
 /* A valid SP payload containing USB magic stays one complete SP frame. */
 uint8_t sp[29]={'S','P',0x5a,28,0x20};uint16_t crc=nc_crc(sp,27);sp[27]=crc;sp[28]=crc>>8;
 nav_mux_t mux={0};feed(&mux,sp,29,10);assert(sp_seen==1&&!nav_seen);
 c=(rm_command_t){.route=2,.session=1};assert(rm_encode_command(&c,usb));
 feed(&mux,usb,12,20);feed(&mux,usb+12,22,21);assert(nav_seen==1);
 usb[30]^=1;feed(&mux,usb,34,30);assert(nav_seen==1);usb[30]^=1;
 feed(&mux,usb,12,40);feed(&mux,usb+12,22,100);assert(nav_seen==1);
 feed(&mux,usb,34,101);assert(nav_seen==2);
 puts("PASS protocol: typed CAN fields, CRC/order/version/DLC/EXT/RTR/drop/timeout, quantization, wrap, BUSY expiry, session STOP, USB SP mux");
}
