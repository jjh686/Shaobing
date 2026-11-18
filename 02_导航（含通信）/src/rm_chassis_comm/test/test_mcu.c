#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include "rm_chassis_comm/mcu_link.h"
static int deliver(rm_mcu_link_t*l,rm_command_t*c,uint32_t t){uint8_t b[34];assert(rm_encode_command(c,b));
 return rm_mcu_receive(l,b,34,t);}
static void zero(rm_mcu_link_t*l,uint32_t t){rm_output_t o;rm_mcu_get_output(l,t,&o);assert(!o.enabled&&o.vx==0&&o.vy==0&&o.wz==0);}
int main(void){rm_mcu_link_t l;rm_command_t c={0};rm_output_t o;rm_feedback_t f;uint8_t b[34];
 rm_mcu_init(&l,1);zero(&l,0);rm_mcu_feedback(&l,0,false,1,2,3,&f);
 assert(f.session==0&&!(f.status&RM_STATUS_ACK_VALID)&&!(f.status&RM_STATUS_MEASURED)&&f.vx==0);
 assert(rm_encode_feedback(&f,b));rm_mcu_set_safety(&l,true,false,false,0);
 c.session=7;c.route=1;c.sequence=65534;c.flags=1;c.ttl_ms=100;c.vx=2;
 assert(!deliver(&l,&c,1));zero(&l,1);
 c.flags=0;c.ttl_ms=0;c.vx=0;assert(deliver(&l,&c,2));
 c.flags=1;c.ttl_ms=100;c.vx=2;c.sequence=65535;assert(deliver(&l,&c,3));
 rm_mcu_get_output(&l,102,&o);assert(o.enabled&&o.vx==2);
 assert(!deliver(&l,&c,102));zero(&l,103); /* Duplicate did not renew TTL. */
 rm_mcu_feedback(&l,103,false,4,5,6,&f);assert((f.status&RM_STATUS_TIMEOUT)&&!(f.status&RM_STATUS_MEASURED)&&f.vx==0);
 c.sequence=0;assert(!deliver(&l,&c,104)); /* Expired link requires STOP. */
 c.flags=0;c.ttl_ms=0;c.vx=0;assert(deliver(&l,&c,105));
 rm_mcu_feedback(&l,105,true,1,-2,3,&f);assert(!(f.status&RM_STATUS_TIMEOUT)&&(f.status&RM_STATUS_MEASURED)&&f.vy==-2);
 c.flags=1;c.ttl_ms=20;c.vx=1;c.sequence=1;assert(deliver(&l,&c,106));
 c.sequence=0;assert(!deliver(&l,&c,107));c.sequence=32769;assert(!deliver(&l,&c,108));
 c.sequence=2;c.session=8;assert(!deliver(&l,&c,109));
 c.flags=0;c.ttl_ms=0;c.vx=0;assert(deliver(&l,&c,110));
 c.flags=1;c.ttl_ms=100;c.vx=1;c.sequence=3;assert(deliver(&l,&c,111));
 for(unsigned safety=0;safety<4;safety++){
  rm_mcu_set_safety(&l,safety!=0,safety==1,safety==2,safety==3?1:0);zero(&l,112);
  rm_mcu_set_safety(&l,true,false,false,0);c.sequence++;assert(!deliver(&l,&c,113));
  c.flags=0;c.ttl_ms=0;c.vx=0;assert(deliver(&l,&c,114));
  c.flags=1;c.ttl_ms=100;c.vx=1;c.sequence++;assert(deliver(&l,&c,115));
 }
 c.route=2;c.sequence++;assert(!deliver(&l,&c,116));c.route=1;
 assert(rm_encode_command(&c,b));b[32]^=1;assert(!rm_mcu_receive(&l,b,34,200));zero(&l,215);
 c.flags=0;c.ttl_ms=0;c.vx=0;c.session=9;c.sequence=65535;assert(deliver(&l,&c,0xfffffff0u));
 c.flags=1;c.ttl_ms=20;c.vx=1;c.sequence=0;assert(deliver(&l,&c,0xfffffff5u));
 rm_mcu_get_output(&l,8,&o);assert(o.enabled);zero(&l,9); /* uint32 wrap and exact TTL boundary. */
 rm_mcu_feedback(&l,10,true,NAN,0,0,&f);assert(!(f.status&RM_STATUS_MEASURED)&&f.vx==0);
 /* Replayed STOP cannot reset the baseline after timeout, or permit replayed active. */
 rm_mcu_init(&l,1);rm_mcu_set_safety(&l,true,false,false,0);
 c.session=12;c.sequence=10;c.flags=0;c.ttl_ms=0;c.vx=0;
 assert(deliver(&l,&c,100));
 c.sequence=11;c.flags=1;c.ttl_ms=20;c.vx=1;
 assert(deliver(&l,&c,101));zero(&l,121);
 c.sequence=10;c.flags=0;c.ttl_ms=0;c.vx=0;
 assert(!deliver(&l,&c,122));
 c.sequence=11;c.flags=1;c.ttl_ms=20;c.vx=1;
 assert(!deliver(&l,&c,123));zero(&l,123);
 c.sequence=12;c.flags=0;c.ttl_ms=0;c.vx=0;
 assert(deliver(&l,&c,124));
 c.sequence=13;c.flags=1;c.ttl_ms=20;c.vx=1;
 assert(deliver(&l,&c,125));
 /* Even a rejected stale STOP stops immediately, without making its old active valid. */
 c.sequence=12;c.flags=0;c.ttl_ms=0;c.vx=0;
 assert(!deliver(&l,&c,126));zero(&l,126);
 c.sequence=14;c.flags=1;c.ttl_ms=20;c.vx=1;
 assert(!deliver(&l,&c,127));
 c.sequence=15;c.flags=0;c.ttl_ms=0;c.vx=0;
 assert(deliver(&l,&c,128));
 c.sequence=16;c.flags=1;c.ttl_ms=20;c.vx=1;
 assert(deliver(&l,&c,129));
 rm_mcu_set_safety(&l,false,false,false,0);
 rm_mcu_set_safety(&l,true,false,false,0);
 c.sequence=15;c.flags=0;c.ttl_ms=0;c.vx=0;
 assert(!deliver(&l,&c,130));
 c.sequence=17;c.flags=1;c.ttl_ms=20;c.vx=1;
 assert(!deliver(&l,&c,131));zero(&l,131);
 return 0;}
