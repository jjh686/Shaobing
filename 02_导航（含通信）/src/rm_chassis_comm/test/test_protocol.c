#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <string.h>
#include "rm_chassis_comm/protocol.h"
static void seal(uint8_t*p){unsigned i,b;uint8_t a=255;uint16_t c=65535;
 for(i=0;i<3;i++){a^=p[i];for(b=0;b<8;b++)a=(uint8_t)((a>>1)^((a&1)?0x8c:0));}p[3]=a;
 for(i=0;i<32;i++){c^=p[i];for(b=0;b<8;b++)c=(uint16_t)((c>>1)^((c&1)?0x8408:0));}p[32]=(uint8_t)c;p[33]=(uint8_t)(c>>8);}
int main(void){uint8_t f[34],bad[35],out[34];rm_command_t c={0},d;rm_feedback_t fb={0},decoded;rm_parser_t parser;unsigned i,j,count;
 c.uptime_ms=0x12345678;c.session=0x23455a78;c.sequence=0x1234;c.route=2;
 assert(rm_encode_command(&c,f));assert(f[0]==0x5a&&f[1]==28&&f[2]==0x20&&f[4]==0x78&&f[7]==0x12&&f[10]==0x34&&f[11]==0x12);
 assert(rm_decode_command(f,34,&d)&&d.session==c.session&&d.ttl_ms==0&&d.vx==0);
 assert(!rm_decode_command(f,33,&d));assert(!rm_decode_command(f,35,&d));
 c.flags=1;c.ttl_ms=100;c.vx=1;c.vy=-2;c.wz=3;
 assert(rm_encode_command(&c,f));assert(f[20]==0&&f[21]==0&&f[22]==0x80&&f[23]==0x3f);
 assert(rm_decode_command(f,34,&d)&&d.vy==-2&&d.wz==3);
 for(i=0;i<34;i++){memcpy(bad,f,34);bad[i]^=1;assert(!rm_decode_command(bad,34,&d));}
 for(i=0;i<7;i++){
  memcpy(bad,f,34);
  if(i==0)bad[1]=29;
 if(i==1)bad[2]=0x22;
 if(i==2)bad[8]=2;
 if(i==3)bad[9]=3;
  if(i==4)bad[14]=2;
 if(i==5)memset(bad+16,0,4);
  if(i==6){bad[20]=0;bad[21]=0;bad[22]=0x80;bad[23]=0x7f;}
  seal(bad);assert(!rm_decode_command(bad,34,&d));
 }
 c.vx=NAN;assert(!rm_encode_command(&c,out));c.vx=INFINITY;assert(!rm_encode_command(&c,out));
 c.vx=21;assert(!rm_encode_command(&c,out));c.vx=0;c.ttl_ms=19;assert(!rm_encode_command(&c,out));
 c.ttl_ms=251;assert(!rm_encode_command(&c,out));c.flags=0;c.ttl_ms=0;c.vy=1;assert(!rm_encode_command(&c,out));
 c.vy=c.wz=0;assert(rm_encode_command(&c,out));
 fb.route=1;assert(rm_encode_feedback(&fb,out));assert(rm_decode_feedback(out,34,&decoded)&&decoded.session==0);
 fb.status=RM_STATUS_ACK_VALID;assert(!rm_encode_feedback(&fb,out));fb.session=1;fb.status|=RM_STATUS_MEASURED;fb.vx=2;
 assert(rm_encode_feedback(&fb,out)&&rm_decode_feedback(out,34,&decoded)&&decoded.vx==2);
 fb.status&=(uint16_t)~RM_STATUS_MEASURED;assert(!rm_encode_feedback(&fb,out));
 /* Split/coalesced byte feeds, noise, payload SOF and a bad frame followed by a good one. */
 rm_parser_init(&parser);count=0;
 for(j=0;j<2;j++)for(i=0;i<34;i++)count+=(unsigned)rm_parser_push(&parser,f[i],out);
 assert(count==2&&memcmp(f,out,34)==0);
 for(i=0;i<10;i++)assert(!rm_parser_push(&parser,0x5a,out));
 memcpy(bad,f,34);bad[33]^=1;
 for(i=0;i<34;i++)assert(!rm_parser_push(&parser,bad[i],out));
 count=0;for(i=0;i<34;i++){count+=(unsigned)rm_parser_push(&parser,f[i],out);assert(parser.used<=34);}
 assert(count==1&&memcmp(f,out,34)==0);
 rm_parser_init(&parser);for(i=0;i<12;i++)assert(!rm_parser_push(&parser,f[i],out));
 count=0;for(i=0;i<34;i++)count+=(unsigned)rm_parser_push(&parser,f[i],out);assert(count==1);
 return 0;}
