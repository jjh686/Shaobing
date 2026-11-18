#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <math.h>
#include <string.h>
#include "rm_chassis_comm/gimbal_protocol.h"
static void seal(uint8_t*p){uint8_t c=255;uint16_t d=65535;
 for(unsigned i=0;i<3;i++){c^=p[i];for(unsigned b=0;b<8;b++)c=(uint8_t)((c>>1)^((c&1)?0x8c:0));}p[3]=c;
 for(unsigned i=0;i<44;i++){d^=p[i];for(unsigned b=0;b<8;b++)d=(uint16_t)((d>>1)^((d&1)?0x8408:0));}p[44]=(uint8_t)d;p[45]=(uint8_t)(d>>8);}
int main(void){uint8_t b[46],bad[47];rm_gimbal_t d;
 rm_gimbal_t g={0};g.route=2;g.uptime_ms=1000;g.sample_ms=995;g.sample_age_ms=5;
 g.command_received_ms=990;g.session=0x12345678;g.sequence=0x1234;g.status=7;g.yaw=1;g.pitch=-.5f;g.yaw_rate=2;g.pitch_rate=-3;
 assert(rm_encode_gimbal(&g,b));assert(b[0]==0x5a&&b[1]==40&&b[2]==0x22&&b[8]==1&&b[9]==2);
 assert(b[10]==0x34&&b[11]==0x12&&b[16]==0x78&&b[19]==0x12&&b[28]==0&&b[30]==0x80&&b[31]==0x3f);
 assert(rm_decode_gimbal(b,46,&d)&&d.sample_ms==995&&d.command_received_ms==990&&d.pitch_rate==-3);
 assert(!rm_decode_gimbal(b,45,&d)&&!rm_decode_gimbal(b,47,&d));
 for(unsigned i=0;i<46;i++){memcpy(bad,b,46);bad[i]^=1;assert(!rm_decode_gimbal(bad,46,&d));}
 for(unsigned k=0;k<7;k++){
  memcpy(bad,b,46);
  if(k==0)bad[8]=2;
  if(k==1)bad[9]=1;
  if(k==2)bad[12]=16;
  if(k==3)bad[14]=6;
  if(k==4)bad[1]=41;
  if(k==5)bad[2]=0x21;
  if(k==6){bad[28]=0;bad[29]=0;bad[30]=0x80;bad[31]=0x7f;}
  seal(bad);assert(!rm_decode_gimbal(bad,46,&d));
 }
 g.yaw=NAN;assert(!rm_encode_gimbal(&g,b));g.yaw=1;
 g.pitch_rate=INFINITY;assert(!rm_encode_gimbal(&g,b));g.pitch_rate=-3;
 g.sample_age_ms=501;g.uptime_ms=g.sample_ms+501;assert(!rm_encode_gimbal(&g,b));
 g.uptime_ms=4;g.sample_ms=0xfffffffeu;g.sample_age_ms=6;assert(rm_encode_gimbal(&g,b));
 assert(rm_decode_gimbal(b,46,&d));
 g.status=0;g.yaw=g.pitch=g.yaw_rate=g.pitch_rate=0;assert(rm_encode_gimbal(&g,b));
 g.yaw=1;assert(!rm_encode_gimbal(&g,b));g.yaw=0;
 g.status=RM_GIMBAL_FAULT;assert(rm_encode_gimbal(&g,b));
 return 0;}
