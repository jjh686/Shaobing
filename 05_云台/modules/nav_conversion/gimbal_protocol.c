#include "rm_chassis_comm/gimbal_protocol.h"
#include <math.h>
#include <float.h>
#include <string.h>
typedef char rm_gimbal_binary32_required[(sizeof(float)==4&&FLT_RADIX==2&&FLT_MANT_DIG==24&&FLT_MAX_EXP==128)?1:-1];
static uint8_t c8(const uint8_t*p,size_t n){uint8_t c=255;for(size_t i=0;i<n;i++){c^=p[i];for(unsigned b=0;b<8;b++)c=(uint8_t)((c>>1)^((c&1)?0x8c:0));}return c;}
static uint16_t c16(const uint8_t*p,size_t n){uint16_t c=65535;for(size_t i=0;i<n;i++){c^=p[i];for(unsigned b=0;b<8;b++)c=(uint16_t)((c>>1)^((c&1)?0x8408:0));}return c;}
static uint16_t r16(const uint8_t*p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static uint32_t r32(const uint8_t*p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void w16(uint8_t*p,uint16_t x){p[0]=(uint8_t)x;p[1]=(uint8_t)(x>>8);}
static void w32(uint8_t*p,uint32_t x){for(unsigned i=0;i<4;i++)p[i]=(uint8_t)(x>>(8*i));}
static float rf(const uint8_t*p){uint32_t x=r32(p);float f;memcpy(&f,&x,4);return f;}
static void wf(uint8_t*p,float f){uint32_t x;memcpy(&x,&f,4);w32(p,x);}
static int valid(const rm_gimbal_t*g){return g&&g->route==2&&!(g->status&~15u)&&
 (uint32_t)(g->uptime_ms-g->sample_ms)==g->sample_age_ms&&g->sample_age_ms<=500&&
 isfinite(g->yaw)&&isfinite(g->pitch)&&isfinite(g->yaw_rate)&&isfinite(g->pitch_rate)&&
 fabsf(g->yaw)<=3.141593f&&fabsf(g->pitch)<=3.141593f&&
 (!(g->status&RM_GIMBAL_YAW_VALID)?(g->yaw==0&&g->yaw_rate==0):1)&&
 (!(g->status&RM_GIMBAL_PITCH_VALID)?(g->pitch==0&&g->pitch_rate==0):1);}
int rm_encode_gimbal(const rm_gimbal_t*g,uint8_t out[RM_GIMBAL_FRAME_SIZE]){
 if(!out||!valid(g))return 0;
 memset(out,0,RM_GIMBAL_FRAME_SIZE);out[0]=0x5a;out[1]=40;out[2]=RM_ID_GIMBAL;out[3]=c8(out,3);
 w32(out+4,g->uptime_ms);out[8]=1;out[9]=g->route;w16(out+10,g->sequence);w16(out+12,g->status);
 w16(out+14,g->sample_age_ms);w32(out+16,g->session);w32(out+20,g->sample_ms);w32(out+24,g->command_received_ms);
 wf(out+28,g->yaw);wf(out+32,g->pitch);wf(out+36,g->yaw_rate);wf(out+40,g->pitch_rate);w16(out+44,c16(out,44));return 1;}
int rm_decode_gimbal(const uint8_t*p,size_t n,rm_gimbal_t*out){rm_gimbal_t g;
 if(!p||!out||n!=RM_GIMBAL_FRAME_SIZE||p[0]!=0x5a||p[1]!=40||p[2]!=RM_ID_GIMBAL||p[3]!=c8(p,3)||p[8]!=1||r16(p+44)!=c16(p,44))return 0;
 g.uptime_ms=r32(p+4);g.route=p[9];g.sequence=r16(p+10);g.status=r16(p+12);g.sample_age_ms=r16(p+14);g.session=r32(p+16);
 g.sample_ms=r32(p+20);g.command_received_ms=r32(p+24);g.yaw=rf(p+28);g.pitch=rf(p+32);g.yaw_rate=rf(p+36);g.pitch_rate=rf(p+40);
 if(!valid(&g))return 0;
 *out=g;return 1;}
