#include "rm_chassis_comm/protocol.h"
#include <float.h>
#include <math.h>
#include <string.h>
#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error IEEE754 binary32 float required
#endif
typedef char rm_float_must_be_four_bytes[(sizeof(float)==4)?1:-1];
static uint8_t crc8(const uint8_t *p,size_t n){uint8_t c=255;size_t i;unsigned b;
 for(i=0;i<n;i++){c^=p[i];for(b=0;b<8;b++)c=(uint8_t)((c>>1)^((c&1)?0x8c:0));}return c;}
static uint16_t crc16(const uint8_t *p,size_t n){uint16_t c=65535;size_t i;unsigned b;
 for(i=0;i<n;i++){c^=p[i];for(b=0;b<8;b++)c=(uint16_t)((c>>1)^((c&1)?0x8408:0));}return c;}
static uint16_t u16(const uint8_t*p){return (uint16_t)(p[0]|((uint16_t)p[1]<<8));}
static uint32_t u32(const uint8_t*p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static void w16(uint8_t*p,uint16_t v){p[0]=(uint8_t)v;p[1]=(uint8_t)(v>>8);}
static void w32(uint8_t*p,uint32_t v){unsigned i;for(i=0;i<4;i++)p[i]=(uint8_t)(v>>(8*i));}
static float f32(const uint8_t*p){uint32_t v=u32(p);float f;
 memcpy(&f,&v,4);
 return f;}
static void wf32(uint8_t*p,float f){uint32_t v;
 memcpy(&v,&f,4);w32(p,v);}
static int route_ok(uint8_t r){return r==1||r==2;}
static int velocity_ok(float x,float y,float z){return isfinite(x)&&isfinite(y)&&isfinite(z)&&fabsf(x)<=20&&fabsf(y)<=20&&fabsf(z)<=100;}
static int frame_ok(const uint8_t*p,size_t n){return p&&n==RM_FRAME_SIZE&&p[0]==0x5a&&p[1]==28&&
 (p[2]==RM_ID_COMMAND||p[2]==RM_ID_FEEDBACK)&&p[3]==crc8(p,3)&&u16(p+32)==crc16(p,32);}
static int command_ok(const rm_command_t*c){return c&&route_ok(c->route)&&c->session&&!(c->flags&~RM_FLAG_ENABLE)&&
 velocity_ok(c->vx,c->vy,c->wz)&&((c->flags&RM_FLAG_ENABLE)?(c->ttl_ms>=20&&c->ttl_ms<=250):
 (c->ttl_ms==0&&c->vx==0&&c->vy==0&&c->wz==0));}
static int feedback_ok(const rm_feedback_t*f){return f&&route_ok(f->route)&&(!(f->status&RM_STATUS_ACK_VALID)||f->session)&&!(f->status&~127u)&&
 velocity_ok(f->vx,f->vy,f->wz)&&((f->status&RM_STATUS_MEASURED)||(f->vx==0&&f->vy==0&&f->wz==0));}
static void base(uint8_t*p,uint8_t id,uint32_t uptime,uint8_t route,uint16_t seq,uint16_t a,uint16_t b,uint32_t session,float x,float y,float z){
 memset(p,0,RM_FRAME_SIZE);p[0]=0x5a;p[1]=28;p[2]=id;p[3]=crc8(p,3);w32(p+4,uptime);p[8]=1;p[9]=route;
 w16(p+10,seq);w16(p+12,a);w16(p+14,b);w32(p+16,session);wf32(p+20,x);wf32(p+24,y);wf32(p+28,z);w16(p+32,crc16(p,32));}
int rm_encode_command(const rm_command_t*c,uint8_t out[RM_FRAME_SIZE]){if(!out||!command_ok(c))return 0;
 base(out,RM_ID_COMMAND,c->uptime_ms,c->route,c->sequence,c->ttl_ms,c->flags,c->session,c->vx,c->vy,c->wz);
 return 1;}
int rm_decode_command(const uint8_t*p,size_t n,rm_command_t*out){rm_command_t c;
 if(!out||!frame_ok(p,n)||p[2]!=RM_ID_COMMAND||p[8]!=1)return 0;
 c.uptime_ms=u32(p+4);c.route=p[9];c.sequence=u16(p+10);c.ttl_ms=u16(p+12);c.flags=u16(p+14);c.session=u32(p+16);
 c.vx=f32(p+20);c.vy=f32(p+24);c.wz=f32(p+28);
 if(!command_ok(&c))return 0;
 *out=c;
 return 1;}
int rm_encode_feedback(const rm_feedback_t*f,uint8_t out[RM_FRAME_SIZE]){if(!out||!feedback_ok(f))return 0;
 base(out,RM_ID_FEEDBACK,f->uptime_ms,f->route,f->sequence,f->status,f->fault_mask,f->session,f->vx,f->vy,f->wz);
 return 1;}
int rm_decode_feedback(const uint8_t*p,size_t n,rm_feedback_t*out){rm_feedback_t f;
 if(!out||!frame_ok(p,n)||p[2]!=RM_ID_FEEDBACK||p[8]!=1)return 0;
 f.uptime_ms=u32(p+4);f.route=p[9];f.sequence=u16(p+10);f.status=u16(p+12);f.fault_mask=u16(p+14);f.session=u32(p+16);
 f.vx=f32(p+20);f.vy=f32(p+24);f.wz=f32(p+28);
 if(!feedback_ok(&f))return 0;
 *out=f;
 return 1;}
void rm_parser_init(rm_parser_t*p){if(p)p->used=0;}
int rm_parser_push(rm_parser_t*p,uint8_t b,uint8_t out[RM_FRAME_SIZE]){if(!p||!out)return 0;
 if(p->used>=RM_FRAME_SIZE)p->used=0;
 p->data[p->used++]=b;
 while(p->used){
  if(p->data[0]!=0x5a||(p->used>=2&&p->data[1]!=28)||(p->used>=3&&p->data[2]!=RM_ID_COMMAND&&p->data[2]!=RM_ID_FEEDBACK)||
    (p->used>=4&&p->data[3]!=crc8(p->data,3))){memmove(p->data,p->data+1,--p->used);continue;}
  if(p->used<RM_FRAME_SIZE)return 0;
  if(frame_ok(p->data,p->used)){memcpy(out,p->data,RM_FRAME_SIZE);p->used=0;
 return 1;}
  memmove(p->data,p->data+1,--p->used);
 }return 0;}
