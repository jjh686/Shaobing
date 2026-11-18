#include "rm_chassis_comm/mcu_link.h"
#include <math.h>
#include <string.h>
static bool safe(const rm_mcu_link_t*l){return l->ready&&!l->estop&&!l->manual&&!l->fault_mask;}
void rm_mcu_init(rm_mcu_link_t*l,uint8_t route){if(!l)return;
 memset(l,0,sizeof(*l));
 l->route=route;
 l->requires_stop=true;}
void rm_mcu_set_safety(rm_mcu_link_t*l,bool ready,bool estop,bool manual,uint16_t fault){if(!l)return;
 l->ready=ready;
 l->estop=estop;
 l->manual=manual;
 l->fault_mask=fault;
 if(!safe(l)){l->active=false;
 l->requires_stop=true;}}
void rm_mcu_get_output(rm_mcu_link_t*l,uint32_t now,rm_output_t*out){if(!out)return;
 memset(out,0,sizeof(*out));
 if(!l)return;
 if(l->active&&(uint32_t)(now-l->received_ms)>=l->command.ttl_ms){l->active=false;
 l->requires_stop=true;
 l->timed_out=true;}
 if(!safe(l)){l->active=false;
 l->requires_stop=true;}
 if(l->active&&!l->requires_stop){out->enabled=true;
 out->vx=l->command.vx;
 out->vy=l->command.vy;
 out->wz=l->command.wz;}}
int rm_mcu_receive(rm_mcu_link_t*l,const uint8_t*p,size_t n,uint32_t now){rm_command_t c;rm_output_t ignored;uint16_t diff;
 if(!l||!rm_decode_command(p,n,&c)||c.route!=l->route)return 0;
 rm_mcu_get_output(l,now,&ignored); /* Expiry is enforced even if output polling was delayed. */
 if(!(c.flags&RM_FLAG_ENABLE)){
  /* STOP always removes output. A replay from this session must not roll the
   * sequence baseline backwards or unlock a timeout/safety latch. */
  l->active=false;
  if(l->ack_valid&&c.session==l->command.session){
    diff=(uint16_t)(c.sequence-l->command.sequence);
    if(!diff||diff>=32768u){l->requires_stop=true;return 0;}
  }
  l->command=c;
 l->received_ms=now;
 l->ack_valid=true;
 l->active=false;
 l->timed_out=false;
  l->requires_stop=!safe(l);
 return 1;
 }
 if(!l->ack_valid||l->requires_stop||!safe(l)||c.session!=l->command.session)return 0;
 diff=(uint16_t)(c.sequence-l->command.sequence);
 if(!diff||diff>=32768u)return 0;
 l->command=c;
 l->received_ms=now;
 l->active=true;
 l->timed_out=false;
 return 1;}
void rm_mcu_feedback(rm_mcu_link_t*l,uint32_t now,bool valid,float x,float y,float z,rm_feedback_t*out){rm_output_t current;
 if(!l||!out)return;
 rm_mcu_get_output(l,now,&current);
 memset(out,0,sizeof(*out));
 out->uptime_ms=now;
 out->route=l->route;
 out->session=l->ack_valid?l->command.session:0u;
 out->sequence=l->ack_valid?l->command.sequence:0;
 out->fault_mask=l->fault_mask;
 if(l->ready)out->status|=RM_STATUS_READY;
 if(current.enabled)out->status|=RM_STATUS_ACTIVE;
 if(l->estop)out->status|=RM_STATUS_ESTOP;
 if(l->manual)out->status|=RM_STATUS_MANUAL;
 if(l->timed_out)out->status|=RM_STATUS_TIMEOUT;
 if(l->ack_valid)out->status|=RM_STATUS_ACK_VALID;
 if(valid&&isfinite(x)&&isfinite(y)&&isfinite(z)&&fabsf(x)<=20&&fabsf(y)<=20&&fabsf(z)<=100){
  out->status|=RM_STATUS_MEASURED;
 out->vx=x;
 out->vy=y;
 out->wz=z;}}
