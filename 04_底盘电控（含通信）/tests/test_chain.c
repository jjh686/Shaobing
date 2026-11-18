/* Execute both actual firmware ports; only peripheral I/O is simulated. */
#include "bsp_can.h"
#include "bsp_usb.h"
#include "remote_control.h"
#include "nav_gateway_port.h"
#include "chassis_comm_port.h"
#include "chassis_comm_config.h"
#include "nav_can.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
CAN_HandleTypeDef hcan1,hcan2;int huart3;
static uint32_t tick;static int side,rc_online=1,configured=1,usb_busy,can_busy;
static CANInstance inst[2][6];static unsigned registered[2],ack_count;
static unsigned rx_fifo[2][6];
typedef struct { unsigned index; uint8_t bytes[8]; } WireFrame;
static WireFrame wire[2][2][32];
static unsigned wire_count[2][2];
static void deliver(unsigned board)
{
 /* Emulate both FIFO interrupts pending: FIFO0 can be serviced before FIFO1. */
 for(unsigned fifo=0;fifo<2;fifo++){
  for(unsigned n=0;n<wire_count[board][fifo];n++){
   WireFrame *p=&wire[board][fifo][n];CANInstance *i=&inst[board][p->index];
   memcpy(i->rx_buff,p->bytes,8);i->rx_len=8;i->callback(i);
  }
  wire_count[board][fifo]=0;
 }
}
static FS_RC_ctrl_t remote;static rm_feedback_t ack;
static Chassis_Ctrl_Cmd_s output;static Chassis_Upload_Data_s measured={.real_vx=.1234f,.real_vy=-.2346f,.real_wz=45};
uint32_t HAL_GetTick(void){return tick;}
FS_RC_ctrl_t *FS_RemoteControlInit(int *p){assert(p==&huart3);return &remote;}
uint8_t FS_RemoteControlIsOnline(void){return rc_online;}
uint8_t FS_RemoteControlGetSnapshot(FS_RC_ctrl_t *p){*p=remote;p->received_ms=tick;return rc_online;}
uint8_t USBIsConfigured(void){return configured;}
uint8_t USBTransmit(uint8_t *p,uint16_t n)
{if(usb_busy)return USBD_BUSY;if(n==34){assert(rm_decode_feedback(p,n,&ack));ack_count++;}else assert(n==46);return USBD_OK;}
CANInstance *CANRegister(CAN_Init_Config_s *c)
{assert(c->can_handle==(side==0?&hcan1:&hcan2));assert(registered[side]<(side==0?3:6));
 unsigned n=registered[side]++;
 rx_fifo[side][n]=c->rx_fifo_policy==CAN_RX_FIFO_POLICY_FIFO0?0:
  c->rx_fifo_policy==CAN_RX_FIFO_POLICY_FIFO1?1:((n&1)?0:1);
 CANInstance *i=&inst[side][n];i->rx_id=c->rx_id;i->callback=c->can_module_callback;return i;}
unsigned HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *p){assert(p==(side==0?&hcan1:&hcan2));return can_busy?0:3;}
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *p,uint32_t m){assert(p==(side==0?&hcan1:&hcan2));(void)m;return 0;}
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *p,uint32_t m){assert(p==(side==0?&hcan1:&hcan2));(void)m;return HAL_OK;}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *p,CAN_TxHeaderTypeDef *h,uint8_t *b,uint32_t *m)
{
 assert(p==(side==0?&hcan1:&hcan2));
 assert(h->DLC==8&&h->IDE==CAN_ID_STD&&h->RTR==CAN_RTR_DATA);*m=1;
 for(unsigned j=0;j<registered[!side];j++){CANInstance *i=&inst[!side][j];if(i->rx_id==h->StdId){
 unsigned f=rx_fifo[!side][j],n=wire_count[!side][f]++;assert(n<32);
 wire[!side][f][n].index=j;memcpy(wire[!side][f][n].bytes,b,8);return HAL_OK;}}
 assert(!"unexpected CAN ID");return -1;
}
static void step(int motors)
{side=0;NavGatewayTask();deliver(1);side=1;ChassisCommPortCommand(&output,motors);ChassisCommPortFeedback(&measured,motors);deliver(0);side=0;NavGatewayTask();}
static void send_velocity(uint16_t seq,int enabled,float vx,float vy,float wz)
{
 rm_command_t c={.route=2,.session=0x10203040,.sequence=seq,.uptime_ms=tick,
 .flags=enabled?1:0,.ttl_ms=enabled?150:0,.vx=enabled?vx:0,.vy=enabled?vy:0,.wz=enabled?wz:0};
 uint8_t b[34];assert(rm_encode_command(&c,b));NavGatewayReceive(b,34,tick);
}
static void send(uint16_t seq,int enabled){send_velocity(seq,enabled,.1239f,-.0989f,.1239f);}
static void stopped(void){assert(output.chassis_mode==CHASSIS_ZERO_FORCE&&output.vx==0&&output.vy==0&&output.wz==0);}
int main(void)
{
 assert(CHASSIS_COMM_VX_SIGN==1.0f && CHASSIS_COMM_VY_SIGN==-1.0f && CHASSIS_COMM_WZ_SIGN==-1.0f);
 remote.channel[6]=remote.channel[7]=0;side=0;NavGatewayInit();side=1;ChassisCommPortInit();
 tick=0;step(1); /* Healthy non-nav sample is required before arming. */
 remote.channel[6]=remote.channel[7]=1;
 tick=20;send(1,0);step(1);assert(ack_count==1&&ack.sequence==1&&(ack.status&RM_STATUS_ACK_VALID));
 assert(fabsf(ack.vx-.123f)<1e-6f&&fabsf(ack.vy-.235f)<1e-6f&&fabsf(ack.wz+.785f)<1e-6f);
 tick=40;send(2,1);step(1);
#if CHASSIS_COMM_ALLOW_MOTION && CHASSIS_COMM_AXES_VERIFIED
 assert(output.chassis_mode==CHASSIS_NO_FOLLOW&&output.offset_angle==0);
 const float scale=6.28318530718f*.06f/(360*19);
 assert(fabsf(output.vx*scale-.123f)<1e-6f&&fabsf(output.vy*scale-.098f)<1e-6f);
 assert(fabsf(output.wz*(1000.0f*.01745329252f*scale)+.123f)<1e-6f);
 assert((ack.status&RM_STATUS_ACTIVE)&&ack.fault_mask==0);
#else
 stopped();assert((ack.fault_mask&NC_FAULT_LOCK)&&(ack.fault_mask&NC_FAULT_UNITS));
#endif
 rc_online=0;remote.fault_generation++;tick=60;step(1);stopped();
 rc_online=1;tick=80;send(3,1);step(1);stopped(); /* reconnection cannot auto-arm */
 remote.channel[6]=0;tick=100;send(4,0);step(1);
 remote.channel[6]=1;tick=120;send(5,1);step(1);
 tick=280;step(1);stopped(); /* navigation TTL */
 remote.channel[9]=2;tick=300;send(6,1);step(1);stopped(); /* CH10 stop */
 remote.channel[9]=0;tick=320;send(7,1);step(1);stopped();
 remote.channel[6]=0;tick=340;send(8,0);step(1);
 remote.channel[6]=1;tick=360;send(9,1);step(0);stopped(); /* motor offline */
 tick=380;send(10,0);step(1);
 tick=400;send(11,1);tick=411;step(1);stopped();assert(ack.sequence==10); /* stale USB */
 tick=431;nc_group_t g;rm_command_t c={.route=2,.session=9};assert(nc_command_group(&c,&g));
 for(unsigned n=0;n<6;n++)for(unsigned j=0;j<3;j++){CANInstance *i=&inst[1][j];memcpy(i->rx_buff,g.data[j],8);i->rx_len=8;i->callback(i);}
 step(1);stopped();assert(ack.fault_mask&NC_FAULT_RX);
 remote.channel[6]=0;tick=451;send(12,0);step(1);assert(!(ack.fault_mask&NC_FAULT_RX));
 remote.channel[6]=1;tick=471;send(13,0);step(1);
 usb_busy=1;tick=491;send(14,0);step(1);unsigned before=ack_count;
 tick=502;usb_busy=0;step(1);assert(ack_count==before);
 configured=0;tick=511;step(1);configured=1;tick=531;send(15,1);step(1);stopped();
 tick=551;send(16,0);step(1);
 for(unsigned n=0;n<10000;n++){
  tick+=20;send((uint16_t)(17+n),1);step(1);
  assert(ack.session==0x10203040&&(ack.status&RM_STATUS_ACK_VALID));
#if CHASSIS_COMM_ALLOW_MOTION && CHASSIS_COMM_AXES_VERIFIED
  assert(ack.sequence==(uint16_t)(17+n));assert(output.chassis_mode==CHASSIS_NO_FOLLOW);
#else
  assert(ack.sequence==16);stopped();
#endif
 }
#if CHASSIS_COMM_ALLOW_MOTION && CHASSIS_COMM_AXES_VERIFIED
 /* Six independent ROS directions: driver signs differ only for Y/yaw.
  * Simulated measured velocity checks feedback as well as commands. */
 const float directions[6][3]={{.05f,0,0},{-.05f,0,0},{0,.05f,0},
                              {0,-.05f,0},{0,0,.1f},{0,0,-.1f}};
 for(unsigned n=0;n<6;n++){
  const float *v=directions[n];
  measured.real_vx=v[0];measured.real_vy=-v[1];
  measured.real_wz=-v[2]/.01745329252f;
  tick+=20;send_velocity((uint16_t)(10017+n),1,v[0],v[1],v[2]);step(1);
  assert(output.chassis_mode==CHASSIS_NO_FOLLOW);
  assert(fabsf(output.vx*scale-v[0])<1e-6f);
  assert(fabsf(output.vy*scale+v[1])<1e-6f);
  assert(fabsf(output.wz*(1000.0f*.01745329252f*scale)+v[2])<1e-6f);
  assert(fabsf(ack.vx-v[0])<.00051f && fabsf(ack.vy-v[1])<.00051f && fabsf(ack.wz-v[2])<.00051f);
 }
 puts("PASS six axes: forward/back, left/right, ccw/cw commands and measured feedback signs");
#endif
 /* Interrupt forwarding while PC navigation commands continue arriving. */
 tick+=101;side=1;ChassisCommPortCommand(&output,1);stopped();
 puts("PASS actual ports: 10000 USB/CAN round trips with forwarded RC, failsafe, CH10 stop, rearm, RC transport timeout, motion lock, motor loss, TTL, queue overflow, USB BUSY/disconnect");
}
