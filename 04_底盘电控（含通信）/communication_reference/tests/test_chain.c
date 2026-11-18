/* Execute both actual firmware ports; only peripheral I/O is simulated. */
#include "bsp_can.h"
#include "bsp_usb.h"
#include "remote_control.h"
#include "nav_gateway_port.h"
#include "nav_can_adapter.h"
#include "nav_relay_config.h"
#include "nav_can.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
CAN_HandleTypeDef hcan1,hcan2;int huart3;
static uint32_t tick;static int side,rc_online=1,configured=1,usb_busy,can_busy;
static CANInstance inst[2][3];static unsigned registered[2],ack_count;
static FS_RC_ctrl_t remote;static rm_feedback_t ack;
static Chassis_Ctrl_Cmd_s output;static Chassis_Upload_Data_s measured={.real_vx=.1234f,.real_vy=-.2346f,.real_wz=45};
uint32_t HAL_GetTick(void){return tick;}
FS_RC_ctrl_t *FS_RemoteControlInit(int *p){assert(p==&huart3);return &remote;}
uint8_t FS_RemoteControlIsOnline(void){return rc_online;}
uint8_t USBIsConfigured(void){return configured;}
uint8_t USBTransmit(uint8_t *p,uint16_t n)
{if(usb_busy)return USBD_BUSY;if(n==34){assert(rm_decode_feedback(p,n,&ack));ack_count++;}else assert(n==46);return USBD_OK;}
CANInstance *CANRegister(CAN_Init_Config_s *c)
{assert(c->can_handle==(side==0?&hcan1:&hcan2));assert(registered[side]<3);CANInstance *i=&inst[side][registered[side]++];i->rx_id=c->rx_id;i->callback=c->can_module_callback;return i;}
unsigned HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *p){assert(p==(side==0?&hcan1:&hcan2));return can_busy?0:3;}
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *p,uint32_t m){assert(p==(side==0?&hcan1:&hcan2));(void)m;return 0;}
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *p,uint32_t m){assert(p==(side==0?&hcan1:&hcan2));(void)m;return HAL_OK;}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *p,CAN_TxHeaderTypeDef *h,uint8_t *b,uint32_t *m)
{
 assert(p==(side==0?&hcan1:&hcan2));
 assert(h->DLC==8&&h->IDE==CAN_ID_STD&&h->RTR==CAN_RTR_DATA);*m=1;
 for(unsigned j=0;j<3;j++){CANInstance *i=&inst[!side][j];if(i->rx_id==h->StdId){memcpy(i->rx_buff,b,8);i->rx_len=8;i->callback(i);return HAL_OK;}}
 assert(!"unexpected CAN ID");return -1;
}
static void step(int motors)
{side=0;NavGatewayTask();side=1;NavCanCommand(&output,motors);NavCanFeedback(&measured,motors);side=0;NavGatewayTask();}
static void send(uint16_t seq,int enabled)
{
 rm_command_t c={.route=2,.session=0x10203040,.sequence=seq,.uptime_ms=tick,
 .flags=enabled?1:0,.ttl_ms=enabled?150:0,.vx=enabled?.1239f:0,.vy=enabled?-.0989f:0,.wz=enabled?.1239f:0};
 uint8_t b[34];assert(rm_encode_command(&c,b));NavGatewayReceive(b,34,tick);
}
static void stopped(void){assert(output.chassis_mode==CHASSIS_ZERO_FORCE&&output.vx==0&&output.vy==0&&output.wz==0);}
int main(void)
{
 remote.channel[6]=remote.channel[7]=1;side=0;NavGatewayInit();side=1;NavCanInit();
 tick=1;send(0,1);step(1);stopped();
 tick=20;send(1,0);step(1);assert(ack_count==1&&ack.sequence==1&&(ack.status&RM_STATUS_ACK_VALID));
 assert(fabsf(ack.vx-.123f)<1e-6f&&fabsf(ack.vy+.235f)<1e-6f);
 tick=40;send(2,1);step(1);
#if NAV_RELAY_ALLOW_MOTION && NAV_AXES_VERIFIED
 assert(output.chassis_mode==CHASSIS_NO_FOLLOW&&output.offset_angle==0);
 const float scale=6.28318530718f*.06f/(360*19);
 assert(fabsf(output.vx*scale-.123f)<1e-6f&&fabsf(output.vy*scale+.098f)<1e-6f);
 assert(fabsf(output.wz*1000*.01745329252f*scale-.123f)<1e-6f);
 assert((ack.status&RM_STATUS_ACTIVE)&&ack.fault_mask==0);
#else
 stopped();assert((ack.fault_mask&NC_FAULT_LOCK)&&(ack.fault_mask&NC_FAULT_UNITS));
#endif
 rc_online=0;tick=41;step(1);stopped();rc_online=1;tick=42;send(3,1);step(1);stopped();
 tick=60;send(4,0);step(1);tick=80;send(5,1);step(1);
 tick=220;step(1);stopped(); /* gateway reduces TTL by 10 ms */
 tick=221;send(6,1);step(1);stopped();
 tick=240;send(7,0);step(1);tick=260;send(8,1);step(0);stopped();
 assert(!(ack.status&RM_STATUS_MEASURED)&&(ack.fault_mask&NC_FAULT_MOTOR));
 tick=280;send(9,0);step(1);tick=300;send(10,1);tick=311;step(1);stopped();assert(ack.sequence==9);
 /* Bottom inbox overflow: valid groups cannot keep old actuator output alive. */
 tick=331;nc_group_t g;rm_command_t c={.route=2,.session=9};assert(nc_command_group(&c,&g));
 for(unsigned n=0;n<6;n++)for(unsigned j=0;j<3;j++){CANInstance *i=&inst[1][j];memcpy(i->rx_buff,g.data[j],8);i->rx_len=8;i->callback(i);}
 step(1);stopped();assert(ack.fault_mask&NC_FAULT_RX);
 tick=351;send(11,0);step(1);assert(!(ack.fault_mask&NC_FAULT_RX));
 usb_busy=1;tick=371;send(12,0);step(1);unsigned before=ack_count;
 tick=382;usb_busy=0;step(1);assert(ack_count==before); /* stale pending feedback dropped */
 configured=0;tick=391;step(1);configured=1;tick=411;send(13,1);step(1);stopped();
 tick=431;send(14,0);step(1);
 remote.channel[6]=255;tick=432;step(1);stopped();
 remote.channel[6]=1;tick=433;send(15,1);step(1);stopped();
 tick=451;send(16,0);step(1);
 for(unsigned n=0;n<10000;n++){
  tick+=20;send((uint16_t)(17+n),1);step(1);
  assert(ack.session==0x10203040&&(ack.status&RM_STATUS_ACK_VALID));
#if NAV_RELAY_ALLOW_MOTION && NAV_AXES_VERIFIED
  assert(ack.sequence==(uint16_t)(17+n));
  assert(output.chassis_mode==CHASSIS_NO_FOLLOW);
#else
  assert(ack.sequence==16); /* rejected ENABLE must never receive a fake ACK */
  stopped();
#endif
 }
 puts("PASS actual gimbal CAN1/chassis CAN2 ports: 10000 software USB/CAN round trips, STOP/ACK, motion lock, SI units, RC/motor loss, TTL, stale queue, overflow, USB BUSY/disconnect");
}
