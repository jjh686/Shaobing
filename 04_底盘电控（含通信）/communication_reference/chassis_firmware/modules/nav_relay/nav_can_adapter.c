#include "nav_can_adapter.h"
#include "nav_relay_config.h"
#include "nav_can.h"
#include "rm_chassis_comm/mcu_link.h"
#include "bsp_can.h"
#include "nav_can_hal.h"
#include "remote_control.h"
#include <math.h>
#include <string.h>
typedef struct {uint8_t bytes[8],dlc;uint16_t id;uint32_t tick;} packet_t;
static packet_t incoming[16];static volatile uint8_t head,tail,overrun;
static FS_RC_ctrl_t *rc;static nc_rx_t receiver;
static nch_tx_t sender={.can=&hcan2}; /* Chassis board keeps CAN2. */
static rm_mcu_link_t link;static uint32_t feedback_ms;static uint8_t rx_fault;
volatile uint32_t nav_can_received,nav_can_accepted,nav_can_errors;
static bool selected(void)
{
    return FS_RemoteControlIsOnline()&&rc[TEMP].channel[6]==FS_RC_SW_MID&&rc[TEMP].channel[7]==FS_RC_SW_MID;
}
static uint16_t faults(bool motors)
{
    return (!NAV_RELAY_ALLOW_MOTION?NC_FAULT_LOCK:0)|(!NAV_AXES_VERIFIED?NC_FAULT_UNITS:0)|
        (!selected()?NC_FAULT_RC:0)|(!motors?NC_FAULT_MOTOR:0)|(rx_fault?NC_FAULT_RX:0);
}
static void safety(bool motors)
{
    uint16_t f=faults(motors);
    rm_mcu_set_safety(&link,f==0,!FS_RemoteControlIsOnline(),!selected(),f);
}
static void receive(CANInstance *can)
{
    uint8_t next=(head+1)%16;
    if(next==tail){overrun=1;nav_can_errors++;return;}
    memcpy(incoming[head].bytes,can->rx_buff,8);incoming[head].dlc=can->rx_len;
    incoming[head].id=(uint16_t)can->rx_id;incoming[head].tick=HAL_GetTick();__DMB();head=next;nav_can_received++;
}
void NavCanInit(void)
{
    rm_mcu_init(&link,2);rc=FS_RemoteControlInit(&huart3);
    for(unsigned i=0;i<3;i++){
        CAN_Init_Config_s c={.can_handle=&hcan2,.tx_id=NC_FEEDBACK_BASE+i,
            .rx_id=NC_COMMAND_BASE+i,.can_module_callback=receive};CANRegister(&c);
    }
}
void NavCanCommand(Chassis_Ctrl_Cmd_s *out,int motors_online)
{
    uint32_t now=HAL_GetTick();
    if(overrun){
        uint32_t irq=__get_PRIMASK();__disable_irq();tail=head;overrun=0;__set_PRIMASK(irq);
        receiver.active=0;rx_fault=1;
    }
    safety(motors_online);
    while(tail!=head){
        packet_t p=incoming[tail];__DMB();tail=(tail+1)%16;
        nc_group_t group;rm_command_t command;uint8_t usb[RM_FRAME_SIZE];
        if((uint32_t)(now-p.tick)>NC_GROUP_MS){receiver.active=0;rx_fault=1;continue;}
        if(!nc_rx_push(&receiver,NC_COMMAND_BASE,p.id,false,false,p.dlc,p.bytes,p.tick,&group))continue;
        if(!nc_command_decode(&group,&command)){nav_can_errors++;continue;}
        if(!(command.flags&RM_FLAG_ENABLE)){rx_fault=0;safety(motors_online);}
        if(rm_encode_command(&command,usb)&&rm_mcu_receive(&link,usb,sizeof(usb),receiver.started_ms))nav_can_accepted++;
    }
    safety(motors_online);rm_output_t current;rm_mcu_get_output(&link,now,&current);
    memset(out,0,sizeof(*out));out->chassis_mode=CHASSIS_ZERO_FORCE;
    if(!current.enabled)return;
    const float aps_to_mps=6.28318530718f*(RADIUS_WHEEL/1000.0f)/(360.0f*REDUCTION_RATIO_WHEEL);
    out->vx=current.vx*NAV_RELAY_VX_SIGN/aps_to_mps;
    out->vy=current.vy*NAV_RELAY_VY_SIGN/aps_to_mps;
    out->wz=current.wz*NAV_RELAY_WZ_SIGN/(1000.0f*0.01745329252f*aps_to_mps);
    out->offset_angle=0;out->chassis_speed_buff=100;out->chassis_mode=CHASSIS_NO_FOLLOW;
}
static void pump(uint32_t now)
{
    nch_pump(&sender,now);
}
void NavCanFeedback(const Chassis_Upload_Data_s *measured,int motors_online)
{
    uint32_t now=HAL_GetTick();pump(now);
    if((uint32_t)(now-feedback_ms)<20)return;
    feedback_ms=now;safety(motors_online);
    float vx=measured->real_vx*NAV_RELAY_VX_SIGN,vy=measured->real_vy*NAV_RELAY_VY_SIGN;
    float wz=measured->real_wz*0.01745329252f*NAV_RELAY_WZ_SIGN;
    bool valid=motors_online&&isfinite(vx)&&isfinite(vy)&&isfinite(wz)&&fabsf(vx)<=20&&fabsf(vy)<=20&&fabsf(wz)<=32.767f;
    rm_feedback_t f;rm_mcu_feedback(&link,now,valid,vx,vy,wz,&f);nc_group_t group;
    if(nc_feedback_group(&f,&group)&&nch_start(&sender,&group,now))pump(now);
}
