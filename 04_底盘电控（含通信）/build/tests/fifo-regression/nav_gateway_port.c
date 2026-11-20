/* HOST NEGATIVE TEST ONLY: intentionally restores old defect. */
#include "nav_gateway_port.h"
#include "nav_joint_config.h"
#include "nav_gateway.h"
#include "rm_chassis_comm/gimbal_protocol.h"
#include "bsp_can.h"
#include "nav_can_hal.h"
#include "bsp_usb.h"
#include "remote_control.h"
#include "remote_can.h"
#include <string.h>
typedef struct {uint8_t bytes[RM_FRAME_SIZE];uint32_t tick;} command_packet_t;
typedef struct {uint8_t bytes[8];uint16_t id;uint8_t dlc;uint32_t tick;} can_packet_t;
static command_packet_t commands[8];static can_packet_t incoming[16];
static volatile uint8_t qh,qt,rh,rt,reset_requested;
static nav_gateway_t gateway;static nc_rx_t receiver;
static nch_tx_t sender={.can=&hcan1}; /* Gimbal CAN1 <-> chassis CAN2. */
static uint8_t pending[RM_FRAME_SIZE],pending_valid;
static uint32_t pending_ms,joint_ms;
static nch_tx_t remote_sender={.can=&hcan1};
static uint32_t remote_sent_ms;
static uint16_t remote_sequence;
static uint8_t remote_started;
volatile uint32_t nav_rc_groups, nav_rc_online, nav_rc_estop, nav_rc_age_ms;
static void forward_remote(uint32_t now)
{
    nch_pump(&remote_sender,now);
    if(nch_busy(&remote_sender)||(remote_started&&(uint32_t)(now-remote_sent_ms)<20u))return;
    FS_RC_ctrl_t rc={0};RemoteCanSample sample={0};nc_group_t group;
    bool online=FS_RemoteControlGetSnapshot(&rc)!=0;
    sample.sequence=++remote_sequence;sample.generation=(uint16_t)rc.fault_generation;
    uint32_t age=now-rc.received_ms;sample.age_ms=age>65535u?65535u:(uint16_t)age;
    sample.axes[0]=rc.rc.rocker_r_;sample.axes[1]=rc.rc.rocker_r1;
    sample.axes[2]=rc.rc.rocker_l1;sample.axes[3]=rc.rc.rocker_l_;
    sample.axes[4]=rc.rc.knob_l+240;sample.axes[5]=rc.rc.knob_r+240;
    for(unsigned i=0;i<4;i++){
        int value=rc.channel[6+i];
        if(value<0||value>2){online=false;value=0;}
        sample.switches[i]=(uint8_t)value;
    }
    online=online&&age<=RC_CAN_TIMEOUT_MS;
    sample.flags=(online?RC_CAN_ONLINE:0)|(!online||sample.switches[3]==2?RC_CAN_ESTOP:0);
    if(!rc_sample_ok(&sample)){
        memset(sample.axes,0,sizeof(sample.axes));sample.flags=RC_CAN_ESTOP;
    }
    nav_rc_online=(sample.flags&RC_CAN_ONLINE)!=0;nav_rc_estop=(sample.flags&RC_CAN_ESTOP)!=0;
    nav_rc_age_ms=sample.age_ms;
    if(rc_can_encode(&sample,&group)&&nch_start(&remote_sender,&group,now)){
        remote_started=1;remote_sent_ms=now;nav_rc_groups++;nch_pump(&remote_sender,now);
    }
}
volatile uint32_t nav_usb_received,nav_can_groups,nav_feedback_received,nav_gateway_drops;
/* Read-only SRAM mirrors let SWD distinguish bus errors from decode errors.
 * HAL ErrorCode alone stays zero unless CAN error notifications are enabled. */
volatile uint32_t nav_can_esr,nav_can_tsr,nav_can_rx_frames,nav_can_decode_errors;
volatile uint32_t nav_can_task_ticks;
volatile uint32_t nav_command_tx_mailbox_loaded_frames,nav_command_tx_addtx_errors;
volatile uint32_t nav_command_tx_timeout_groups,nav_command_tx_aborted_groups;
volatile uint32_t nav_remote_tx_mailbox_loaded_frames,nav_remote_tx_addtx_errors;
volatile uint32_t nav_remote_tx_timeout_groups,nav_remote_tx_aborted_groups;
static void update_tx_debug(void)
{
    nav_command_tx_mailbox_loaded_frames=sender.tx_mailbox_loaded_frames;
    nav_command_tx_addtx_errors=sender.tx_addtx_errors;
    nav_command_tx_timeout_groups=sender.tx_timeout_groups;
    nav_command_tx_aborted_groups=sender.tx_aborted_groups;
    nav_remote_tx_mailbox_loaded_frames=remote_sender.tx_mailbox_loaded_frames;
    nav_remote_tx_addtx_errors=remote_sender.tx_addtx_errors;
    nav_remote_tx_timeout_groups=remote_sender.tx_timeout_groups;
    nav_remote_tx_aborted_groups=remote_sender.tx_aborted_groups;
}
static void can_receive(CANInstance *can)
{
    nav_can_rx_frames++;
    uint8_t next=(rh+1)%16;
    if(next==rt){nav_gateway_drops++;reset_requested=1;return;}
    memcpy(incoming[rh].bytes,can->rx_buff,8);incoming[rh].id=(uint16_t)can->rx_id;
    incoming[rh].dlc=can->rx_len;incoming[rh].tick=HAL_GetTick();__DMB();rh=next;
}
void NavGatewayReset(void){reset_requested=1;}
void NavGatewayReceive(const uint8_t *bytes,unsigned n,uint32_t received)
{
    uint8_t next=(qh+1)%8;
    if(n!=RM_FRAME_SIZE||next==qt){nav_gateway_drops++;reset_requested=1;return;}
    memcpy(commands[qh].bytes,bytes,n);commands[qh].tick=received;__DMB();qh=next;nav_usb_received++;
}
void NavGatewayInit(void)
{
    ng_init(&gateway);
    remote_started=0;remote_sequence=0;
    for(unsigned i=0;i<3;i++){
        CAN_Init_Config_s c={.can_handle=&hcan1,.tx_id=NC_COMMAND_BASE+i,
            .rx_fifo_policy=CAN_RX_FIFO_POLICY_AUTO,
            .rx_id=NC_FEEDBACK_BASE+i,.can_module_callback=can_receive};
        CANRegister(&c);
    }
}
static void pump(uint32_t now)
{
    nch_pump(&sender,now);
}
void NavGatewayTask(void)
{
    uint32_t now=HAL_GetTick();
    nav_can_task_ticks=now;
    /* RC transport runs independently of USB connection / navigation sessions. */
    forward_remote(now);
    update_tx_debug();
#ifndef CHASSIS_COMM_HOST_TEST
    nav_can_esr=hcan1.Instance->ESR;
    nav_can_tsr=hcan1.Instance->TSR;
#endif
    if(reset_requested||!USBIsConfigured()){
        uint32_t irq=__get_PRIMASK();__disable_irq();qh=qt=rh=rt=0;reset_requested=0;__set_PRIMASK(irq);
        ng_init(&gateway);memset(&receiver,0,sizeof(receiver));nch_reset(&sender);pending_valid=0;
        if(!USBIsConfigured()){update_tx_debug();return;}
    }
    pump(now);
    while(qt!=qh){
        command_packet_t p=commands[qt];__DMB();qt=(qt+1)%8;nc_group_t group;
        if(nch_busy(&sender)||!ng_command(&gateway,p.bytes,p.tick,now,&group)||!nch_start(&sender,&group,now)){
            nav_gateway_drops++;continue;
        }
        nav_can_groups++;pump(now);
    }
    while(rt!=rh){
        can_packet_t p=incoming[rt];__DMB();rt=(rt+1)%16;
        nc_group_t group;rm_feedback_t f;
        if((uint32_t)(now-p.tick)>NC_GROUP_MS){receiver.active=0;continue;}
        if(nc_rx_push(&receiver,NC_FEEDBACK_BASE,p.id,false,false,p.dlc,p.bytes,p.tick,&group)){
            if(nc_feedback_decode(&group,&f)&&rm_encode_feedback(&f,pending)){
                pending_ms=p.tick;pending_valid=1;nav_feedback_received++;
            }else nav_can_decode_errors++;
        }
    }
    /* Prioritize chassis ACK over optional joint telemetry. CDC owns accepted
     * buffers and arbitrates with the original high-priority SP status sender. */
    if(pending_valid){
        if((uint32_t)(now-pending_ms)>NC_GROUP_MS||USBTransmit(pending,sizeof(pending))==USBD_OK)pending_valid=0;
        update_tx_debug();
        return;
    }
    if(gateway.gate.ack_valid&&(uint32_t)(now-joint_ms)>=20){
        joint_ms=now;
        rm_gimbal_t g={0};g.route=2;g.uptime_ms=now;g.sample_ms=now;
        g.command_received_ms=gateway.command_received_ms;
        g.session=gateway.gate.command.session;g.sequence=gateway.gate.command.sequence;
        /* No chassis-relative calibration exists in the supplied sources.
         * Zero values carry NO VALID/CALIBRATED bits, so the PC rejects them. */
        g.status=0;
        uint8_t frame[RM_GIMBAL_FRAME_SIZE];
        if(rm_encode_gimbal(&g,frame))USBTransmit(frame,sizeof(frame));
    }
    update_tx_debug();
}
