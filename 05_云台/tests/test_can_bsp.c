#include "main.h"
#include "bsp_can.h"
#include "nav_can_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan1,hcan2;
static uint32_t basepri,primask,pending,aborted;
static unsigned sce_enabled;
static CAN_TypeDef regs;
static unsigned assigned,tx_count,rx_count,rx_pending,rx_fail;
static CAN_RxHeaderTypeDef received;
static CAN_TxHeaderTypeDef transmitted;
static uint8_t transmitted_bytes[8];
static nch_tx_t navigation={.can=&hcan1};
uint32_t __get_BASEPRI(void){return basepri;}
void __set_BASEPRI(uint32_t value){basepri=value;}
uint32_t __get_PRIMASK(void){return primask;}
void __disable_irq(void){primask=1;}
void __set_PRIMASK(uint32_t value){primask=value;}
int HAL_CAN_Start(CAN_HandleTypeDef *can){(void)can;return HAL_OK;}
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can,uint32_t flags)
{(void)can;(void)flags;return HAL_OK;}
void HAL_NVIC_SetPriority(int irq,unsigned priority,unsigned sub)
{assert(irq==CAN1_SCE_IRQn && priority==5 && sub==0);}
void HAL_NVIC_EnableIRQ(int irq){assert(irq==CAN1_SCE_IRQn);sce_enabled++;}
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *can){HAL_CAN_ErrorCallback(can);}
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *can){return can->ErrorCode;}
int HAL_CAN_ResetError(CAN_HandleTypeDef *can){can->ErrorCode=0;return HAL_OK;}
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can,CAN_FilterTypeDef *filter)
{
    if(can==&hcan1 && filter->FilterIdLow==(0x338u<<5))
        assert(filter->FilterFIFOAssignment==CAN_RX_FIFO0);
    return HAL_OK;
}
void CANOnTxMailboxAssigned(CAN_HandleTypeDef *can,uint32_t mailbox)
{
    assert(basepri==0x50u || primask);
    assigned++;
    if(navigation.can==can)navigation.mailboxes&=~mailbox;
}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can,CAN_TxHeaderTypeDef *h,uint8_t *bytes,uint32_t *mailbox)
{
    assert(can==&hcan1 && basepri==0x50u);
    if(pending)return -1;
    pending=1;*mailbox=1;transmitted=*h;memcpy(transmitted_bytes,bytes,8);tx_count++;
    return HAL_OK;
}
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *can,uint32_t mask)
{assert(can==&hcan1);aborted|=mask;pending&=~mask;return HAL_OK;}
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *can,uint32_t mask)
{assert(can==&hcan1);return pending&mask;}
unsigned HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can)
{assert(can==&hcan1);return !pending;}
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *can,uint32_t fifo)
{assert(can==&hcan1 && fifo==CAN_RX_FIFO0);return rx_pending;}
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *can,uint32_t fifo,CAN_RxHeaderTypeDef *h,uint8_t *bytes)
{(void)can;(void)fifo;rx_pending=0;*h=received;memset(bytes,0x55,8);return rx_fail?-1:HAL_OK;}
static void receive(CANInstance *instance)
{assert(instance->rx_len==8 && instance->rx_buff[0]==0x55);rx_count++;}
static void inject(unsigned ide,unsigned rtr,unsigned dlc,unsigned fail)
{
    received=(CAN_RxHeaderTypeDef){.StdId=0x338,.IDE=ide,.RTR=rtr,.DLC=dlc};
    rx_pending=1;rx_fail=fail;HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}
int main(void)
{
    hcan1.Instance=&regs;
    CAN_Init_Config_s config={.can_handle=&hcan1,.tx_id=0x1ff,.rx_id=0x338,
        .rx_fifo_policy=CAN_RX_FIFO_POLICY_FIFO0,.can_module_callback=receive};
    CANInstance *motor=CANRegister(&config);
    /* Hardware has finished navigation, but its software owner has not polled.
     * The real motor FIFO reuses its mailbox and must revoke stale ownership. */
    navigation.mailboxes=1;
    for(unsigned i=0;i<8;i++)motor->tx_buff[i]=(uint8_t)i;
    CANTransmit(motor,8);
    assert(assigned==1 && tx_count==1 && !navigation.mailboxes && basepri==0);
    assert(transmitted.StdId==0x1ff && transmitted.DLC==8);
    assert(memcmp(transmitted_bytes,motor->tx_buff,8)==0);
    nch_reset(&navigation);assert(!aborted && pending==1 && !primask);
    /* Existing nonblocking queue and TX-complete draining still deliver yaw. */
    motor->tx_buff[0]=42;CANTransmit(motor,8);assert(tx_count==1);
    pending=0;HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
    assert(tx_count==2 && assigned==2 && transmitted_bytes[0]==42);
    inject(0,0,8,0);assert(rx_count==1);
    inject(4,0,8,0);inject(0,2,8,0);inject(0,0,9,0);inject(0,0,8,1);
    assert(rx_count==1);
    inject(0,0,8,0);assert(rx_count==2);
    CANEnableDiagnostics(&hcan1);assert(sce_enabled==1);
    regs.ESR=(17u<<16)|(9u<<24);regs.TSR=123;
    hcan1.ErrorCode=HAL_CAN_ERROR_BOF|HAL_CAN_ERROR_RX_FOV0|HAL_CAN_ERROR_TX_TERR0;
    CAN1_SCE_IRQHandler();
    CANBusDiagnostics *d=&can_bus_diagnostics[0];
    assert(d->tec==17 && d->rec==9 && d->max_tec==17 && d->tsr==123);
    assert(d->error_events==1 && d->bus_off_events==1 && d->fifo0_overruns==1 && d->tx_errors==1);
    assert(!hcan1.ErrorCode && !primask);
    regs.ESR=0;CANUpdateDiagnostics(&hcan1);assert(d->tec==0 && d->max_tec==17);
    hcan1.ErrorCode=HAL_CAN_ERROR_RX_FOV1;CAN1_SCE_IRQHandler();
    assert(d->error_events==2 && d->bus_off_events==1 && d->fifo1_overruns==1);
    puts("PASS real CAN diagnostics: SCE service, TEC/REC peaks, independent event bits and HAL error reset");
    puts("PASS real CAN BSP: yaw queue/IRQ delivery, mailbox ownership hook, FIFO0, EXT/RTR/DLC/HAL rejection");
}
