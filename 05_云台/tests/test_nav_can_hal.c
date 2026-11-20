#include "nav_can_hal.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan1,hcan2;
static nch_tx_t navigation,remote;
static uint32_t pending[2],aborted[2],loaded[2];
static unsigned delay_abort,fail_add,blocked;
static unsigned bus(CAN_HandleTypeDef *can) { return can==&hcan1?0:1; }
void CANOnTxMailboxAssigned(CAN_HandleTypeDef *can,uint32_t mailbox)
{
    if(navigation.can==can)navigation.mailboxes&=~mailbox;
    if(remote.can==can)remote.mailboxes&=~mailbox;
}
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *can,uint32_t mailbox)
{ return pending[bus(can)]&mailbox; }
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *can,uint32_t mailboxes)
{
    unsigned b=bus(can);aborted[b]|=mailboxes;
    if(!delay_abort)pending[b]&=~mailboxes;
    return HAL_OK;
}
unsigned HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *can)
{ return !blocked && pending[bus(can)]!=7; }
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *can,CAN_TxHeaderTypeDef *h,uint8_t *bytes,uint32_t *mailbox)
{
    (void)bytes;
    assert(h->IDE==CAN_ID_STD && h->RTR==CAN_RTR_DATA && h->DLC==8);
    if(fail_add)return -1;
    unsigned b=bus(can);
    *mailbox=!(pending[b]&1)?1:!(pending[b]&2)?2:4;
    pending[b]|=*mailbox;loaded[b]++;
    return HAL_OK;
}
static void prepare(void)
{
    memset(pending,0,sizeof(pending));memset(aborted,0,sizeof(aborted));
    navigation=(nch_tx_t){.can=&hcan1};remote=(nch_tx_t){.can=&hcan1};
    delay_abort=fail_add=blocked=0;
}
int main(void)
{
    nc_group_t group={.base=NC_COMMAND_BASE};
    prepare();
    assert(NC_TX_MS==8 && NC_GROUP_MS==10);
    assert(nch_start(&navigation,&group,100));nch_pump(&navigation,100);
    assert(pending[0]==7 && loaded[0]==3 && nch_busy(&navigation));
    nch_pump(&navigation,108);assert(!aborted[0]);
    delay_abort=1;nch_pump(&navigation,109);
    assert(aborted[0]==7 && pending[0]==7 && nch_busy(&navigation));
    assert(!nch_start(&navigation,&group,109));
    /* A completed slot is immediately refilled by the yaw FIFO before the next
     * gateway tick. Its pending bit now belongs to the motor, not navigation. */
    CANOnTxMailboxAssigned(&hcan1,1);aborted[0]=0;delay_abort=0;
    nch_pump(&navigation,110);
    assert(aborted[0]==6 && pending[0]==1); /* Motor was not aborted. */
    nch_pump(&navigation,111);assert(!nch_busy(&navigation));
    nch_reset(&navigation);assert(pending[0]==1);

    prepare();
    assert(nch_start(&navigation,&group,200));nch_pump(&navigation,200);
    pending[0]=0; /* All three frames completed, no foreground tick yet. */
    assert(nch_start(&remote,&group,201));nch_pump(&remote,201);
    assert(!navigation.mailboxes && remote.mailboxes==7);
    nch_reset(&navigation);assert(pending[0]==7);
    nch_reset(&remote);nch_pump(&remote,202);assert(!nch_busy(&remote));

    prepare();blocked=1;
    assert(nch_start(&navigation,&group,UINT32_MAX-3));
    nch_pump(&navigation,4);assert(navigation.tx.active);
    nch_pump(&navigation,5);assert(!nch_busy(&navigation));
    assert(navigation.tx_timeout_groups==1);
    blocked=0;fail_add=1;
    assert(nch_start(&navigation,&group,20));nch_pump(&navigation,20);
    assert(navigation.tx_addtx_errors==1 && navigation.tx.next==0);
    fail_add=0;nch_pump(&navigation,21);assert(pending[0]==7);
    pending[1]=7;nch_reset(&navigation);assert(pending[1]==7 && !aborted[1]);
    puts("PASS 8 ms expiry, async abort ownership, yaw/RC mailbox reuse, blocked bus, HAL retry, tick wrap, independent CAN2");
}
