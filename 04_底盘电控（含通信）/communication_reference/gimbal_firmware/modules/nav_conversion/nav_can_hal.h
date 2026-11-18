#ifndef NAV_CAN_HAL_H
#define NAV_CAN_HAL_H
#include "nav_can.h"
#include "bsp_can.h"
/* Own the hardware mailboxes until completion as well as the software group.
 * NART alone cannot remove a frame waiting for a busy bus to become idle. */
/* Each board selects its local controller; the wire protocol is identical. */
typedef struct {nc_tx_t tx;uint32_t mailboxes;CAN_HandleTypeDef *can;
    uint32_t tx_mailbox_loaded_frames,tx_addtx_errors,tx_timeout_groups,tx_aborted_groups;} nch_tx_t;
static inline void nch_reset(nch_tx_t *s)
{
    if(s->tx.active||s->mailboxes)s->tx_aborted_groups++;
    if(s->mailboxes)HAL_CAN_AbortTxRequest(s->can,s->mailboxes);
    s->tx.active=0;
}
static inline void nch_pump(nch_tx_t *s,uint32_t now)
{
    uint32_t pending=0;
    for(uint32_t bit=1;bit<=4;bit<<=1)
        if((s->mailboxes&bit)&&HAL_CAN_IsTxMessagePending(s->can,bit))pending|=bit;
    s->mailboxes=pending;
    if(pending&&(uint32_t)(now-s->tx.started_ms)>NC_TX_MS){s->tx_timeout_groups++;nch_reset(s);return;}
    uint8_t bytes[8];uint16_t id;uint32_t mailbox;
    CAN_TxHeaderTypeDef h={0};h.IDE=CAN_ID_STD;h.RTR=CAN_RTR_DATA;h.DLC=8;
    while(HAL_CAN_GetTxMailboxesFreeLevel(s->can)&&nc_tx_peek(&s->tx,now,&id,bytes)){
        h.StdId=id;
        if(HAL_CAN_AddTxMessage(s->can,&h,bytes,&mailbox)!=HAL_OK){s->tx_addtx_errors++;break;}
        s->mailboxes|=mailbox;nc_tx_commit(&s->tx);
        s->tx_mailbox_loaded_frames++;
    }
}
static inline bool nch_busy(const nch_tx_t *s){return s->mailboxes||s->tx.active;}
static inline bool nch_start(nch_tx_t *s,const nc_group_t *g,uint32_t now)
{return !nch_busy(s)&&nc_tx_start(&s->tx,g,now);}
#endif
