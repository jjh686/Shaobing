#include "nav_can_hal.h"
#include <assert.h>
#include <stdio.h>
CAN_HandleTypeDef hcan1,hcan2;
static uint32_t pending[2],aborted[2],sent[2];
static unsigned bus(CAN_HandleTypeDef *p){assert(p==&hcan1||p==&hcan2);return p==&hcan1?0:1;}
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *p,uint32_t m){return pending[bus(p)]&m;}
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *p,uint32_t m){unsigned b=bus(p);aborted[b]|=m;pending[b]&=~m;return HAL_OK;}
unsigned HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *p){return pending[bus(p)]==7?0:1;}
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *p,CAN_TxHeaderTypeDef *h,uint8_t *b,uint32_t *m)
{unsigned i=bus(p);(void)h;(void)b;*m=!(pending[i]&1)?1:!(pending[i]&2)?2:4;pending[i]|=*m;sent[i]++;return HAL_OK;}
int main(void)
{
 nch_tx_t gimbal={.can=&hcan1},chassis={.can=&hcan2};nc_group_t g={.base=NC_COMMAND_BASE};
 assert(nch_start(&gimbal,&g,100));nch_pump(&gimbal,100);assert(pending[0]==7&&sent[0]==3&&nch_busy(&gimbal));
 assert(nch_start(&chassis,&g,104));nch_pump(&chassis,104);assert(pending[1]==7&&sent[1]==3);
 assert(!nch_start(&gimbal,&g,101));nch_pump(&gimbal,108);assert(!aborted[0]&&!aborted[1]);
 nch_pump(&gimbal,109);assert(aborted[0]==7&&!pending[0]&&pending[1]==7&&!aborted[1]);
 nch_pump(&gimbal,110);assert(!nch_busy(&gimbal));
 nch_pump(&chassis,112);assert(!aborted[1]);nch_pump(&chassis,113);assert(aborted[1]==7&&!pending[1]);
 nch_pump(&chassis,114);assert(!nch_busy(&chassis));
 assert(nch_start(&gimbal,&g,115));nch_pump(&gimbal,115);nch_reset(&gimbal);assert(!pending[0]);
 puts("PASS independent CAN1/CAN2 hardware mailboxes: in-flight group blocks replacement, expiry aborts only the owning bus, USB reset abort");
}
