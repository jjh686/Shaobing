#ifndef TEST_BSP_CAN_HAL_H
#define TEST_BSP_CAN_HAL_H
#include <stdint.h>
#define __weak __attribute__((weak))
#define HAL_OK 0
#define CAN_ID_STD 0u
#define CAN_RTR_DATA 0u
#define CAN_FILTERMODE_IDLIST 1u
#define CAN_FILTERSCALE_16BIT 0u
#define CAN_FILTER_ENABLE 1u
#define CAN_RX_FIFO0 0u
#define CAN_RX_FIFO1 1u
#define CAN_IT_RX_FIFO0_MSG_PENDING 1u
#define CAN_IT_RX_FIFO1_MSG_PENDING 2u
#define CAN_IT_TX_MAILBOX_EMPTY 4u
typedef int CAN_HandleTypeDef;
typedef struct { uint32_t StdId,IDE,RTR,DLC; } CAN_TxHeaderTypeDef;
typedef struct { uint32_t StdId,IDE,RTR,DLC; } CAN_RxHeaderTypeDef;
typedef struct {
    uint32_t FilterMode,FilterScale,FilterFIFOAssignment,SlaveStartFilterBank;
    uint32_t FilterIdLow,FilterIdHigh,FilterBank,FilterActivation;
} CAN_FilterTypeDef;
extern CAN_HandleTypeDef hcan1,hcan2;
int HAL_CAN_Start(CAN_HandleTypeDef *);
int HAL_CAN_ActivateNotification(CAN_HandleTypeDef *,uint32_t);
int HAL_CAN_ConfigFilter(CAN_HandleTypeDef *,CAN_FilterTypeDef *);
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *,CAN_TxHeaderTypeDef *,uint8_t *,uint32_t *);
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *,uint32_t);
int HAL_CAN_GetRxMessage(CAN_HandleTypeDef *,uint32_t,CAN_RxHeaderTypeDef *,uint8_t *);
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *,uint32_t);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *,uint32_t);
unsigned HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *);
#endif
