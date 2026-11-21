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
typedef struct {uint32_t ESR,TSR;} CAN_TypeDef;
typedef struct {CAN_TypeDef *Instance;uint32_t ErrorCode;} CAN_HandleTypeDef;
#define CAN1_SCE_IRQn 22
#define CAN_IT_ERROR 8u
#define CAN_IT_LAST_ERROR_CODE 16u
#define CAN_IT_BUSOFF 32u
#define CAN_IT_ERROR_WARNING 64u
#define CAN_IT_ERROR_PASSIVE 128u
#define CAN_IT_RX_FIFO0_OVERRUN 256u
#define CAN_IT_RX_FIFO1_OVERRUN 512u
#define HAL_CAN_ERROR_BOF 1u
#define HAL_CAN_ERROR_RX_FOV0 2u
#define HAL_CAN_ERROR_RX_FOV1 4u
#define HAL_CAN_ERROR_TX_TERR0 8u
#define HAL_CAN_ERROR_TX_TERR1 16u
#define HAL_CAN_ERROR_TX_TERR2 32u
#define HAL_CAN_ERROR_TX_ALST0 64u
#define HAL_CAN_ERROR_TX_ALST1 128u
#define HAL_CAN_ERROR_TX_ALST2 256u
void HAL_NVIC_SetPriority(int,unsigned,unsigned);
void HAL_NVIC_EnableIRQ(int);
void HAL_CAN_IRQHandler(CAN_HandleTypeDef *);
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *);
int HAL_CAN_ResetError(CAN_HandleTypeDef *);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *);
void CAN1_SCE_IRQHandler(void);
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
