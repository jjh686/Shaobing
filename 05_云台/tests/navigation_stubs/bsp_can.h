#ifndef TEST_CAN_H
#define TEST_CAN_H
#include <stdint.h>
#define __DMB() ((void)0)
#define __get_PRIMASK() 0u
#define __set_PRIMASK(x) ((void)(x))
#define __disable_irq() ((void)0)
#define CAN_ID_STD 0u
#define CAN_RTR_DATA 0u
#define HAL_OK 0
typedef int CAN_HandleTypeDef;
typedef struct {unsigned StdId,IDE,RTR,DLC;} CAN_TxHeaderTypeDef;
typedef struct CANInstance {uint8_t rx_buff[8],rx_len;uint32_t rx_id;void (*callback)(struct CANInstance *);} CANInstance;
typedef enum {CAN_RX_FIFO_POLICY_AUTO=0,CAN_RX_FIFO_POLICY_FIFO0=1,CAN_RX_FIFO_POLICY_FIFO1=2} CAN_RxFifoPolicy_e;
typedef struct {CAN_HandleTypeDef *can_handle;unsigned tx_id,rx_id;void (*can_module_callback)(CANInstance *);CAN_RxFifoPolicy_e rx_fifo_policy;} CAN_Init_Config_s;
extern CAN_HandleTypeDef hcan1,hcan2;
CANInstance *CANRegister(CAN_Init_Config_s *);
uint32_t HAL_GetTick(void);
unsigned HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *);
int HAL_CAN_AddTxMessage(CAN_HandleTypeDef *,CAN_TxHeaderTypeDef *,uint8_t *,uint32_t *);
uint32_t HAL_CAN_IsTxMessagePending(CAN_HandleTypeDef *,uint32_t);
int HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *,uint32_t);
void CANOnTxMailboxAssigned(CAN_HandleTypeDef *,uint32_t);
#endif
