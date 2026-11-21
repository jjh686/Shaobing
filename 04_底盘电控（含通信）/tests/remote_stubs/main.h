#ifndef REMOTE_TEST_MAIN_H
#define REMOTE_TEST_MAIN_H
#include <stdint.h>
typedef struct {uint32_t ErrorCode;void *hdmarx;int gState;} UART_HandleTypeDef;
uint32_t HAL_GetTick(void);
#define __get_PRIMASK() 0u
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
#define HAL_OK 0
#define HAL_UART_STATE_BUSY_TX 1
#define HAL_UART_ERROR_PE 1u
#define HAL_UART_ERROR_FE 2u
#define HAL_UART_ERROR_NE 4u
#define HAL_UART_ERROR_ORE 8u
#define HAL_UART_ERROR_DMA 16u
#define DMA_IT_HT 1u
#define __HAL_DMA_DISABLE_IT(a,b) ((void)(a),(void)(b))
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
int HAL_UART_Transmit(UART_HandleTypeDef *,uint8_t *,uint16_t,unsigned);
int HAL_UART_Transmit_IT(UART_HandleTypeDef *,uint8_t *,uint16_t);
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *,uint8_t *,uint16_t);
#endif
