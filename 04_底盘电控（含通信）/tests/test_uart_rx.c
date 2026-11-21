#include "bsp_usart.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint8_t *target;
static unsigned armed,callbacks;
static USARTInstance *instance;
int HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef *h,uint8_t *b,uint16_t n)
{(void)h;assert(n==25);target=b;armed=1;return HAL_OK;}
int HAL_UART_Transmit(UART_HandleTypeDef *h,uint8_t *b,uint16_t n,unsigned t)
{(void)h;(void)b;(void)n;(void)t;return HAL_OK;}
int HAL_UART_Transmit_IT(UART_HandleTypeDef *h,uint8_t *b,uint16_t n)
{(void)h;(void)b;(void)n;return HAL_OK;}
int HAL_UART_Transmit_DMA(UART_HandleTypeDef *h,uint8_t *b,uint16_t n)
{return HAL_UART_Transmit_IT(h,b,n);}
static void parsed(void)
{
    assert(armed&&target!=instance->recv_buff);callbacks++;
    assert(instance->recv_buff[0]==0x0f);
    memset(target,0xee,25);assert(instance->recv_buff[0]==0x0f);
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *,uint16_t);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *);
int main(void)
{
    UART_HandleTypeDef h={0};USART_Init_Config_s c={.recv_buff_size=25,.usart_handle=&h,.module_callback=parsed};
    instance=USARTRegister(&c);target[0]=0x0f;armed=0;HAL_UARTEx_RxEventCallback(&h,9);
    assert(callbacks==1&&instance->recv_len==9&&instance->rx_short_events==1&&instance->rx_bytes==9);
    target[0]=0x0f;armed=0;HAL_UARTEx_RxEventCallback(&h,25);
    assert(callbacks==2&&instance->rx_events==2&&instance->rx_bytes==34);
    h.ErrorCode=HAL_UART_ERROR_PE|HAL_UART_ERROR_FE|HAL_UART_ERROR_ORE;HAL_UART_ErrorCallback(&h);
    assert(instance->error_events==1&&instance->parity_errors==1&&instance->framing_errors==1&&instance->overrun_errors==1);
    puts("PASS actual UART BSP: DMA rearmed before parser, stable copied bytes, short events and classified errors");
}
