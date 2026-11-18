#include "BMI088Middleware.h"
#include "main.h"

SPI_HandleTypeDef *BMI088_SPI;
/* [USB_SP 修改 01 开始] SPI接收变量初始化、HAL错误累积和有界传输超时。 */
static uint8_t spi_read_ok;
void BMI088_BeginRead(void) { spi_read_ok = 1; }
uint8_t BMI088_ReadOK(void) { return spi_read_ok; }
/* [USB_SP 修改 01 结束] */

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
/* [USB_SP 修改 02 开始] SPI接收变量初始化、HAL错误累积和有界传输超时。 */
    uint8_t rx_data = 0;
    /* Bounded transfer: a stalled sensor must not block control for 1 second/byte. */
    if (!BMI088_SPI || HAL_SPI_TransmitReceive(BMI088_SPI, &txdata, &rx_data, 1, 2) != HAL_OK)
        spi_read_ok = 0;
/* [USB_SP 修改 02 结束] */
    return rx_data;
}
