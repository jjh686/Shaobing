#ifndef TRACKING_TEST_MAIN_H
#define TRACKING_TEST_MAIN_H
#include <stdint.h>
typedef struct { int unused; } SPI_HandleTypeDef;
uint32_t HAL_GetTick(void);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t value);
#endif
