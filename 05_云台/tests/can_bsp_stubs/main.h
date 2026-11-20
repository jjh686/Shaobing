#ifndef TEST_BSP_MAIN_H
#define TEST_BSP_MAIN_H
#include <stdint.h>
uint32_t __get_BASEPRI(void);
void __set_BASEPRI(uint32_t);
uint32_t __get_PRIMASK(void);
void __disable_irq(void);
void __set_PRIMASK(uint32_t);
#endif
