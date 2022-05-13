#ifndef __DELAY_H_
#define __DELAY_H_

#include "stm32f4xx_hal.h"

void delay_init(uint8_t SYSCLK);

void delay_us(uint32_t nus);

void delay_ms(uint32_t nms);

#endif
