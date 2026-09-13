/*
 * functions.h
 *
 *  Created on: Sep 12, 2026
 *  Target: STM32G474CET3
 */

#ifndef INC_FUNCTIONS_H_
#define INC_FUNCTIONS_H_

#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

void DWT_Init(void);
void delay_micros(uint32_t us);
void delay_tick(uint32_t ticks);

#endif /* INC_FUNCTIONS_H_ */
