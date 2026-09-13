/*
 * functions.c
 *
 *  Created on: Sep 12, 2026
 *  Target: STM32G474CET3
 */

#include "functions.h"

void DWT_Init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     // разрешаем использовать счётчик
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;               // запускаем счётчик
}

void delay_micros(uint32_t us)
{
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = us * (SystemCoreClock / 1000000U);
	while ((DWT->CYCCNT - start) < ticks);
}

void delay_tick(uint32_t ticks)
{
	uint32_t start = DWT->CYCCNT;
	while ((DWT->CYCCNT - start) < ticks);
}
