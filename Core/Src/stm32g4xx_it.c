/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g4xx_it.c
  * @brief   Interrupt Service Routines for STM32G474
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g4xx_it.h"

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart1;
extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

extern void ADC_Process_ISR(void);

volatile uint32_t dma_period_ticks = 0;
volatile uint32_t isr_exec_ticks = 0;

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  static uint32_t last_dwt = 0;
  uint32_t entry_dwt = DWT->CYCCNT;
  dma_period_ticks = entry_dwt - last_dwt;
  last_dwt = entry_dwt;

  DMA1->IFCR = DMA_IFCR_CGIF1;
  ADC_Process_ISR();

  isr_exec_ticks = DWT->CYCCNT - entry_dwt;
}

/**
  * @brief This function handles TIM1 update interrupt and TIM16 global interrupt.
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
}

/**
  * @brief This function handles ADC1 and ADC2 global interrupt.
  */
void ADC1_2_IRQHandler(void)
{
  ADC1->ISR = ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_OVR;
}
