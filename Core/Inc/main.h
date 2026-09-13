/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Target: STM32G474CET3 (LQFP48)
  * Clock: 16 MHz Ext. Generator D5 -> PLL -> 84 MHz SYSCLK
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sync_tracker.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct __attribute__((packed)) {
	uint8_t sync_status; /* 0: SEARCH, 1: LOCKED, 2: HOLDOVER */
	float difference;    /* Frequency drift / time offset in microseconds */
	float snr;           /* Signal-to-Noise Ratio */
} OutputPacket_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define SYSCLK_FREQ_HZ           84000000UL
#define APB1_TIM_FREQ_HZ         84000000UL
#define APB2_TIM_FREQ_HZ         84000000UL
#define SYNC_PERIOD_TICKS        168000000UL /* 2.0s at 84 MHz */
#define DIFF_DIVIDER             42.0f
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
float Read_Temperature_TMP235(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RX_SIG_ADC_Pin           GPIO_PIN_1
#define RX_SIG_ADC_GPIO_Port     GPIOA
#define AMP_EN_Pin               GPIO_PIN_4
#define AMP_EN_GPIO_Port         GPIOA
#define TEST_GEN_Pin             GPIO_PIN_0
#define TEST_GEN_GPIO_Port       GPIOB
#define SYNC_OUT_Pin             GPIO_PIN_11
#define SYNC_OUT_GPIO_Port       GPIOB
#define TEMP_SENS_Pin            GPIO_PIN_12
#define TEMP_SENS_GPIO_Port      GPIOB
#define LED_PIN                  GPIO_PIN_15
#define LED_GPIO_Port            GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
