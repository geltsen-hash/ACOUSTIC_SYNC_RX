/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Acoustic Receiver firmware for STM32G474CET3
  ******************************************************************************
  * @attention
  *
  * Target: STM32G474CET3
  * Clock: 16 MHz Ext. Generator D5 (PF0) -> PLL -> SYSCLK 84 MHz
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "functions.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define SLEEP_MODE
#define REQUEST_RESPONSE_MODE

typedef enum
{
    SYNC,
	PRINT,
	STAY,
	SLEEP
} rx_status;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES_PER_BIT          66U
#define SYNC_SEQ_LEN             2046U /* 31 * 66 */
#define CONV_BUF_SIZE            16U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc4;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SyncTracker_t sync_tracker;
uint32_t sleep_pause = 1920;
uint16_t sequence_size = 2048;
uint16_t byte_size = 66;
volatile int32_t Signal_ma[2048];
volatile uint32_t S_data[66];

volatile uint16_t BUFF[4];
uint32_t sgn = 0;
float sqr = 0;
uint16_t w = (uint16_t)-1;

volatile uint8_t w_ma = (uint8_t)-1;
uint8_t w_c = (uint8_t)-1;
uint8_t p_c = 0;
volatile uint32_t S_summ = 0;
volatile int32_t convolution_data[16];
volatile float convolution = 0;
volatile uint32_t sub = 0;
volatile int32_t  pwrMax = 0, pwrMin = 0;
volatile float32_t SNR = 0;
volatile uint32_t N = 0;
volatile bool sync_catch_flg = false;
volatile rx_status Rx_sts = SYNC;
uint32_t freq = 0;
float current_temperature = 0.0f;

const uint32_t ticks_per_period = 544;
const uint32_t ticks_per_bit = 544 * 66;
const uint32_t ticks_per_stop_bit = 544 * 66;
const uint32_t led_pulse_off_time = ticks_per_bit * 31 - ticks_per_period * 6;
const uint32_t print_start_time = SYSCLK_FREQ_HZ / 2;

volatile uint32_t local_ticks_per_sync_period = 0;
uint8_t artur_income[3];
OutputPacket_t artur_output;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/**
  * @brief  TMP235 temperature reader on PB12 (ADC4_IN3)
  *         TMP235: 500 mV at 0 °C, 10 mV/°C slope
  * @retval Temperature in degrees Celsius
  */
float Read_Temperature_TMP235(void)
{
    uint32_t raw_adc = 0;
    HAL_ADC_Start(&hadc4);
    if (HAL_ADC_PollForConversion(&hadc4, 10) == HAL_OK)
    {
        raw_adc = HAL_ADC_GetValue(&hadc4);
    }
    HAL_ADC_Stop(&hadc4);

    float voltage = ((float)raw_adc * 3.3f) / 4095.0f;
    float temp_c = (voltage - 0.500f) / 0.010f;
    return temp_c;
}

// TIM1 Period Elapsed: Wake up from sleep
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM1)
	{
#ifdef SLEEP_MODE
		HAL_ResumeTick();
		__HAL_RCC_ADC12_CLK_ENABLE();
		__HAL_RCC_DMA1_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();
	 	__HAL_RCC_GPIOB_CLK_ENABLE();

		// Разрешение работы усилителя (лог. «1» - работа)
		HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_SET);
#endif
		p_c++;
		if (p_c >= 10) {
			p_c = 0;
			pwrMax = 0;
			S_summ = 0;
			NVIC_DisableIRQ(ADC1_2_IRQn);
			memset((void*)Signal_ma, 0x00, sequence_size * sizeof(Signal_ma[0]));
			memset((void*)S_data, 0x00, byte_size * sizeof(S_data[0]));
			memset((void*)convolution_data, 0x00, sizeof(convolution_data));
			NVIC_EnableIRQ(ADC1_2_IRQn);
		}

		// Режим Holdover: если в прошедшем периоде синхросигнал не был детектирован
		if (sync_catch_flg == 0 && sync_tracker.lock_count > 0) {
			current_temperature = Read_Temperature_TMP235();
			uint32_t holdover_period = SyncTracker_OnSyncMissed(&sync_tracker, current_temperature);
			TIM2->ARR = holdover_period - 1;
			artur_output.sync_status = (uint8_t)SyncTracker_GetState(&sync_tracker);
			artur_output.difference = SyncTracker_GetDifferenceUs(&sync_tracker);
		}

		NVIC_EnableIRQ(ADC1_2_IRQn);
		Rx_sts = SYNC;
	}
}

// ADC1 Data Processing ISR
void ADC_Process_ISR(void)
{
	if(Rx_sts == SYNC){
	    sub =  __SHSUB16(BUFF[0], BUFF[1]);
	    sgn = __SMUAD(sub, sub);
	    arm_sqrt_f32(sgn, &sqr);

	    // скользящее среднее
	    w_ma++;
	   	if(w_ma >= SAMPLES_PER_BIT)
	   		w_ma = 0;
	 	S_summ -= S_data[w_ma];
	    S_data[w_ma] = (uint32_t)sqr;
	    S_summ += S_data[w_ma];

	    // главная последовательность 66*31=2046
		w++;
		if (w >= SYNC_SEQ_LEN)
			w = 0;
		Signal_ma[w] = S_summ;

		// пиковый детектор
		if(Signal_ma[w] > pwrMax) {
			pwrMax = Signal_ma[w];
		}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//                        свертка 1-0-0+1-0-0+1+1+1+1+1-0+1+1+1-0-0-0+1-0+1-0+1+1-0+1-0-0-0-0+1
		w_c++;
		w_c = w_c & 0x7;

		uint16_t idx = w;
		int32_t conv = Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv -= Signal_ma[idx];
		idx += 66; if (idx >= SYNC_SEQ_LEN) idx -= SYNC_SEQ_LEN; conv += Signal_ma[idx];

		convolution_data[w_c] = conv;
		convolution = convolution_data[(w_c + 3) & 0x7];

	    if(
	        convolution_data[(w_c + 3) & 0x7] > pwrMax * 10
		    && convolution_data[(w_c + 3) & 0x7] > convolution_data[(w_c + 2) & 0x7]
		    && convolution_data[(w_c + 3) & 0x7] > convolution_data[(w_c + 4) & 0x7]
		  ){
	    	 HAL_GPIO_WritePin(LED_GPIO_Port, LED_PIN, GPIO_PIN_SET); // включение светодиода PB15
	    	 TIM2->CNT = 0x0;
	    	 local_ticks_per_sync_period = DWT->CYCCNT;
	    	 DWT->CYCCNT = 0U;
	    	 sync_catch_flg = 1;
		     NVIC_DisableIRQ(ADC1_2_IRQn);
	    }
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock: 16 MHz HSE Bypass -> 84 MHz SYSCLK */
  SystemClock_Config();

  /* USER CODE BEGIN Init */
  DWT_Init();
  /* USER CODE END Init */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  // Инициализация адаптивного трекера синхронизации
  SyncTracker_Init(&sync_tracker);

  // Калибровка АЦП
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  HAL_ADCEx_Calibration_Start(&hadc4, ADC_SINGLE_ENDED);

  // Включение усилителя (лог. 1)
  HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_SET);

  // Запуск АЦП1 в непрерывном режиме с DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)BUFF, 2);

  // Запуск таймеров
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // PB11 Pulse output

  freq = HAL_RCC_GetHCLKFreq();

  // Индикация старта (мигание LED на PB15)
  for(int i = 0; i < 6; i++){
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_PIN);
		HAL_Delay(60);
  }
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_PIN, GPIO_PIN_RESET);

  artur_income[0] = 0x00; artur_income[1] = 0x00; artur_income[2] = 0x00;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  w = (uint16_t)-1;
  w_ma = (uint8_t)-1;
  w_c = (uint8_t)-1;

#ifdef REQUEST_RESPONSE_MODE
  HAL_HalfDuplex_EnableReceiver(&huart1);
  HAL_UART_Receive_IT(&huart1, artur_income, 1);
#else
  NVIC_DisableIRQ(USART1_IRQn);
  HAL_HalfDuplex_EnableTransmitter(&huart1);
#endif

  while (1)
  {
      if (sync_catch_flg == 1)
      {
	      N++;
	      sync_catch_flg = 0;
	      S_summ = 0;
	      pwrMin = (Signal_ma[(w + 1848) % 0x7fe] + Signal_ma[(w + 1056) % 0x7fe]) / 2;
	      SNR = (pwrMax - pwrMin) / pwrMin;
	      pwrMax = 0;
	      pwrMin = 0;
	      p_c = 0;

	      // Периодическое чтение температуры
	      current_temperature = Read_Temperature_TMP235();

	      // Адаптивная фильтрация периода и подстройка TIM2->ARR
	      uint32_t recommended_arr_period = SyncTracker_OnSyncDetected(&sync_tracker, local_ticks_per_sync_period, current_temperature);
	      TIM2->ARR = recommended_arr_period - 1;

	      NVIC_DisableIRQ(ADC1_2_IRQn);
	      memset((void*)Signal_ma, 0x00, sequence_size * sizeof(Signal_ma[0]));
	      memset((void*)S_data, 0x00, byte_size * sizeof(S_data[0]));
	      memset((void*)convolution_data, 0x00, sizeof(convolution_data));
	      NVIC_EnableIRQ(ADC1_2_IRQn);

	      w = (uint16_t)-1;
	      w_ma = (uint8_t)-1;
	      w_c = (uint8_t)-1;
	      TIM1->CNT = 0x0;

	      while((int32_t)(TIM2->CNT - led_pulse_off_time) < 0) {;}
	      HAL_GPIO_WritePin(LED_GPIO_Port, LED_PIN, GPIO_PIN_RESET);
	      artur_output.sync_status = (uint8_t)SyncTracker_GetState(&sync_tracker);
	      artur_output.difference = SyncTracker_GetDifferenceUs(&sync_tracker);
	      artur_output.snr = SNR;

#ifndef REQUEST_RESPONSE_MODE
	      while((int32_t)(TIM2->CNT - print_start_time) < 0) {;}
	      Rx_sts = PRINT;
	      if(Rx_sts == PRINT)
	      {
		      HAL_HalfDuplex_EnableTransmitter(&huart1);
		      HAL_UART_Transmit(&huart1, (uint8_t*)(&artur_output), sizeof(artur_output), 2);
		      Rx_sts = SLEEP;
	      }
#endif
      }

      if(Rx_sts == SLEEP){
#ifdef SLEEP_MODE
        	// Блокировка работы усилителя (лог. «0» - запрет)
        	HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_RESET);
        	__HAL_RCC_ADC12_CLK_DISABLE();
        	__HAL_RCC_DMA1_CLK_DISABLE();
        	__HAL_RCC_GPIOA_CLK_DISABLE();
        	__HAL_RCC_GPIOB_CLK_DISABLE();
        	HAL_SuspendTick();
        	HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
#endif
       }
  }
  /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration (16 MHz HSE Bypass -> 84 MHz SYSCLK)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  * D5 generator: 16 MHz on PF0 (HSE_BYPASS)
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4; // 16 / 4 = 4 MHz
  RCC_OscInitStruct.PLL.PLLN = 42;             // 4 * 42 = 168 MHz VCO
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // 168 / 2 = 84 MHz SYSCLK
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function (PA1 - Signal Barker)
  */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_2; // PA1 is ADC1_IN2
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC4 Initialization Function (PB12 - TMP235 Temp Sensor)
  */
static void MX_ADC4_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc4.Instance = ADC4;
  hadc4.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV256;
  hadc4.Init.Resolution = ADC_RESOLUTION_12B;
  hadc4.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc4.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc4.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc4.Init.ContinuousConvMode = DISABLE;
  hadc4.Init.NbrOfConversion = 1;
  hadc4.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc4.Init.DMAContinuousRequests = DISABLE;
  hadc4.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc4) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_3; // PB12 is ADC4_IN3
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  if (HAL_ADC_ConfigChannel(&hadc4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function (Periodic wakeup)
  */
static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8400 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19500 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function (32-bit Sync Timer / Pulse Generator)
  */
static void MX_TIM2_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 167999999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 16800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) // PB11
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim2);
}

/**
  * @brief USART1 Initialization Function (500 000 baud, Single-Wire / Half-Duplex)
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 500000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AMP_EN_GPIO_Port, AMP_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 (AMP_EN) */
  GPIO_InitStruct.Pin = AMP_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(AMP_EN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB15 (LED) */
  GPIO_InitStruct.Pin = LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 (TEST_GEN) */
  GPIO_InitStruct.Pin = TEST_GEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(TEST_GEN_GPIO_Port, &GPIO_InitStruct);
}

#ifdef REQUEST_RESPONSE_MODE
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if(huart == &huart1)
    {
		NVIC_DisableIRQ(ADC1_2_IRQn);
		if(artur_output.sync_status == SYNC_STATE_LOCKED || artur_output.sync_status == SYNC_STATE_HOLDOVER)
		    Rx_sts = SLEEP;
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_PIN);
    	if(artur_income[0] == 0x37)
    	{
    		delay_micros(100);
    	    artur_output.difference = SyncTracker_GetDifferenceUs(&sync_tracker);
    	    artur_output.sync_status = (uint8_t)SyncTracker_GetState(&sync_tracker);
    	    artur_output.snr = SNR;
    	    HAL_HalfDuplex_EnableTransmitter(&huart1);
    	    HAL_UART_Transmit(&huart1, (uint8_t*)(&artur_output), sizeof(artur_output), 2);
    	}
    	artur_income[0] = 0x00; artur_income[1] = 0x00; artur_income[2] = 0x00;
    	NVIC_EnableIRQ(ADC1_2_IRQn);
    	HAL_HalfDuplex_EnableReceiver(&huart1);
        HAL_UART_Receive_IT(&huart1, artur_income, 1);
    }
}
#endif

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
