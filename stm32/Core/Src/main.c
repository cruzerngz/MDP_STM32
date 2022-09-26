/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"

#include "move.h"
#include "uart_state_machine.h"
#include "oled.h"
#include "encoder.h"
#include "ir_adc.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for movement_task */
osThreadId_t movement_taskHandle;
const osThreadAttr_t movement_task_attributes = {
  .name = "movement_task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for uart_state_mach */
osThreadId_t uart_state_machHandle;
const osThreadAttr_t uart_state_mach_attributes = {
  .name = "uart_state_mach",
  .stack_size = 400 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for encoder_display */
osThreadId_t encoder_displayHandle;
const osThreadAttr_t encoder_display_attributes = {
  .name = "encoder_display",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encoder_poll_ro */
osThreadId_t encoder_poll_roHandle;
const osThreadAttr_t encoder_poll_ro_attributes = {
  .name = "encoder_poll_ro",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for ir_adc_task */
osThreadId_t ir_adc_taskHandle;
const osThreadAttr_t ir_adc_task_attributes = {
  .name = "ir_adc_task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ir_adc_poller_t */
osThreadId_t ir_adc_poller_tHandle;
const osThreadAttr_t ir_adc_poller_t_attributes = {
  .name = "ir_adc_poller_t",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void movement(void *argument);
void state_machine(void *argument);
void encoder(void *argument);
void encoder_poller(void *argument);
void ir_adc(void *argument);
void ir_adc_poller(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t UART_RX_BUF[20]; // 20 char UART receive buffer
uint8_t UART_RX_CHAR;    // single char receive buffer
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
    OLED_Init();

    ir_adc_init(&hadc1);

    HAL_UART_Receive_IT(&huart3, &UART_RX_CHAR, 1);

    motor_init_timer(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2);
    motor_init_gpio_left(GPIOA, AIN1_Pin, AIN2_Pin);
    motor_init_gpio_right(GPIOA, BIN1_Pin, BIN2_Pin);
    motor_stop();
    servo_init(&htim1, TIM_CHANNEL_4);

    encoder_init(&htim2, &htim3, TIM_CHANNEL_ALL, TIM_CHANNEL_ALL);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of movement_task */
//  movement_taskHandle = osThreadNew(movement, NULL, &movement_task_attributes);

  /* creation of uart_state_mach */
//  uart_state_machHandle = osThreadNew(state_machine, NULL, &uart_state_mach_attributes);

  /* creation of encoder_display */
  encoder_displayHandle = osThreadNew(encoder, NULL, &encoder_display_attributes);

  /* creation of encoder_poll_ro */
  encoder_poll_roHandle = osThreadNew(encoder_poller, NULL, &encoder_poll_ro_attributes);

  /* creation of ir_adc_task */
//  ir_adc_taskHandle = osThreadNew(ir_adc, NULL, &ir_adc_task_attributes);

  /* creation of ir_adc_poller_t */
//  ir_adc_poller_tHandle = osThreadNew(ir_adc_poller, NULL, &ir_adc_poller_t_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
    /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the analog watchdog
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 2900;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_10;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE5 PE6 PE7 PE8
                           LED3_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin BIN1_Pin BIN2_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin|BIN1_Pin|BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUZZER_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_Pin */
  GPIO_InitStruct.Pin = BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

// USART receive callback
// State machine functions only called here
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    // prevent unused arg warning during compilation
    UNUSED(huart);

    __disable_irq();
    uint8_t response = state_machine_interpreter(UART_RX_CHAR);
    __enable_irq();

    // send back data in non blocking mode
    HAL_UART_Transmit_IT(&huart3, (uint8_t *)&response, 1);

    HAL_UART_Receive_IT(&huart3, &UART_RX_CHAR, sizeof(UART_RX_CHAR));
}

/**
 * @brief  Analog watchdog callback in non blocking mode.
 * @param  hadc: ADC handle
 * @retval None
 */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
    /* Set variable to report analog watchdog out of window status to main program. */
//    ubAnalogWatchdogStatus = SET;
}

// Interrupt routine for GPIO pins
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	// button routine: activate/deactivate move to obstacle
	if(GPIO_Pin == BUTTON_Pin) {
		FLAG_APPROACH_OBSTACLE ^= 0x1;
	}
}



/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	static uint32_t ticks = 0;
    /* Infinite loop */
    for (;;)
    {
    	ticks = osKernelGetTickCount();
    	_set_motor_speed_pid(MotorDirForward, MotorLeft, 100);
    	_set_motor_speed_pid(MotorDirForward, MotorRight, 100);

        osDelayUntil(ticks + 40); //25 hz
    }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_movement */
extern void (*HARDCODE_DIRECTION)(void); // func from state machine
/**
 * @brief Function implementing the movement_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_movement */
void movement(void *argument)
{
  /* USER CODE BEGIN movement */
    static uint32_t ticks = 0;

    static volatile uint16_t mvmt_dist = 0;
    static volatile uint16_t turn_angle = 0;
    static volatile uint8_t cardinal = 0;
    static volatile int8_t appr_obstacle = 0;

    static volatile int8_t move_dir = 0;
    static volatile int8_t turn_dir = 0;

    // Flags set in the UART interrupt routine set here
    /* Infinite loop */
    for (;;)
    {
        // HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
        ticks = osKernelGetTickCount();


        __disable_irq();
        mvmt_dist = FLAG_MOVEMENT_DISTANCE;
        turn_angle = FLAG_TURN_ANGLE;
        cardinal = FLAG_IN_PLACE_CARDINAL;
        appr_obstacle = FLAG_APPROACH_OBSTACLE;

        move_dir = FLAG_MOVE_DIR;
        turn_dir = FLAG_TURN_DIR;

        FLAG_MOVEMENT_DISTANCE = 0;
        FLAG_TURN_ANGLE = 0;
        FLAG_IN_PLACE_CARDINAL = 0;
        FLAG_APPROACH_OBSTACLE = 0;

        FLAG_MOVE_DIR = 0;
        FLAG_TURN_DIR = 0;
        __enable_irq();

        if (mvmt_dist != 0)
        {

            if (move_dir < 0)
            {
                // HAL_UART_Transmit(&huart3, (uint8_t *)"MoveB\r\n", 10, HAL_MAX_DELAY);
                move_backward_calc(mvmt_dist);
                USART3_SEND_CHAR('&'); // algo needs this to know when to send the next command
            }
            else if (move_dir > 0)
            {
                // HAL_UART_Transmit(&huart3, (uint8_t *)"MoveF\r\n", 10, HAL_MAX_DELAY);
                 move_forward_calc(mvmt_dist);
                 USART3_SEND_CHAR('&');
//                move_to_obstacle();
                // HAL_UART_Transmit(&huart3, (uint8_t *)"MoveOK\r\n", 10, HAL_MAX_DELAY);
            }
        }

        if (turn_angle != 0)
        {

            if (move_dir < 0)
            {
                move_turn_backward_by(turn_dir > 0 ? MoveDirRight : MoveDirLeft, turn_angle);
                USART3_SEND_CHAR('&');
            }
            else if (move_dir > 0)
            {
                move_turn_forward_by(turn_dir > 0 ? MoveDirRight : MoveDirLeft, turn_angle);
                USART3_SEND_CHAR('&');
            }
        }

        if (cardinal != 0)
        {
            move_in_place_turn_cardinal(cardinal);
            USART3_SEND_CHAR('&');
        }

        if(appr_obstacle == 1) {
          move_to_obstacle();
          USART3_SEND_CHAR('&');
          appr_obstacle = 0;
        }

        osDelayUntil(ticks + 100); // 10hz polling
    }
  /* USER CODE END movement */
}

/* USER CODE BEGIN Header_state_machine */
/**
 * @brief Function implementing the uart_state_mach thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_state_machine */
void state_machine(void *argument)
{
  /* USER CODE BEGIN state_machine */
    /* Infinite loop */
    for (;;)
    {
        osDelay(1);
    }
  /* USER CODE END state_machine */
}

/* USER CODE BEGIN Header_encoder */
/**
 * @brief Function implementing the encoder_display thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_encoder */
void encoder(void *argument)
{
  /* USER CODE BEGIN encoder */
    static uint32_t ticks = 0;
    static char oled_lines[6][17] = {0};

    OLED_Display_On();

    /* Infinite loop */
    for (;;)
    {
        ticks = osKernelGetTickCount();

        taskENTER_CRITICAL();
        // note here that strings are max 16 (+1 null) chars long
        sprintf(oled_lines[0], "L<%05lu  %05lu>R", ENCODER_POS_DIRECTIONAL[0], ENCODER_POS_DIRECTIONAL[1]);
        sprintf(oled_lines[1], "L<%+05d  %+05d>R", ENCODER_SPEED_DIRECTIONAL[0], ENCODER_SPEED_DIRECTIONAL[1]);
        sprintf(oled_lines[2], " "); // blank line
        taskEXIT_CRITICAL();

        OLED_ShowString(0, 0, (uint8_t *)oled_lines[0]);
        OLED_ShowString(0, 10, (uint8_t *)oled_lines[1]);
        OLED_ShowString(0, 20, (uint8_t *)oled_lines[2]);

        OLED_Refresh_Gram();

        osDelayUntil(ticks + 100); // 10hz display freq
    }
  /* USER CODE END encoder */
}

/* USER CODE BEGIN Header_encoder_poller */
/**
 * @brief Function implementing the encoder_poll_ro thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_encoder_poller */
void encoder_poller(void *argument)
{
  /* USER CODE BEGIN encoder_poller */

    // Polls and updates the encoder values

    static uint32_t ticks = 0;
    /* Infinite loop */
    for (;;)

    {
        ticks = osKernelGetTickCount();

        taskENTER_CRITICAL();
        encoder_poll();
        taskEXIT_CRITICAL();

        osDelayUntil(ticks + MOTOR_ENCODER_REFRESH_INTERVAL_TICKS);
    }
  /* USER CODE END encoder_poller */
}

/* USER CODE BEGIN Header_ir_adc */
/**
 * @brief Function implementing the ir_adc_task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ir_adc */
void ir_adc(void *argument)
{
  /* USER CODE BEGIN ir_adc */
    // static uint16_t IR_READOUT = 0;
    static uint32_t ticks = 0;
    static char buffer_ADC[20];

    for (;;)
    {
        taskENTER_CRITICAL();
        sprintf(
        		buffer_ADC,
				"%d %d %d\r\n",
				IR_ADC_AVERAGE_READOUT,
				ENCODER_SPEED_DIRECTIONAL[0],
				ENCODER_SPEED_DIRECTIONAL[1]
		);

        taskEXIT_CRITICAL();

        HAL_UART_Transmit(&huart3, (uint8_t *)buffer_ADC, strlen(buffer_ADC), HAL_MAX_DELAY);

        osDelayUntil(ticks + IR_ADC_POLLING_RATE_TICKS);
    }

  /* USER CODE END ir_adc */
}

/* USER CODE BEGIN Header_ir_adc_poller */
/**
 * @brief Function implementing the ir_adc_poller_t thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ir_adc_poller */
void ir_adc_poller(void *argument)
{
  /* USER CODE BEGIN ir_adc_poller */

    // Polls and updates the ir adc values

    static uint32_t ticks = 0;
    /* Infinite loop */
    for (;;)

    {
        ticks = osKernelGetTickCount();

        taskENTER_CRITICAL();
        ir_adc_poll();
        taskEXIT_CRITICAL();

        osDelayUntil(ticks + IR_ADC_POLLING_RATE_TICKS);
    }
  /* USER CODE END ir_adc_poller */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
