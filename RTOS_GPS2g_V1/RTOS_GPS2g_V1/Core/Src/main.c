/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#include <string.h>
#include <math.h>




float SpeedThr=80;
char DataString[300];
char GPSString[100];
char GPSLine[100];
int GPSLineN=0;
int seqNo=0;
int gpsSt=0;
char Regno[20];
char IMEI[20];
char in3[1000];
char inn[2];
char INSMSno[30];
char OUTSMSno[30];
char EmgIP[50];
char RegIP[50];
char TracIP[50];
int AccGyroStatus=0;
int AccGyroStatus1=0;
char simop[25];
int TimeCount=0;
int RunCnt=0;
int DMAon=1;
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_tx;

static void Debug_Tx(char[]);
static char* Debug_Rx(void);
static void GSM_Tx(const char[]);
//static void GPS_Tx(const char[]);
static void GSM_TxL(const char[]);
//static char* GSM_Rx(void);
//static char* GSM_RxL(void);
float stor(const char*);
void initFirstRun(void);
void Run(void);
void timedWork(void);
void TestRun(void);
void ReadAllGPIO(void);

#include "gpio.h"
#include "gsm.h"
#include "mem.h"
#include "acc.h"
#include "gps.h"
#include "testCode.h"
#include "runtime.h"
//#include "FOTA.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 16
};
/* Definitions for GPS_read */
osThreadId_t GPS_readHandle;
const osThreadAttr_t GPS_read_attributes = {
  .name = "GPS_read",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 2
};
/* Definitions for StatusLED */
osThreadId_t StatusLEDHandle;
const osThreadAttr_t StatusLED_attributes = {
  .name = "StatusLED",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for testQ */
osMessageQueueId_t testQHandle;
const osMessageQueueAttr_t testQ_attributes = {
  .name = "testQ"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

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

  /* Create the queue(s) */
  /* creation of testQ */
  testQHandle = osMessageQueueNew (16, sizeof(uint16_t), &testQ_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of GPS_read */
  GPS_readHandle = osThreadNew(StartTask02, NULL, &GPS_read_attributes);

  /* creation of StatusLED */
  StatusLEDHandle = osThreadNew(StartTask03, NULL, &StatusLED_attributes);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;

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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 4;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00000E14;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 8000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 500-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);
  /* DMA1_Ch4_7_DMAMUX1_OVR_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Ch4_7_DMAMUX1_OVR_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(DMA1_Ch4_7_DMAMUX1_OVR_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_MEM_Pin|DO_GPS_VCC_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DO_GSM_VCC_EN_Pin|DO_PWRKEY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DO_LED_PWR_Pin|DO_LED_NET_Pin|DO_LED_GPS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DO_OUT1_Pin|DO_OUT2_Pin|DO_5V_OUT_EN_Pin|DO_OUT3_P_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : SPI_CS_MEM_Pin DO_GPS_VCC_EN_Pin */
  GPIO_InitStruct.Pin = SPI_CS_MEM_Pin|DO_GPS_VCC_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_IN2_Pin DI_IN3_Pin DI_MAINS_STATE_Pin DI_ACC_STATE_Pin
                            DI_INT1_ACCEL_Pin */
  GPIO_InitStruct.Pin = DI_IN2_Pin|DI_IN3_Pin|DI_MAINS_STATE_Pin|DI_ACC_STATE_Pin
                          | DI_INT1_ACCEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  /*Configure GPIO pins :  DI_BOX_STATE_Pin  */
  GPIO_InitStruct.Pin =  DI_BOX_STATE_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;//GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_SOS_STATE_Pin  */
  GPIO_InitStruct.Pin = DI_SOS_STATE_Pin ;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;//GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins :  DI_IN1_Pin */
  GPIO_InitStruct.Pin = DI_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DO_GSM_VCC_EN_Pin DO_PWRKEY_Pin */
  GPIO_InitStruct.Pin = DO_GSM_VCC_EN_Pin|DO_PWRKEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DO_LED_PWR_Pin DO_LED_NET_Pin DO_LED_GPS_Pin */
  GPIO_InitStruct.Pin = DO_LED_PWR_Pin|DO_LED_NET_Pin|DO_LED_GPS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DO_OUT1_Pin DO_OUT2_Pin DO_5V_OUT_EN_Pin DO_OUT3_P_LED_Pin */
  GPIO_InitStruct.Pin = DO_OUT1_Pin|DO_OUT2_Pin|DO_5V_OUT_EN_Pin|DO_OUT3_P_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */




void timedWork(){
	HAL_GPIO_TogglePin(GPIOD, DO_LED_GPS_Pin);

		//%%%%%%%    AccGyro Rest    %%%%%%
	    if (AccGyroStatus1==0){AccGyroStatus=detectAcc();}
	    if (AccGyroStatus>0){AccGyroStatus1=AccGyroStatus;}

}

float stor(const char* str) {
    float result = 0;
    float sign = *str == '-' ? str++, -1 : 1;
    while (*str >= '0' && *str <= '9') {
        result *= 10;
        result += *str - '0';
        str++;
    }
    if (*str == ',' || *str == '.') {
        str++;
        float multiplier = 0.1;
        while (*str >= '0' && *str <= '9') {
            result += (*str - '0') * multiplier;
            multiplier /= 10;
            str++;
        }
    }
    result *= sign;
    if (*str == 'e' || *str == 'E') {
        str++;
        float powerer = *str == '-'? str++, 0.1 : 10;
        float power = 0;
        while (*str >= '0' && *str <= '9') {
            power *= 10;
            power += *str - '0';
            str++;
        }
        result *= pow(powerer, power);
    }
    return result;
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{


}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	{

	  HAL_GPIO_TogglePin (GPIOD, DO_LED_GPS_Pin);
	//Debug_Tx("V");
	if(huart==&huart2){
		if(dnlfile==0){
		if(strlen((char*)gpsData)>10){

	  		//Debug_Tx((char*)gpsData);
			ProcessGPS((char*)gpsData);
			getGPSString();
			timedWork();
			ReadAllGPIO();
		}
		if(DMAon==1){
		if(strlen((char*)gpsData)>0) memset((char*)gpsData,0,999);
		//__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF|UART_CLEAR_OREF);
		HAL_UART_Receive_DMA(&huart2, gpsData, 900);}
		}
	}
	if(huart==&huart1){
		if(strlen((char*)GSMData)<1900){
		    if(GSMBuff[0]!=0)	strcat(GSMData,(char*)GSMBuff);
		}


		if(DMAon==1){
		GSMBuff[0]=0;
		//__HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
		HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);
		}
	}
	}

}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart){
	if(huart==&huart2){

		if(dnlfile==0){
		if(DMAon==1){
		//Debug_Tx("GPSErr");
    	if(strlen((char*)gpsData)>0) memset((char*)gpsData,0,999);
    	__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF|UART_CLEAR_OREF);
    	HAL_UART_Receive_DMA(&huart2, gpsData, 900);
		}
		}
	}
	if(huart==&huart1){

		if(DMAon==1){
		//Debug_Tx("GSMErr");
		GSMBuff[0]=0;
		__HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
		__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF|UART_CLEAR_OREF);
			    	HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);
		}
	}





}



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
  /* Infinite loop */



	 // TestDigitalOutput();
initAcc();
//TestDigitalOutput();
SET_GPS_VCC_EN(1);
SET_GSM_VCC_EN(1);
SET_PWRKEY(1);
Init_ADC();
SET_5V_OUT_EN(1);
SET_GPS_VCC_EN(1);
SET_GSM_VCC_EN(1);
SET_PWRKEY(1);
GSMBuff[0]=0;
 memset((char*)gpsData,0,1000);
__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF|UART_CLEAR_OREF);
HAL_UART_Receive_DMA(&huart2, gpsData, 900);
__HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);
initGPS();
initFirstRun();
InitMEMQ();
HAL_Delay(10000);
InitGSM();
TestMEM();

 memset(IMEI,0,20);
 memset(Regno,0,20);
strcpy(IMEI,GSMIMEI());
 strcpy(Regno,readRegNo());

 Debug_Tx("IMEI:");
	  Debug_Tx(IMEI);

	  //StartTCPConnection(0);
	  //StartTCPConnection(1);
	  //StartTCPConnection(2);
while (1){//DownloadFile();
  GSMSigQuality();
 TestRun();
}
  while(1)
  {
	  	//
	 // 	TestMEMQ();

  	//	 TestDigitalInput();
	//  	 TestAnalogInput();
		//TestGPS();
	  	// TestGSM();
	//  	TestACC();
	 // 	TestMEM();
	   	//
		 //
		 //

	//
	//
    //osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the GPS_read thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */

while(1){osDelay(1);
	  }





/*
	 // TestDigitalOutput();
	  SET_5V_OUT_EN(1);
	 	  SET_GPS_VCC_EN(1);
	 	  SET_GSM_VCC_EN(1);
	 	  SET_PWRKEY(1);
	  		//  TestDigitalInput();
	  		//  Init_ADC();
	  		  //TestAnalogInput();
	  		  initGPS();

		  InitGSM();
			HAL_Delay(10000);
		  while (1){

		 		  HAL_UART_Receive_DMA(&huart2, gpsData, 900);
		 			HAL_GPIO_TogglePin(GPIOD, DO_LED_PWR_Pin);
		 		  int whiletotal=tic();

		 		  GSMSigQuality();
		 			  TestRun();
		 		  toc(whiletotal,"_________________________While loop Total");



		 	  }
	  while(1){

		  TestGPS();

		  TestGSM();
		  initAcc();

		  TestACC();
		  GetLastAddress();
		  InitMEMQ();
		  TestMEM();
		  //TestMEMQ();
	  }
	  initGPS();
	  //GSM Init Block
	    GSMBuff[0]=0;
		__HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
		HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);
	    Debug_Tx("---------GSM init start");
	   int nn=tic();
		HAL_Delay(3000);
	   InitGSM();
	   toc(nn,"_________________________GSM INIT");


	    memset((char*)gpsData,0,1000);
		    __HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF|UART_CLEAR_OREF);
		    HAL_UART_Receive_DMA(&huart2, gpsData, 900);

	  Debug_Tx("----------GSM init done");
	  //END GSM Init Block


	  // Memory init block
	  initFirstRun();
	  GetLastAddress();
	  InitMEMQ();
	  // END Memory init block






	  initAcc();
	  Init_ADC();



		HAL_Delay(10000);



	  Debug_Tx("IMEI:");
	  Debug_Tx(IMEI);
	  Debug_Tx("RegNo:");
	  Debug_Tx(Regno);

	  StartTCPConnection();
	  gpsSt=0;



*/

	  /* USER CODE END 2 */

	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */

  for(;;)
  {


    osDelay(1);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the StatusLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}




void restartGSMuart(){GSMBuff[0]=0;__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF|UART_CLEAR_OREF);

__HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);

}
void initFirstRun(){//20.210.207.21\",5001
	ClearQueue();
	writeConfig("AS-o1-A-9222\0","in sim no1234567891234in sim no\0","out sim no 123456789123456out sim no\0",
 	  				  "20.210.207.21","20.210.207.21","20.210.207.21",
	  				  "oooooooootttttthheerrrrrrOtherdatadddaaatttttttttaaaaaaaaa\0");

}




void ReadAllGPIO(){
	memset(StatusStrng,0,20);
	memset(Dig_io,0,30);

	//%%%%%%%    GPIO Read    %%%%%%
	Dig_in[0]=Read_DI_IN1();
	Dig_in[1]=Read_DI_IN2();
	Dig_in[2]=Read_DI_IN3();

	MAINS_STATE=Read_DI_MAINS_STATE();
	ACC_STATE=Read_DI_ACC_STATE();
	BOX_STATE=Read_DI_BOX_STATE();
	SOS_STATE=Read_DI_SOS_STATE();
	EXT_B=Read_EXT_B_SENSE();
	INT_B=Read_INT_B_SENSE();


	adc[0]=Read_ADC1();
	adc[1]=Read_ADC2();

	memset(StatusStrng,0,20);
	memset(Dig_io,0,30);

    sprintf(Dig_io, "%d%d%d0,%d%d,%d,0.0,",Dig_in[0],Dig_in[1],Dig_in[2],Digout1,Digout2,seqNo);//Dig_io
	sprintf(StatusStrng, "%d,%d,%.1f,%.1f,%d,%c",ACC_STATE,MAINS_STATE,EXT_B,INT_B,SOS_STATE,BOX_STATE);
	//Debug_Tx("GPIO STATUS");
	//Debug_Tx(StatusStrng);

}


void GetHead(){

	memset(Head,0,100);
	strcpy(Head,InitStr);
	strcat(Head,",");
	strcat(Head,VerStr);

	if((EmergencyStateON==-1 )& (SOS_STATE==1 )){EmergencyStateON=1;EmergencyStateOFF=-1;}
	if((EmergencyStateOFF==-1 )& (SOS_STATE==0) ){EmergencyStateOFF=1;EmergencyStateON=-1;}
	if((IgnitionTurnedON==-1 )&( MAINS_STATE==1 )){IgnitionTurnedON=1;IgnitionTurnedOFF=-1;}
	if((IgnitionTurnedOFF==-1) &( MAINS_STATE==0 )){IgnitionTurnedOFF=1;IgnitionTurnedON=-1;}
	if((InternalBatterLow==-1) & (INT_B<3.0 )){InternalBatterLow=1;InternalBatteryChargedAgain=-1;}
	if((InternalBatteryChargedAgain==-1 )& (INT_B>=3.0)){InternalBatteryChargedAgain=1;InternalBatterLow=-1;}
	if((MainBatteryDisconnect==-1 )& (INT_B<3.0 )){MainBatteryDisconnect=1;MainBatteryReconnect=-1;}
	if((MainBatteryReconnect==-1 )& (INT_B>=3.0)){MainBatteryReconnect=1;MainBatteryDisconnect=-1;}
	if(GSMSignal<=10){
		strcat(Head,AlartStr_HistoryPVTData);
	}
	else if(getSpeed()>SpeedThr){
		strcat(Head,AlartStr_OverSpeed);
	}
	else if(EmergencyWireBreak>0){
		strcat(Head,AlartStr_EmergencyWireBreak);
		EmergencyWireBreak=0;
	}
	else if(AccGyroStatus==1){
		strcat(Head,AlartStr_HarshAcceleration);
	}

	else if(AccGyroStatus==2){
		strcat(Head,AlartStr_HarshBreaking);
	}
	else if(AccGyroStatus==3){
		strcat(Head,AlartStr_RashTurning);
	}
	else if(EmergencyStateON==1){
		strcat(Head,AlartStr_EmergencyStateON);
		//EmergencyStateON=0;
	}
	else if(EmergencyStateOFF==1){
		strcat(Head,AlartStr_EmergencyStateOFF);
		//EmergencyStateOFF=0;
	}
	else if(IgnitionTurnedON>0){
		strcat(Head,AlartStr_IgnitionTurnedON);
		IgnitionTurnedON=0;
	}
	else if(IgnitionTurnedOFF>0){
		strcat(Head,AlartStr_IgnitionTurnedOFF);
		IgnitionTurnedOFF=0;
	}
	else if(BOX_STATE=='C'){
		strcat(Head,AlartStr_BoxTemper);
	}
	else if(InternalBatterLow>0){
		strcat(Head,AlartStr_InternalBatterLow);
		InternalBatterLow=0;
	}
	else if(InternalBatteryChargedAgain>0){
		strcat(Head,AlartStr_InternalBatteryChargedAgain);
		InternalBatteryChargedAgain=0;
	}
	else if(MainBatteryDisconnect>0){
		strcat(Head,AlartStr_MainBatteryDisconnect);
		MainBatteryDisconnect=0;

	}
	else if(MainBatteryReconnect>0){
		strcat(Head,AlartStr_MainBatteryReconnect);
		MainBatteryReconnect=0;
	}
	else if(HistoryPVTData>0){
		strcat(Head,AlartStr_HistoryPVTData);
		HistoryPVTData=0;
	}
	else if(HealthPacket>0){
		strcat(Head,AlartStr_HealthPacket);
		HealthPacket=0;
	}
	else if(OTAParameterChange>0){
		strcat(Head,AlartStr_OTAParameterChange);
		OTAParameterChange=0;
	}
	else{
		strcat(Head,AlartStr_NormalPkt);
	}


	if(GSMSignal<=10){
			strcat(Head,PacketStatusStrHist);
	}else{
		strcat(Head,PacketStatusStrLive);
	}

	//%%%%%%%    AccGyro Rest    %%%%%%
    if (AccGyroStatus1>0){AccGyroStatus1=0;}


}

void TestRun(){



	if(errorlen>10){
		Debug_Tx("GSMUART error rebooting device");
		NVIC_SystemReset();
	}




	printInt(seqNo);


	MAINS_STATE=Read_DI_MAINS_STATE();
	ACC_STATE=Read_DI_ACC_STATE();
	BOX_STATE=Read_DI_BOX_STATE();
	SOS_STATE=Read_DI_SOS_STATE();
	EXT_B=Read_EXT_B_SENSE();
	INT_B=Read_INT_B_SENSE();


	Debug_Tx("TEST1");
	adc[0]=Read_ADC1();
	adc[1]=Read_ADC2();
	if(EXT_B>7){
		SET_LED_PWR(1);
	}
	else{
		SET_LED_PWR(0);
	}



memset(INSMSno,0,30);
memset(OUTSMSno,0,30);
memset(EmgIP,0,50);
memset(RegIP,0,50);
memset(TracIP,0,50);
memset(simop,0,25);

Debug_Tx("TEST2");
strcpy(INSMSno,readINSMSno());
strcpy(OUTSMSno,readOUTSMSno());
Debug_Tx("TEST2-1");
strcpy(EmgIP,readEmgIP());
strcpy(RegIP,readRegIP());
strcpy(TracIP,readTracIP());
//Debug_Tx("BEFORE SIM OP");
Debug_Tx("TEST2-2");
strcpy(simop, GSMSimOperator());

Debug_Tx("TEST3");
//Debug_Tx("AFTER SIM OP");
//strcpy(simop, GSMSimOperator());
//strcpy(simop, GSMSimOperator());


	//%%%%%%%    GSM Info Read    %%%%%%
	int GSMinfoT=tic();
	GSMCellInfo();

	//toc(GSMinfoT,"_________________________While loop GSMinfo");


	// %%%%%%%%%%%%%%%%%%%%%%%%%Create Protocall %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	GetHead();
	Debug_Tx("TEST4");

/*

	if (debug==122){
		memset(DataString,0,300);
		strcat(DataString,"Head:");
		strcat(DataString,Head);strcat(DataString,"\nRegno:");
		strcat(DataString,Regno);strcat(DataString,"\nGPSDS:");
		strcat(DataString,gpsDataRet);strcat(DataString,"\nIMEI:");
		strcat(DataString,IMEI);strcat(DataString,"\nSIMOP:");
		strcat(DataString,simop);strcat(DataString,"\nStratusString:");
		strcat(DataString,StatusStrng);strcat(DataString,"\nGsminfo:");
		strcat(DataString,gsminfo);strcat(DataString,"\nDig_io:");
		strcat(DataString,Dig_io);strcat(DataString,",\n\0");
		Debug_Tx(DataString);
	}**/
	char v='V';
	if (GPSInfo.fix>0)v='A';
	sprintf(gpsDataRet2, "%s,%s,%c,%0.6f,%1s,%0.6f,%1s,%0.1f,%0.1f,%0.1f,G",GPSInfo.date,GPSInfo.time,v,GPSInfo.lat,GPSInfo.latD,GPSInfo.lon,GPSInfo.lonD,GPSInfo.alt,GPSInfo.speed,GPSInfo.head);


	Debug_Tx("TEST5");
	memset(DataString_em1,0,150);
	strcat(DataString_em1,"$,EPB,EMR,");
	strcat(DataString_em1,IMEI);strcat(DataString_em1,",NM,");
	strcat(DataString_em1,gpsDataRet2);strcat(DataString_em1,",");
	strcat(DataString_em1,Regno);strcat(DataString_em1,",0000000000,");
	sprintf(checksum, "%02x",nmea0183_checksum(DataString_em1));
	strcat(DataString_em1,checksum);
	strcat(DataString_em1,",*\0");

	memset(DataString_em2,0,150);
	strcat(DataString_em2,"$,EPB,EMR,");
	strcat(DataString_em2,IMEI);strcat(DataString_em2,",NM,");
	strcat(DataString_em2,gpsDataRet2);strcat(DataString_em2,",");
	strcat(DataString_em2,Regno);strcat(DataString_em2,",0000000000,");
	sprintf(checksum, "%02x",nmea0183_checksum(DataString_em2));
	strcat(DataString_em2,checksum);
	strcat(DataString_em2,",*\0");

	sprintf(gpsDataRet2, "%0.6f%1s%0.6f%1s", GPSInfo.lat,GPSInfo.latD,GPSInfo.lon,GPSInfo.lonD);


	Debug_Tx("TEST6");

	memset(data_LOGIN,0,100);
	strcat(data_LOGIN,"$");
	strcat(data_LOGIN,Regno);strcat(data_LOGIN,",");
	strcat(data_LOGIN,"$");
	strcat(data_LOGIN,IMEI);strcat(data_LOGIN,",");
	strcat(data_LOGIN,"$");
	strcat(data_LOGIN,VerStr);strcat(data_LOGIN,",");
	strcat(data_LOGIN,"$");
	strcat(data_LOGIN,VerStr);strcat(data_LOGIN,",");strcat(data_LOGIN,gpsDataRet2);strcat(data_LOGIN,",");
	sprintf(checksum, "%02x",nmea0183_checksum(data_LOGIN));
	strcat(data_LOGIN,checksum);
	strcat(data_LOGIN,",*\0");
	//Debug_Tx("Login Packet Created");
	//Debug_Tx(data_LOGIN);


    memset(checksum,0,3);
    memset(DataString,0,300);
	strcat(DataString,Head);strcat(DataString,",");
	strcat(DataString,IMEI);strcat(DataString,",");
	strcat(DataString,Regno);strcat(DataString,",");
	strcat(DataString,gpsDataRet);strcat(DataString,",");
	strcat(DataString,simop);strcat(DataString,",");
 	strcat(DataString,StatusStrng);strcat(DataString,",");
 	strcat(DataString,gsminfo);strcat(DataString,",");
 	strcat(DataString,Dig_io);//strcat(DataString,"\0");
 	// %%%%%%%%%%%%%%%%%%%%%%%%%Add Checksum %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

	Debug_Tx("TEST7");
    sprintf(checksum, "%02x",nmea0183_checksum(DataString));
    strcat(DataString,checksum);
    strcat(DataString,",*\0");




    if (debug==1){
    }

	Debug_Tx("TEST8");
    if(GSMSignal > 5) 		{
    	while(ReadQdata()>0){
			//
    		ProcessTCPAll( ReadMDataS,0);
    	}
    	// %%%%%%%%%%%%%%%%%%%%%%%%Send Protocall %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    	int tcpsendT=tic();

     	ProcessTCPAll(DataString,0);
     	if(EmergencyStateON==1 && EmergencyStateOFF==-1){
         	ProcessTCPAll(DataString_em1,1);
     	}
     	if(EmergencyStateON==-1 && EmergencyStateOFF==1){
         	ProcessTCPAll(DataString_em2,1);
         	EmergencyStateOFF=0;
         	//EmergencyStateON=0;
     	}
     	toc( tcpsendT,"_________________________While loop tcp send data");
     	RunCnt=0;
    }
    else{
    	if (debug==1){
    		Debug_Tx("No GSM Signal Saving Data to Memory----");
    	}
    	WriteQdata((uint8_t*)DataString, strlen(DataString)+1);
    	RunCnt++;
    	if(RunCnt>10){
    		//InitGSM();
    		RunCnt=0;
    	}

    	//HAL_Delay(5000);
    }


	Debug_Tx("TEST reloop1");
	/*while(TimeDelay(TimeCount,5)==1){
		HAL_Delay(10);
		timedWork();

	}
	*/
	//TimeCount=tic();


 	//ProcessTCPAll(TracIP,"6055", DataString);
 	//ProcessTCPAll("34.74.249.18","300", DataString);
 	//ProcessTCPAll("34.74.249.18","300", DataString, "taisysnet");

 	seqNo=seqNo+1;


}




static void Debug_Tx(char _out[]){

	__HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_NEF|UART_CLEAR_OREF);
	HAL_UART_Transmit(&huart3, (uint8_t *) _out, strlen(_out), 5000);
	char newline[2] = "\r\n";
	HAL_UART_Transmit(&huart3, (uint8_t *) newline, 2, 10);


}


static char* Debug_Rx(){

	__HAL_UART_CLEAR_IT(&huart3, UART_CLEAR_NEF|UART_CLEAR_OREF);
	HAL_UART_Receive(&huart3, (uint8_t *)inn, 1000, 300);
	return(inn);

}




static void GSM_Tx(const char _out[]){
	 __HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
		HAL_UART_Transmit(&huart1, (uint8_t *) _out, strlen(_out), 300);
	//memset(_out,0,strlen(_out));
}

static void GSM_TxL(const char _out[]){

	  __HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
	HAL_UART_Transmit(&huart1, (uint8_t *) _out, strlen(_out), 5000);
	//memset(_out,0,strlen(_out));
}



/*
static void GPS_Tx(const char _out[]){
	__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF|UART_CLEAR_OREF);
	HAL_UART_Transmit(&huart2, (uint8_t *) _out, strlen(_out), 300);
	//memset(_out,0,strlen(_out));
}




static char* GSM_RxL(){
	memset(GSMData,0,1000);
	//memset(ret,0,1000);
	__HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
	HAL_UART_Receive(&huart1, (uint8_t *)GSMData, 500, 350);
	//timedWork();
	return(GSMData);

}


static char* GSM_Rx(){
	GSMBuff[0]=0;
	    	//__HAL_UART_CLEAR_IT(&huart2, UART_CLEAR_NEF|UART_CLEAR_OREF);
	HAL_UART_Receive_DMA(&huart1, GSMBuff, 1);
	memset(GSMData,0,1000);

	  __HAL_UART_CLEAR_IT(&huart1, UART_CLEAR_NEF|UART_CLEAR_OREF);
	HAL_UART_Receive(&huart1, (uint8_t *)GSMData, 100, 50);

	return(GSMData);
}



*/







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
