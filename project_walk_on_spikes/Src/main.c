/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define ARM_MATH_CM4
#include "stm32l4s5i_iot01_qspi.h"
#include "arm_math.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define ITM_Port32(n) (*((volatile unsigned long *) (0xE0000000+4*n)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DMA_HandleTypeDef hdma_dfsdm1_flt0;

I2C_HandleTypeDef hi2c2;

OSPI_HandleTypeDef hospi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

osThreadId buttonTaskHandle;
osThreadId transmitTaskHandle;
osThreadId readTaskHandle;
osThreadId musicTaskHandle;
/* USER CODE BEGIN PV */
#define buffer_size	1000
int32_t pre_Buffer[buffer_size];
int32_t aft_Buffer[buffer_size];
uint32_t music[42] = {1,1,5,5,6,6,5,4,4,3,3,2,2,1,5,5,4,4,3,3,2,5,5,4,4,3,3,2,1,1,5,5,6,6,5,4,4,3,3,2,1,1};
uint32_t music_index = 0;
uint8_t music_started = 0;
uint8_t first_half_done = 0;
uint8_t second_half_done = 0;
uint8_t start_recording = 0;
uint8_t volume_height = 0;
uint32_t start_status = 0;
int32_t buffmax = 0;
int32_t buffmin = 0;
int32_t mean = 0;
int32_t sum = 0;
uint32_t i;
uint32_t j;
//uint32_t data_index = 0;
uint32_t puttonPushed = 0;
//uint32_t musicCounter = 0;
uint32_t resetCounter = 0;
uint32_t blank_count = 0;
HAL_StatusTypeDef UART_status;
char str[240];
char row1[20];
char row2[20];
char row3[20];
char row4[20];
char row5[20];
char row6[20];
char row7[20];
char row8[20];
char row9[20];
char row10[20];
char row11[20];
int distance = 0;
uint32_t num = 0;
uint32_t done = 0;
uint8_t original[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
uint8_t copy[10];
uint32_t SIN_OUT_800[50];
uint32_t SIN_OUT_1000[40];
uint32_t SIN_OUT_1250[32];
uint32_t SIN_OUT_1600[25];
uint32_t SIN_OUT_2000[20];
uint32_t SIN_OUT_2500[16];
uint32_t offset = 20000;
uint8_t in_Buffer[20000];
uint32_t out_Buffer[20000];
uint8_t wave0[20000];
uint8_t wave1[20000];
uint8_t wave2[20000];
uint8_t wave3[20000];
uint8_t wave4[20000];
uint8_t wave5[20000];
uint8_t wave6[20000];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_OCTOSPI1_Init(void);
void StartButtonTask(void const * argument);
void StartTransmitTask(void const * argument);
void StartReadTask(void const * argument);
void StartMusicTask(void const * argument);

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
  MX_DAC1_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_DFSDM1_Init();
  MX_OCTOSPI1_Init();
  /* USER CODE BEGIN 2 */
  BSP_QSPI_Init();


  for (int i = 0; i < offset; i++){
      float radians = 3.14 * i / 20;
      wave0[i] = (uint8_t) roundf(128 * (1.0 + arm_sin_f32(radians)));
  }
  for (int i = 0; i < offset; i++){
	  float radians = 3.14 * i / 18;
	  wave1[i] = (uint8_t) roundf(128 * (1.0 + arm_sin_f32(radians)));
  }
  for (int i = 0; i < offset; i++){
      float radians = 3.14 * i / 16;
      wave2[i] = (uint8_t) roundf(128 * (1.0 + arm_sin_f32(radians)));
  }
  for (int i = 0; i < offset; i++){
	  float radians = 3.14 * i / 15;
	  wave3[i] = (uint8_t) roundf(128 * (1.0 + arm_sin_f32(radians)));
  }
  for (int i = 0; i < offset; i++){
	  float radians = 3.14 * i / 13;
	  wave4[i] = (uint8_t) roundf(128 * (1.0 + arm_sin_f32(radians)));
  }
  for (int i = 0; i < offset; i++){
	  float radians = 3.14 * i / 12;
	  wave5[i] = (uint8_t) roundf(128 * (1.0 + arm_sin_f32(radians)));
  }
  for (int i = 0; i < offset; i++){
	  float radians = 3.14 * i / 11;
	  wave6[i] = (uint8_t) roundf(128 * (1.0 + arm_sin_f32(radians)));
  }

//  for (uint32_t k = 0; k < 50; k++) {
//	  SIN_OUT_800[k] = (uint32_t) roundf(125*(arm_sin_f32(k*(M_PI/25.0))+1));//1365
//  	}
//  for (uint32_t k = 0; k < 40; k++) {
//	  SIN_OUT_1000[k] = (uint32_t) roundf(125*(arm_sin_f32(k*(M_PI/20.0))+1));
//  	}
//  for (uint32_t k = 0; k < 32; k++) {
//	  SIN_OUT_1250[k] = (uint32_t) roundf(125*(arm_sin_f32(k*(M_PI/16.0))+1));
//  	}
//  for (uint32_t k = 0; k < 25; k++) {
//	  SIN_OUT_1600[k] = (uint32_t) roundf(125*(arm_sin_f32(k*(M_PI/12.5))+1));
//  	}
//  for (uint32_t k = 0; k < 20; k++) {
//	  SIN_OUT_2000[k] = (uint32_t) roundf(125*(arm_sin_f32(k*(M_PI/10.0))+1));
//  	}
//  for (uint32_t k = 0; k < 16; k++) {
//	  SIN_OUT_2500[k] = (uint32_t) roundf(125*(arm_sin_f32(k*(M_PI/8.0))+1));
//  	}

//  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

  if (BSP_QSPI_Erase_Block((uint32_t) 0x007F0000) != QSPI_OK){
	  Error_Handler();
  }
  if (BSP_QSPI_Erase_Block((uint32_t) 0x007E0000) != QSPI_OK){
	  Error_Handler();
  }
  if (BSP_QSPI_Erase_Block((uint32_t) 0x007D0000) != QSPI_OK){
	  Error_Handler();
  }
  if (BSP_QSPI_Erase_Block((uint32_t) 0x007C0000) != QSPI_OK){
	  Error_Handler();
  }
  if (BSP_QSPI_Erase_Block((uint32_t) 0x007B0000) != QSPI_OK){
	  Error_Handler();
  }
  if (BSP_QSPI_Erase_Block((uint32_t) 0x007A0000) != QSPI_OK){
	  Error_Handler();
  }
  if (BSP_QSPI_Erase_Block((uint32_t) 0x00790000) != QSPI_OK){
	  Error_Handler();
  }
//  for (uint32_t i = 0; i < 30000; i++) {
//	  in_Buffer[i] = (uint8_t)SIN_OUT_800[(i % 50)];
//  	}


  if (BSP_QSPI_Write(wave0, (uint32_t) 0x007F0000, 20000) != QSPI_OK){
	  Error_Handler();
  }

//  for (uint32_t i = 0; i < 30000; i++) {
//	  in_Buffer[i] = (uint8_t)SIN_OUT_1000[(i % 40)];
//  	}
  if (BSP_QSPI_Write(wave1, (uint32_t) 0x007E0000, 20000) != QSPI_OK){
	  Error_Handler();
  }

//  for (uint32_t i = 0; i < 30000; i++) {
//	  in_Buffer[i] = (uint8_t)SIN_OUT_1250[(i % 32)];
//  	}
  if (BSP_QSPI_Write(wave2, (uint32_t) 0x007D0000, 20000) != QSPI_OK){
	  Error_Handler();
  }

//  for (uint32_t i = 0; i < 30000; i++) {
//	  in_Buffer[i] = (uint8_t)SIN_OUT_1600[(i % 25)];
//  	}
  if (BSP_QSPI_Write(wave3, (uint32_t) 0x007C0000, 20000) != QSPI_OK){
	  Error_Handler();
  }

//  for (uint32_t i = 0; i < 30000; i++) {
//	  in_Buffer[i] = (uint8_t)SIN_OUT_2000[(i % 20)];
//  	}
  if (BSP_QSPI_Write(wave4, (uint32_t) 0x007B0000, 20000) != QSPI_OK){
	  Error_Handler();
  }

//  for (uint32_t i = 0; i < 30000; i++) {
//	  in_Buffer[i] = (uint8_t)SIN_OUT_2500[(i % 16)];
//  	}
  if (BSP_QSPI_Write(wave5, (uint32_t) 0x007A0000, 20000) != QSPI_OK){
	  Error_Handler();
  }

  if (BSP_QSPI_Write(wave6, (uint32_t) 0x00790000, 20000) != QSPI_OK){
	  Error_Handler();
  }

  /* USER CODE END 2 */

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
  /* definition and creation of buttonTask */
  osThreadDef(buttonTask, StartButtonTask, osPriorityNormal, 0, 128);
  buttonTaskHandle = osThreadCreate(osThread(buttonTask), NULL);

  /* definition and creation of transmitTask */
  osThreadDef(transmitTask, StartTransmitTask, osPriorityNormal, 0, 128);
  transmitTaskHandle = osThreadCreate(osThread(transmitTask), NULL);

  /* definition and creation of readTask */
  osThreadDef(readTask, StartReadTask, osPriorityNormal, 0, 128);
  readTaskHandle = osThreadCreate(osThread(readTask), NULL);

  /* definition and creation of musicTask */
  osThreadDef(musicTask, StartMusicTask, osPriorityNormal, 0, 128);
  musicTaskHandle = osThreadCreate(osThread(musicTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	    HAL_Delay(100);
//		BSP_ACCELERO_AccGetXYZ(accelero);
//		BSP_GYRO_GetXYZ(gyro);
//		BSP_MAGNETO_GetXYZ(magneto);
//		hsensor = BSP_HSENSOR_ReadHumidity();
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T2_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_ABOVE_80MHZ;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC3_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 250;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = ENABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 40;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_EXTERNAL_INPUTS;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_INTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0;
  hdfsdm1_channel2.Init.RightBitShift = 0x00;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10909CEC;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief OCTOSPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OCTOSPI1_Init(void)
{

  /* USER CODE BEGIN OCTOSPI1_Init 0 */

  /* USER CODE END OCTOSPI1_Init 0 */

  OSPIM_CfgTypeDef OSPIM_Cfg_Struct = {0};

  /* USER CODE BEGIN OCTOSPI1_Init 1 */

  /* USER CODE END OCTOSPI1_Init 1 */
  /* OCTOSPI1 parameter configuration*/
  hospi1.Instance = OCTOSPI1;
  hospi1.Init.FifoThreshold = 1;
  hospi1.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
  hospi1.Init.MemoryType = HAL_OSPI_MEMTYPE_MACRONIX;
  hospi1.Init.DeviceSize = 32;
  hospi1.Init.ChipSelectHighTime = 1;
  hospi1.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
  hospi1.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
  hospi1.Init.ClockPrescaler = 1;
  hospi1.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_NONE;
  hospi1.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
  hospi1.Init.ChipSelectBoundary = 0;
  hospi1.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
  if (HAL_OSPI_Init(&hospi1) != HAL_OK)
  {
    Error_Handler();
  }
  OSPIM_Cfg_Struct.ClkPort = 1;
  OSPIM_Cfg_Struct.NCSPort = 1;
  OSPIM_Cfg_Struct.IOLowPort = HAL_OSPIM_IOPORT_1_LOW;
  if (HAL_OSPIM_Config(&hospi1, &OSPIM_Cfg_Struct, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OCTOSPI1_Init 2 */

  /* USER CODE END OCTOSPI1_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB_Pin */
  GPIO_InitStruct.Pin = PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac){
	done = 1;
	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
}
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter){
	first_half_done = 1;
}
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter){
	second_half_done = 1;
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	puttonPushed = 1;

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartButtonTask */
/**
  * @brief  Function implementing the buttonTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartButtonTask */
void StartButtonTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    //ITM_Port32(31) = 1;
    if (puttonPushed==1){
    	puttonPushed = 0;
    	distance = 0;
    	resetCounter = 1;//this indicates the start of the game
    	//initialize 10 rows
        sprintf(row1, "\r*|          |*\n");
        sprintf(row2, "\r*|          |*\n");
        sprintf(row3, "\r*|          |*\n");
        sprintf(row4, "\r*|          |*\n");
        sprintf(row5, "\r*|          |*\n");
        sprintf(row6, "\r*|          |*\n");
        sprintf(row7, "\r*|          |*\n");
        sprintf(row8, "\r*|          |*\n");
        sprintf(row9, "\r*|          |*\n");
        sprintf(row10, "\r*|         O|*\n");
        sprintf(row11, "\r**************");
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTransmitTask */
/**
* @brief Function implementing the transmitTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTransmitTask */
void StartTransmitTask(void const * argument)
{
  /* USER CODE BEGIN StartTransmitTask */
	  HAL_UART_Init(&huart1);
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
    while(resetCounter ==0){
    //do nothing until the button is pressed
    }
    //for each row, copy the last row's content
    for(uint16_t copyIndex =0;copyIndex<20;copyIndex++){
    row10[copyIndex] = row9[copyIndex];
    row9[copyIndex] = row8[copyIndex];
    row8[copyIndex] = row7[copyIndex];
    row7[copyIndex] = row6[copyIndex];
    row6[copyIndex] = row5[copyIndex];
    row5[copyIndex] = row4[copyIndex];
    row4[copyIndex] = row3[copyIndex];
    row3[copyIndex] = row2[copyIndex];
    row2[copyIndex] = row1[copyIndex];
    }
    if (row10[12-volume_height]=='_'){
    	row10[12-volume_height] = 'X';
    	resetCounter = 0;
    }
    else{
    	row10[12-volume_height] = 'O';
    	distance += 1;
    }
    //generate the row1
    if (blank_count == 0){ //when blank_count=0, we generate spikes, otherwise we leave it blank
    	blank_count +=1;
    	uint8_t first_spike =  rand()%7;//generate a random number between 0-6
    	//generate different spikes
    	if (first_spike == 0){
    		sprintf(row1, "\r*|    ______|*\n");
    	}
    	else if(first_spike == 1){
    		sprintf(row1, "\r*|_    _____|*\n");
    	}
    	else if(first_spike == 2){
    		sprintf(row1, "\r*|__    ____|*\n");
    	}
    	else if(first_spike == 3){
    		sprintf(row1, "\r*|___    ___|*\n");
    	}
    	else if(first_spike == 4){
    		sprintf(row1, "\r*|____    __|*\n");
    	}
    	else if(first_spike == 5){
    		sprintf(row1, "\r*|_____    _|*\n");
    	}
    	else if(first_spike == 6){
    		sprintf(row1, "\r*|______    |*\n");
    	}
    }
    else if(blank_count == 1){//leave blank(no spikes)
    	blank_count +=1;
    	sprintf(row1, "\r*|          |*\n");
    }
    else{//leave blank(no spikes)
    	blank_count = 0;
    	sprintf(row1, "\r*|          |*\n");
    }
    sprintf(str, "%s%s%s%s%s%s%s%s%s%s%s distance=%d\n", row1, row2, row3, row4, row5, row6, row7, row8, row9, row10, row11, distance);
    HAL_UART_Transmit(&huart1, (uint8_t*) str, (uint16_t) strlen(str), 10000);
  }
  /* USER CODE END StartTransmitTask */
}

/* USER CODE BEGIN Header_StartReadTask */
/**
* @brief Function implementing the readTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadTask */
void StartReadTask(void const * argument)
{
  /* USER CODE BEGIN StartReadTask */

  /* Infinite loop */
  for(;;)
  {
    osDelay(100);
    while(resetCounter ==0){
    //do nothing until the button is pressed
    }
    if(start_recording==0){
    HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, pre_Buffer, buffer_size);
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    start_recording=1;
    sum = 0;
    }
	  if (first_half_done == 1){
		  for(i = 0; i<buffer_size/2; i++){
			  aft_Buffer[i] = pre_Buffer[i]>>20;
			  if (aft_Buffer[i]>buffmax){
				  buffmax = aft_Buffer[i];
			  }
			  if (aft_Buffer[i]<buffmin){
				  buffmin = aft_Buffer[i];
			  }
			  if(aft_Buffer[i]<0){
				  sum -= aft_Buffer[i];
			  }
			  else{
				  sum += aft_Buffer[i];
			  }
		  }
		  first_half_done = 0;
	  }
	  if (second_half_done == 1){
		  for(i = buffer_size/2; i<buffer_size; i++){
			  aft_Buffer[i] = pre_Buffer[i]>>20;
			  if (aft_Buffer[i]>buffmax){
				  buffmax = aft_Buffer[i];
			  }
			  if (aft_Buffer[i]<buffmin){
				  buffmin = aft_Buffer[i];
			  }
			  if(aft_Buffer[i]<0){
				  sum -= aft_Buffer[i];
			  }
			  else{
				  sum += aft_Buffer[i];
			  }
		  }
		  mean = sum/buffer_size;
		  second_half_done = 0;
		  HAL_DFSDM_FilterRegularStop_DMA(&hdfsdm1_filter0);
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		  start_recording=0;
	  }
	  if(mean<10){
			  volume_height = 0;
		  }
		  else if(mean<25){
			  volume_height = 1;
		  }
		  else if(mean<50){
			  volume_height = 2;
		  }
		  else if(mean<75){
			  volume_height = 3;
		  }
		  else if(mean<100){
			  volume_height = 4;
		  }
		  else if(mean<125){
			  volume_height = 5;
		  }
		  else if(mean<150){
			  volume_height = 6;
		  }
		  else if(mean<175){
			  volume_height = 7;
		  }
		  else if(mean<200){
			  volume_height = 8;
		  }
		  else if(mean<225){
			  volume_height = 9;
		  }
		  else{
			  volume_height = 9;
		  }

  }
  /* USER CODE END StartReadTask */
}

/* USER CODE BEGIN Header_StartMusicTask */
/**
* @brief Function implementing the musicTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMusicTask */
void StartMusicTask(void const * argument)
{
  /* USER CODE BEGIN StartMusicTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(500);
    while(resetCounter ==0){
    	HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    	music_index=0;
    	music_started = 0;
    }
    if(music_started==0){
    	music_started=1;
    	music_index = music_index+1;
    	  if (BSP_QSPI_Read(in_Buffer, (uint32_t) 0x007F0000, 20000) != QSPI_OK){
    		  Error_Handler();
    	  }
    	  for (uint32_t i = 0; i < 20000; i++) {
    		  out_Buffer[i] = (uint32_t)in_Buffer[i]*10;
    	  	}
    	  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, out_Buffer, 20000, DAC_ALIGN_12B_R);
    	  HAL_TIM_Base_Start_IT(&htim2);
    }
    while(done == 1 && music_started==1){
    		  done = 0;
    		  num = music[music_index]-1;
    		if (num == 0 ){
    			num = num+1;
    			  if (BSP_QSPI_Read(in_Buffer, (uint32_t) 0x007F0000, 20000) != QSPI_OK){
    				  Error_Handler();
    			  }
    			  for (uint32_t i = 0; i < 20000; i++) {
    				  out_Buffer[i] = (uint32_t)in_Buffer[i]*10;
    			  	}
//    			  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    			  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, out_Buffer, 20000, DAC_ALIGN_12B_R);
//    			  int a = 0;
    		}
    		else if(num == 1){
    			num = num+1;
    			  if (BSP_QSPI_Read(in_Buffer, (uint32_t) 0x007E0000, 20000) != QSPI_OK){
    				  Error_Handler();
    			  }
    			  for (uint32_t i = 0; i < 20000; i++) {
    				  out_Buffer[i] = (uint32_t)in_Buffer[i]*10;
    			  	}
//    			  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    			  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, out_Buffer, 20000, DAC_ALIGN_12B_R);
    		}
    		else if(num == 2){
    			num = num+1;
    			  if (BSP_QSPI_Read(in_Buffer, (uint32_t) 0x007D0000, 20000) != QSPI_OK){
    				  Error_Handler();
    			  }
    			  for (uint32_t i = 0; i < 20000; i++) {
    				  out_Buffer[i] = (uint32_t)in_Buffer[i]*10;
    			  	}
//    			  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    			  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, out_Buffer, 20000, DAC_ALIGN_12B_R);
    		}
    		else if(num == 3){
    			num = num+1;
    			  if (BSP_QSPI_Read(in_Buffer, (uint32_t) 0x007C0000, 20000) != QSPI_OK){
    				  Error_Handler();
    			  }
    			  for (uint32_t i = 0; i < 20000; i++) {
    				  out_Buffer[i] = (uint32_t)in_Buffer[i]*10;
    			  	}
//    			  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    			  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, out_Buffer, 20000, DAC_ALIGN_12B_R);
    		}
    		else if(num == 4){
    			num = num+1;
    			  if (BSP_QSPI_Read(in_Buffer, (uint32_t) 0x007B0000, 20000) != QSPI_OK){
    				  Error_Handler();
    			  }
    			  for (uint32_t i = 0; i < 20000; i++) {
    				  out_Buffer[i] = (uint32_t)in_Buffer[i]*10;
    			  	}
//    			  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    			  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, out_Buffer, 20000, DAC_ALIGN_12B_R);
    		}
    		else if(num == 5){
    			num = num+1;
    			  if (BSP_QSPI_Read(in_Buffer, (uint32_t) 0x007A0000, 20000) != QSPI_OK){
    				  Error_Handler();
    			  }
    			  for (uint32_t i = 0; i < 20000; i++) {
    				  out_Buffer[i] = (uint32_t)in_Buffer[i]*10;
    			  	}
//    			  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    			  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, out_Buffer, 20000, DAC_ALIGN_12B_R);
    		}
    		else{
    			num = 0;
    			  if (BSP_QSPI_Read(in_Buffer, (uint32_t) 0x00790000, 20000) != QSPI_OK){
    				  Error_Handler();
    			  }
    			  for (uint32_t i = 0; i < 20000; i++) {
    				  out_Buffer[i] = (uint32_t)in_Buffer[i]*10;
    			  	}
//    			  HAL_DAC_Stop_DMA(&hdac1, DAC_CHANNEL_1);
    			  HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, out_Buffer, 20000, DAC_ALIGN_12B_R);
    		}
    		music_index = music_index+1;
    		if(music_index==41){
    			music_index=0;
    		}
      }
  }
  /* USER CODE END StartMusicTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
