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
#include "string.h"
#include "stdio.h"
#include "stm32l4s5i_iot01.h"

#include "stm32l4s5i_iot01_magneto.h"
#include "stm32l4s5i_iot01_accelero.h"
#include "stm32l4s5i_iot01_psensor.h"
#include "stm32l4s5i_iot01_hsensor.h"

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
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

osThreadId press_buttonHandle;
osThreadId read_sensorHandle;
osThreadId transmit_dataHandle;
/* USER CODE BEGIN PV */
int sensorType = 0;

uint8_t humidity;
uint8_t pressure;

int16_t gyro[3];
int16_t magneto[3];
int16_t accelero[3];

char buffer[100];
int pressed=0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
void press_button_task(void const * argument);
void read_sensor_task(void const * argument);
void transmit_data_task(void const * argument);

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
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize I2C sensors */
  BSP_ACCELERO_Init();
  BSP_MAGNETO_Init();
  BSP_PSENSOR_Init();
  BSP_HSENSOR_Init();

  /* Set low power mode for accelerometer and magnetometer */
  BSP_ACCELERO_LowPower(1);
  BSP_MAGNETO_LowPower(1);


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
  /* definition and creation of press_button */
  osThreadDef(press_button, press_button_task, osPriorityNormal, 0, 128);
  press_buttonHandle = osThreadCreate(osThread(press_button), NULL);

  /* definition and creation of read_sensor */
  osThreadDef(read_sensor, read_sensor_task, osPriorityIdle, 0, 128);
  read_sensorHandle = osThreadCreate(osThread(read_sensor), NULL);

  /* definition and creation of transmit_data */
  osThreadDef(transmit_data, transmit_data_task, osPriorityIdle, 0, 128);
  transmit_dataHandle = osThreadCreate(osThread(transmit_data), NULL);

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
	  // part 1
//	  if (sensorType == 0){
//		  humidity = BSP_HSENSOR_ReadHumidity();
//		  sprintf(buffer, "Humidity: %d \n", (int) humidity);
//	  }else if (sensorType == 1){
//		  pressure = BSP_PSENSOR_ReadPressure();
//		  sprintf(buffer, "Pressure: %d \n", (int) pressure);
//	  }else if (sensorType == 2){
//		  BSP_MAGNETO_GetXYZ(magneto);
//		  sprintf(buffer, "Magneto X, Y, Z: %.2d, %.2d, %.2d \n", (int) magneto[0], (int) magneto[1], (int) magneto[2]);
//	  }else if (sensorType == 3){
//		  BSP_ACCELERO_AccGetXYZ(accelero);
//		  sprintf(buffer, "Accelero X, Y, Z: %.2d, %.2d, %.2d \n", (int) accelero[0], (int) accelero[1], (int) accelero[2]);
//	  }
//
//	  HAL_UART_Transmit(&huart1, (uint8_t*) buffer, (uint16_t) strlen(buffer), 100);
//
//	  // 10hz
//	  HAL_Delay(100);
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : push_button_Pin */
  GPIO_InitStruct.Pin = push_button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(push_button_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	if(GPIO_PIN == GPIO_PIN_13){
		pressed=1;

		if(sensorType>=3 ){
			sensorType = 0;
		}else{
			sensorType ++;
		}
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_press_button_task */
/**
  * @brief  Function implementing the press_button thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_press_button_task */
void press_button_task(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */

	for(;;){
		osDelay(30000);
		if(pressed==0){
			pressed=1;
			if(sensorType >=3 ){
				sensorType = 0;
			}else{
				sensorType ++;
			}
		}
	}

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_read_sensor_task */
/**
* @brief Function implementing the read_sensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_read_sensor_task */
void read_sensor_task(void const * argument)
{
  /* USER CODE BEGIN read_sensor_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(100);

    		  humidity = BSP_HSENSOR_ReadHumidity();

    		  pressure = BSP_PSENSOR_ReadPressure();

    		  BSP_MAGNETO_GetXYZ(magneto);

    		  BSP_ACCELERO_AccGetXYZ(accelero);



  }
  /* USER CODE END read_sensor_task */
}

/* USER CODE BEGIN Header_transmit_data_task */
/**
* @brief Function implementing the transmit_data thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_transmit_data_task */
void transmit_data_task(void const * argument)
{
  /* USER CODE BEGIN transmit_data_task */
  /* Infinite loop */
	  for(;;)
	  {
	    osDelay(100);


	    	  if (sensorType == 0 && pressed == 1){
	    		  sprintf(buffer, "Humidity: %d \n", (int) humidity);

				  pressed = 0;
	    	  }else if (sensorType == 1 && pressed == 1){
	    		  sprintf(buffer, "Pressure: %d \n", (int) pressure);

				  pressed = 0;
	    	  }else if (sensorType == 2 && pressed == 1){
	    		  sprintf(buffer, "Magneto X, Y, Z: %.2d, %.2d, %.2d \n", (int) magneto[0], (int) magneto[1], (int) magneto[2]);

				  pressed = 0;
	    	  }else if (sensorType == 3 && pressed == 1){
	    		  sprintf(buffer, "Accelero X, Y, Z: %.2d, %.2d, %.2d \n", (int) accelero[0], (int) accelero[1], (int) accelero[2]);

				  pressed = 0;
	    	  }
	    	  HAL_UART_Transmit(&huart1, (uint8_t*) buffer, (uint16_t) strlen(buffer), 100);
		}

  /* USER CODE END transmit_data_task */
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
