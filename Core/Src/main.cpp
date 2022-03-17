/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.cpp
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include <stdio.h>
#include "bno055_stm32.h"
#include "PID.h"
#include "stm32l4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_HIGH 20315.85
#define PWM_LOW 6500
#define PWM_MID 12000
#define KP_y 3.1
#define KD_y 0.0005
#define KI_y 0.008
#define KP_p 3.1
#define KD_p 0.0005
#define KI_p 0.008

#define debounceDelay 10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for controlSysTask */
osThreadId_t controlSysTaskHandle;
const osThreadAttr_t controlSysTask_attributes = {
  .name = "controlSysTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ledBattTask */
osThreadId_t ledBattTaskHandle;
const osThreadAttr_t ledBattTask_attributes = {
  .name = "ledBattTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for imuTask */
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};
/* Definitions for targetSetTask */
osThreadId_t targetSetTaskHandle;
const osThreadAttr_t targetSetTask_attributes = {
  .name = "targetSetTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for spatialSmphr */
osSemaphoreId_t spatialSmphrHandle;
const osSemaphoreAttr_t spatialSmphr_attributes = {
  .name = "spatialSmphr"
};
/* Definitions for targetSmphr */
osSemaphoreId_t targetSmphrHandle;
const osSemaphoreAttr_t targetSmphr_attributes = {
  .name = "targetSmphr"
};
/* USER CODE BEGIN PV */
osEventFlagsId_t setPointButtonEvents;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
void StartCtrlSysTask(void *argument);
void StartLedBattTask(void *argument);
void StartIMUTask(void *argument);
void targetSetTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void yawPWM(float CCR_val);
float getYaw();

void pitchPWM(float CCR_val);
float getPitch();

void rollPWM(float CCR_val);
float getRoll();

bno055_vector_t spatialOrientation;
int CCR1,CCR2,CCR4;
PIDController<float> yawCtrl(KP_y,KD_y,KI_y, getYaw, yawPWM), pitchCtrl(KP_p,KD_p,KI_p, getPitch, pitchPWM);//,rollCtrl(2.5,0.0001,0.001, getRoll, rollPWM);
float setpointYaw, setpointPitch, setpointRoll;
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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of spatialSmphr */
  spatialSmphrHandle = osSemaphoreNew(1, 1, &spatialSmphr_attributes);

  /* creation of targetSemphr */
  targetSmphrHandle = osSemaphoreNew(1, 1, &targetSmphr_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  setpointYaw = setpointPitch = setpointRoll = 0;
  /* Create the thread(s) */
  /* creation of controlSysTask */
  controlSysTaskHandle = osThreadNew(StartCtrlSysTask, NULL, &controlSysTask_attributes);

  /* creation of ledBattTask */
  ledBattTaskHandle = osThreadNew(StartLedBattTask, NULL, &ledBattTask_attributes);

  /* creation of imuTask */
  imuTaskHandle = osThreadNew(StartIMUTask, NULL, &imuTask_attributes);

  /* creation of downButtonTask */
  targetSetTaskHandle = osThreadNew(targetSetTask, NULL, &targetSetTask_attributes);


  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  setPointButtonEvents = osEventFlagsNew( NULL );
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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
  hi2c1.Init.Timing = 0x00707CBB;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 3;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LED_1_Pin|LED_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 button_down_Pin button_up_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|button_down_Pin|button_up_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : button_right_Pin button_left_Pin */
  GPIO_InitStruct.Pin = button_right_Pin|button_left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : button_capture_Pin */
  GPIO_InitStruct.Pin = button_capture_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(button_capture_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : temperature_Pin batt_voltage_Pin */
  GPIO_InitStruct.Pin = temperature_Pin|batt_voltage_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LED_1_Pin LED_2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LED_1_Pin|LED_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_3_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_3_GPIO_Port, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);


}

/* USER CODE BEGIN 4 */

void yawPWM(float CCR_val){

	CCR1 = CCR1+(int)CCR_val;

	if (CCR1 < PWM_LOW){
		TIM2->CCR1 = PWM_LOW;
	}
	else if (CCR1 >PWM_HIGH){
		TIM2->CCR1 = PWM_HIGH;
	}
	else{
		TIM2->CCR1 = CCR1;
	}

	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

void pitchPWM(float CCR_val){

	CCR2 = CCR2+(int)CCR_val;

	if (CCR2 <PWM_LOW){
		TIM2->CCR2 = PWM_LOW;
	}
	else if (CCR2 >PWM_HIGH){
		TIM2->CCR2 = PWM_HIGH;
	}
	else{
		TIM2->CCR2 = CCR2;
	}

	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void rollPWM(float CCR_val){

	CCR4 = CCR4+(int)CCR_val;

		if (CCR4 <PWM_LOW){
			TIM2->CCR4 = PWM_LOW;
		}
		else if (CCR4 >PWM_HIGH){
			TIM2->CCR4 = PWM_HIGH;
		}
		else{
			TIM2->CCR4 = CCR4;
		}

		//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}

float getYaw(){
	float yaw;
	if (spatialOrientation.x > 180){
		yaw = spatialOrientation.x-360;
	}
	else{
		yaw = spatialOrientation.x;
	}
	return yaw;
}

float getPitch(){
	return spatialOrientation.y;
}

float getRoll(){
	return spatialOrientation.z;
}

void updatePitchSetPoint(float newSet){
	pitchCtrl.setTarget(newSet);
}



void setpointButtons(){

	osEventFlagsSet(setPointButtonEvents, 0x50);

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCtrlSysTask */
/**
  * @brief  Function implementing the controlSysTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartCtrlSysTask */
void StartCtrlSysTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	CCR1 = CCR2 = CCR4 = PWM_MID;
	TIM2->CCR1 = CCR1; TIM2->CCR2 = CCR2; TIM2->CCR4 = CCR4;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);

	yawCtrl.setTarget(setpointYaw); yawCtrl.registerTimeFunction(HAL_GetTick);
	pitchCtrl.setTarget(setpointPitch); pitchCtrl.registerTimeFunction(HAL_GetTick);
	//rollCtrl.setTarget(setpointRoll); pitchCtrl.RegisterTimeFunction(HAL_GetTick);
  /* Infinite loop */
  for(;;)
  {
	osSemaphoreAcquire( spatialSmphrHandle, osWaitForever );
	osSemaphoreAcquire( targetSmphrHandle, osWaitForever );

		//yawCtrl.tick();
		//pitchCtrl.tick();
		//rollCtrl.tick();

//		if (polar){
//			CCR2 = CCR2 + 30;
//		}
//		else{
//			CCR2 = CCR2 - 30;
//		}
//
//		if (CCR2>PWM_HIGH){
//			polar = 0;
//		}
//		else if(CCR2 < PWM_LOW){
//			polar = 1;
//		}
//
//		TIM2->CCR2 = CCR2;
	osSemaphoreRelease( spatialSmphrHandle );
	osSemaphoreRelease( targetSmphrHandle );
	osDelay(3);
  }

  osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLedBattTask */
/**
* @brief Function implementing the ledBattTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLedBattTask */
void StartLedBattTask(void *argument)
{
  /* USER CODE BEGIN StartLedBattTask */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
    osDelay(800);
  }
  osThreadTerminate(NULL);
  /* USER CODE END StartLedBattTask */
}

/* USER CODE BEGIN Header_StartIMUTask */
/**
* @brief Function implementing the imuTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUTask */
void StartIMUTask(void *argument)
{
  /* USER CODE BEGIN StartIMUTask */
  /* Infinite loop */
	bno055_assignI2C(&hi2c1);
	bno055_setup();
	bno055_setOperationModeNDOF();
  for(;;)
  {
	osSemaphoreAcquire( spatialSmphrHandle, osWaitForever );
	osSemaphoreRelease( targetSmphrHandle );
	spatialOrientation = bno055_getVectorEuler();
	//printf("y%.2fyp%.2fpr%.2fr\n", spatialOrientation.x, spatialOrientation.y, spatialOrientation.z);
	osSemaphoreRelease( spatialSmphrHandle );
	osDelay(3);
  }
  osThreadTerminate(NULL);
  /* USER CODE END StartIMUTask */
}

/* USER CODE BEGIN Header_downButton */
/**
* @brief Function implementing the downButtonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_downButton */
void targetSetTask(void *argument)
{
  /* USER CODE BEGIN downButton */
  /* Infinite loop */
  for(;;)
  {
	osEventFlagsWait(setPointButtonEvents,0x50, osFlagsWaitAll, osWaitForever);
	osSemaphoreAcquire( targetSmphrHandle, osWaitForever );
	//setpointPitch = setpointPitch+3;
	//pitchCtrl.setTarget(setpointPitch);

	if (!HAL_GPIO_ReadPin (GPIOA,GPIO_PIN_6)){
		CCR2 += 1000;
			TIM2->CCR2 = CCR2;

	}
		if (!HAL_GPIO_ReadPin (GPIOA,GPIO_PIN_7)){
			CCR2 -= 1000;
			TIM2->CCR2 = CCR2;
	}
	osSemaphoreRelease( targetSmphrHandle );
    osEventFlagsClear(setPointButtonEvents, 0x50);
	osDelay(debounceDelay);
  }
  /* USER CODE END downButton */
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

