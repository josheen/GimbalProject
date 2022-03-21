/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void setpointButtons();
void modeChangeButton();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define on_off_mode_Pin GPIO_PIN_4
#define on_off_mode_GPIO_Port GPIOA
#define on_off_mode_EXTI_IRQn EXTI4_IRQn
#define button_down_Pin GPIO_PIN_6
#define button_down_GPIO_Port GPIOA
#define button_down_EXTI_IRQn EXTI9_5_IRQn
#define button_up_Pin GPIO_PIN_7
#define button_up_GPIO_Port GPIOA
#define button_up_EXTI_IRQn EXTI9_5_IRQn
#define button_right_Pin GPIO_PIN_0
#define button_right_GPIO_Port GPIOB
#define button_right_EXTI_IRQn EXTI0_IRQn
#define button_left_Pin GPIO_PIN_1
#define button_left_GPIO_Port GPIOB
#define button_left_EXTI_IRQn EXTI1_IRQn
#define button_capture_Pin GPIO_PIN_8
#define button_capture_GPIO_Port GPIOA
#define button_capture_EXTI_IRQn EXTI9_5_IRQn
#define temperature_Pin GPIO_PIN_11
#define temperature_GPIO_Port GPIOA
#define batt_voltage_Pin GPIO_PIN_12
#define batt_voltage_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#define LED_1_Pin GPIO_PIN_4
#define LED_1_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_5
#define LED_2_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOH
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
