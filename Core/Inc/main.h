/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f1xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ESC1_Pin GPIO_PIN_0
#define ESC1_GPIO_Port GPIOA
#define ESC2_Pin GPIO_PIN_1
#define ESC2_GPIO_Port GPIOA
#define ESC3_Pin GPIO_PIN_2
#define ESC3_GPIO_Port GPIOA
#define ESC4_Pin GPIO_PIN_3
#define ESC4_GPIO_Port GPIOA
#define Extra_Setup_Button_Pin GPIO_PIN_4
#define Extra_Setup_Button_GPIO_Port GPIOA
#define Motor_C1_Pin GPIO_PIN_6
#define Motor_C1_GPIO_Port GPIOA
#define Motor_C2_Pin GPIO_PIN_7
#define Motor_C2_GPIO_Port GPIOA
#define Motor_D1_Pin GPIO_PIN_0
#define Motor_D1_GPIO_Port GPIOB
#define Motor_D2_Pin GPIO_PIN_1
#define Motor_D2_GPIO_Port GPIOB
#define Input_UART_Pin GPIO_PIN_10
#define Input_UART_GPIO_Port GPIOB
#define Input_UARTB11_Pin GPIO_PIN_11
#define Input_UARTB11_GPIO_Port GPIOB
#define Motor_B1_Pin GPIO_PIN_8
#define Motor_B1_GPIO_Port GPIOA
#define Motor_B2_Pin GPIO_PIN_9
#define Motor_B2_GPIO_Port GPIOA
#define Control_UART_Pin GPIO_PIN_11
#define Control_UART_GPIO_Port GPIOA
#define Control_UARTA12_Pin GPIO_PIN_12
#define Control_UARTA12_GPIO_Port GPIOA
#define I2C_CLOCK_Pin GPIO_PIN_6
#define I2C_CLOCK_GPIO_Port GPIOB
#define I2C_DATA_Pin GPIO_PIN_7
#define I2C_DATA_GPIO_Port GPIOB
#define Motor_A1_Pin GPIO_PIN_8
#define Motor_A1_GPIO_Port GPIOB
#define Motor_A2_Pin GPIO_PIN_9
#define Motor_A2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
