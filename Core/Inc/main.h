/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define led_r_Pin GPIO_PIN_13
#define led_r_GPIO_Port GPIOC
#define led_g_Pin GPIO_PIN_14
#define led_g_GPIO_Port GPIOC
#define led_b_Pin GPIO_PIN_15
#define led_b_GPIO_Port GPIOC
#define pitch_lim_Pin GPIO_PIN_2
#define pitch_lim_GPIO_Port GPIOC
#define pitch_dir_Pin GPIO_PIN_3
#define pitch_dir_GPIO_Port GPIOC
#define CS_accel_Pin GPIO_PIN_4
#define CS_accel_GPIO_Port GPIOC
#define CS_gyro_Pin GPIO_PIN_5
#define CS_gyro_GPIO_Port GPIOC
#define dir2_485_Pin GPIO_PIN_8
#define dir2_485_GPIO_Port GPIOC
#define dir1_485_Pin GPIO_PIN_8
#define dir1_485_GPIO_Port GPIOA
#define key_b_Pin GPIO_PIN_15
#define key_b_GPIO_Port GPIOA
#define key_d_Pin GPIO_PIN_11
#define key_d_GPIO_Port GPIOC
#define key_d_EXTI_IRQn EXTI15_10_IRQn
#define key_a_Pin GPIO_PIN_3
#define key_a_GPIO_Port GPIOB
#define key_c_Pin GPIO_PIN_4
#define key_c_GPIO_Port GPIOB
#define yaw_lim_Pin GPIO_PIN_5
#define yaw_lim_GPIO_Port GPIOB
#define yaw_dir_Pin GPIO_PIN_7
#define yaw_dir_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
