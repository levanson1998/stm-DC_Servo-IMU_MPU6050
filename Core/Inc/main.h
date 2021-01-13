/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f4xx_hal.h"
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
#define ENCODER_R1_Pin GPIO_PIN_5
#define ENCODER_R1_GPIO_Port GPIOA
#define EN_L_Pin GPIO_PIN_9
#define EN_L_GPIO_Port GPIOE
#define EN_R_Pin GPIO_PIN_11
#define EN_R_GPIO_Port GPIOE
#define LPWM_R_Pin GPIO_PIN_8
#define LPWM_R_GPIO_Port GPIOD
#define RPWM_R_Pin GPIO_PIN_9
#define RPWM_R_GPIO_Port GPIOD
#define LPWM_L_Pin GPIO_PIN_10
#define LPWM_L_GPIO_Port GPIOD
#define RPWM_L_Pin GPIO_PIN_11
#define RPWM_L_GPIO_Port GPIOD
#define LED_GRE_Pin GPIO_PIN_12
#define LED_GRE_GPIO_Port GPIOD
#define LED_ORG_Pin GPIO_PIN_13
#define LED_ORG_GPIO_Port GPIOD
#define LED_RED_Pin GPIO_PIN_14
#define LED_RED_GPIO_Port GPIOD
#define LED_BLU_Pin GPIO_PIN_15
#define LED_BLU_GPIO_Port GPIOD
#define ENCODER_R2_Pin GPIO_PIN_3
#define ENCODER_R2_GPIO_Port GPIOB
#define ENCODER_L2_Pin GPIO_PIN_6
#define ENCODER_L2_GPIO_Port GPIOB
#define ENCODER_L1_Pin GPIO_PIN_7
#define ENCODER_L1_GPIO_Port GPIOB
#define IUM_SCL_Pin GPIO_PIN_8
#define IUM_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
