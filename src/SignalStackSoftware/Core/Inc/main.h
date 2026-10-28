/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h5xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmag5273.h"
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
#define DRIVER_EN_Pin GPIO_PIN_13
#define DRIVER_EN_GPIO_Port GPIOC
#define DRIVER_M0_Pin GPIO_PIN_14
#define DRIVER_M0_GPIO_Port GPIOC
#define DRIVER_M1_Pin GPIO_PIN_15
#define DRIVER_M1_GPIO_Port GPIOC
#define DRIVER_M2_Pin GPIO_PIN_0
#define DRIVER_M2_GPIO_Port GPIOH
#define DRIVER_STEP_CLOCK_Pin GPIO_PIN_1
#define DRIVER_STEP_CLOCK_GPIO_Port GPIOH
#define HEARTBEAT_Pin GPIO_PIN_0
#define HEARTBEAT_GPIO_Port GPIOC
#define DRIVER_nSTBY_Pin GPIO_PIN_13
#define DRIVER_nSTBY_GPIO_Port GPIOB
#define DRIVER_DIRECTION_Pin GPIO_PIN_10
#define DRIVER_DIRECTION_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
