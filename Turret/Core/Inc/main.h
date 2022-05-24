/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
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
#define Friction1_Pin GPIO_PIN_0
#define Friction1_GPIO_Port GPIOA
#define Friction2_Pin GPIO_PIN_1
#define Friction2_GPIO_Port GPIOA
#define Bopan_IN1_Pin GPIO_PIN_2
#define Bopan_IN1_GPIO_Port GPIOA
#define Bopan_IN2_Pin GPIO_PIN_3
#define Bopan_IN2_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_4
#define IN1_GPIO_Port GPIOA
#define IN2_Pin GPIO_PIN_5
#define IN2_GPIO_Port GPIOA
#define IN3_Pin GPIO_PIN_6
#define IN3_GPIO_Port GPIOA
#define IN4_Pin GPIO_PIN_7
#define IN4_GPIO_Port GPIOA
#define FrictionL_Pin GPIO_PIN_0
#define FrictionL_GPIO_Port GPIOB
#define FrictionR_Pin GPIO_PIN_1
#define FrictionR_GPIO_Port GPIOB
#define Bopan_A_Pin GPIO_PIN_8
#define Bopan_A_GPIO_Port GPIOA
#define Bopan_B_Pin GPIO_PIN_9
#define Bopan_B_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define A2312
//#define L298N
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
