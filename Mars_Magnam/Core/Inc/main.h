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

extern int control_mode;
extern int ps2_mode;


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
#define Turret_Pin GPIO_PIN_5
#define Turret_GPIO_Port GPIOE
#define Back_LA_Pin GPIO_PIN_0
#define Back_LA_GPIO_Port GPIOA
#define Back_LB_Pin GPIO_PIN_1
#define Back_LB_GPIO_Port GPIOA
#define Back_L_IN1_Pin GPIO_PIN_5
#define Back_L_IN1_GPIO_Port GPIOA
#define Back_L_IN2_Pin GPIO_PIN_6
#define Back_L_IN2_GPIO_Port GPIOA
#define Back_R_IN1_Pin GPIO_PIN_0
#define Back_R_IN1_GPIO_Port GPIOB
#define Back_R_IN2_Pin GPIO_PIN_1
#define Back_R_IN2_GPIO_Port GPIOB
#define Back_RA_Pin GPIO_PIN_9
#define Back_RA_GPIO_Port GPIOE
#define Back_RB_Pin GPIO_PIN_11
#define Back_RB_GPIO_Port GPIOE
#define Forward_R_IN2_Pin GPIO_PIN_10
#define Forward_R_IN2_GPIO_Port GPIOB
#define Forward_R_IN1_Pin GPIO_PIN_11
#define Forward_R_IN1_GPIO_Port GPIOB
#define Steer_B_Pin GPIO_PIN_14
#define Steer_B_GPIO_Port GPIOB
#define Steer_F_Pin GPIO_PIN_15
#define Steer_F_GPIO_Port GPIOB
#define Forward_RA_Pin GPIO_PIN_12
#define Forward_RA_GPIO_Port GPIOD
#define Forward_RB_Pin GPIO_PIN_13
#define Forward_RB_GPIO_Port GPIOD
#define Forward_LA_Pin GPIO_PIN_6
#define Forward_LA_GPIO_Port GPIOC
#define Forward_LB_Pin GPIO_PIN_7
#define Forward_LB_GPIO_Port GPIOC
#define CLK_Pin GPIO_PIN_0
#define CLK_GPIO_Port GPIOD
#define CS_Pin GPIO_PIN_2
#define CS_GPIO_Port GPIOD
#define DO_Pin GPIO_PIN_4
#define DO_GPIO_Port GPIOD
#define DI_Pin GPIO_PIN_6
#define DI_GPIO_Port GPIOD
#define Forward_L_IN1_Pin GPIO_PIN_3
#define Forward_L_IN1_GPIO_Port GPIOB
#define Forward_L_IN2_Pin GPIO_PIN_5
#define Forward_L_IN2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
