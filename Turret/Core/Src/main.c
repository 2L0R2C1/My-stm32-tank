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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "delay.h"
#include "pid.h"
#include "communication.h"
#include "control.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void show_motor(MOTOR_TypeDef *motor){
	printf("\r\n***\r\n");
/*	if(motor==&Forward_L)printf("FL\r\n");
	if(motor==&Forward_R)printf("FR\r\n");
	if(motor==&Back_L)printf("BL\r\n");
	if(motor==&Back_R)printf("BR\r\n");
*/	
//	DataScope(motor->actual_angle,motor->target_angle);
//	DataScope(motor->actual_speed,motor->target_speed);
	
	
	printf("actual speed: %f\r\n",motor->actual_speed);
	printf("target speed: %f\r\n",motor->target_speed);
	
//	printf("err: %f\r\n",(float)(motor->capture_count-motor->last_count)/(beipin*xianshu*jiansubi*0.01));
	
/*	printf("kp: %f\r\n",motor->k_speed.p);
	printf("ki: %f\r\n",motor->k_speed.i);
	printf("kd: %f\r\n",motor->k_speed.d);
	printf("integral: %f\r\n",motor->k_speed.integral);
	printf("err_last: %f\r\n",motor->k_speed.err_last);
	printf("err_last1: %f\r\n",motor->k_speed.err_last1);
	printf("err_last2: %f\r\n",motor->k_speed.err_last2);
	printf("Motor fi: %f\r\n",motor->k_speed.fi);
	
	printf("capture_count: %d\r\n",motor->capture_count);
	printf("last_count: %d\r\n",motor->last_count);
*/	printf("pwm: %d\r\n",motor->pwm);
/*	printf("motor encoder_overflow: %d\r\n",motor->encoder_overflow);
	
	printf("kp: %f\r\n",motor->k_angle.p);
//	printf("ki: %f\r\n",motor->k_angle.i);
//	printf("kd: %f\r\n",motor->k_angle.d);
//	printf("integral: %f\r\n",motor->k_angle.integral);
//	printf("err_last: %f\r\n",motor->k_angle.err_last);
//	printf("target angle: %f\r\n",motor->target_angle);
//	printf("actual angle: %f\r\n",motor->actual_angle);
//	printf("fp: %f\r\n",motor->k_angle.fp);
*/
}
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
  delay_init(72);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

	MOTOR_Init(); 
	
//	set_pid(&Bopan.k_double,1.2,0.002,25);
	
	MOTOR_Tim_Init(&Bopan);	
//	MOTOR_Tim_Init(&Friction_L);	
//	MOTOR_Tim_Init(&Friction_R);	
	
//	Bopan.target_speed = 0.5; Bopan.mode = 0;
	
	Bopan.target_angle = 40;	Bopan.mode = 1;	
	
	HAL_UART_Receive_IT(&huart6,(uint8_t *)&receiver6,1);
	HAL_UART_Receive_IT(&huart1,(uint8_t *)&receiver1,1);
	
  /* USER CODE END 2 */

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){  
//		HAL_UART_Transmit(&huart1,(uint8_t *)&receiver1,1,1);
//		HAL_UART_Transmit(&huart6,(uint8_t *)&receiver1,1,1);		
		HAL_UART_Receive_IT(&huart1,(uint8_t *)&receiver1,1);
	}
	if(huart->Instance == USART6){
//		HAL_UART_Transmit(&huart6,(uint8_t *)&receiver6,1,1);	
//		HAL_UART_Transmit(&huart1,(uint8_t *)&receiver6,1,1);
		f411_control(receiver6);
		HAL_UART_Receive_IT(&huart6,(uint8_t *)&receiver6,1);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){	//计数溢出更新中断
	if(htim==(&htim1)){
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			Bopan.encoder_overflow--;			//反转过程中溢出
		else 
			Bopan.encoder_overflow++;			//正转过程中溢出
	}
	
	if(htim==(&htim10)){	
		if(Bopan.mode){                                             //control_mode=1 角度;control_mode=0 速度
//			feedback_angle(&Bopan);			//反馈电机
			feedback_angle_double(&Bopan);
		}else{
			feedback_speed(&Bopan);																					 	//反馈电机
		}
	}
}

/* USER CODE END 4 */

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
