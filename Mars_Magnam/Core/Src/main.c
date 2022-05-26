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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "wwdg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "ps2.h"
#include "delay.h"
#include "motor.h"

#include "steer.h"
#include "vision.h"
#include "control.h"
#include "communication.h"

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
int control_mode=0;
int ps2_mode=0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void show_motor(MOTOR_TypeDef *motor){
	printf("\r\n***\r\n");
	if(motor==&Forward_L)printf("FL\r\n");
	if(motor==&Forward_R)printf("FR\r\n");
	if(motor==&Back_L)printf("BL\r\n");
	if(motor==&Back_R)printf("BR\r\n");
/**/	
//	DataScope(motor->actual_angle,motor->target_angle);
//	DataScope(motor->actual_speed,motor->target_speed);
	
	
	printf("actual speed: %f\r\n",motor->actual_speed);
	printf("target speed: %f\r\n",motor->target_speed);
	
//	printf("err: %f\r\n",(float)(motor->capture_count-motor->last_count)/(beipin*xianshu*jiansubi*0.01));
/*	
	printf("kp: %f\r\n",motor->k_speed.p);
	printf("ki: %f\r\n",motor->k_speed.i);
	printf("kd: %f\r\n",motor->k_speed.d);
	printf("integral: %f\r\n",motor->k_speed.integral);
	printf("err_last: %f\r\n",motor->k_speed.err_last);
	printf("err_last1: %f\r\n",motor->k_speed.err_last1);
	printf("err_last2: %f\r\n",motor->k_speed.err_last2);
	printf("Motor fi: %f\r\n",motor->k_speed.fi);
*/	
	printf("capture_count: %d\r\n",motor->capture_count);
	printf("last_count: %d\r\n",motor->last_count);
	printf("pwm: %d\r\n",motor->pwm);
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
	delay_init(72);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_USART1_UART_Init();
  MX_WWDG_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	steer_init();

	HAL_UART_Receive_IT(&huart1,(uint8_t *)&receiver1,1);
	HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
//	HAL_UART_Receive_IT(&huart3,(uint8_t *)&receiver3,1);
	
//	HAL_UART_Receive_IT(&huart2,rx2_buffer,10);
//	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
//	HAL_UART_Receive_DMA(&huart2,rx2_buffer,BUFFERSIZE);//打开DMA接收
	
	control_mode = 0;
	ps2_mode = 0;
	MOTOR_Init(); 
	MOTOR_Tim_Init(&Forward_L);
	MOTOR_Tim_Init(&Forward_R);
	MOTOR_Tim_Init(&Back_L); 
	MOTOR_Tim_Init(&Back_R); 
	
	printf("READY\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//  set_steer_pwm(&Steer_f);
	 // set_steer_pwm(&Steer_b);
//	  set_steer_pwm(&Turret);
	  steer_turn_slow(&Turret); 	
/*
//	  printf("turret angle %f\r\n",Turret.angle);
//	  delay_ms(1000);
//	  show_motor(&Forward_L);
//	  show_motor(&Forward_R);
//	  show_motor(&Back_L);
//	  show_motor(&Back_R);
*/
	  
	  if(ps2_mode){	//ps2手柄控制模式
		 delay_ms(10);
		  
		 ps2_control(PS2_DataKey());
			
		 if(control_mode)ps2_angle();
		 else ps2_speed();		 
	  }
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
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

/*三个串口通信

串口一: 核心控制与调试

串口二：与树莓派通信

串口三：与炮台主控芯片（stm32f411ceu6）通信

*/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart ->Instance == USART1){		//串口1
		bluetooth_control(receiver1);	//非ps2控制下用手机蓝牙控制小车
		
		HAL_UART_Receive_IT(&huart1,(uint8_t *)&receiver1,1);
	}
	
	if(huart ->Instance == USART2){	//串口二，与树莓派通信
		
/*		uint32_t tmp_flag = 0;
		uint32_t temp;
		if(__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)!=RESET){
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);//清除标志位
			HAL_UART_DMAStop(&huart2); //  停止DMA传输
			temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);// 获取DMA中未传输的数据个数   
			rx2_len =  BUFFERSIZE - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
			
			HAL_UART_Transmit(&huart1,rx2_buffer,rx2_len,0xff);
//			memset(rx2_buffer,0,BUFFERSIZE);
		}
		HAL_UART_Receive_DMA(&huart2,rx2_buffer,BUFFERSIZE);//重新打开DMA接收
*/		

		vision_control(receiver2);
		
		HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
	}
	
	if(huart ->Instance == USART3){
//		printf("receiver3! %c",receiver3);
//		HAL_UART_Transmit(&huart1,(u8 *)&receiver3,1,1);
		HAL_UART_Receive_IT(&huart3,(uint8_t *)&receiver3,1);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){	//计数溢出更新中断
	if(htim==(&htim8)){ 
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			Forward_L.encoder_overflow--;			//反转过程中溢出
		else 
			Forward_L.encoder_overflow++;			//正转过程中溢出
	}
	if(htim==(&htim4)){		
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			Forward_R.encoder_overflow--;		//反转过程中溢出
		else 
			Forward_R.encoder_overflow++;		//正转过程中溢出
	}
	if(htim==(&htim5)){	
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			Back_L.encoder_overflow--;			//反转过程中溢出
		else 
			Back_L.encoder_overflow++;			//正转过程中溢出
	}
	if(htim==(&htim1)){	
		if(__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
			Back_R.encoder_overflow--;			//反转过程中溢出
		else 
			Back_R.encoder_overflow++;			//正转过程中溢出
	}
	
	
	if(htim==(&htim6)){		
		if(control_mode==1){ 							//control_mode=1 角度;control_mode=0 速度
			feedback_angle(&Forward_L);
			feedback_angle(&Forward_R);
			feedback_angle(&Back_L);
			feedback_angle(&Back_R);
		}else{
			feedback_speed(&Forward_L);
			feedback_speed(&Forward_R);
			feedback_speed(&Back_L);
			feedback_speed(&Back_R);
		}
	}
}

void HAL_WWDG_EarlyWakeupCallback(WWDG_HandleTypeDef *hwwdg){
	HAL_WWDG_Refresh(hwwdg);      //喂窗口看门狗
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
