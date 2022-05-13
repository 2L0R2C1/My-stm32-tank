#include "motor.h"
#include "pid.h"
#include "tim.h"
#include "delay.h"
#include "usart.h"
#include "string.h"

/*********************************************
初始化所有电机的所有相关参数，

并且建立电机与encoder、pwm输出通道、IN1、IN2的一一对应关系

初始化并开启encoder、定时器溢出更新中断，开启pwm输出口
**********************************************/

MOTOR_TypeDef Bopan   = {0};
//MOTOR_TypeDef Friction_R   = {0};
//MOTOR_TypeDef Friction_L   = {0};

void MOTOR_Init(void){
	Bopan.encoder = htim1;
	Bopan.timepwm1 = htim9;
	Bopan.channelpwm1 = TIM_CHANNEL_1;
	Bopan.timepwm2 = htim9;
	Bopan.channelpwm2 = TIM_CHANNEL_2;
	Bopan.capturetim = htim10;
	
/*	Friction_R.encoder = htim2;
	Friction_R.timepwm1 = htim4;
	Friction_R.channelpwm1 = TIM_CHANNEL_1;
	Friction_R.timepwm2 = htim4;
	Friction_R.channelpwm2 = TIM_CHANNEL_2;
	Friction_R.capturetim = htim10;
	
	Friction_L.encoder = htim3;
	Friction_L.timepwm1 = htim4;
	Friction_L.channelpwm1 = TIM_CHANNEL_3;
	Friction_L.timepwm2 = htim4;
	Friction_L.channelpwm2 = TIM_CHANNEL_4;
	Friction_L.capturetim = htim10;
*/
}

void MOTOR_Tim_Init(MOTOR_TypeDef *motor){	
	set_pid(&motor->k_speed, 20, 5, 0);
	set_pid(&motor->k_angle, 50, 0.5, 0);
	set_pid(&motor->k_double, 1.2, 0.005, 25);
	
	__HAL_TIM_CLEAR_IT(&motor->encoder,TIM_IT_UPDATE);	//清零encoder中断标志位
	__HAL_TIM_SetCounter(&motor->encoder,0);			//归零encoder计数值
	HAL_TIM_Base_Start_IT(&motor->encoder);			//开启encoder计数中断

	HAL_TIM_Encoder_Start(&motor->encoder,TIM_CHANNEL_1);	//开启encoder输入捕获通道
	HAL_TIM_Encoder_Start(&motor->encoder,TIM_CHANNEL_2);	//开启encoder输入捕获通道
	
	HAL_TIM_PWM_Start(&motor->timepwm1,motor->channelpwm1);	//开启pwm输出
	HAL_TIM_PWM_Start(&motor->timepwm2,motor->channelpwm2);	//开启pwm输出
	
	__HAL_TIM_CLEAR_IT(&motor->capturetim,TIM_IT_UPDATE);	//清零tim6中断标志位(定时中断获取电机状态并反馈）
	__HAL_TIM_SetCounter(&motor->capturetim,0);				//归零tim6计数值
	HAL_TIM_Base_Start_IT(&motor->capturetim);				//开启tim6计数中断	
}

void MOTOR_Tim_Stop(MOTOR_TypeDef *motor){			//用这个函数将关闭定时器6，也就意味着实际上所有电机停摆
	motor->pwm = 0;
	set_motor_pwm(motor);
//	memset(motor,0,sizeof(Bopan));
	motor->actual_angle=0;
	motor->actual_speed=0;
	motor->target_angle=0;
	motor->target_speed=0;
	
	motor->encoder_overflow=0;
	motor->capture_count=0;
	motor->last_count=0;
	motor->pwm=0;
	
	PID_init(&motor->k_angle);
	PID_init(&motor->k_speed);
	PID_init(&motor->k_double);
	
	HAL_TIM_Base_Stop_IT(&motor->capturetim);				//关闭tim6计数中断	
//	__HAL_TIM_CLEAR_IT(&motor->capturetim,TIM_IT_UPDATE);	//清零tim6中断标志位(定时中断获取电机状态并反馈）
//	__HAL_TIM_SetCounter(&motor->capturetim,0);				//归零tim6计数值
	
	HAL_TIM_Base_Stop_IT(&motor->encoder);			//关闭encoder计数中断
//	__HAL_TIM_CLEAR_IT(&motor->encoder,TIM_IT_UPDATE);	//清零encoder中断标志位
//	__HAL_TIM_SetCounter(&motor->encoder,0);			//归零encoder计数值
	
//	HAL_TIM_PWM_Stop(&motor->timepwm1,motor->channelpwm1);	//关闭pwm输出
}




float get_speed(float ns, MOTOR_TypeDef *motor){		//测速
	
	motor->capture_count = __HAL_TIM_GetCounter(&motor->encoder) + (motor->encoder_overflow*0xffff);	//当前捕获值 = 捕获计数器的值 + 溢出次数*捕获计数器最大装载值

	motor->actual_speed = (float) (motor->capture_count - motor->last_count) / (beipin*xianshu*jiansubi*ns); //实际转速 = （当前捕获值 - 前一段时间捕获值） / （倍频数*线数*这段时间）
	
	motor->last_count = motor->capture_count;
	
	return motor->actual_speed;
}	

float get_angle(MOTOR_TypeDef *motor){		//测角度（位置）
	
	float angle=0;

	motor->capture_count = __HAL_TIM_GetCounter(&motor->encoder) + (motor->encoder_overflow*0xffff);	//当前捕获值 = 捕获计数器的值 + 溢出次数*捕获计数器最大装载值

	angle = 360.0f * (float)motor->capture_count/(beipin*xianshu*jiansubi);		//实际角度 = 360度 * 捕获脉冲数 / （倍频数*线数*减速比）
	
	return angle;			
}



void set_motor_pwm(MOTOR_TypeDef *motor){	//输出pwm，大于0正转，小于0反转
//AT8236电机驱动板控制代码	电机电压 = IN1电压 - IN2电压  
	if(motor->pwm == 0){
		__HAL_TIM_SetCompare( &motor->timepwm1, motor->channelpwm1, PWM_MAX);	
		__HAL_TIM_SetCompare( &motor->timepwm2, motor->channelpwm2, PWM_MAX);
	}
	
	if(motor->pwm > 0){		
		if(motor->pwm > PWM_MAX) motor->pwm = PWM_MAX;
		
		__HAL_TIM_SetCompare( &motor->timepwm1, motor->channelpwm1, PWM_MAX - motor->pwm);	
		__HAL_TIM_SetCompare( &motor->timepwm2, motor->channelpwm2, PWM_MAX);	
	}
	if(motor->pwm < 0){	
		if(motor->pwm < -PWM_MAX) motor->pwm = -PWM_MAX;
		
		__HAL_TIM_SetCompare( &motor->timepwm1, motor->channelpwm1, PWM_MAX);	
		__HAL_TIM_SetCompare( &motor->timepwm2, motor->channelpwm2, PWM_MAX + motor->pwm);
	}

/*	//tb6612电机驱动板控制代码
	if(motor->pwm > 0){						//IN1 = 1    IN2 = 0   正转
		HAL_GPIO_WritePin( motor->in1_port, motor->in1_pin, GPIO_PIN_SET),	
		HAL_GPIO_WritePin( motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
	}
	
	if(motor->pwm < 0){						//IN1 = 0    IN2 = 1   反转
		motor->pwm = -motor->pwm;
		
		HAL_GPIO_WritePin( motor->in1_port, motor->in1_pin, GPIO_PIN_RESET),
		HAL_GPIO_WritePin( motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
	}
	
	if(motor->pwm > PWM_MAX) motor->pwm = PWM_MAX;
	
	__HAL_TIM_SetCompare( &motor->timepwm, motor->channelpwm, motor->pwm);
*/
	
}	

void limit_pwm(MOTOR_TypeDef *motor){
	if(motor->k_angle.integral <-(PWM_MAX/6) )motor->k_angle.integral = -(PWM_MAX/6);//限制位置式pid中的积分项过大
	if(motor->k_angle.integral > (PWM_MAX/6) )motor->k_angle.integral = (PWM_MAX/6);
	
	if(motor->pwm <-(PWM_MAX/2) )motor->pwm = -(PWM_MAX/2);
	if(motor->pwm > (PWM_MAX/2) )motor->pwm = (PWM_MAX/2);
}

void limit_speed(MOTOR_TypeDef *motor){
	if(motor->target_speed > 0.5)motor->target_speed = 0.5;
	if(motor->target_speed < -0.5)motor->target_speed = -0.5;
}

void check_pwm_speed(MOTOR_TypeDef *motor){		// v = pwm占空比 * vmax
	float f = (motor->k_speed.fi / PWM_MAX) * SPEED_MAX;
	float v = motor->target_speed;
	
	if( -1.5f < f-v && f-v < 1.5f )return;		//误差在允许范围内
	else motor->pwm = motor->k_speed.fi = ( v / SPEED_MAX ) * PWM_MAX; //同时改变pwm和pid累加项fi
}

void feedback_angle(MOTOR_TypeDef *motor){
	motor->actual_angle = get_angle(motor);													//获取实际角度
	motor->pwm = (int)PID_position(motor->target_angle, motor->actual_angle, PWM_MAX, &motor->k_angle);	//pid算出需要输出的pwm
	limit_pwm(motor);
	set_motor_pwm(motor);																	//反馈电机
}

void feedback_speed(MOTOR_TypeDef *motor){
	motor->actual_speed = get_speed(0.01,motor); 		//printf("actual speed=%f\r\n",motor->actual_speed);														//获取实际速度
	motor->pwm = (int)PID_incremental(motor->target_speed*jiansubi, motor->actual_speed*jiansubi,  PWM_MAX, &motor->k_speed); 	//pid算出需要输出的pwm
	//check_pwm_speed(&Back_L);																					//检查pwm值是否偏离过大
	set_motor_pwm(motor);																					 	//反馈电机
}

void feedback_angle_double(MOTOR_TypeDef *motor){ //双环pid，让电机在匀速状态下转到指定角度
	motor->actual_angle = get_angle(motor);	
	
	motor->target_speed = PID_position(motor->target_angle, motor->actual_angle, SPEED_MAX*jiansubi, &motor->k_double)/jiansubi; //位置环pid算出需要的速度
	limit_speed(&Bopan);
	
	motor->actual_speed = (float) (motor->capture_count - motor->last_count) / (beipin*xianshu*jiansubi*0.01); //实际转速 
	motor->last_count = motor->capture_count;
	
	motor->pwm = (int)PID_incremental(motor->target_speed*jiansubi, motor->actual_speed*jiansubi, PWM_MAX, &motor->k_speed); //速度环pid算出应输出的pwm
		
	limit_pwm(&Bopan);
	set_motor_pwm(motor);																	//反馈电机
}

