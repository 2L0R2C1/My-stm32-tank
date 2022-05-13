#include "motor.h"
#include "pid.h"
#include "tim.h"
#include "delay.h"
#include "usart.h"

/*********************************************
初始化所有电机的所有相关参数，

并且建立电机与encoder、pwm输出通道、IN1、IN2的一一对应关系

初始化并开启encoder、定时器溢出更新中断，开启pwm输出口
**********************************************/

MOTOR_TypeDef Forward_L = {0};
MOTOR_TypeDef Forward_R = {0};
MOTOR_TypeDef Back_L   = {0};
MOTOR_TypeDef Back_R   = {0};

void MOTOR_Init(void){
	Forward_L.encoder = htim8;	//前左电机encoder 》》 定时器TIM8
	Forward_R.encoder = htim4;	//前右电机encoder 》》 定时器TIM4
	Back_L.encoder 	  = htim5;	//后左电机encoder 》》 定时器TIM5
	Back_R.encoder    = htim1;	//后右电机encoder 》》 定时器TIM1
		
	Forward_L.timepwm1 = htim2;	//time2四个通道pwm输出	
	Forward_R.timepwm1 = htim2;
	Back_L.timepwm1    = htim2;	
	Back_R.timepwm1    = htim3;
	
	Forward_L.channelpwm1 = TIM_CHANNEL_2;
	Forward_R.channelpwm1 = TIM_CHANNEL_4;
	Back_L.channelpwm1    = TIM_CHANNEL_1;
	Back_R.channelpwm1    = TIM_CHANNEL_3;

	
	Forward_L.timepwm2 = htim3;	//time3四个通道pwm输出	
	Forward_R.timepwm2 = htim2;	
	Back_L.timepwm2    = htim3;	
	Back_R.timepwm2    = htim3;
	
	Forward_L.channelpwm2 = TIM_CHANNEL_2;
	Forward_R.channelpwm2 = TIM_CHANNEL_3;
	Back_L.channelpwm2    = TIM_CHANNEL_1;
	Back_R.channelpwm2    = TIM_CHANNEL_4;
	
//	__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	//清零tim6中断标志位(定时中断获取电机状态并反馈）
//	__HAL_TIM_SetCounter(&htim6,0);				//归零tim6计数值
//	HAL_TIM_Base_Start_IT(&htim6);				//开启tim6计数中断
	
	
/*	Forward_L.in1_port = Forward_L_IN1_GPIO_Port;	Forward_L.in2_port = Forward_L_IN2_GPIO_Port;
	Forward_R.in1_port = Forward_R_IN1_GPIO_Port;	Forward_R.in2_port = Forward_R_IN2_GPIO_Port;
	Back_L.in1_port    = Back_L_IN1_GPIO_Port;		Back_L.in2_port   =  Back_L_IN2_GPIO_Port;
	Back_R.in1_port    = Back_R_IN1_GPIO_Port;		Back_R.in2_port   =  Back_R_IN2_GPIO_Port;

	Forward_L.in1_pin = Forward_L_IN1_Pin;		Forward_L.in2_pin = Forward_L_IN2_Pin;
	Forward_R.in1_pin = Forward_R_IN1_Pin;		Forward_R.in2_pin = Forward_R_IN2_Pin;
	Back_L.in1_pin    = Back_L_IN1_Pin;			Back_L.in2_pin    = Back_L_IN2_Pin;
	Back_R.in1_pin    = Back_R_IN1_Pin;			Back_R.in2_pin    = Back_R_IN2_Pin;
	
	PID_restart(&Forward_L.k_speed,20,5,0); 		PID_restart(&Forward_L.k_angle,50,0.5,0); 	
	PID_restart(&Forward_R.k_speed,20,5,0); 		PID_restart(&Forward_R.k_angle,50,0.5,0);	
	PID_restart(&Back_L.k_speed,20,5,0);			PID_restart(&Back_L.k_angle,50,0.5,0);		
	PID_restart(&Back_R.k_speed,20,5,0);			PID_restart(&Back_R.k_angle,50,0.5,0);	
*/	
}

void MOTOR_Tim_Init(MOTOR_TypeDef *motor){			//启用某个电机

	set_pid(&motor->k_speed, 15, 3, 50);
	set_pid(&motor->k_angle, 50, 0.5, 100);
	set_pid(&motor->k_double, 1.2, 0.005, 25);
	
	motor->actual_angle=0;
	motor->actual_speed=0;
	motor->target_angle=0;
	motor->target_speed=0;
	
	motor->encoder_overflow=0;
	motor->capture_count=0;
	motor->last_count=0;
	motor->pwm=0;

	
	__HAL_TIM_CLEAR_IT(&motor->encoder,TIM_IT_UPDATE);	//清零encoder中断标志位
	__HAL_TIM_SetCounter(&motor->encoder,0);			//归零encoder计数值
	HAL_TIM_Base_Start_IT(&motor->encoder);			//开启encoder计数中断

	HAL_TIM_Encoder_Start(&motor->encoder,TIM_CHANNEL_1);	//开启encoder输入捕获通道
	HAL_TIM_Encoder_Start(&motor->encoder,TIM_CHANNEL_2);	//开启encoder输入捕获通道
	
	HAL_TIM_PWM_Start(&motor->timepwm1,motor->channelpwm1);	//开启pwm输出
	HAL_TIM_PWM_Start(&motor->timepwm2,motor->channelpwm2);	//开启pwm输出
	
	motor->pwm = 0;
	set_motor_pwm(motor);
	delay_ms(500);
}

void MOTOR_Tim_Stop(MOTOR_TypeDef *motor){			//停用某个电机
	motor->pwm = 0;
	set_motor_pwm(motor);

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
	
//	HAL_TIM_Base_Stop_IT(&motor->capturetim);				//关闭tim6计数中断	关闭定时器6，也就意味着实际上所有电机停摆
//	__HAL_TIM_CLEAR_IT(&motor->capturetim,TIM_IT_UPDATE);	//清零tim6中断标志位(定时中断获取电机状态并反馈）
//	__HAL_TIM_SetCounter(&motor->capturetim,0);				//归零tim6计数值
	
	HAL_TIM_Base_Stop_IT(&motor->encoder);			//关闭encoder计数中断
//	__HAL_TIM_CLEAR_IT(&motor->encoder,TIM_IT_UPDATE);	//清零encoder中断标志位
//	__HAL_TIM_SetCounter(&motor->encoder,0);			//归零encoder计数值
	
//	HAL_TIM_PWM_Stop(&motor->timepwm1,motor->channelpwm1);	//关闭pwm输出
}


void MOTOR_reset(){	//重置所有电机
	MOTOR_Tim_Stop(&Forward_L);
	MOTOR_Tim_Stop(&Forward_R);
	MOTOR_Tim_Stop(&Back_L);
	MOTOR_Tim_Stop(&Back_R);
	
	delay_ms(50);
	
	MOTOR_Init();
	
	MOTOR_Tim_Init(&Forward_L);
	MOTOR_Tim_Init(&Forward_R);
	MOTOR_Tim_Init(&Back_L); 
	MOTOR_Tim_Init(&Back_R);
}


float get_speed(float ns, MOTOR_TypeDef *motor){		//测速
	volatile float speed = 0.0f;
	
	motor->capture_count = __HAL_TIM_GetCounter(&motor->encoder) + (motor->encoder_overflow*0xffff);	//当前捕获值 = 捕获计数器的值 + 溢出次数*捕获计数器最大装载值

	speed = (float) (motor->capture_count - motor->last_count) / (beipin*xianshu*jiansubi*ns); //实际转速 = （当前捕获值 - 前一段时间捕获值） / （倍频数*线数*这段时间）
	
	if(speed < SPEED_MAX && speed > -SPEED_MAX)motor->last_count = motor->capture_count;	//剔除过大测量值（硬件纹波有噪点）
	
	return speed;
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


void limit_pwm_angle(MOTOR_TypeDef *motor){
	if(motor->k_angle.integral <-((float)PWM_MAX/4.0f) )motor->k_angle.integral = -((float)PWM_MAX/4.0f);//限制位置式pid中的积分项过大
	if(motor->k_angle.integral > ((float)PWM_MAX/4.0f) )motor->k_angle.integral = ((float)PWM_MAX/4.0f);
	
	if(motor->pwm <-((float)PWM_MAX/2.0f) )motor->pwm = -((float)PWM_MAX/2.0f);
	if(motor->pwm > ((float)PWM_MAX/2.0f) )motor->pwm = ((float)PWM_MAX/2.0f);
}

void limit_pwm_speed(MOTOR_TypeDef *motor){
	if(motor->pwm <-((float)PWM_MAX/1.2f) )motor->pwm = -((float)PWM_MAX/1.2f) ;
	if(motor->pwm > ((float)PWM_MAX/1.2f) )motor->pwm = ((float)PWM_MAX/1.2f);
}


void feedback_angle(MOTOR_TypeDef *motor){
	volatile float angle = get_angle(motor);	//获取实际角度
	volatile float target = motor -> target_angle;
	
	motor->pwm = (int)PID_position(target,angle, PWM_MAX/1.2, &motor->k_angle);	//pid算出需要输出的pwm
	
	motor->actual_angle = angle;
	limit_pwm_angle(motor);
	set_motor_pwm(motor);		//反馈电机
}

void feedback_speed(MOTOR_TypeDef *motor){
	volatile float speed = get_speed(0.01,motor); 	//获取实际速度
	volatile float target = motor ->target_speed;
	volatile int pwm=0;
	
	if( speed - (float)SPEED_MAX/1.5f > 0.5f ) speed = (float)SPEED_MAX/1.5f;	//剔除过大测量值（硬件纹波有噪点）
	if( speed + (float)SPEED_MAX/1.5f < -0.5f) speed = -(float)SPEED_MAX/1.5f;
//	if( speed - motor->actual_speed > SPEED_MAX ) speed = motor->actual_speed + SPEED_MAX;	
//	if( speed - motor->actual_speed < -SPEED_MAX ) speed = motor->actual_speed - SPEED_MAX;
	
	motor -> actual_speed = speed;
	
	pwm = (int)PID_incremental(target*jiansubi, speed*jiansubi,  PWM_MAX/1.2, &motor->k_speed); 	//pid算出需要输出的pwm
	
	if(pwm > PWM_MAX/1.2) pwm = PWM_MAX/1.2;	//限pwm
	if(pwm < -PWM_MAX/1.2)pwm = -PWM_MAX/1.2;
	
	motor->pwm = pwm;
	
	set_motor_pwm(motor);		//反馈电机
}


void feedback_angle_double(MOTOR_TypeDef *motor, float ms){ //双环pid，让电机在匀速状态下转到指定角度
	motor->actual_angle = get_angle(motor);	
	
	motor->target_speed = PID_position(motor->target_angle, motor->actual_angle, SPEED_MAX*jiansubi, &motor->k_double)/jiansubi; //位置环pid算出需要的速度
	
	if(motor->target_speed > (float)SPEED_MAX/3.0f ) motor->target_speed = (float)SPEED_MAX/3.0f;
	if(motor->target_speed < -(float)SPEED_MAX/3.0f ) motor->target_speed = -(float)SPEED_MAX/3.0f;
	
	motor->actual_speed = (float) (motor->capture_count - motor->last_count) / (beipin*xianshu*jiansubi*ms); //实际转速 
	motor->last_count = motor->capture_count;
	
	motor->pwm = (int)PID_incremental(motor->target_speed*jiansubi, motor->actual_speed*jiansubi, PWM_MAX, &motor->k_speed); //速度环pid算出应输出的pwm
		
	limit_pwm_angle(motor);
	set_motor_pwm(motor);	//反馈电机
}

