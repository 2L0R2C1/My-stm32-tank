#include "control.h"
#include "motor.h"
#include "delay.h"
#include "usart.h"
#include "tim.h"


/**************���Ŀ��ƴ���****************/
u8 friction_enable = 0;
#define SPEEDMAX 200

void friction_init(void){
#ifdef L298N
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
#endif	
	
#ifdef A2312
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,SPEEDMAX);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,SPEEDMAX);

	delay_ms(3000);
	
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,100);
	__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,100);
	
	delay_ms(3000);
	
#endif
	
}

void friction_start(void){
	friction_enable = 1;
	int pwm=0;
	
#ifdef L298N	
	HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_RESET);
	
	for(pwm=0;pwm<PWM_MAX;pwm+=100){	//����������������������ջ�����
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,pwm);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,pwm);
		delay_ms(50);
	}
	
#endif
	
#ifdef A2312
	for(pwm=100;pwm<=SPEEDMAX;pwm+=2){		//����������������������ջ�����
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,pwm);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,pwm);
		delay_ms(50);
	}
#endif	
}
void friction_stop(void){
	friction_enable = 0;
	int pwm=0;
	
#ifdef L298N	
	for(pwm=PWM_MAX;pwm>0;pwm-=100){	//����������������������ջ�����
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_3,pwm);
		__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_4,pwm);
		delay_ms(50);
	}
	
	HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_RESET);
#endif
	
#ifdef A2312
	for(pwm=SPEEDMAX;pwm>=100;pwm-=5){		//����������������������ջ�����
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,pwm);
		__HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,pwm);
		delay_ms(50);
	}
#endif	
}

void single_shot(void) //����
{
	if(!friction_enable)friction_start();
	Bopan.target_angle =40;
}
void repeating_shot1(void) //���� 5
{
	if(!friction_enable)friction_start();
	Bopan.target_angle=200;
}
void repeating_shot2(void) //���� 
{
	if(!friction_enable)friction_start();	
	Bopan.target_angle=400;
}

void single_shot_(void){
	if(!friction_enable)friction_start();	
	Bopan.target_angle=-40;
}

void f411_control(u8 order)     
{
	MOTOR_Tim_Stop(&Bopan);
	delay_ms(50);
	switch(order){
		case 'm':	friction_start();	break;
		case 'n':	friction_stop();	break;
		case 'o':	single_shot(); 	    break;	//����	        
		case 'w':	repeating_shot1(); 	break;	//����5	
		case 'k':	repeating_shot2();  break;	//����10	
		case 'p':	single_shot_();		break;
	}
	MOTOR_Init();
	PID_init(&Bopan.k_double);
	MOTOR_Tim_Init(&Bopan);
}


