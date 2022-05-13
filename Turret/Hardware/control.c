#include "control.h"
#include "motor.h"
#include "delay.h"
#include "usart.h"


/**************核心控制代码****************/
u8 friction_enable = 0;

void friction_start(void){
	friction_enable = 1;
	HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_RESET);
	delay_ms(2500);
}
void friction_stop(void){
	friction_enable = 0;
	HAL_GPIO_WritePin(IN1_GPIO_Port,IN1_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN2_GPIO_Port,IN2_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN3_GPIO_Port,IN3_Pin,GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN4_GPIO_Port,IN4_Pin,GPIO_PIN_RESET);
	delay_ms(2500);
}

void single_shot(void) //单发
{
	if(!friction_enable)friction_start();
	Bopan.target_angle =40;
}
void repeating_shot1(void) //连发 5
{
	if(!friction_enable)friction_start();
	Bopan.target_angle=200;
}
void repeating_shot2(void) //连发 
{
	if(!friction_enable)friction_start();	
	Bopan.target_angle=400;
}


void f411_control(u8 order)     
{
	MOTOR_Tim_Stop(&Bopan);
	delay_ms(50);
	switch(order){
		case 'm':	friction_start();	break;
		case 'n':	friction_stop();		break;
		case 'o':	single_shot(); 	    break;	//单发	        
		case 'w':	repeating_shot1(); 	break;	//连发5	
		case 'k':	repeating_shot2();  break;	//连发10		          				        
	}
	MOTOR_Init();
	PID_init(&Bopan.k_double);
	MOTOR_Tim_Init(&Bopan);
}


