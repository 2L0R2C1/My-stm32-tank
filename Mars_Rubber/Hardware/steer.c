#include "steer.h"
#include "tim.h"
//#include "usart.h"

STEER_TypeDef Steer_f={0},Steer_b={0},Turret;

void steer_init(void){
	Steer_f.timepwm = htim12;
	Steer_b.timepwm = htim12;
	Turret.timepwm = htim9;
	Steer_f.channelpwm = TIM_CHANNEL_1;
	Steer_b.channelpwm = TIM_CHANNEL_2;
	Turret.channelpwm = TIM_CHANNEL_1;
	
	Steer_f.angle = 0;
	Steer_b.angle = 0;
	Turret.angle = 0;
	
	Steer_f.max_angle = 180.0f;
	Steer_b.max_angle = 180.0f;
	Turret.max_angle = 270.0f;
	
	Steer_f.A = 25.0f/18.0f;
	Steer_b.A = 25.0f/18.0f;
	Turret.A = 20.0f/27.0f;
	
	Steer_f.B = Steer_b.B = Turret.B = 50.0f;
	
	steer_start(&Steer_f);
	steer_start(&Steer_b);
	steer_start(&Turret);
}

void steer_start(STEER_TypeDef *steer){
	HAL_TIM_PWM_Start(&steer->timepwm,steer->channelpwm);
	steer->angle = 0;
	set_steer_pwm(steer);
}

void set_steer_pwm(STEER_TypeDef *steer){
	int pwm = steer->A * steer->angle + steer->B;	//y = ax + b （pwm占空比与舵机转动角度成线性关系）
	__HAL_TIM_SetCompare(&steer->timepwm,steer->channelpwm,pwm);
}
