#include "steer.h"
#include "tim.h"
#include "delay.h"
#include "usart.h"

STEER_TypeDef Steer_f={0},Steer_b={0},Turret;

void steer_init(void){
	Steer_f.timepwm = htim12;
	Steer_b.timepwm = htim12;
	Turret.timepwm = htim9;
	Steer_f.channelpwm = TIM_CHANNEL_2;
	Steer_b.channelpwm = TIM_CHANNEL_1;
	Turret.channelpwm = TIM_CHANNEL_1;
	
	Steer_f.angle = 0;
	Steer_b.angle = 0;
	Turret.angle = 10;
	
	Steer_f.max_angle = 180.0f;
	Steer_b.max_angle = 180.0f;
	Turret.max_angle = 270.0f;
	
	Steer_f.A = -250.0f/18.0f;
	Steer_b.A = -250.0f/18.0f;
	Turret.A = 200.0f/27.0f;
	
	Steer_f.B = Steer_b.B = 3000;
	Turret.B = 500.0f;
	
	steer_start(&Steer_f);
	steer_start(&Steer_b);
	steer_start(&Turret);
}

void steer_start(STEER_TypeDef *steer){
	HAL_TIM_PWM_Start(&steer->timepwm,steer->channelpwm);
	set_steer_pwm(steer);
}

void set_steer_pwm(STEER_TypeDef *steer){
	steer->pwm = steer->A * steer->angle + steer->B;
	__HAL_TIM_SetCompare(&steer->timepwm,steer->channelpwm,steer->pwm);
}

void steer_turn_slow(STEER_TypeDef *steer){
	int pwm = steer->A * steer->angle + steer->B;
	if( steer->pwm > pwm ) steer->pwm -- , delay_ms(3);
	if( steer->pwm < pwm ) steer->pwm ++ , delay_ms(3);
	__HAL_TIM_SetCompare(&steer->timepwm,steer->channelpwm,steer->pwm);
}

