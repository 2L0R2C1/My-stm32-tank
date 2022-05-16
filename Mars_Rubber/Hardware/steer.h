#ifndef __STEER_H_
#define __STEER_H_

#include "main.h"

typedef struct
{
	volatile float angle;
	volatile int pwm;
	float max_angle;
	float A,B;
	u16 channelpwm;
	TIM_HandleTypeDef timepwm; 
	
}STEER_TypeDef;

extern STEER_TypeDef Steer_f,Steer_b,Turret;


void steer_init(void);

void steer_start(STEER_TypeDef *steer);

void set_steer_pwm(STEER_TypeDef *steer);

void steer_turn_slow(STEER_TypeDef *steer);

#endif

