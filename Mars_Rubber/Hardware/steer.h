#ifndef __STEER_H_
#define __STEER_H_

#include "main.h"

#define INIT_ANGLE 300
#define OVER_ANGLE 50

typedef struct
{
	volatile int angle;
	float max_angle;
	float A,B;
	volatile u16 channelpwm;
	TIM_HandleTypeDef timepwm; 
	
}STEER_TypeDef;

extern STEER_TypeDef Steer_f,Steer_b,Turret;

void steer_init(void);

void steer_start(STEER_TypeDef *steer);

void set_steer_pwm(STEER_TypeDef *steer);


#endif
