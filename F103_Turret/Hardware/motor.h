#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"

#define beipin 4
#define xianshu 11
#define jiansubi 65
#define PWM_MAX 7200
#define SPEED_MAX 1

#define CAPTURE htim6//定时中断获取电机状态

typedef struct
{
	volatile float actual_speed;		//电机实际转速
	volatile float target_speed;		//电机目标转速
	volatile float actual_angle;		//电机实际转过角度
	volatile float target_angle;		//电机目标转过角度
	
	volatile int encoder_overflow;	//各电机encoder溢出次数
	volatile int capture_count;		//当前编码器计数值
	volatile int last_count;			//上一时刻编码器计数值
	volatile int pwm;				//输出给电机pwm值
	
	volatile u8 mode;
	
	PID_TypeDef k_angle;	//带编码器电机对应位置环pid参数
	PID_TypeDef k_speed;	//带编码器电机对应速度环pid参数
	PID_TypeDef k_double;	//带编码器电机对应双环pid参数
	
	TIM_HandleTypeDef timepwm1; //电机in1 pwm对应定时器
	u16 channelpwm1;			//电机in1对应pwm输入通道
	
	
	TIM_HandleTypeDef timepwm2; //电机in2 pwm对应定时器
	u16 channelpwm2;			//电机in2对应pwm输入通道
	
	
	TIM_HandleTypeDef encoder;	//电机encoder对应定时器
	TIM_HandleTypeDef capturetim;//定时负反馈
	
}MOTOR_TypeDef;	
	
extern MOTOR_TypeDef Bopan;

void MOTOR_Init(void);		

void MOTOR_Tim_Init(MOTOR_TypeDef *motor);

void MOTOR_Tim_Stop(MOTOR_TypeDef *motor);

void MOTOR_reset(void);					//重置所有电机

float get_speed(float nms,MOTOR_TypeDef *motor);	//测速
float get_angle(MOTOR_TypeDef *motor);	//测角度位置

void set_motor_pwm(MOTOR_TypeDef *motor);	//pwm输出	

void limit_pwm(MOTOR_TypeDef *motor);		//限幅pwm
void limit_speed(MOTOR_TypeDef *motor);		//限速
void check_pwm_speed(MOTOR_TypeDef *motor);	//检查pid算出的pwm与目标速度关系是否近似对应
void feedback_angle(MOTOR_TypeDef *motor);	//反馈角度信号给电机
void feedback_speed(MOTOR_TypeDef *motor);  //反馈速度信号给电机
void feedback_angle_double(MOTOR_TypeDef *motor);//双环pid


#ifdef __cplusplus
}
#endif

#endif 

