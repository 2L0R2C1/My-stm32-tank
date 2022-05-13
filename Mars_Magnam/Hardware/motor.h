#ifndef __MOTOR_H__
#define __MOTOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"

#define beipin 4
#define xianshu 13
#define jiansubi 30
#define PWM_MAX 7200
#define SPEED_MAX 6

#define CAPTURE htim6//��ʱ�жϻ�ȡ���״̬

typedef struct
{
	volatile float actual_speed;		//���ʵ��ת��
	volatile float target_speed;		//���Ŀ��ת��
	volatile float actual_angle;		//���ʵ��ת���Ƕ�
	volatile float target_angle;		//���Ŀ��ת���Ƕ�
	
	volatile int encoder_overflow;	//�����encoder�������
	volatile int capture_count;		//��ǰ����������ֵ
	volatile int last_count;			//��һʱ�̱���������ֵ
	volatile int pwm;				//��������pwmֵ
	
	volatile u8 mode;
	
	PID_TypeDef k_angle;	//�������������Ӧλ�û�pid����
	PID_TypeDef k_speed;	//�������������Ӧ�ٶȻ�pid����
	PID_TypeDef k_double;	//�������������Ӧ˫��pid����
	
	TIM_HandleTypeDef timepwm1; //���in1 pwm��Ӧ��ʱ��
	u16 channelpwm1;			//���in1��Ӧpwm����ͨ��
	
	
	TIM_HandleTypeDef timepwm2; //���in2 pwm��Ӧ��ʱ��
	u16 channelpwm2;			//���in2��Ӧpwm����ͨ��
	
//	GPIO_TypeDef *in1_port,*in2_port;//�������������IN1��IN2��ƽ��ϵ��������ת
//	u16 in1_pin,in2_pin;
	
	TIM_HandleTypeDef encoder;	//���encoder��Ӧ��ʱ��
	TIM_HandleTypeDef capturetim;//��ʱ������
	
}MOTOR_TypeDef;	
	
extern MOTOR_TypeDef Forward_L,Forward_R,Back_L,Back_R;

void MOTOR_Init(void);		//��ʼ�����е��

void MOTOR_Tim_Init(MOTOR_TypeDef *motor);//����ĳ�����

void MOTOR_Tim_Stop(MOTOR_TypeDef *motor);//ͣ��ĳ�����

void MOTOR_reset(void);					  //�������е��

float get_speed(float nms,MOTOR_TypeDef *motor);	//����
float get_angle(MOTOR_TypeDef *motor);	//��Ƕ�λ��

void set_motor_pwm(MOTOR_TypeDef *motor);	//pwm���	

void limit_pwm_angle(MOTOR_TypeDef *motor);		//�޷�pwm
void limit_pwm_speed(MOTOR_TypeDef *motor);		//����
void check_pwm_speed(MOTOR_TypeDef *motor);	//���pid�����pwm��Ŀ���ٶȹ�ϵ�Ƿ���ƶ�Ӧ
void feedback_angle(MOTOR_TypeDef *motor);	//�����Ƕ��źŸ����
void feedback_speed(MOTOR_TypeDef *motor);  //�����ٶ��źŸ����
void feedback_angle_double(MOTOR_TypeDef *motor, float ms);//˫��pid


#ifdef __cplusplus
}
#endif

#endif 

