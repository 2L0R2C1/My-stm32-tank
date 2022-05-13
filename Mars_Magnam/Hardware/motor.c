#include "motor.h"
#include "pid.h"
#include "tim.h"
#include "delay.h"
#include "usart.h"

/*********************************************
��ʼ�����е����������ز�����

���ҽ��������encoder��pwm���ͨ����IN1��IN2��һһ��Ӧ��ϵ

��ʼ��������encoder����ʱ����������жϣ�����pwm�����
**********************************************/

MOTOR_TypeDef Forward_L = {0};
MOTOR_TypeDef Forward_R = {0};
MOTOR_TypeDef Back_L   = {0};
MOTOR_TypeDef Back_R   = {0};

void MOTOR_Init(void){
	Forward_L.encoder = htim8;	//ǰ����encoder ���� ��ʱ��TIM8
	Forward_R.encoder = htim4;	//ǰ�ҵ��encoder ���� ��ʱ��TIM4
	Back_L.encoder 	  = htim5;	//������encoder ���� ��ʱ��TIM5
	Back_R.encoder    = htim1;	//���ҵ��encoder ���� ��ʱ��TIM1
		
	Forward_L.timepwm1 = htim2;	//time2�ĸ�ͨ��pwm���	
	Forward_R.timepwm1 = htim2;
	Back_L.timepwm1    = htim2;	
	Back_R.timepwm1    = htim3;
	
	Forward_L.channelpwm1 = TIM_CHANNEL_2;
	Forward_R.channelpwm1 = TIM_CHANNEL_4;
	Back_L.channelpwm1    = TIM_CHANNEL_1;
	Back_R.channelpwm1    = TIM_CHANNEL_3;

	
	Forward_L.timepwm2 = htim3;	//time3�ĸ�ͨ��pwm���	
	Forward_R.timepwm2 = htim2;	
	Back_L.timepwm2    = htim3;	
	Back_R.timepwm2    = htim3;
	
	Forward_L.channelpwm2 = TIM_CHANNEL_2;
	Forward_R.channelpwm2 = TIM_CHANNEL_3;
	Back_L.channelpwm2    = TIM_CHANNEL_1;
	Back_R.channelpwm2    = TIM_CHANNEL_4;
	
//	__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);	//����tim6�жϱ�־λ(��ʱ�жϻ�ȡ���״̬��������
//	__HAL_TIM_SetCounter(&htim6,0);				//����tim6����ֵ
//	HAL_TIM_Base_Start_IT(&htim6);				//����tim6�����ж�
	
	
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

void MOTOR_Tim_Init(MOTOR_TypeDef *motor){			//����ĳ�����

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

	
	__HAL_TIM_CLEAR_IT(&motor->encoder,TIM_IT_UPDATE);	//����encoder�жϱ�־λ
	__HAL_TIM_SetCounter(&motor->encoder,0);			//����encoder����ֵ
	HAL_TIM_Base_Start_IT(&motor->encoder);			//����encoder�����ж�

	HAL_TIM_Encoder_Start(&motor->encoder,TIM_CHANNEL_1);	//����encoder���벶��ͨ��
	HAL_TIM_Encoder_Start(&motor->encoder,TIM_CHANNEL_2);	//����encoder���벶��ͨ��
	
	HAL_TIM_PWM_Start(&motor->timepwm1,motor->channelpwm1);	//����pwm���
	HAL_TIM_PWM_Start(&motor->timepwm2,motor->channelpwm2);	//����pwm���
	
	motor->pwm = 0;
	set_motor_pwm(motor);
	delay_ms(500);
}

void MOTOR_Tim_Stop(MOTOR_TypeDef *motor){			//ͣ��ĳ�����
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
	
//	HAL_TIM_Base_Stop_IT(&motor->capturetim);				//�ر�tim6�����ж�	�رն�ʱ��6��Ҳ����ζ��ʵ�������е��ͣ��
//	__HAL_TIM_CLEAR_IT(&motor->capturetim,TIM_IT_UPDATE);	//����tim6�жϱ�־λ(��ʱ�жϻ�ȡ���״̬��������
//	__HAL_TIM_SetCounter(&motor->capturetim,0);				//����tim6����ֵ
	
	HAL_TIM_Base_Stop_IT(&motor->encoder);			//�ر�encoder�����ж�
//	__HAL_TIM_CLEAR_IT(&motor->encoder,TIM_IT_UPDATE);	//����encoder�жϱ�־λ
//	__HAL_TIM_SetCounter(&motor->encoder,0);			//����encoder����ֵ
	
//	HAL_TIM_PWM_Stop(&motor->timepwm1,motor->channelpwm1);	//�ر�pwm���
}


void MOTOR_reset(){	//�������е��
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


float get_speed(float ns, MOTOR_TypeDef *motor){		//����
	volatile float speed = 0.0f;
	
	motor->capture_count = __HAL_TIM_GetCounter(&motor->encoder) + (motor->encoder_overflow*0xffff);	//��ǰ����ֵ = �����������ֵ + �������*������������װ��ֵ

	speed = (float) (motor->capture_count - motor->last_count) / (beipin*xianshu*jiansubi*ns); //ʵ��ת�� = ����ǰ����ֵ - ǰһ��ʱ�䲶��ֵ�� / ����Ƶ��*����*���ʱ�䣩
	
	if(speed < SPEED_MAX && speed > -SPEED_MAX)motor->last_count = motor->capture_count;	//�޳��������ֵ��Ӳ���Ʋ�����㣩
	
	return speed;
}	

float get_angle(MOTOR_TypeDef *motor){		//��Ƕȣ�λ�ã�
	
	float angle=0;

	motor->capture_count = __HAL_TIM_GetCounter(&motor->encoder) + (motor->encoder_overflow*0xffff);	//��ǰ����ֵ = �����������ֵ + �������*������������װ��ֵ

	angle = 360.0f * (float)motor->capture_count/(beipin*xianshu*jiansubi);		//ʵ�ʽǶ� = 360�� * ���������� / ����Ƶ��*����*���ٱȣ�
	
	return angle;			
}



void set_motor_pwm(MOTOR_TypeDef *motor){	//���pwm������0��ת��С��0��ת
//AT8236�����������ƴ���	�����ѹ = IN1��ѹ - IN2��ѹ  
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

/*	//tb6612�����������ƴ���
	if(motor->pwm > 0){						//IN1 = 1    IN2 = 0   ��ת
		HAL_GPIO_WritePin( motor->in1_port, motor->in1_pin, GPIO_PIN_SET),	
		HAL_GPIO_WritePin( motor->in2_port, motor->in2_pin, GPIO_PIN_RESET);
	}
	
	if(motor->pwm < 0){						//IN1 = 0    IN2 = 1   ��ת
		motor->pwm = -motor->pwm;
		
		HAL_GPIO_WritePin( motor->in1_port, motor->in1_pin, GPIO_PIN_RESET),
		HAL_GPIO_WritePin( motor->in2_port, motor->in2_pin, GPIO_PIN_SET);
	}
	
	if(motor->pwm > PWM_MAX) motor->pwm = PWM_MAX;
	
	__HAL_TIM_SetCompare( &motor->timepwm, motor->channelpwm, motor->pwm);
*/
	
}	


void limit_pwm_angle(MOTOR_TypeDef *motor){
	if(motor->k_angle.integral <-((float)PWM_MAX/4.0f) )motor->k_angle.integral = -((float)PWM_MAX/4.0f);//����λ��ʽpid�еĻ��������
	if(motor->k_angle.integral > ((float)PWM_MAX/4.0f) )motor->k_angle.integral = ((float)PWM_MAX/4.0f);
	
	if(motor->pwm <-((float)PWM_MAX/2.0f) )motor->pwm = -((float)PWM_MAX/2.0f);
	if(motor->pwm > ((float)PWM_MAX/2.0f) )motor->pwm = ((float)PWM_MAX/2.0f);
}

void limit_pwm_speed(MOTOR_TypeDef *motor){
	if(motor->pwm <-((float)PWM_MAX/1.2f) )motor->pwm = -((float)PWM_MAX/1.2f) ;
	if(motor->pwm > ((float)PWM_MAX/1.2f) )motor->pwm = ((float)PWM_MAX/1.2f);
}


void feedback_angle(MOTOR_TypeDef *motor){
	volatile float angle = get_angle(motor);	//��ȡʵ�ʽǶ�
	volatile float target = motor -> target_angle;
	
	motor->pwm = (int)PID_position(target,angle, PWM_MAX/1.2, &motor->k_angle);	//pid�����Ҫ�����pwm
	
	motor->actual_angle = angle;
	limit_pwm_angle(motor);
	set_motor_pwm(motor);		//�������
}

void feedback_speed(MOTOR_TypeDef *motor){
	volatile float speed = get_speed(0.01,motor); 	//��ȡʵ���ٶ�
	volatile float target = motor ->target_speed;
	volatile int pwm=0;
	
	if( speed - (float)SPEED_MAX/1.5f > 0.5f ) speed = (float)SPEED_MAX/1.5f;	//�޳��������ֵ��Ӳ���Ʋ�����㣩
	if( speed + (float)SPEED_MAX/1.5f < -0.5f) speed = -(float)SPEED_MAX/1.5f;
//	if( speed - motor->actual_speed > SPEED_MAX ) speed = motor->actual_speed + SPEED_MAX;	
//	if( speed - motor->actual_speed < -SPEED_MAX ) speed = motor->actual_speed - SPEED_MAX;
	
	motor -> actual_speed = speed;
	
	pwm = (int)PID_incremental(target*jiansubi, speed*jiansubi,  PWM_MAX/1.2, &motor->k_speed); 	//pid�����Ҫ�����pwm
	
	if(pwm > PWM_MAX/1.2) pwm = PWM_MAX/1.2;	//��pwm
	if(pwm < -PWM_MAX/1.2)pwm = -PWM_MAX/1.2;
	
	motor->pwm = pwm;
	
	set_motor_pwm(motor);		//�������
}


void feedback_angle_double(MOTOR_TypeDef *motor, float ms){ //˫��pid���õ��������״̬��ת��ָ���Ƕ�
	motor->actual_angle = get_angle(motor);	
	
	motor->target_speed = PID_position(motor->target_angle, motor->actual_angle, SPEED_MAX*jiansubi, &motor->k_double)/jiansubi; //λ�û�pid�����Ҫ���ٶ�
	
	if(motor->target_speed > (float)SPEED_MAX/3.0f ) motor->target_speed = (float)SPEED_MAX/3.0f;
	if(motor->target_speed < -(float)SPEED_MAX/3.0f ) motor->target_speed = -(float)SPEED_MAX/3.0f;
	
	motor->actual_speed = (float) (motor->capture_count - motor->last_count) / (beipin*xianshu*jiansubi*ms); //ʵ��ת�� 
	motor->last_count = motor->capture_count;
	
	motor->pwm = (int)PID_incremental(motor->target_speed*jiansubi, motor->actual_speed*jiansubi, PWM_MAX, &motor->k_speed); //�ٶȻ�pid���Ӧ�����pwm
		
	limit_pwm_angle(motor);
	set_motor_pwm(motor);	//�������
}

