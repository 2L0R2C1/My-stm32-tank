#include "motor.h"
#include "pid.h"
#include "tim.h"
#include "delay.h"
#include "usart.h"
#include "string.h"

/*********************************************
��ʼ�����е����������ز�����

���ҽ��������encoder��pwm���ͨ����IN1��IN2��һһ��Ӧ��ϵ

��ʼ��������encoder����ʱ����������жϣ�����pwm�����
**********************************************/

MOTOR_TypeDef Bopan   = {0};
//MOTOR_TypeDef Friction_R   = {0};
//MOTOR_TypeDef Friction_L   = {0};

void MOTOR_Init(void){
	Bopan.encoder = htim1;
	Bopan.timepwm1 = htim9;
	Bopan.channelpwm1 = TIM_CHANNEL_1;
	Bopan.timepwm2 = htim9;
	Bopan.channelpwm2 = TIM_CHANNEL_2;
	Bopan.capturetim = htim10;
	
/*	Friction_R.encoder = htim2;
	Friction_R.timepwm1 = htim4;
	Friction_R.channelpwm1 = TIM_CHANNEL_1;
	Friction_R.timepwm2 = htim4;
	Friction_R.channelpwm2 = TIM_CHANNEL_2;
	Friction_R.capturetim = htim10;
	
	Friction_L.encoder = htim3;
	Friction_L.timepwm1 = htim4;
	Friction_L.channelpwm1 = TIM_CHANNEL_3;
	Friction_L.timepwm2 = htim4;
	Friction_L.channelpwm2 = TIM_CHANNEL_4;
	Friction_L.capturetim = htim10;
*/
}

void MOTOR_Tim_Init(MOTOR_TypeDef *motor){	
	set_pid(&motor->k_speed, 20, 5, 0);
	set_pid(&motor->k_angle, 50, 0.5, 0);
	set_pid(&motor->k_double, 1.2, 0.005, 25);
	
	__HAL_TIM_CLEAR_IT(&motor->encoder,TIM_IT_UPDATE);	//����encoder�жϱ�־λ
	__HAL_TIM_SetCounter(&motor->encoder,0);			//����encoder����ֵ
	HAL_TIM_Base_Start_IT(&motor->encoder);			//����encoder�����ж�

	HAL_TIM_Encoder_Start(&motor->encoder,TIM_CHANNEL_1);	//����encoder���벶��ͨ��
	HAL_TIM_Encoder_Start(&motor->encoder,TIM_CHANNEL_2);	//����encoder���벶��ͨ��
	
	HAL_TIM_PWM_Start(&motor->timepwm1,motor->channelpwm1);	//����pwm���
	HAL_TIM_PWM_Start(&motor->timepwm2,motor->channelpwm2);	//����pwm���
	
	__HAL_TIM_CLEAR_IT(&motor->capturetim,TIM_IT_UPDATE);	//����tim6�жϱ�־λ(��ʱ�жϻ�ȡ���״̬��������
	__HAL_TIM_SetCounter(&motor->capturetim,0);				//����tim6����ֵ
	HAL_TIM_Base_Start_IT(&motor->capturetim);				//����tim6�����ж�	
}

void MOTOR_Tim_Stop(MOTOR_TypeDef *motor){			//������������رն�ʱ��6��Ҳ����ζ��ʵ�������е��ͣ��
	motor->pwm = 0;
	set_motor_pwm(motor);
//	memset(motor,0,sizeof(Bopan));
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
	
	HAL_TIM_Base_Stop_IT(&motor->capturetim);				//�ر�tim6�����ж�	
//	__HAL_TIM_CLEAR_IT(&motor->capturetim,TIM_IT_UPDATE);	//����tim6�жϱ�־λ(��ʱ�жϻ�ȡ���״̬��������
//	__HAL_TIM_SetCounter(&motor->capturetim,0);				//����tim6����ֵ
	
	HAL_TIM_Base_Stop_IT(&motor->encoder);			//�ر�encoder�����ж�
//	__HAL_TIM_CLEAR_IT(&motor->encoder,TIM_IT_UPDATE);	//����encoder�жϱ�־λ
//	__HAL_TIM_SetCounter(&motor->encoder,0);			//����encoder����ֵ
	
//	HAL_TIM_PWM_Stop(&motor->timepwm1,motor->channelpwm1);	//�ر�pwm���
}




float get_speed(float ns, MOTOR_TypeDef *motor){		//����
	
	motor->capture_count = __HAL_TIM_GetCounter(&motor->encoder) + (motor->encoder_overflow*0xffff);	//��ǰ����ֵ = �����������ֵ + �������*������������װ��ֵ

	motor->actual_speed = (float) (motor->capture_count - motor->last_count) / (beipin*xianshu*jiansubi*ns); //ʵ��ת�� = ����ǰ����ֵ - ǰһ��ʱ�䲶��ֵ�� / ����Ƶ��*����*���ʱ�䣩
	
	motor->last_count = motor->capture_count;
	
	return motor->actual_speed;
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

void limit_pwm(MOTOR_TypeDef *motor){
	if(motor->k_angle.integral <-(PWM_MAX/6) )motor->k_angle.integral = -(PWM_MAX/6);//����λ��ʽpid�еĻ��������
	if(motor->k_angle.integral > (PWM_MAX/6) )motor->k_angle.integral = (PWM_MAX/6);
	
	if(motor->pwm <-(PWM_MAX/2) )motor->pwm = -(PWM_MAX/2);
	if(motor->pwm > (PWM_MAX/2) )motor->pwm = (PWM_MAX/2);
}

void limit_speed(MOTOR_TypeDef *motor){
	if(motor->target_speed > 0.5)motor->target_speed = 0.5;
	if(motor->target_speed < -0.5)motor->target_speed = -0.5;
}

void check_pwm_speed(MOTOR_TypeDef *motor){		// v = pwmռ�ձ� * vmax
	float f = (motor->k_speed.fi / PWM_MAX) * SPEED_MAX;
	float v = motor->target_speed;
	
	if( -1.5f < f-v && f-v < 1.5f )return;		//���������Χ��
	else motor->pwm = motor->k_speed.fi = ( v / SPEED_MAX ) * PWM_MAX; //ͬʱ�ı�pwm��pid�ۼ���fi
}

void feedback_angle(MOTOR_TypeDef *motor){
	motor->actual_angle = get_angle(motor);													//��ȡʵ�ʽǶ�
	motor->pwm = (int)PID_position(motor->target_angle, motor->actual_angle, PWM_MAX, &motor->k_angle);	//pid�����Ҫ�����pwm
	limit_pwm(motor);
	set_motor_pwm(motor);																	//�������
}

void feedback_speed(MOTOR_TypeDef *motor){
	motor->actual_speed = get_speed(0.01,motor); 		//printf("actual speed=%f\r\n",motor->actual_speed);														//��ȡʵ���ٶ�
	motor->pwm = (int)PID_incremental(motor->target_speed*jiansubi, motor->actual_speed*jiansubi,  PWM_MAX, &motor->k_speed); 	//pid�����Ҫ�����pwm
	//check_pwm_speed(&Back_L);																					//���pwmֵ�Ƿ�ƫ�����
	set_motor_pwm(motor);																					 	//�������
}

void feedback_angle_double(MOTOR_TypeDef *motor){ //˫��pid���õ��������״̬��ת��ָ���Ƕ�
	motor->actual_angle = get_angle(motor);	
	
	motor->target_speed = PID_position(motor->target_angle, motor->actual_angle, SPEED_MAX*jiansubi, &motor->k_double)/jiansubi; //λ�û�pid�����Ҫ���ٶ�
	limit_speed(&Bopan);
	
	motor->actual_speed = (float) (motor->capture_count - motor->last_count) / (beipin*xianshu*jiansubi*0.01); //ʵ��ת�� 
	motor->last_count = motor->capture_count;
	
	motor->pwm = (int)PID_incremental(motor->target_speed*jiansubi, motor->actual_speed*jiansubi, PWM_MAX, &motor->k_speed); //�ٶȻ�pid���Ӧ�����pwm
		
	limit_pwm(&Bopan);
	set_motor_pwm(motor);																	//�������
}

