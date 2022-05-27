#include "control.h"
#include "motor.h"
#include "delay.h"
#include "usart.h"
#include "steer.h"
#include "vision.h"

/**************���Ŀ��ƴ���****************/



const float a=0.18, b=0.25;  //����2a = 0.18m������ 2b = 0.25m	

/**************PS2���ƴ���****************/


#include "ps2.h"

//float V[50+1]={0};	//С����n+1��ǰ������, ����n��ƽ��ֵ
//float W[50+1]={0};	//С����n+1��ת����,  ����n��ƽ��ֵ
float v=0;				//С���ٶ�
float w=0;				//С��ת��
float geer=3;			//С���ٶȵ�λ����4������Ӧ����ٶ�1��2��3��4r/s��Ĭ�ϵ�����
u8 i=0, n=15;			//n���ٶȵ�λ�й�		

void auto_step(void)
{
	stop();
	Steer_f.angle = 110; //��ǰצ
	set_steer_pwm(&Steer_f);
	delay_ms(500);	
	
	Forward_L.target_speed = 0.5;
	Forward_R.target_speed = 0.5;
	Back_L.target_speed = 0.5;
	Back_R.target_speed = 0.5;
	delay_ms(1500);
	
	stop();
	Steer_f.angle =0;			//��ǰצ
	Steer_b.angle = 125;          //����צ
	set_steer_pwm(&Steer_f);
	set_steer_pwm(&Steer_b);
	delay_ms(1500);
	
	Forward_L.target_speed = 3;
	Forward_R.target_speed = 3;
	Back_L.target_speed = 3;
	Back_R.target_speed = 3;
	delay_ms(800);
	 
	stop();
	Steer_b.angle = 0;            //����צ
	set_steer_pwm(&Steer_b);
	delay_ms(500);
}

void ps2_angle(void){		//������ģʽ�����
	
	i16 x = PS2_AnologData(PSS_RX) - 128; //��ҡ��x��y������
	i16 y = 127 - PS2_AnologData(PSS_RY);
	i16 w = PS2_AnologData(PSS_LX) - 128; //��ҡ��x������
	i16 z = PS2_AnologData(PSS_LY);
	if(x>-20&&x<20) x=0;	//��������
	if(y>-20&&y<20) y=0;
	if(w>-40&&w<40) w=0;
	if(z<30)z=-2; else if(z>220)z=2; else z=0;
	
	float vx = 2.5 * (float)x / 128;	//x��y����ӳ��ΪС��x��y������ٶȣ����2m/s��
	float vy = 2.5 * (float)y / 128;
	float ww = 4 * (float)w / 128;		//w����ӳ��ΪС��ת�٣����3rad/s��
	
//	printf("vx= %f\r\n vy= %f\r\n ww= %f\r\n",vx,vy,ww);
	
	Forward_R.target_speed = vy - vx - ww*(a+b);
	Forward_L.target_speed = vy + vx + ww*(a+b);
	Back_L.target_speed = vy - vx + ww*(a+b);
	Back_R.target_speed = vy + vx - ww*(a+b);
	
//	delay_ms(100);
	
/*	if(order==PSB_PAD_UP)Turret.angle+=10;
	
	һ��ǰ���ٶ���ת����������С���ٶ�ͻ��
	//��������һ��Ϊ��ģ����ʵ����������С���ٶ�ͻ��Ϊ0�ᵼ��ps2�ֱ�������		
//	printf("v = %f\r\n w = %f\r\n",v,w);
*/		
	
	if(0<=Turret.angle+z&&Turret.angle+z<=48){
		Turret.angle += z;
	}
}

void ps2_speed(void){		//����ǰ��ģʽ	
	
	i16 x = PS2_AnologData(PSS_RX) - 128; //��ҡ��x��y������
	i16 y = 127 - PS2_AnologData(PSS_RY);
	i16 w = PS2_AnologData(PSS_LX) - 128; //��ҡ��x������
	i16 z = PS2_AnologData(PSS_LY);
	
	if(x>-40&&x<40) x=0;	//��������
	if(y>-40&&y<40) y=0;
	if(w>-40&&w<40) w=0;
	if(z<30)z=-2; else if(z>220)z=2; else z=0;
	
	float vx = 2.5 * (float)x / 128;	//x��y����ӳ��ΪС��x��y������ٶȣ����2m/s��
	float vy = 2.5 * (float)y / 128;
	float ww = 4 * (float)w / 128;		//w����ӳ��ΪС��ת�٣����3rad/s��
	
//	printf("vx= %f\r\n vy= %f\r\n ww= %f\r\n",vx,vy,ww);
	
	Forward_R.target_speed = vy - vx - ww*(a+b);
	Forward_L.target_speed = vy + vx + ww*(a+b);
	Back_L.target_speed = vy - vx + ww*(a+b);
	Back_R.target_speed = vy + vx - ww*(a+b);
	
	if(0<=Turret.angle+z&&Turret.angle+z<=48){
		Turret.angle += z;
	}
}


void ps2_control(u8 order){
	u8 t=0;
	if(!order)return;
	
	switch(order){
		
		case PSB_START : {	//reset
			while(PS2_DataKey()==PSB_START)delay_ms(50);//�ȴ������ɿ������ư��²��ɿ�Ϊһ����Ч����
			HAL_NVIC_SystemReset();	//ǿ�Ƹ�λ����
		}
		
		case PSB_PAD_UP : {	//ǰצ������צ��λ
//			while(PS2_DataKey()==PSB_PAD_UP&&t<3)t++,delay_ms(10);
			if(Steer_f.angle < 110){
				//Steer_f.angle += 10, Steer_b.angle = 0; 
				Steer_f.angle = 110; Steer_b.angle = 0;
				set_steer_pwm(&Steer_f);
				set_steer_pwm(&Steer_b);
			}
			break;
		}
		
		case PSB_PAD_DOWN : {		//��צ����ǰצ��λ
//			while(PS2_DataKey()==PSB_PAD_DOWN&&t<3)t++,delay_ms(10);
			if(Steer_b.angle < 125){
				//Steer_f.angle = 0, Steer_b.angle += 10;
				Steer_f.angle = 0; Steer_b.angle = 125;
				set_steer_pwm(&Steer_f);
				set_steer_pwm(&Steer_b);
			}
			break;
		}
		
		case PSB_PAD_LEFT : {	//ǰ��צ��λ
//			while(PS2_DataKey()==PSB_PAD_LEFT&&t<3)t++,delay_ms(10);
			Steer_f.angle = 0, Steer_b.angle = 0;
			set_steer_pwm(&Steer_f);
			set_steer_pwm(&Steer_b);
		}
		
		case PSB_PAD_RIGHT : {	//ȫ�Զ�����
//			while(PS2_DataKey()==PSB_PAD_RIGHT&&t<3)t++,delay_ms(10);//�ȴ������ɿ������ư��²��ɿ�Ϊһ����Ч����
			auto_step();			
			break;
		}
		
		case PSB_L1 : {		//�ӵ���С���ٶȵ�+1
//			while(PS2_DataKey()==PSB_L1&&t<3)t++,delay_ms(10);	//�ȴ������ɿ������ư�һ�μ�һ��
			if(geer<4){
				geer+=1;	
	//			for(i=n;i>0;i--)V[i]=W[i]=0;
			}
			break;
		}
		case PSB_L2 : {		//������С���ٶȵ�-1
//			while(PS2_DataKey()==PSB_L2&&t<3)t++,delay_ms(10);	//�ȴ������ɿ������ư�һ�μ�һ��
			if(geer>1){
				geer-=1;	
	//			for(i=n;i>0;i--)V[i]=W[i]=0;
			}
			break;
		}
		
		case PSB_R1 : {		//����̨���ص�Ƭ������ ����һ���ӵ� ��ָ��
//			while(PS2_DataKey()==PSB_R1&&t<3)t++,delay_ms(10);	//�ȴ������ɿ������ư�һ�η�һ���ӵ�
			receiver3 = 'o';
			HAL_UART_Transmit(&huart3,(u8 *)&receiver3,1,1);
			break;
		}
		case PSB_R2 : {		//����̨���ص�Ƭ������ ��������ӵ� ��ָ��
//			while(PS2_DataKey()==PSB_R2&&t<3)t++,delay_ms(10);	//�ȴ������ɿ������ư�һ�η�����ӵ�
			receiver3 = 'w';
			HAL_UART_Transmit(&huart3,(u8 *)&receiver3,1,1);
			break;
		}
		
		case PSB_PINK : {	//����ݮ�ɷ��͡�����ģʽ��ָ��
//			while(PS2_DataKey()==PSB_PINK&&t<3)t++,delay_ms(10);
			MOTOR_reset();
			receiver2 = '%';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			control_mode=1;
			break;
		}
			
		case PSB_GREEN : {	//����ݮ�ɷ��͡����ģʽ��ָ��
//			while(PS2_DataKey()==PSB_GREEN&&t<3)t++,delay_ms(10);
			control_mode=1;
			MOTOR_reset();
			receiver2 = '&';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			break;
		}
		
		case PSB_BLUE : {	//����ݮ�ɷ��͡�����ģʽ��ָ��
//			while(PS2_DataKey()==PSB_BLUE&&t<3)t++,delay_ms(10);
			control_mode=0;
			MOTOR_reset(); 
			receiver2 = '~';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
//			HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			break;
		}
		
		case PSB_RED : {	//ͣ��Ħ���ֵ��
//			while(PS2_DataKey()==PSB_RED&&t<3)t++,delay_ms(10); 
			receiver3 = 'n';
			HAL_UART_Transmit(&huart3,(u8 *)&receiver3,1,1);
			break;
		}
		
	}
}





/*****************�������ƴ���****************/


const float WSPEED =1;;
u8 order_last='Z';

static float v_x = 0;
static float v_y = 0;
static float w_w = 0;
static int dirx=0;
static int diry=0;
static int dirw=0;
static float mul=1;


void stop(void)         //ֹͣ
{
	Forward_L.target_speed=0;
	Forward_R.target_speed=0;
	Back_L.target_speed=0;
	Back_R.target_speed=0;
	delay_ms(100);	
}

void direction_change(void)
{

	v_x = 1.2f * (float)dirx / 128;	//x��y����ӳ��ΪС��x��y������ٶȣ����0.6m/s��
	v_y = 1.2f * (float)diry / 128;
    w_w = 3.0f * (float)dirw / 128;		//w����ӳ��ΪС��ת�٣����6rad/s��
	Forward_R.target_speed = mul*(v_y - v_x) + w_w*(a+b);
	Forward_L.target_speed = mul*(v_y + v_x) - w_w*(a+b);
	Back_L.target_speed = mul*(v_y - v_x) - w_w*(a+b);
	Back_R.target_speed = mul*(v_y + v_x) + w_w*(a+b);
}

void forward(void)      //ǰ��
{
	if(order_last== 'Z'){mul=1;}
	dirx =0;
	diry =127;
	dirw =0;
	direction_change();
	delay_ms(100);	
}
void back(void)         //����
{
	if(order_last== 'Z'){mul=1;}
	dirx =0;
	diry =-128;
	dirw =0;
	direction_change();
	delay_ms(100);	
}

void right(void)        //��ת
{
	if(order_last== 'Z'){mul=1;}
	dirx =127;
	diry =0;
	dirw =0;
	direction_change();
	delay_ms(100);	
}
void left(void)         //��ת
{
	if(order_last== 'Z'){mul=1;}
	dirx =-128;
	diry =0;
	dirw =0;
	direction_change();
	delay_ms(100);	
}

void fright(void)       //ǰ��
{
	if(order_last== 'Z'){mul=1;}
	dirx =127;
	diry =127;
	dirw =0;
	direction_change();
	delay_ms(100);	
}
void bright(void)       //����
{
	if(order_last== 'Z'){mul=1;}
	dirx =127;
	diry =-128;
	dirw =0;
	direction_change();
	delay_ms(100);	
}
void fleft(void)        //ǰ��
{
	if(order_last== 'Z'){mul=1;}
	dirx =-128;
	diry =127;
	dirw =0;
	direction_change();
	delay_ms(100);	
}
void bleft(void)        //����
{
	if(order_last== 'Z'){mul=1;}
	dirx =-128;
	diry =-128;
	dirw =0;
	direction_change();
	delay_ms(100);	
}

void speed_up(void)     //����
{
	mul+=0.5f;
}
	
void speed_down(void)   //����
{
	if(mul==0)return;
	mul-=0.5f;
}

void turn_left(void)    //����
{
	dirx =0;
	diry =0;
	dirw =-128;
	direction_change();
	delay_ms(100);	
}

void turn_right(void)   //����
{
	dirx =0;
	diry =0;
	dirw =127;
	direction_change();
	delay_ms(100);	
}
	
void Steer_f_up(void)  //��ǰצ
{
	Steer_f.angle = 0; set_steer_pwm(&Steer_f);
}
void Steer_f_down(void) //��ǰצ
{
	Steer_f.angle = 110; 
	set_steer_pwm(&Steer_f);
	Steer_b_up();
}
void Steer_b_up(void)   //����צ
{
	Steer_b.angle = 0 ;
	set_steer_pwm(&Steer_b);
}
void Steer_b_down(void) //����צ
{
	Steer_b.angle = 125; 
	set_steer_pwm(&Steer_b);
	Steer_f_up();
}

void Turret_up(void)   //��̨����
{
	if(Turret.angle < 48)Turret.angle += 2;	//����ڳ��׳�30��ǣ��ٴ�Ͳ��ܱ�֤�ӵ����
}
void Turret_down(void) //��̨�µ�
{
	if(Turret.angle > 0)Turret.angle -= 2;
}

void bluetooth_control(u8 order)     
{
	switch(order){
		case 'A':	forward(); 	 break;	//��ǰ		        
		case 'B':	fright();    break;	//ǰ�� 									        
		case 'C':	right();   	 break;	//����				        	        
		case 'D':   bright();  	 break;	//����            				        
		case 'E':	back();    	 break;  //����            				        
		case 'F':	bleft();  	 break;  //����            				        
		case 'G':	left();    	 break;  //����            				        
		case 'H':	fleft();     break;  //ǰ��            				        
		case 'Z':	stop();    	 break;	//ֹͣ           				        
		case 'X':	turn_left(); break;	//����           				        
		case 'Y':	turn_right();break;	//����  

		case 'b': Steer_f_up();		break;	//��ǰצ
		case 'c': Steer_b_up();		break;  //����צ
		case 'e': Steer_f_down();	break;	//��ǰצ
		case 'f': Steer_b_down();	break;	//����צ
		case 't': auto_step();		break;  //�Զ�����
		//��̨���
		case 'h': Turret_up();    break;  //��������
		case 'i': Turret_down();  break;  //�����µ�	
		
		case 'd': Steer_f_up(); Steer_b_up(); break;
		case 'g': Steer_b_down(); delay_ms(20); Steer_f_up();break;
		
		case 'm': HAL_UART_Transmit(&huart3,(u8*)&order,1,1);break; //����̨���ص�Ƭ������ָ��
		case 'n': HAL_UART_Transmit(&huart3,(u8*)&order,1,1);break;
		case 'o': HAL_UART_Transmit(&huart3,(u8*)&order,1,1);break;
		case 'w': HAL_UART_Transmit(&huart3,(u8*)&order,1,1);break;
		case 'k': HAL_UART_Transmit(&huart3,(u8*)&order,1,1);break;
		
		case 'l': HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET); //����
		case 'r': HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET); //�ص�
			
		case '#': ps2_mode = 1;	break;//����������Ϊps2�ֱ�����
		case '$': ps2_mode = 0;	break;								 //��ps2����Ϊ��������
		
		case '~' : {	//����ģʽ
			MOTOR_reset();
			receiver2 = '~';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
//			HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			control_mode=0;
			break;
		}; 		
		case '%' : {	//����ģʽ
			MOTOR_reset(); 
			receiver2 = '%';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			control_mode=1;
			break;
		}
		case '&' : {	//���ģʽ
			MOTOR_reset(); 
			receiver2 = '&';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			control_mode=1;
			break;
		}
		
			
		case '!' : HAL_NVIC_SystemReset();	break;	//ǿ�Ƹ�λ����
		
		case '@' : {
			printf("debug mode start\r\n");
			printf("please select one motor\r\n");
			scanf("%s",rx1_buffer);
			if(rx1_buffer[0]=='f'&&rx1_buffer[1]=='l'){	//ǰ����
				printf("angle or speed?\r\n");
				scanf("%c",&receiver1);
				if(receiver1=='a'){
					printf("input target angle\r\n");
					MOTOR_reset(); 
					control_mode=1;
					scanf("%f",&Forward_L.target_angle);
				}else{
					printf("input target speed\r\n");
					MOTOR_reset(); 
					control_mode=0;
					scanf("%f",&Forward_L.target_speed);
				}
			}
			if(rx1_buffer[0]=='f'&&rx1_buffer[1]=='r'){	//ǰ�ҵ��
				printf("angle or speed?\r\n");
				scanf("%c",&receiver1);
				if(receiver1=='a'){
					printf("input target angle\r\n");
					MOTOR_reset(); 
					
					control_mode=1;
					scanf("%f",&Forward_R.target_angle);
				}else{
					printf("input target speed\r\n");
					MOTOR_reset(); 
					control_mode=0;
					scanf("%f",&Forward_R.target_speed);
				}
			}
			if(rx1_buffer[0]=='b'&&rx1_buffer[1]=='l'){	//������
				printf("angle or speed?\r\n");
				scanf("%c",&receiver1);
				if(receiver1=='a'){
					printf("input target angle\r\n");
					MOTOR_reset(); 
					control_mode=1;
					scanf("%f",&Back_L.target_angle);
				}else{
					printf("input target speed\r\n");
					MOTOR_reset(); 
					control_mode=0;
					scanf("%f",&Back_L.target_speed);
				}
			}
			if(rx1_buffer[0]=='b'&&rx1_buffer[1]=='r'){	//���ҵ��
				printf("angle or speed?\r\n");
				scanf("%c",&receiver1);
				if(receiver1=='a'){
					printf("input target angle\r\n");
					MOTOR_reset(); 
					control_mode=1;
					scanf("%f",&Back_R.target_angle);
				}else{
					printf("input target speed\r\n");
					MOTOR_reset(); 
					control_mode=0;
					scanf("%f",&Back_R.target_speed);
				}
			}
			if(rx1_buffer[0]=='s'&&rx1_buffer[1]=='f'){	//ǰצ���
				printf("input target angle\r\n");
				scanf("%f",&Steer_f.angle);
			}
			if(rx1_buffer[0]=='s'&&rx1_buffer[1]=='b'){	//��צ���
				printf("input target angle\r\n");
				scanf("%f",&Steer_b.angle);
			}
			if(rx1_buffer[0]=='s'&&rx1_buffer[1]=='t'){	//��̨�������
				printf("input target angle\r\n");
				scanf("%f",&Turret.angle);
			}			
			break;
		}
	}
	
	order_last = order;

}
