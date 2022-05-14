#include "control.h"
#include "motor.h"
#include "delay.h"
#include "usart.h"
#include "steer.h"
#include "vision.h"

/**************核心控制代码****************/



const float a=0.09, b=0.11;  //车长2a = 0.18m，车宽 2b = 0.22m	

/**************PS2控制代码****************/


#include "ps2.h"

//float V[50+1]={0};	//小车近n+1次前进速率, 及近n次平均值
//float W[50+1]={0};	//小车近n+1次转弯量,  及近n次平均值
float v=0;				//小车速度
float w=0;				//小车转速
float geer=3;			//小车速度挡位，共4挡，对应最高速度1、2、3、4r/s，默认第三档
u8 i=0, n=15;			//n与速度挡位有关		


void ps2_angle(void){		//调方向模式，打靶
	
	i8 x = PS2_AnologData(PSS_LX) - 128; //摇杆x、y轴坐标
	i8 y = PS2_AnologData(PSS_LY) - 127; 
	
	float xx = 70*(float)x/128.0f;
	float yy = 5*(float)y/128.0f;
	
	Forward_L.target_angle += xx;
	Forward_R.target_angle -= xx;
	Back_L.target_angle += xx,	
	Back_R.target_angle -= xx;
	
	if(0<=Turret.angle+yy&&Turret.angle+yy<=48){
		Turret.angle += yy;
	}
	
//	delay_ms(100);
	
/*	if(order==PSB_PAD_UP)Turret.angle+=10;
	
	if(order==PSB_PAD_DOWN)Turret.angle-=10;
	
		if(order==PSB_PAD_RIGHT){			//右转几度
		Forward_L.target_angle += 10;
		Forward_R.target_angle -= 10;
		Back_L.target_angle += 10,	
		Back_R.target_angle -= 10;
		delay_ms(500);	//防止极短时间内连按
	}
	
	if(order==PSB_PAD_LEFT){			//左转几度
		Forward_L.target_angle -= 10;
		Forward_R.target_angle += 10;
		Back_L.target_angle -= 10,
		Back_R.target_angle += 10;
		delay_ms(500);	//防止极短时间内连按
	}
*/
}


void ps2_speed(void){		//正常前进模式
	
	i8 x = PS2_AnologData(PSS_RX) - 128; //摇杆x、y轴坐标
	i8 y = 127 - PS2_AnologData(PSS_RY);
	i8 w = PS2_AnologData(PSS_LX) - 128; //左摇杆x轴坐标
	i8 z = PS2_AnologData(PSS_LY) - 127; 


/*	n = geer*5	; //if(n==5)n=2;
	
	for(i=n;i>0;i--)V[i]=V[i-1], W[i]=W[i-1];//更新近n次计算值 
		  
	V[0] = geer*(float)y/128.0f; //摇杆y坐标映射为小车前进速度，速度范围-geer ~ geer
											 
	W[0] = (geer/3)*(float)x/128.0f; //摇杆x坐标映射为小车转弯量
	
	v += (V[0]-V[n])/(float)n;   w += (W[0]-W[n])/(float)n;											 
	//以近n次速度与转弯量平均值作为小车新一轮前进速度与转弯量，抑制小车速度突变
	//（这样做一是为了模拟真实开车，二是小车速度突变为0会导致ps2手柄断联）		
//	printf("v = %f\r\n w = %f\r\n",v,w);
*/	

	float vx = 2.5 * (float)x / 128;	//x、y坐标映射为小车x、y方向分速度（最大2m/s）
	float vy = 2.5 * (float)y / 128;
	float ww = 4 * (float)w / 128;		//w坐标映射为小车转速（最大3rad/s）
	z = 5*(float)z/128.0f;
	
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
	
	if(control_mode)ps2_angle();
	else ps2_speed();
	
	switch(order){
		
		case PSB_START : {	//reset
			while(PS2_DataKey()==PSB_START)delay_ms(50);//等待按键松开，限制按下并松开为一次有效按键
			HAL_NVIC_SystemReset();	//强制复位程序
		}
		
		case PSB_PAD_DOWN : {	//前爪降，后爪归位
			while(PS2_DataKey()==PSB_PAD_DOWN&&t<3)t++,delay_ms(50);
			if(Steer_f.angle < 80){
				Steer_f.angle += 10, Steer_b.angle = 0; 
				set_steer_pwm(&Steer_f);
				set_steer_pwm(&Steer_b);
			}
			break;
		}
		
		case PSB_PAD_UP : {		//后爪降，前爪归位
			while(PS2_DataKey()==PSB_PAD_UP&&t<3)t++,delay_ms(50);
			if(Steer_b.angle < 80){
				Steer_f.angle = 0, Steer_b.angle += 10;
				set_steer_pwm(&Steer_f);
				set_steer_pwm(&Steer_b);
			}
			break;
		}
		
		case PSB_PAD_LEFT : {	//前后爪归位
			while(PS2_DataKey()==PSB_PAD_LEFT&&t<3)t++,delay_ms(50);
			Steer_f.angle = 0, Steer_b.angle = 0;
			set_steer_pwm(&Steer_f);
			set_steer_pwm(&Steer_b);
		}
		
		case PSB_PAD_RIGHT : {	//全自动爬梯
			while(PS2_DataKey()==PSB_PAD_RIGHT)delay_ms(50);//等待按键松开，限制按下并松开为一次有效按键
			
			MOTOR_reset(),control_mode=1;	delay_ms(500);
			
			Steer_f.angle = 80;	 
			delay_ms(3000);	
			Back_L.target_angle = 400; 
			Back_R.target_angle = 400;
			delay_ms(3000);	
			Steer_f.angle =0;
			Steer_b.angle = 80;
			delay_ms(3000);
			Forward_L.target_angle = 1080; 
			Forward_R.target_angle = 1080;
			delay_ms(3000);
	
			MOTOR_reset(),control_mode=0;	delay_ms(500);
			
			break;
		}
		
		case PSB_L1 : {		//加档，小车速度挡+1
			while(PS2_DataKey()==PSB_L1)delay_ms(50);	//等待按键松开，限制按一次加一档
			if(geer<4){
				geer+=1;	
	//			for(i=n;i>0;i--)V[i]=W[i]=0;
			}
			break;
		}
		case PSB_L2 : {		//减档，小车速度挡-1
			while(PS2_DataKey()==PSB_L2)delay_ms(50);	//等待按键松开，限制按一次加一档
			if(geer>1){
				geer-=1;	
	//			for(i=n;i>0;i--)V[i]=W[i]=0;
			}
			break;
		}
		
		case PSB_R1 : {		//给炮台主控单片机发送 单发一颗子弹 的指令
			while(PS2_DataKey()==PSB_R1)delay_ms(50);	//等待按键松开，限制按一次发一颗子弹
			receiver3 = 'o';
			HAL_UART_Transmit(&huart3,(u8 *)&receiver3,1,1);
			break;
		}
		case PSB_R2 : {		//给炮台主控单片机发送 连发五颗子弹 的指令
			while(PS2_DataKey()==PSB_R2)delay_ms(50);	//等待按键松开，限制按一次发五颗子弹
			receiver3 = 'w';
			HAL_UART_Transmit(&huart3,(u8 *)&receiver3,1,1);
			break;
		}
		
		case PSB_PINK : {	//向树莓派发送“符文模式”指令
			while(PS2_DataKey()==PSB_PINK)delay_ms(50);
			MOTOR_reset();
			receiver2 = '%';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			control_mode=1;
			break;
		}
			
		case PSB_GREEN : {	//向树莓派发送“打靶模式”指令
			while(PS2_DataKey()==PSB_GREEN)delay_ms(50);
			control_mode=1;
			MOTOR_reset();
			receiver2 = '&';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			break;
		}
		
		case PSB_BLUE : {	//向树莓派发送“机动模式”指令
			while(PS2_DataKey()==PSB_BLUE)delay_ms(50);
			control_mode=0;
			MOTOR_reset(); 
			receiver2 = '~';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
		//	HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			break;
		}
		
		case PSB_RED : {	//停下摩擦轮电机
			while(PS2_DataKey()==PSB_RED)delay_ms(50); 
			receiver3 = 'n';
			HAL_UART_Transmit(&huart3,(u8 *)&receiver3,1,1);
			break;
		}
		
	}
}





/*****************蓝牙控制代码****************/


const float WSPEED =1;;
u8 order_last='Z';

static float v_x = 0;
static float v_y = 0;
static float w_w = 0;
static int dirx=0;
static int diry=0;
static int dirw=0;
static float mul=1;


void stop(void)         //停止
{
	Forward_L.target_speed=0;
	Forward_R.target_speed=0;
	Back_L.target_speed=0;
	Back_R.target_speed=0;
	delay_ms(100);	
}

void direction_change(void)
{

	v_x = 1.2 * (float)dirx / 128;	//x、y坐标映射为小车x、y方向分速度（最大0.6m/s）
	v_y = 1.2 * (float)diry / 128;
  w_w = 3 * (float)dirw / 128;		//w坐标映射为小车转速（最大6rad/s）
	Forward_R.target_speed = mul*(v_y - v_x) + w_w*(a+b);
	Forward_L.target_speed = mul*(v_y + v_x) - w_w*(a+b);
	Back_L.target_speed = mul*(v_y - v_x) - w_w*(a+b);
	Back_R.target_speed = mul*(v_y + v_x) + w_w*(a+b);
}

void forward(void)      //前进
{
	if(order_last== 'Z'){mul=1;}
	dirx =0;
	diry =127;
	dirw =0;
	direction_change();
	delay_ms(100);	
}
void back(void)         //后退
{
	if(order_last== 'Z'){mul=1;}
	dirx =0;
	diry =-128;
	dirw =0;
	direction_change();
	delay_ms(100);	
}

void right(void)        //右转
{
	if(order_last== 'Z'){mul=1;}
	dirx =127;
	diry =0;
	dirw =0;
	direction_change();
	delay_ms(100);	
}
void left(void)         //左转
{
	if(order_last== 'Z'){mul=1;}
	dirx =-128;
	diry =0;
	dirw =0;
	direction_change();
	delay_ms(100);	
}

void fright(void)       //前右
{
	if(order_last== 'Z'){mul=1;}
	dirx =127;
	diry =127;
	dirw =0;
	direction_change();
	delay_ms(100);	
}
void bright(void)       //后右
{
	if(order_last== 'Z'){mul=1;}
	dirx =127;
	diry =-128;
	dirw =0;
	direction_change();
	delay_ms(100);	
}
void fleft(void)        //前左
{
	if(order_last== 'Z'){mul=1;}
	dirx =-128;
	diry =127;
	dirw =0;
	direction_change();
	delay_ms(100);	
}
void bleft(void)        //后左
{
	if(order_last== 'Z'){mul=1;}
	dirx =-128;
	diry =-128;
	dirw =0;
	direction_change();
	delay_ms(100);	
}

void speed_up(void)     //加速
{
	mul+=0.5;
}
	
void speed_down(void)   //减速
{
	if(mul==0)return;
	mul-=0.5;
}

void turn_left(void)    //左旋
{
	dirx =0;
	diry =0;
	dirw =-128;
	direction_change();
	delay_ms(100);	
}

void turn_right(void)   //右旋
{
	dirx =0;
	diry =0;
	dirw =127;
	direction_change();
	delay_ms(100);	
}
	
void Steer_f_up(void)  //升前爪
{
	Steer_f.angle = 0;
}
void Steer_f_down(void) //降前爪
{
	if(Steer_f.angle<80)Steer_f.angle +=10;
}
void Steer_b_up(void)   //升后爪
{
	Steer_b.angle = 0;
}
void Steer_b_down(void) //降后爪
{
	if(Steer_b.angle<80)Steer_b.angle += 10;
}

void Turret_up(void)   //炮台上仰
{
	if(Turret.angle < 48)Turret.angle += 2;	//相对于车底成30°角，再大就不能保证子弹射出
}
void Turret_down(void) //炮台下调
{
	if(Turret.angle > 0)Turret.angle -= 2;
}

void bluetooth_control(u8 order)     
{
	switch(order){
		case 'A':	forward(); 	break;	//正前		        
		case 'B':	fright();  	break;	//前右 									        
		case 'C':	right();   	break;	//向右				        	        
		case 'D': bright();  	break;	//后右            				        
		case 'E':	back();    	break;  //正后            				        
		case 'F':	bleft();  	break;  //后左            				        
		case 'G':	left();    	break;  //向左            				        
		case 'H':	fleft();   	break;  //前左            				        
		case 'Z':	stop();    	break;	//停止           				        
		case 'X':	speed_up();	break;	//加速            				        
		case 'Y':	speed_down();break;	//减速   

		case 'b': Steer_f_up();		break;	//升前爪
		case 'c': Steer_b_up();		break;  //升后爪
		case 'e': Steer_f_down();	break;	//降前爪
		case 'f': Steer_b_down();	break;	//降后爪
		//云台舵机
		case 'h': Turret_up();    break;  //炮塔上仰
		case 'i': Turret_down();  break;  //炮塔下调	
		
		case 'd': Steer_f_up(); Steer_b_up(); break;
		case 'g': Steer_b_down(); delay_ms(20); Steer_f_up();break;
		
		case 'm': HAL_UART_Transmit(&huart3,(u8*)&order,1,1);break; //向炮台主控单片机发送指令
		case 'n': HAL_UART_Transmit(&huart3,(u8*)&order,1,1);break;
		case 'o': HAL_UART_Transmit(&huart3,(u8*)&order,1,1);break;
		case 'w': HAL_UART_Transmit(&huart3,(u8*)&order,1,1);break;
		case 'k': HAL_UART_Transmit(&huart3,(u8*)&order,1,1);break;
			
		case '#': ps2_mode = 1;	break;//改蓝牙控制为ps2手柄控制
		case '$': ps2_mode = 0;	break;								 //改ps2控制为蓝牙控制
		
		case '~' : {	//机动模式
			MOTOR_reset();
			receiver2 = '~';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
//			HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			control_mode=0;
			break;
		}; 		
		case '%' : {	//符文模式
			MOTOR_reset(); 
			receiver2 = '%';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			control_mode=1;
			break;
		}
		case '&' : {	//打靶模式
			MOTOR_reset(); 
			receiver2 = '&';
			HAL_UART_Transmit(&huart2,(uint8_t *)&receiver2,1,1);
			HAL_UART_Receive_IT(&huart2,(uint8_t *)&receiver2,1);
			control_mode=1;
			break;
		}
		
			
		case '!' : HAL_NVIC_SystemReset();	break;	//强制复位程序
		
		case '@' : {
			printf("debug mode start\r\n");
			printf("please select one motor\r\n");
			scanf("%s",rx1_buffer);
			if(rx1_buffer[0]=='f'&&rx1_buffer[1]=='l'){	//前左电机
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
			if(rx1_buffer[0]=='f'&&rx1_buffer[1]=='r'){	//前右电机
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
			if(rx1_buffer[0]=='b'&&rx1_buffer[1]=='l'){	//后左电机
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
			if(rx1_buffer[0]=='b'&&rx1_buffer[1]=='r'){	//后右电机
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
			if(rx1_buffer[0]=='s'&&rx1_buffer[1]=='f'){	//前爪舵机
				printf("input target angle\r\n");
				scanf("%d",&Steer_f.angle);
			}
			if(rx1_buffer[0]=='s'&&rx1_buffer[1]=='b'){	//后爪舵机
				printf("input target angle\r\n");
				scanf("%d",&Steer_b.angle);
			}
			if(rx1_buffer[0]=='s'&&rx1_buffer[1]=='t'){	//云台俯仰舵机
				printf("input target angle\r\n");
				scanf("%d",&Turret.angle);
			}			
			break;
		}
	}
	
	order_last = order;

}
