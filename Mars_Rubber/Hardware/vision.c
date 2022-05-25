#include "vision.h"
#include "usart.h"
#include "dma.h"
#include "motor.h"
#include "Steer.h"

float x=0,y=0; 
const int m=10;
float xp[50],yp[50];
float xx=0.0f,yy=0.0f;


void getdata(){		//���ղ�ת����ݮ������ 
	HAL_UART_Receive(&huart2,rx2_buffer,12,12);	//����һ������֡
	
	x = (rx2_buffer[2]-'0')*10 + (rx2_buffer[3]-'0') + (float)(rx2_buffer[4]-'0')*0.1f;//����ת��
	if(rx2_buffer[1]=='-')x=-x;
	y = (rx2_buffer[8]-'0')*10 + (rx2_buffer[9]-'0') + (float)(rx2_buffer[10]-'0')*0.1f;
	if(rx2_buffer[7]=='-')y=-y;
	
	for(int t=0;t<12;t++)rx2_buffer[t]=0;	//������ݻ���
}


void vision_control(u8 order){
	static int t=0;  if(t>m)t=0; t++; int q=0;	
	
	if(order=='G')getdata();
	if(order=='N')x=0,y=0,printf("{#no target}$\r\n");
	if(order=='V')printf("{#lock target}$\r\n");
	
//	if(x>-60&&x<60)xp[t] = x; else xp[t]=0;		//���˹���ֵ
//	if(y>-15&&y<15)yp[t] = y; else yp[t]=0;
	
	
//	xx=0.0f,yy=0.0f;
//	while(q<m)xx += xp[q], yy += yp[q], q++;	//��ֵ����ʹ�ƶ�ƽ��
//	xx = xx*70/(float)m; yy = yy/(float)m;
	printf("{#x=%f y=%f}$\r\n",x,y);
	
	if(x<-60)x=-60;if(x>60)x=60;
	if(y<-5)y=-5;if(y>5)y=5;
	xx=x*70; yy=y;	
	

	Forward_L.target_angle += xx;
	Forward_R.target_angle += xx;
	Back_L.target_angle += xx,	
	Back_R.target_angle += xx;
	if(10<Turret.angle+yy&&Turret.angle+yy<=50)Turret.angle += yy;
}

