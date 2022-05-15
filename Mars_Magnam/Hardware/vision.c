#include "vision.h"
#include "usart.h"
#include "dma.h"
#include "motor.h"
#include "Steer.h"

int x=0,y=0; const int m=10;

float xp[50],yp[50];
float xx=0.0f,yy=0.0f;


void getdata(){		//���ղ�ת����ݮ������ 
	HAL_UART_Receive(&huart2,rx2_buffer,10,10);	//����һ������֡
	
	x = (rx2_buffer[2]-'0')*10 + (rx2_buffer[3]-'0');//����ת��
	if(rx2_buffer[1]=='-')x=-x;
	y = (rx2_buffer[7]-'0')*10 + (rx2_buffer[8]-'0');
	if(rx2_buffer[6]=='-')y=-y;
	
	for(int t=0;t<10;t++)rx2_buffer[t]=0;	//������ݻ���
}


void vision_control(u8 order){
	static int t=0;  if(t>m)t=0; t++; int q=0;	
	
	if(order=='G')getdata(), y=-y;
	if(order=='N')x=0,y=0,printf("can not fine the target\r\n");
	
	if(x>-30&&x<30)xp[t] = x; else xp[t]=0;		//���˹���ֵ
	if(y>-15&&y<15)yp[t] = y;	else yp[t]=0;
	xx=0.0f,yy=0.0f;
//	xx=x; yy=y;
	while(q<m)xx += xp[q]/(float)m, yy += yp[q]/(float)m, q++;	//��ֵ����ʹ�ƶ�ƽ��
	xx *= 7; yy *= 0.5;
	
	Forward_L.target_angle += xx;
	Forward_R.target_angle -= xx;
	Back_L.target_angle += xx,	
	Back_R.target_angle -= xx;
	if(0<=Turret.angle+yy&&Turret.angle+yy<=50)Turret.angle += yy;
	
	printf("xx=%f yy=%f\r\n",xx,yy);
}
