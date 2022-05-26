#include "vision.h"
#include "usart.h"
#include "dma.h"
#include "motor.h"
#include "Steer.h"

float x=0,y=0; 
const int m=20;
float xp[50],yp[50];
float xx=0.0f,yy=0.0f;


void getdata(){		//接收并转换树莓派数据 
	HAL_UART_Receive(&huart2,rx2_buffer,12,12);	//接收一组数据帧
	
	x = (rx2_buffer[2]-'0')*10 + (rx2_buffer[3]-'0') + (float)(rx2_buffer[4]-'0')*0.1f;//数据转换
	if(rx2_buffer[1]=='-')x=-x;
	y = (rx2_buffer[8]-'0')*10 + (rx2_buffer[9]-'0') + (float)(rx2_buffer[10]-'0')*0.1f;
	if(rx2_buffer[7]=='-')y=-y;
	
	for(int t=0;t<12;t++)rx2_buffer[t]=0;	//清空数据缓存
}


void vision_control(u8 order){
	static int t=0;  if(t>m)t=0; t++; int q=0;	
	
	if(order=='G')getdata();
	if(order=='N')x=0,y=0,printf("{#no target}$\r\n");
	if(order=='V')printf("{#lock target}$\r\n");
	
//	printf("{#x=%f y=%f}$\r\n",x,y);
	
	if(x<-30)x=0; if(x>30)x=0;	//过滤过大值
	if(x<-10)x=-10;if(x>10)x=10;		
	if(y<-20)y=0; if(y>20)y=0;
	if(y<-5)y=-5;if(y>5)y=5;
	
	xp[t]=x; yp[t]=y;
	
	xx=0.0f,yy=0.0f;
	while(q<m)xx += xp[q], yy += yp[q], q++;	//均值处理，使移动平滑
	xx = xx*3/(float)m; yy = yy/(float)m;

//	printf("{#xx=%f yy=%f}$\r\n",xx,yy);
//	xx=x*70; yy=y;	
	
//	Forward_L.target_angle += xx;
//	Forward_R.target_angle += xx;
	Back_L.target_angle -= xx,	
	Back_R.target_angle += xx;
	if(10<Turret.angle+yy&&Turret.angle+yy<=50)Turret.angle += yy;
}

