#include "vision.h"
#include "usart.h"
#include "dma.h"
#include "motor.h"
#include "Steer.h"

volatile float x=0.0f,y=0.0f;


int getdata(void){		//接收并转换树莓派数据
	volatile int num=0, t=0, flag=1; 
//	HAL_UART_Receive_DMA(&huart2,rx2_buffer,4);
	HAL_UART_Receive(&huart2,rx2_buffer,4,4);
	while(rx2_buffer[t]!='@'){
		printf("%d",rx2_buffer[t]);
		if(rx2_buffer[t]=='+')continue;
		if(rx2_buffer[t]=='-')flag=-1;
		num = num*10 + (rx2_buffer[t]-'0');
		t++;
	}
	printf("\r\n");
	HAL_UART_DMAStop(&huart2);
	return flag*num;
}


void vision_control(u8 order){
	if(order=='G')HAL_UART_Transmit(&huart1, (u8 *)&order,1,1);
    if(order=='N')HAL_UART_Transmit(&huart1, (u8 *)&order,1,1);
	
	if(order=='x'){
		x = 7*getdata();
		Forward_L.target_angle += x;
		Forward_R.target_angle -= x;
		Back_L.target_angle += x,	
		Back_R.target_angle -= x;
	}
	if(order=='y'){
		y = getdata();
		if(0<=Turret.angle+y&&Turret.angle+y<=50)Turret.angle += y;
	}
	
	printf("x=%f y=%f\r\n",x,y);
}
