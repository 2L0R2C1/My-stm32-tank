#include "communication.h"
#include "DataScope_DP.h"
#include "usart.h"

#ifdef WAVE_CHANNEL1
void showwave(int a){
	printf("{B%d}$",a);
}
#endif

#ifdef WAVE_CHANNEL2
void showwave(int a,int b){
	printf("{B%d:%d}$",a,b);
}
void DataScope(float a, float b){
	int Send_Count,i;//计数需要的变量
	
	DataScope_Get_Channel_Data( a, 1 );      
	DataScope_Get_Channel_Data( b, 2 );      
//	DataScope_Get_Channel_Data( 0, 3 );              
//	DataScope_Get_Channel_Data( 0 , 4 );   
//	DataScope_Get_Channel_Data(0, 5 ); //用您要显示的数据替换0就行了
//	DataScope_Get_Channel_Data(0 , 6 );//用您要显示的数据替换0就行了
//	DataScope_Get_Channel_Data(0, 7 );
//	DataScope_Get_Channel_Data( 0, 8 ); 
//	DataScope_Get_Channel_Data(0, 9 );  
//	DataScope_Get_Channel_Data( 0 , 10);//一共可以打印10个数据查看波形
	Send_Count = DataScope_Data_Generate(2);//打印几个数据就在这里改为几
	for( i = 0 ; i < Send_Count; i++) 
	{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[i]; 
	}
}
#endif

#ifdef WAVE_CHANNEL3
void showwave(int a,int b,int c){
printf("{B%d:%d:%d}$",a,b,c);
}
#endif

#ifdef WAVE_CHANNEL4
void showwave(int a,int b,int c,int d){
	printf("{B%d:%d:%d:%d}$",a,b,c,d);
}
#endif

#ifdef WAVE_CHANNEL5
void showwave(int a,int b,int c,int d,int e){
	printf("{B%d:%d:%d:%d:%d}$",a,b,c,d,e);
}
#endif

void getpid(PID_TypeDef *k){
	printf("{#Please input P,I,D}$");
	scanf("%f%f%f", &k->p, &k->i, &k->d); 
}

void get_target_speed(float *p){scanf("%f",p);}

void get_target_position(float *p){scanf("%f",p);}

void outspeed(float speed){int ss = speed*10; printf("{#speed: %d r/s}$",ss);}

void outposition(float position){int pp = position*10; printf("{#position: %d}$",pp);}
