#include "pid.h"
#include "usart.h"
#include "motor.h"

void PID_init(PID_TypeDef *k){					//初始化PID控制中的各项参数，避免积分项错误积累
	k->integral=0;	k->err_last=0;	k->fp=0;
	k->err_last1=0;	k->err_last2=0;	k->fi=0;
}

void set_pid(PID_TypeDef *k,float p,float i,float d){
	PID_init(k);
	k->p = p;	k->i = i;	k->d = d;
}

float PID_position(float target_val, float actual_val, float limit, PID_TypeDef *k){		//PID位置式控制
	float err = target_val - actual_val;	//误差值
	
	k->integral += err;						//积分项
	
	if( k->integral > limit/2 ) k->integral = limit/2;	//限幅
	if( k->integral <-limit/2 ) k->integral = -limit/2;
	
	k->fp = (k->p)*err + (k->i)*k->integral + (k->d)*(err - k->err_last);
	k->err_last = err;						//跟新上一次误差
	
	if( k->fp > limit ) k->fp = limit;	//限幅
	if( k->fp <-limit ) k->fp = -limit;
	
	return k->fp;
}	


float PID_incremental(float target_val, float actual_val, float limit, PID_TypeDef *k){	//PID增量式控制
	float err = target_val - actual_val;		//printf("target= %f\r\n   actual= %f\r\n   err=%f\r\n  err_last1=%f\r\n  err_last2=%f\r\n",target_val,actual_val,err,k->err_last1,k->err_last2);
	
	k->fi += (k->p)*( err - (k->err_last1)) + (k->i)*err + (k->d)*(err-2*(k->err_last1)+(k->err_last2));	//printf("fi= %f\r\n",k->fi);
	k->err_last2 = k->err_last1;						
	k->err_last1 = err;
	
	if( k->fi > limit ) k->fi = limit;	//限幅
	if( k->fi <-limit ) k->fi = -limit;
	
	return k->fi;
}

/*
float PID_incremental_(float target_val, float actual_val, float kp, float ki, float kd){	//PID增量式控制
	float err = target_val - actual_val;		//printf("PID %.2f %.2f %.2f\r\n", k->p, k->i, k->d );
												//printf("err=%.2f  err_last1=%.2f  err_last2=%.2f\r\n",err,err_last1,err_last2);
	fi += kp*(err-err_last1) + ki*err + kd*(err-2*err_last1+err_last2);
	err_last2 = err_last1;						//printf("fi=%.2f\r\n",fi);
	err_last1 = err;
	
	return fi;
}
*/
