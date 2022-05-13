#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"

typedef struct{
	float p,i,d;
	volatile float integral, err_last, fp;
	volatile float err_last1, err_last2, fi;
}PID_TypeDef;


void PID_init(PID_TypeDef *k);
float PID_position(float target_val, float actual_val, float limit, PID_TypeDef *k);		//PID位置式控制
float PID_incremental(float target_val, float actual_val, float limit, PID_TypeDef *k);	//PID增量式控制
void set_pid(PID_TypeDef *k,float p,float i,float d);

//float PID_incremental_(float target_speed, float actual_speed, float kp,float ki,float kd);

#ifdef __cplusplus
}
#endif

#endif 

