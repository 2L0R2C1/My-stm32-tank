#ifndef __SHOW_H__
#define __SHOW_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pid.h"
	
#define WAVE_CHANNEL2

#ifdef WAVE_CHANNEL1
void showwave(int a);
#endif

#ifdef WAVE_CHANNEL2
void showwave(int a,int b);
#endif

#ifdef WAVE_CHANNEL3
void showwave(int a,int b,int c);
#endif

#ifdef WAVE_CHANNEL4
void showwave(int a,int b,int c,int d);
#endif

#ifdef WAVE_CHANNEL5
void showwave(int a,int b,int c,int d,int e);
#endif

void getpid(PID_TypeDef *k);

void get_target_speed(float *p);

void get_target_position(float *p);

void outspeed(float speed);

void outposition(float position);

void DataScope(float a, float b);

#ifdef __cplusplus
}
#endif

#endif 

