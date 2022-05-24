#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"
#define Buff_Max 10 
extern u8 num;
extern u8 RxBuff[Buff_Max];
void friction_init(void);
void friction_start(void);
void friction_stop(void);
void single_shot(void);			//����
void repeating_shot1(void);		//����5
void repeating_shot2(void);		//����10   
void f411_control(u8 order);

#endif


