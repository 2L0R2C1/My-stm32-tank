#ifndef __CONTROL_H
#define __CONTROL_H

#include "main.h"

void auto_step(void);

void ps2_control(u8 order);

void ps2_angle(void);

void ps2_speed(void);




void bluetooth_control(u8 order);

void forward(void);

void back(void);

void right(void);

void left(void);

void fright(void);

void fleft(void);

void bright(void);

void bleft(void);

void speed_up(void);

void speed_down(void);

void stop(void);
void Steer_f_up(void);	  //��ǰצ
void Steer_b_up(void);		//����צ
void Steer_f_down(void);	//��ǰצ
void Steer_b_down(void);	//����צ
		//��̨���
void Turret_up(void);     //��������
void Turret_down(void);   //�����µ�
			

#endif
