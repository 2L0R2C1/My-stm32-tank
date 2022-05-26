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
void Steer_f_up(void);	  //升前爪
void Steer_b_up(void);		//升后爪
void Steer_f_down(void);	//降前爪
void Steer_b_down(void);	//降后爪
		//云台舵机
void Turret_up(void);     //炮塔上仰
void Turret_down(void);   //炮塔下调
			

#endif
