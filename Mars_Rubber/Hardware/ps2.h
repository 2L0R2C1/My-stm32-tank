#ifndef __PS2_H_
#define __PS2_H_

#include "main.h"

#define DI HAL_GPIO_ReadPin(DI_GPIO_Port,DI_Pin)

#define DO_H HAL_GPIO_WritePin( DO_GPIO_Port, DO_Pin, GPIO_PIN_SET)		//����λ��
#define DO_L HAL_GPIO_WritePin( DO_GPIO_Port, DO_Pin, GPIO_PIN_RESET)	//����λ��

#define CS_H HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_SET)     //CS����
#define CS_L HAL_GPIO_WritePin( CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET)    //CS����

#define CLK_H HAL_GPIO_WritePin( CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET)   //ʱ������
#define CLK_L HAL_GPIO_WritePin( CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET) //ʱ������

//These are our button constants
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16
#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      26

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5         //��ҡ��X������
#define PSS_RY 6			//��ҡ��Y������
#define PSS_LX 7		//��ҡ��X������
#define PSS_LY 8			//��ҡ��Y������



extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;

void PS2_Init(void);
u8 PS2_RedLight(void);		  //�ж��Ƿ�Ϊ���ģʽ
void PS2_ReadData(void);	  //��ȡ�ֱ�����
void PS2_Cmd(u8 CMD);		  //���ֱ���������
u8 PS2_DataKey(void);		  //��ֵ��ȡ
u8 PS2_AnologData(u8 button); //�õ�һ��ҡ�˵�ģ����
void PS2_ClearData(void);	  //������ݻ�����

void PS2_Vibration(u8 motor1, u8 motor2);//������motor1  0xFF���������أ�motor2  0x40~0xFF
void PS2_EnterConfing(void);	 //��������
void PS2_TurnOnAnalogMode(void); //����ģ����
void PS2_VibrationMode(void);    //������
void PS2_ExitConfing(void);	     //�������
void PS2_SetInit(void);		     //���ó�ʼ��

#endif
