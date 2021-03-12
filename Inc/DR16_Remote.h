#ifndef __DR16_REMOTE_H__
#define __DR16_REMOTE_H__

#include "usart.h"
#include <stdbool.h>
#pragma anon_unions

/* =========================== PriviteDefine Begin=========================== */
//#define DR16FrameFolk
/*DR16ң������ز���*/
#define DR16BufferNumber 22
#define DR16BufferTruthNumber 18
#define DR16BufferLastNumber 4
/* =========================== PriviteDefine End=========================== */

/* =========================== GroundInit Begin=========================== */
#define DR16_GroundInit \
{ \
{0,0,0,0,0,0,0}, \
{0,0,0,0,0}, \
{0}, \
0, \
0, \
&DR16_ReInit, \
&DR16_Process, \
&DR16_Handler, \
&DR16_USART1_IT_Init,\
} \
/* =========================== GroundInit Begin=========================== */

/* =========================== Structure Begin=========================== */

typedef struct///ң��������ṹ��
{
	/*ң���������ṹ����*/
	struct {
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		uint8_t s_left;
		uint8_t s_right;
		int16_t ch4_DW; 
	}rc;

	/*�������ṹ����*/
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t keyLeft;
		uint8_t keyRight;
	}mouse;

	union {
		uint16_t key_code;
		struct { //λ���ʹ��
			bool press_W : 1;
			bool press_S : 1;
			bool press_A : 1;
			bool press_D : 1;

			bool press_Shift : 1;
			bool press_Ctrl : 1;
			bool press_Q : 1;
			bool press_E : 1;

			bool press_R : 1;
			bool press_F : 1;
			bool press_G : 1;
			bool press_Z : 1;

			bool press_X : 1;
			bool press_C : 1;
			bool press_V : 1;
			bool press_B : 1;
		};
	}keyBoard;
	
	/*֡�ʺ����߱�־*/
	uint16_t DR16_Frame;	
	uint8_t DR16_OffFlag;

	/*ָ�뺯��*/
	void(*DR16_ReInit)(void);//����λ
	void(*DR16_Process)(uint8_t *pData);//DR16����
	void(*DR16_Handler)(UART_HandleTypeDef *huart);//DR16�жϴ���
	void(*DR16_USART1_IT_Init)(void);//DR16�жϳ�ʼ��
} DR16_t;


typedef enum///ң�������Ҳ��˿���ö��
{
	RemotePole_UP = 1, //��
	RemotePole_MID = 3,	//��
	RemotePole_DOWM = 2	//��
}RemotePole_e;


typedef struct///�������������
{
	float DR16_ForwardBack_Value;
	float DR16_Omega_Value;
	float DR16_Left_Right_Value;
	RemotePole_e Switch_Left;
	RemotePole_e Switch_Right;
}DR16_Export_Data_t; //�������ļ�ʹ�õ�������ݡ�


/* =========================== Structure End=========================== */

/* =========================== ShareValue&funtions Begin=========================== */
/****���ⲿ�������****/
extern DR16_t DR16;
extern DR16_Export_Data_t DR16_Export_data;
/* =========================== ShareValue&funtions End=========================== */

#endif
