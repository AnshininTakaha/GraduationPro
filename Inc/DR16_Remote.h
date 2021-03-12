#ifndef __DR16_REMOTE_H__
#define __DR16_REMOTE_H__

#include "usart.h"
#include <stdbool.h>
#pragma anon_unions

/* =========================== PriviteDefine Begin=========================== */
//#define DR16FrameFolk
/*DR16遥控器相关参数*/
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

typedef struct///遥控器解码结构体
{
	/*遥控器基本结构参数*/
	struct {
		int16_t ch0;
		int16_t ch1;
		int16_t ch2;
		int16_t ch3;
		uint8_t s_left;
		uint8_t s_right;
		int16_t ch4_DW; 
	}rc;

	/*鼠标基本结构参数*/
	struct {
		int16_t x;
		int16_t y;
		int16_t z;
		uint8_t keyLeft;
		uint8_t keyRight;
	}mouse;

	union {
		uint16_t key_code;
		struct { //位域的使用
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
	
	/*帧率和离线标志*/
	uint16_t DR16_Frame;	
	uint8_t DR16_OffFlag;

	/*指针函数*/
	void(*DR16_ReInit)(void);//错误复位
	void(*DR16_Process)(uint8_t *pData);//DR16解码
	void(*DR16_Handler)(UART_HandleTypeDef *huart);//DR16中断处理
	void(*DR16_USART1_IT_Init)(void);//DR16中断初始化
} DR16_t;


typedef enum///遥控器左右拨杆开关枚举
{
	RemotePole_UP = 1, //上
	RemotePole_MID = 3,	//中
	RemotePole_DOWM = 2	//下
}RemotePole_e;


typedef struct///机器人总体变量
{
	float DR16_ForwardBack_Value;
	float DR16_Omega_Value;
	float DR16_Left_Right_Value;
	RemotePole_e Switch_Left;
	RemotePole_e Switch_Right;
}DR16_Export_Data_t; //供其他文件使用的输出数据。


/* =========================== Structure End=========================== */

/* =========================== ShareValue&funtions Begin=========================== */
/****可外部共享变量****/
extern DR16_t DR16;
extern DR16_Export_Data_t DR16_Export_data;
/* =========================== ShareValue&funtions End=========================== */

#endif
