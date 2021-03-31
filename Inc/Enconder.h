#ifndef ENCONDER_H__
#define ENCONDER_H__

#include "can.h"
/* =========================== PriviteDefine Begin=========================== */
/*编码器总处理指令*/
#define Encoder_HOST_CMD 0x01

/*编码器起始ID*/
#define Encoder_READ_ID_START 0x02

/*电机编码器值转为角度*/
#define Encoder_TO_ANGLE(x) (4096.0f/360.0f * x)
/* =========================== PriviteDefine End=========================== */

/* =========================== GroundInit Begin=========================== */
#define EnconderFunction_GroundInit \
{ \
	&Encoder_Read_Config, \
	&Encoder_ID_Config, \
	&Encoder_Baud_Config, \
	&Encoder_Read_Mode, \
	&Encoder_Time_Config, \
	&Encoder_Zero_Config, \
	&Encoder_Process, \
	&CAN1_EncHandler, \
	&Enc_CAN1_IT_Init \
}
/* =========================== GroundInit End=========================== */



/* =========================== Structure Begin=========================== */
typedef struct///编码器读回角度
{
	uint8_t Enconder_ID;
	int32_t Enconder_ReadBackAngle;
}Enconder_Folk_t;



/*编码器基本功能功能性函数*/
typedef struct
{
	void(*Encoder_Read_Config)(uint8_t Encoder_id);
	void(*Encoder_ID_Config)(uint8_t old_id,uint8_t new_id);
	void(*Encoder_Baud_Config)(uint8_t Encoder_id,uint32_t Baudrate);
	void(*Encoder_Read_Mode)(uint8_t Encoder_id);
	void(*Encoder_Time_Config)(uint8_t Encoder_id,uint16_t time);
	void(*Encoder_Zero_Config)(uint8_t Encoder_id);
	void(*Encoder_Process)(CAN_RxTypedef RxMessage);
	void(*CAN1_EncHandler)(CAN_HandleTypeDef *hcan);
	void(*Enc_CAN1_IT_Init)(void);
}EnconderFunction_t;
/* =========================== Structure End=========================== */

/* =========================== ShareValue&funtions Begin=========================== */
/*编码器基本功能性函数*/
extern EnconderFunction_t EnconderFunction;

/*编码器数据储存结构体*/
extern Enconder_Folk_t Enconder[4];
/* =========================== ShareValue&funtions End=========================== */

#endif
