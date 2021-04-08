#ifndef ENCONDER_H__
#define ENCONDER_H__

#include "can.h"
/* =========================== PriviteDefine Begin=========================== */
/*�������ܴ���ָ��*/
#define Encoder_HOST_CMD 0x01

/*��������ʼID*/
#define Encoder_READ_ID_START 0x02

#define EncoderLF_ID 0x04
#define EncoderRF_ID 0x05
#define EncoderLB_ID 0x03
#define EncoderRB_ID 0x02

/*����������ʽ��ʼ�Ƕ�(ѧУ�Ķ��ֵ���)*/
#define EncoderLF_BaseAbsEnc 100898   //0
#define EncoderRF_BaseAbsEnc 103913   //1
#define EncoderLB_BaseAbsEnc 96605    //2
#define EncoderRB_BaseAbsEnc 100888   //3


/*���������ֵתΪ�Ƕ�*/
#define ANGLE_TO_Enconder(x) (4096.0f/180.0f * x)
#define Encoder_TO_ANGLE(x) (180.0f/4096.0f * x)
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
typedef struct///���������ؽǶ�
{
	uint8_t Enconder_ID;
	int32_t Enconder_ReadBackAngle;
}Enconder_Folk_t;



/*�������������ܹ����Ժ���*/
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
/*���������������Ժ���*/
extern EnconderFunction_t EnconderFunction;

/*���������ݴ���ṹ��*/
extern Enconder_Folk_t Enconder[4];
/* =========================== ShareValue&funtions End=========================== */

#endif
