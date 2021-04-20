#ifndef DJI_MOTOR_H__
#define DJI_MOTOR_H__

#include "PID.h"
#include "can.h"

/* =========================== PriviteDefine Begin=========================== */
#define USE_3508StepGaining

#define USE_M3508
#define Roll_CountingM3508

//#define USE_M2006



/*DJI M3508 Motor for fourwheel*/
#define M3508_READID_START	0x201
#define M3508_READID_SEC		0x202
#define M3508_READID_THIR   0X203
#define M3508_READID_END	  0x204
#define M3508_SENDID		    0x200
#define M3508_MaxOutput 16384 
#define M3508_CurrentRatio	819.2f	//16384/20A = 819.2->1A

/*DJI M2006 Motor for fourMoonwheel*/
#define M2006_READID_START	0x207
#define M2006_READID_END	0x207
#define M2006_SENDID		0x1FF 
#define M2006_MaxOutput 10000 
//#define M2006_LOADANGLE		36864  //拨弹电机角度		
#define M2006_ReductionRatio	36	

/* =========================== PriviteDefine End=========================== */


/* =========================== GroundInit Begin=========================== */
#define DJIMotorGroundInit \
{ \
	&DJI_Motor_CAN1_IT_Init, \
	&CAN1_DJIHandler, \
	&DJI_Motor3508Process, \
	&DJIMotor_Set3508Current, \
}  
#define DJI_Motor3508GroundInitRB \
{ \
		{0,0,0,0}, \
		{0,0,0,0,0,0}, \
		{PID_GroundSpendInit,PMode_GroundInitOutRB,PMode_GroundInitInRB}, \
		{0,0}, \
}

#define DJI_Motor3508GroundInitLB \
{ \
		{0,0,0,0}, \
		{0,0,0,0,0,0}, \
		{PID_GroundSpendInit,PMode_GroundInitOutLB,PMode_GroundInitInLB}, \
		{0,0}, \
}

#define DJI_Motor3508GroundInitLF \
{ \
		{0,0,0,0}, \
		{0,0,0,0,0,0}, \
		{PID_GroundSpendInit,PMode_GroundInitOutLF,PMode_GroundInitInLF}, \
		{0,0}, \
}

#define DJI_Motor3508GroundInitRF \
{ \
		{0,0,0,0}, \
		{0,0,0,0,0,0}, \
		{PID_GroundSpendInit,PMode_GroundInitOutRF,PMode_GroundInitInRF}, \
		{0,0}, \
}

/* =========================== GroundInit End=========================== */


/* =========================== Structure Begin=========================== */
typedef struct///DJI电机初始化函数结构体
{
	void(*DJI_Motor_CAN1_IT_Init)(void);
	void(*CAN1_DJIHandler)(CAN_HandleTypeDef *hcan);
	void(*DJI_Motor3508Process)(CAN_RxTypedef RxMessage);
	void(*DJIMotor_Set3508Current)(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
}DJI_MotorInit_t;



#ifdef USE_M3508
typedef struct{///3508电机对应结构体
	struct{///反馈部分
	uint16_t ReadAngle;
	int16_t  ReadSpeed;
	int16_t  ReadCurrent;
	uint8_t  temperture;
	}Feedback;
	
	struct{///设置值和计算值部分
	int16_t  targetSpeed;
	int32_t  targetAngle;
	uint16_t lastAngle;
	int32_t  totalAngle;
	int16_t  turnCount;
	int16_t  outCurrent;
	}Value;
	
	struct{///PID部分
	incrementalpid_t Speed_PID;
	positionpid_t Posit_PID_Out;
	positionpid_t Posit_PID_In;
	}PID;
	
	struct{///帧率控制部分
	uint16_t Frame;
	uint8_t OffFlag;
	}Frame;
	
}DJI_Motor3508Folk_t;
#endif



#ifdef USE_M2006
typedef struct{///2006电机对应结构体
	struct{///反馈部分
	uint16_t ReadAngle;
	int16_t  ReadSpeed;
	int16_t  ReadTorque;
	}Feedback;
	
	struct{///设置值和计算值部分
	int16_t  targetSpeed;
	int32_t  targetAngle;
	uint16_t lastAngle;
	int32_t  totalAngle;
	int16_t  turnCount;
	int16_t  outCurrent;
	}Value;
	
	struct{///PID部分
	positionpid_t Posit_PID_Out;
	positionpid_t Posit_PID_In;
	}PID;
	
	struct{///帧率控制部分
	uint16_t Frame;
	uint8_t OffFlag;
	}Frame;
	
	/*指针函数部分*/
	
	
}DJI_Motor2006Folk_t;
#endif

/* =========================== Structure End=========================== */

/* =========================== ShareValue&funtions Begin=========================== */
/****可外部共享变量****/
extern DJI_MotorInit_t DJIMotorFunction;
extern DJI_Motor3508Folk_t M3508_MoonWheel[4];
/* =========================== ShareValue&funtions End=========================== */

#endif
