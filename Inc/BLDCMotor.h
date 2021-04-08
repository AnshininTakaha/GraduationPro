#ifndef BLDC_MOTOR_H__
#define BLDC_MOTOR_H__

#include "can.h"

/* =========================== PriviteDefine Begin=========================== */
/*BLDC电机储存数据变量*/
#define CAN_STATUS_MSGS_TO_STORE 10

/*BLDC第一个电机的ID，为了方便计算数据存储在哪一个结构体*/
#define BLDC_FisMotor_ID 37
#define BLDC_LasMotor_ID 40

#define BLDCMotorLF_ID 39
#define BLDCMotorRF_ID 40
#define BLDCMotorLB_ID 38
#define BLDCMotorRB_ID 37

/* =========================== PriviteDefine End=========================== */



/* =========================== GroundInit Begin=========================== */
#define BLDC_BJMFunctionGroundInit \
{ \
	&BLDC_Motor_CAN2_IT_Init, \
	&BLDCMotor_Process, \
	&CAN2_BLDCHandler, \
	&VSEC_SetDuty, \
	&VSEC_SetCurrent, \
	&VSEC_SetRpm, \
}
/* =========================== GroundInit End=========================== */



/* =========================== Structure Begin=========================== */
typedef enum {///BLDC控制令牌
	CAN_PACKET_SET_DUTY = 0,
	CAN_PACKET_SET_CURRENT,
	CAN_PACKET_SET_CURRENT_BRAKE,
	CAN_PACKET_SET_RPM,
	CAN_PACKET_SET_POS,
	CAN_PACKET_FILL_RX_BUFFER,
	CAN_PACKET_FILL_RX_BUFFER_LONG,
	CAN_PACKET_PROCESS_RX_BUFFER,
	CAN_PACKET_PROCESS_SHORT_BUFFER,
	CAN_PACKET_STATUS,
	CAN_PACKET_SET_CURRENT_REL,
	CAN_PACKET_SET_CURRENT_BRAKE_REL,
	CAN_PACKET_SET_CURRENT_HANDBRAKE,
	CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
	CAN_PACKET_STATUS_2,
	CAN_PACKET_STATUS_3,
	CAN_PACKET_STATUS_4,
	CAN_PACKET_PING,
	CAN_PACKET_PONG,
	CAN_PACKET_DETECT_APPLY_ALL_FOC,
	CAN_PACKET_DETECT_APPLY_ALL_FOC_RES,
	CAN_PACKET_CONF_CURRENT_LIMITS,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS,
	CAN_PACKET_CONF_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_STORE_CURRENT_LIMITS_IN,
	CAN_PACKET_CONF_FOC_ERPMS,
	CAN_PACKET_CONF_STORE_FOC_ERPMS,
	CAN_PACKET_STATUS_5
} CAN_PACKET_ID;




typedef struct {///MSG 1号反馈结构体 具体结构体反馈形式可以在vcse_tool中设置
	int id;
	uint32_t rx_time;
	float rpm;
	float current;
	float duty;
} can_status_msg;



typedef struct {///MSG 2号反馈结构体 具体结构体反馈形式可以在vcse_tool中设置
	int id;
	int rx_time;
	float amp_hours;
	float amp_hours_charged;
} can_status_msg_2;



typedef struct {///MSG 3号反馈结构体 具体结构体反馈形式可以在vcse_tool中设置
	int id;
	uint32_t rx_time;
	float watt_hours;
	float watt_hours_charged;
} can_status_msg_3;



typedef struct {///MSG 4号反馈结构体 具体结构体反馈形式可以在vcse_tool中设置
	int id;
	uint32_t rx_time;
	float temp_fet;
	float temp_motor;
	float current_in;
	float pid_pos_now;
} can_status_msg_4;



typedef struct {///MSG 5号反馈结构体 具体结构体反馈形式可以在vcse_tool中设置
	int id;
	uint32_t rx_time;
	float v_in;
	int32_t tacho_value;
} can_status_msg_5;



typedef struct{///MSG 专用发送结构体
	uint8_t  Data[8];
	CAN_TxHeaderTypeDef TxMessageHeader;
} BLDCCAN_Txmsg ;







typedef struct {///MSG专用功能函数结构体
	void(*BLDC_Motor_CAN2_IT_Init)(void);
	void(*BLDCMotor_Process)(CAN_RxTypedef RxMessage);
	void(*CAN2_BLDCHandler)(CAN_HandleTypeDef *hcan);
	
	/*发送数据函数*/
//	void(*VESC_CANTransmit)(CAN_HandleTypeDef* CANx,uint32_t id, uint8_t *data,uint8_t len);
	void(*VSEC_SetDuty)(CAN_HandleTypeDef* CAN_Num ,uint8_t controller_id, float duty);
	void(*VSEC_SetCurrent)(CAN_HandleTypeDef* CAN_Num ,uint8_t controller_id, float current);
	void(*VSEC_SetRpm)(CAN_HandleTypeDef* CAN_Num ,uint8_t controller_id, float rpm);
}BLDC_BJMFunction_t;




typedef struct {///MSG 对应四个BLDC封装体
can_status_msg    status_msg[4];
can_status_msg_2  status_msg_2[4];
can_status_msg_3  status_msg_3[4];
can_status_msg_4  status_msg_4[4];
can_status_msg_5  status_msg_5[4];
} BLDC_BJMMotors_t;
/* =========================== Structure End=========================== */



/* =========================== ShareValue&funtions Begin=========================== */
extern BLDC_BJMMotors_t BLDC_Motors;
extern BLDC_BJMFunction_t BLDCMotorFunction;
/* =========================== ShareValue&funtions End=========================== */

#endif
