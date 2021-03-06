/**
  ******************************************************************************
  * @file    BLDCMotor.c
  * @author  口无
  * @version V3.21
  * @date		 2021-03-12 解除DJIMoter和BLDCMotor的合并函数结构，重写BLDCMotor函数
						 2021-03-13 初次考虑使用define函数简化bufferapp类型函数，从而减少创建函数的栈开销，但是没有什么思路
						 2021-03-13 更改结构体赋值为非指针赋值类型，将原来的指针赋值变为宏变量方便仿真查看
						 2021-04-28 完整封装完成，最终版完成
						 
  * @brief   BLDC各种电机整合结构
						 
	* @funtion(in) BLDC_Motor_CAN2_IT_Init  BLDC对应的CAN初始化用
								 BLDCMotor_Process        BLDC电机解码
								 CAN2_Handler             CAN2中断处理函数
						 
	* @value(share)BLDC_Motors              存储对应的BLDC的参数状况
								 BLDCMotorFunction        储存对应的BLDC的函数状况
  ******************************************************************************
  */
#include "BLDCMotor.h"
#include <string.h>
#include "handle.h"
/* =========================== FuntionsPulk Begin=========================== */
void BLDC_Motor_CAN2_IT_Init(void);
void BLDCMotor_Process(CAN_RxTypedef RxMessage);
void CAN2_BLDCHandler(CAN_HandleTypeDef *hcan);


void VSEC_SetDuty(CAN_HandleTypeDef* CAN_Num ,uint8_t controller_id, float duty);
void VSEC_SetCurrent(CAN_HandleTypeDef* CAN_Num ,uint8_t controller_id, float current);
void VSEC_SetRpm(CAN_HandleTypeDef* CAN_Num ,uint8_t controller_id, float rpm);

/*VSEC CAN数据发送函数*/
void VESC_CANTransmit(CAN_HandleTypeDef* CANx,uint32_t id, uint8_t *data,uint8_t len);

/*值处理函数*/
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index);
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index);
void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index);
void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index);
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index);
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index);
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index);
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index);
float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index);
float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index);
/* =========================== FuntionsPulk End=========================== */

/* =========================== Value Begin=========================== */
/*BLDC电机参数结构体*/
BLDC_BJMMotors_t BLDC_Motors;

/*BLDC电机功能性参数*/
BLDC_BJMFunction_t BLDCMotorFunction = BLDC_BJMFunctionGroundInit;

/*BLDC启动指令（宏定义是为了方便查看）*/
CAN_PACKET_ID BLDC_Cmd;

/*创建变量指针，方便当前帧数据，直接用指针进行地址操作比较便捷*/
can_status_msg *stat_tmp;
can_status_msg_2 *stat_tmp_2;
can_status_msg_3 *stat_tmp_3;
can_status_msg_4 *stat_tmp_4;
can_status_msg_5 *stat_tmp_5;
/* =========================== Value End=========================== */

/* =========================== Funtions Begin=========================== */
/**
	* @Data   2020-03-13 
  * @brief  buffer转化成为高低位赋值（数据类型转换）
  * @param  uint8_t* buffer   需要转换的数值
						int16_t number    需要转换的数字
						int32_t *index    数组位置，目录
  * @retval void
	* @fallback None
  */
	
void BLDCMotor_Process(CAN_RxTypedef RxMessage)
{
	/*避免数据混乱化问题*/
	uint8_t BLDC_id = 0;
	int32_t list_id = 0;
	
	/*数组目录计数归零*/
	int index = 0;
	
	/*获取对应的拓展id号码，计算需要存储到哪一个结构体*/
	BLDC_id = RxMessage.CAN_RxHeader.ExtId;
	list_id = BLDC_id - BLDC_FisMotor_ID;
	
	
	/*根据命令码进行处理，本杰明最大ID只能到200*/
	if(list_id < 200  && BLDC_id != 0)
	{
		/*取拓展ID高8位，对应上命令表方便操作*/	
		BLDC_Cmd = (RxMessage.CAN_RxHeader.ExtId >>8);
		switch(BLDC_Cmd)///由于
		{
			case CAN_PACKET_STATUS:
				/*将指针指向对应的地址*/
				stat_tmp = &BLDC_Motors.status_msg[list_id];
			
				/*变量赋值*/
				BLDC_Motors.status_msg[list_id].id = BLDC_id;
				BLDC_Motors.status_msg[list_id].rx_time = 0;
				BLDC_Motors.status_msg[list_id].rpm = (float)buffer_get_int32(RxMessage.CAN_RxMessage, &index);
				BLDC_Motors.status_msg[list_id].current = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 10.0f;
				BLDC_Motors.status_msg[list_id].duty = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 1000.0f;
				break;
			
			case CAN_PACKET_STATUS_2:
				/*将指针指向对应的地址*/
				stat_tmp_2 = &BLDC_Motors.status_msg_2[list_id];
			
				/*变量赋值*/
				BLDC_Motors.status_msg_2[list_id].id = BLDC_id;
				BLDC_Motors.status_msg_2[list_id].rx_time = 0;
				BLDC_Motors.status_msg_2[list_id].amp_hours = (float)buffer_get_int32(RxMessage.CAN_RxMessage, &index) / 1e4f;
				BLDC_Motors.status_msg_2[list_id].amp_hours_charged = (float)buffer_get_int32(RxMessage.CAN_RxMessage, &index) / 1e4f;
				break;
			
			case CAN_PACKET_STATUS_3:
				/*将指针指向对应的地址*/
				stat_tmp_3 = &BLDC_Motors.status_msg_3[list_id];
				
				/*变量赋值*/
				BLDC_Motors.status_msg_3[list_id].id = BLDC_id;
				BLDC_Motors.status_msg_3[list_id].rx_time = 0;
				BLDC_Motors.status_msg_3[list_id].watt_hours = (float)buffer_get_int32(RxMessage.CAN_RxMessage, &index) / 1e4f;
				BLDC_Motors.status_msg_3[list_id].watt_hours_charged = (float)buffer_get_int32(RxMessage.CAN_RxMessage, &index) / 1e4f;
				break;
			
			case CAN_PACKET_STATUS_4:
				/*将指针指向对应的地址*/
				stat_tmp_4 = &BLDC_Motors.status_msg_4[list_id];
			
				/*变量赋值*/
				BLDC_Motors.status_msg_4[list_id].id = BLDC_id;
				BLDC_Motors.status_msg_4[list_id].rx_time = 0;
				BLDC_Motors.status_msg_4[list_id].temp_fet = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 10.0f;
				BLDC_Motors.status_msg_4[list_id].temp_motor = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 10.0f;
				BLDC_Motors.status_msg_4[list_id].current_in = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 10.0f;
				BLDC_Motors.status_msg_4[list_id].pid_pos_now = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 50.0f;
				break;
			
			case CAN_PACKET_STATUS_5:
				/*将指针指向对应的地址*/
				stat_tmp_5 = &BLDC_Motors.status_msg_5[list_id];
			
				/*变量赋值*/
				BLDC_Motors.status_msg_5[list_id].id = BLDC_id;
				BLDC_Motors.status_msg_5[list_id].rx_time = 0;
				BLDC_Motors.status_msg_5[list_id].tacho_value = buffer_get_int32(RxMessage.CAN_RxMessage, &index);
				BLDC_Motors.status_msg_5[list_id].v_in = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 1e1f;
				break;
			
			default:
				break;
		}
	}
}



/**
	* @Data   2020-12-07 
  * @brief  CAN2处理函数（使用中断）
  * @param  void
  * @retval void
	* @fallback None
  */
void CAN2_BLDCHandler(CAN_HandleTypeDef *hcan)
{
	if (__HAL_CAN_GET_IT_SOURCE(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING))
	{
		CAN_RxTypedef CAN_RxMessage;
		
		/*标记当前信号为CAN2信号,并接收对应的CAN2的信号*/
		CAN_RxMessage.CAN_Switch = 2;
		HAL_CAN_GetRxMessage(&hcan2,
												CAN_RX_FIFO0,
												&CAN_RxMessage.CAN_RxHeader,
												CAN_RxMessage.CAN_RxMessage);
		
		/*将数据发送到队列里面*/
		xQueueSendToBackFromISR(xQueueCanReceiveHandle,&CAN_RxMessage,0);
		
		/*清理中断标志位*/
		__HAL_CAN_CLEAR_FLAG(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
}




/**
  * @Data    2020-12-07
  * @brief   初始化CAN2中断函数，用于接收BLDCMotor的报文
  * @param   void
  * @retval  void
  */
void BLDC_Motor_CAN2_IT_Init(void)
{
	/*使能滤波器*/
	CAN2_FILTER_Init(CAN2_Filter);
	
	/*启用CAN*/
	HAL_CAN_Start(&hcan2);
	
	/*使能CAN的IT中断*/
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);         //  CAN_IT_FMP0

}


/**
	* @Data   2020-03-13 
  * @brief  VSEC电机设置duty频率
	* @param  CAN_HandleTypeDef* CAN_Num   需要发送的CANx
						uint8_t controller_id        需要控制的电机ID
						float duty    						   需要更改成为的duty是多少
  * @retval void
	* @fallback None
  */
void VSEC_SetDuty(CAN_HandleTypeDef* CAN_Num ,uint8_t controller_id, float duty) 
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(duty * 100000.0f), &send_index);//数据类型转换
	VESC_CANTransmit(CAN_Num,controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8),buffer, send_index);//通过CAN发送
//前半部分是被控制驱动的ID号，后半部分是枚举类型中的命令。

}



/**
	* @Data   2020-03-13 
  * @brief  VSEC电机设置current电流
	* @param  CAN_HandleTypeDef* CAN_Num   需要发送的CANx
						uint8_t controller_id        需要控制的电机ID
						float current    						 需要更改成为的电流是多少
  * @retval void
	* @fallback None
  */
void VSEC_SetCurrent(CAN_HandleTypeDef* CAN_Num ,uint8_t controller_id, float current) 
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
	VESC_CANTransmit(CAN_Num,controller_id | \
			((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}



/**
	* @Data   2020-03-13 
  * @brief  VSEC电机设置rpm转速
	* @param  CAN_HandleTypeDef* CAN_Num   需要发送的CANx
						uint8_t controller_id        需要控制的电机ID
						float rpm    						     需要更改成为的转速是多少
  * @retval void
	* @fallback None
  */
void VSEC_SetRpm(CAN_HandleTypeDef* CAN_Num ,uint8_t controller_id, float rpm) 
{
	int32_t send_index = 0;
	uint8_t buffer[4];
	buffer_append_int32(buffer, (int32_t)rpm, &send_index);
	VESC_CANTransmit(CAN_Num,controller_id |
			((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}



/**
	* @Data   2020-03-13 
	* @brief  VSEC CAN数据发送
  * @param  CAN_HandleTypeDef* CANx, 对应的是哪一个CAN
						uint32_t id, 						 其对应的ID是多少
						uint8_t *data,					 需要发送的数据所指向的指针
						uint8_t len              数据长度
  * @retval void
	* @fallback None
  */
void VESC_CANTransmit(CAN_HandleTypeDef* CANx,uint32_t id, uint8_t *data,uint8_t len)  
{	  
	BLDCCAN_Txmsg BLDC_TxMessage;
	
	if(len > 8)
	{
	   len = 8;
	}
	
	BLDC_TxMessage.TxMessageHeader.ExtId = id;;					 
	BLDC_TxMessage.TxMessageHeader.IDE = CAN_ID_EXT;  //ID类型
	BLDC_TxMessage.TxMessageHeader.RTR = CAN_RTR_DATA;				 //发送的为数据
	BLDC_TxMessage.TxMessageHeader.DLC = len;						 //数据长度为8字节
	BLDC_TxMessage.TxMessageHeader.TransmitGlobalTime = DISABLE;
	
	memcpy(BLDC_TxMessage.Data, data, len);//从CANTxFrame中复制数据
	HAL_CAN_AddTxMessage(CANx,&BLDC_TxMessage.TxMessageHeader,data,(uint32_t*)CAN_TX_MAILBOX0);
}


/**
	* @Data   2020-03-13 
  * @brief  buffer转化成为高低位赋值（数据类型转换）
  * @param  uint8_t* buffer   需要转换的数值
						int16_t number    需要转换的数字
						int32_t *index    数组位置，目录
  * @retval void
	* @fallback None
  */
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) 
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index) 
{
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) 
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index) 
{
	buffer[(*index)++] = number >> 24;
	buffer[(*index)++] = number >> 16;
	buffer[(*index)++] = number >> 8;
	buffer[(*index)++] = number;
}

void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) 
{
    buffer_append_int16(buffer, (int16_t)(number * scale), index);
}

void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) 
{
    buffer_append_int32(buffer, (int32_t)(number * scale), index);
}

int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index)
{
	
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) 
{
	uint16_t res = 	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) 
{
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}


uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) 
{
	uint32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) 
{
    return (float)buffer_get_int16(buffer, index) / scale;
}

float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) 
{
    return (float)buffer_get_int32(buffer, index) / scale;
}
/* =========================== Funtions End=========================== */

