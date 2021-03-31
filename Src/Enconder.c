/**
  ******************************************************************************
  * @file    Enconder.c
  * @author  口无
  * @version V3.21
  * @date		 2020-12-21 初次考虑加入流式编码器，进行选型
						 2021-02-01 流失编码器通过测试，但是协议部分有待优化，会出现收发卡段现象
						 2021-03-13 确定流失编码器协议，开始更改旧协议
						 
  * @brief   编码器各种整合结构
						 
	* @funtion(in) 
						 
	* @value(share)   
  ******************************************************************************
  */
#include "Enconder.h"
#include "handle.h"
/* =========================== FuntionsPulk Begin=========================== */
void Encoder_Read_Config(uint8_t Encoder_id);
void Encoder_ID_Config(uint8_t old_id,uint8_t new_id);
void Encoder_Baud_Config(uint8_t Encoder_id,uint32_t Baudrate);
void Encoder_Read_Mode(uint8_t Encoder_id);
void Encoder_Time_Config(uint8_t Encoder_id,uint16_t time);
void Encoder_Zero_Config(uint8_t Encoder_id);
void Encoder_Process(CAN_RxTypedef RxMessage);
void CAN1_EncHandler(CAN_HandleTypeDef *hcan);
void Enc_CAN1_IT_Init(void);
/* =========================== FuntionsPulk End=========================== */

/* =========================== Value Begin=========================== */
/*编码器基本功能性函数*/
EnconderFunction_t EnconderFunction = EnconderFunction_GroundInit;

/*编码器数据储存结构体*/
Enconder_Folk_t Enconder[4];
/* =========================== Value End=========================== */


/* =========================== Funtions Begin=========================== */
/**
	* @Data   2020-03-13 
  * @brief  读取编码器的值(编码器CMD命令)
  * @param  Encoder_id 编码器的ID（范围：2-127）
  * @retval void
	* @fallback None
  */
void Encoder_Read_Config(uint8_t Encoder_id)
{
	uint8_t data[8];
	
	data[0] = 0x04;
	data[1] = Encoder_id;
	data[2] = 0x01;
	data[3] = 0x00;

	CAN_SendData_F0(&hcan1, CAN_ID_STD, Encoder_id, data);
}



/**
  * @brief  设置编码器的ID(编码器CMD命令)
  * @param  old_id 编码器现在的ID（范围：2-127）
            new_id 编码器新的ID（范围：2-127）
  * @retval void
	* @fallback None
  */
void Encoder_ID_Config(uint8_t old_id,uint8_t new_id)
{
	uint8_t data[8];
	
	data[0] = 0x04;
	data[1] = old_id;
	data[2] = 0x02;
	data[3] = new_id;
	
	CAN_SendData_F0(&hcan1, CAN_ID_STD, old_id, data);
}



/**
  * @brief  设置编码器的波特率（新买的编码器默认波特率500KHZ） (编码器CMD命令)
  * @param  Encoder_id 编码器的ID（范围：2-127）
						Baud 要设置的编码器新波特率（0x00:500KHZ,0x01:1MHZ,0X02:250KHZ,...）
            （注意：不要乱改，CAN版本无上位机，改错就报废了）
  * @retval None
	* @fallback None
  */
void Encoder_Baud_Config(uint8_t Encoder_id,uint32_t Baudrate)
{
	uint8_t data[8];
	
	data[0] = 0x04;
	data[1] = Encoder_id;
	data[2] = 0x03;
	data[3] = Baudrate;
	
	CAN_SendData_F0(&hcan1, CAN_ID_STD, Encoder_id, data);
}



/**
  * @brief  开启编码器自动回传模式 (编码器CMD命令)
  * @param  Encoder_id 编码器的ID（范围：2-127）
  * @retval None
	* @fallback None
  */
void Encoder_Read_Mode(uint8_t Encoder_id)
{
	uint8_t data[8];
	
	data[0] = 0x04;
	data[1] = Encoder_id;
	data[2] = 0x04;
	data[3] = 0xAA;
	
	CAN_SendData_F0(&hcan1, CAN_ID_STD, Encoder_id, data);	
}



/**
  * @brief  设置自动回传周期（注意：改的值太小编码器就废了）(编码器CMD命令)
  * @param  Encoder_id 编码器的ID（范围：2-127）
            time 自动回传的时间（数值范围：50-65535,单位：μs，推荐：0x03E8）
  * @retval None
	* @fallback None
  */
void Encoder_Time_Config(uint8_t Encoder_id,uint16_t time)
{
	uint8_t data[8];
	
	data[0] = 0x05;
	data[1] = Encoder_id;
	data[2] = 0x05;
	data[3] = time;
	data[4] = time>>8;
	
	CAN_SendData_F0(&hcan1, CAN_ID_STD, Encoder_id, data);	
}



/**
  * @brief  设置编码器当前位置为零点(编码器CMD命令)
  * @param  Encoder_id 编码器的ID（范围：2-127）
  * @retval None
  */
void Encoder_Zero_Config(uint8_t Encoder_id)
{
	uint8_t data[8];
	
	data[0] = 0x04;
	data[1] = Encoder_id;
	data[2] = 0x06;	
	data[3] = 0x00;
	
	CAN_SendData_F0(&hcan1, CAN_ID_STD, Encoder_id, data);	
}



/**
  * @brief  获取编码器的值（0~4096*50（分辨率4096））(编码器CMD命令)
  * @param  RxMessage 主机ID默认为0x01
  * @retval None
	* @fallback None
  */
void Encoder_Process(CAN_RxTypedef RxMessage)
{
	int list_id = 0;
	/*确认指令码是否正确，指令码为0x01的则为反馈编码器角度的报文*/
	if(RxMessage.CAN_RxHeader.StdId==0x01 && RxMessage.CAN_RxMessage[2]==0x01)
	{
		/*计算是第几个编码器*/
		list_id = RxMessage.CAN_RxMessage[1] - Encoder_READ_ID_START;
		
		/*编码器解码赋值*/
		Enconder[list_id].Enconder_ID = RxMessage.CAN_RxMessage[1];
    Enconder[list_id].Enconder_ReadBackAngle = RxMessage.CAN_RxMessage[6]<<24| \
	  RxMessage.CAN_RxMessage[5]<<16 | RxMessage.CAN_RxMessage[4]<<8| \
		RxMessage.CAN_RxMessage[3];
		
	}
	else return;
}



/**
  * @brief  CAN1中断处理函数
  * @param  CAN_HandleTypeDef *hcan CAN中断处理函数
  * @retval None
	* @fallback None
  */
void CAN1_EncHandler(CAN_HandleTypeDef *hcan)
{
	if (__HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING))
	{
		CAN_RxTypedef CAN_RxMessage;
		
		/*标记当前信号为CAN2信号,并接收对应的CAN2的信号*/
		CAN_RxMessage.CAN_Switch = 1;
		HAL_CAN_GetRxMessage(&hcan1,
												CAN_RX_FIFO0,
												&CAN_RxMessage.CAN_RxHeader,
												CAN_RxMessage.CAN_RxMessage);
		
		/*将数据发送到队列里面*/
		xQueueSendToBackFromISR(xQueueCanReceiveHandle,&CAN_RxMessage,0);
		
		
		/*清理中断标志位*/
		__HAL_CAN_CLEAR_FLAG(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
}


/**
  * @brief  初始化CAN1中断函数 
  * @param  CAN_HandleTypeDef *hcan CAN中断处理函数
  * @retval None
	* @fallback None
  */
void Enc_CAN1_IT_Init(void)
{
	/*使能CAN1Filter滤波器*/
	CAN1_FILTER_Init(CAN1_Filter);
	
	/*启用CAN1*/
	HAL_CAN_Start(&hcan1);
	
	/*使能CAN1的IT中断*/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		
}
/* =========================== Funtions End=========================== */
