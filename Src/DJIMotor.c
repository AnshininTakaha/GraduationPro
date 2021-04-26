/**
  ******************************************************************************
  * @file    DJIMotor.c
  * @author  口无
  * @version V3.21
  * @date		 2020-12-07 加入3508选型电机
						 2020-12-29 加入M2006选型电机，整合3508电机内容
						 2020-12-30 整合两个电机的内容，封装成typedef
						 2021-02-12 加入分布id判定机制，修复1.26.0固件包导致CAN掉线问题
						 2021-03-06 由于各种因素更换成为1.25.0固件包，使得CAN部分兼容性加强
						 2021-03-08 C extern对象性封装化，调整具体接口兼容性
						 2021-03-10 修复新版本固件库CAN1，CAN2通信不正常丢帧现象，更换1.26.0固件包
						 2021-04-28 完整代码封装，修改代码基本库
						 
  * @brief   DJI各种电机整合结构
						 
	* @funtion(in) DJI_Motor3508Process             3508电机解码
								 DJI_Motor_CAN1_IT_Init           CAN1初始化
								 DJI_Motor_CAN2_IT_Init           CAN2初始化
								 CAN1_Handler                     CAN1中断处理函数
	* @value(share)DJIMotorFunction                 DJI相关功能函数
								 M3508_MoonWheel[x]               相关3508月球轮的具体信息
  ******************************************************************************
  */
	
#include "DJIMotor.h"
#include "handle.h"

/* =========================== FuntionsPulk Begin=========================== */
void DJI_Motor3508Process(CAN_RxTypedef RxMessage);
void DJI_Motor_CAN1_IT_Init(void);
void CAN1_DJIHandler(CAN_HandleTypeDef *hcan);
void DJIMotor_Set3508Current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void CAN_SendData(CAN_HandleTypeDef* CANx, uint8_t id_type, uint32_t id, uint8_t data[8]);
/* =========================== FuntionsPulk End=========================== */

/* =========================== Value Begin=========================== */
DJI_MotorInit_t DJIMotorFunction = DJIMotorGroundInit;
/*3508电机基本运动数据储存结构体，月球轮四个*/

DJI_Motor3508Folk_t M3508_MoonWheel[4] = {DJI_Motor3508GroundInitRB,
																					DJI_Motor3508GroundInitLB,
																					DJI_Motor3508GroundInitLF,
																					DJI_Motor3508GroundInitRF};

/* =========================== Value End=========================== */

/* =========================== Funtions Begin=========================== */

/**
	* @Data   2020-12-07 
  * @brief  解码3508电机函数
  * @param  CAN_RxTypedef RxMessage CAN解码函数
  * @retval void
	* @fallback None
  */
void DJI_Motor3508Process(CAN_RxTypedef RxMessage) 
{
	/*确定是否在ID范围，获取对应排序的轮子是第几个，判断是哪个ID*/
	uint32_t StdId = RxMessage.CAN_RxHeader.StdId;
	uint8_t list_id = (int32_t)(RxMessage.CAN_RxHeader.StdId - M3508_READID_START);
	if(!(StdId >= M3508_READID_START && StdId <= M3508_READID_END))
	{
		return;
	}
			
	/*解码*/
	M3508_MoonWheel[list_id].Feedback.ReadAngle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
	M3508_MoonWheel[list_id].Feedback.ReadSpeed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
	M3508_MoonWheel[list_id].Feedback.ReadCurrent = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
	M3508_MoonWheel[list_id].Feedback.temperture = RxMessage.CAN_RxMessage[6];
			
	/*圈数统计（这里不一定要用）*/
	#ifdef Roll_CountingM3508
	if(M3508_MoonWheel[list_id].Feedback.ReadAngle - M3508_MoonWheel[list_id].Value.lastAngle < -6000)
	{
			M3508_MoonWheel[list_id].Value.turnCount++;
	}
	else if(M3508_MoonWheel[list_id].Feedback.ReadAngle - M3508_MoonWheel[list_id].Value.lastAngle > 6000)
	{
			M3508_MoonWheel[list_id].Value.turnCount--;
	}
			
	/*总圈数计算&值传递*/
	M3508_MoonWheel[list_id].Value.totalAngle = M3508_MoonWheel[list_id].Feedback.ReadAngle + (M3508PerR * M3508_MoonWheel[list_id].Value.turnCount);
	M3508_MoonWheel[list_id].Value.lastAngle = M3508_MoonWheel[list_id].Feedback.ReadAngle;
			
	/*totalAngle fulltank process..*/
	#endif
	
	#ifdef USE_3508StepGaining
	/*帧率统计*/
	M3508_MoonWheel[list_id].Frame.Frame++;
			
	#endif
}



/**
	* @Data   2020-12-07 
  * @brief  CAN1处理函数（使用中断）
  * @param  void
  * @retval void
	* @fallback None
  */
void CAN1_DJIHandler(CAN_HandleTypeDef *hcan)
{
	if (__HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING))
	{
		CAN_RxTypedef CAN_RxMessage;
		
		/*标记当前信号为CAN1信号,并接收对应的CAN1的信号*/
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
	* @Data   2020-12-07 
  * @brief  初始化CAN1中断函数，用于接收Motor的报文
  * @param  void
  * @retval void
	* @fallback None
  */
void DJI_Motor_CAN1_IT_Init(void)
{
	/*使能CAN1Filter滤波器*/
	CAN1_FILTER_Init(CAN1_Filter);
	
	/*启用CAN1*/
	HAL_CAN_Start(&hcan1);
	
	/*使能CAN1的IT中断*/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	
		
}



/**
	* @Data   2021-04-05
  * @brief  设置3508电机电流
  * @param  void
  * @retval void
	* @fallback None
  */
void DJIMotor_Set3508Current(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) 
{

	uint8_t data[8];

	//数据格式详见说明书P32
	data[0] = iq1 >> 8;
	data[1] = iq1;
	data[2] = iq2 >> 8;
	data[3] = iq2;
	data[4] = iq3 >> 8;
	data[5] = iq3;
	data[6] = iq4 >> 8;
	data[7] = iq4;

	CAN_SendData(&hcan1, CAN_ID_STD, M3508_SENDID, data);

}



/**
  * @brief  CAN发送数据
  * @param  CANx 		CAN编号
  * 				id_type ·	id类型 CAN_ID_STD， CAN_ID_EXT
  *					id			id号
  * 				data[8]		8个数据
  * @retval None
  */
void CAN_SendData(CAN_HandleTypeDef* CANx, uint8_t id_type, uint32_t id, uint8_t data[8])  
{	  
	CAN_TxHeaderTypeDef TxMessage;
	
	if(id_type == CAN_ID_STD){
		TxMessage.StdId = id;						 
	}
	else{
		TxMessage.ExtId = id;					 //ID号
	}
	
	TxMessage.IDE = id_type;					 //ID类型
	TxMessage.RTR = CAN_RTR_DATA;				 //发送的为数据
	TxMessage.DLC = 0x08;						 //数据长度为8字节
	TxMessage.TransmitGlobalTime = DISABLE;
	
	HAL_CAN_AddTxMessage(CANx,&TxMessage,data,(uint32_t*)CAN_TX_MAILBOX0);
}

/* =========================== Funtions End=========================== */
