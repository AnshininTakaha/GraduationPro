/**
  ******************************************************************************
  * @file    DR16_Remote.c
  * @author  口无
  * @version V3.21
  * @date		 2020-12-03 第三次迭代完成，优化整体代码框架和结构量，优化整体计算复杂度
						 2020-12-27 优化迭代代码，修改FreeRTOS整体结构框架，考虑到各种因素将FreeRTOS版本由CV2更改为CV1
						 2021-03-02 修复CAN1通信问题，更新使用内存库
						 2021-03-08 完全C extern封装化，调整具体接口
						 
  * @brief   DR16 DJI遥控器整体结构
						 
	* @funtion(in) DR16_ReInit                   DR16错误复位函数
								 DR16_Process                  DR16解码处理函数
								 DR16_Handler 								 DR16中断处理函数
								 USART_Receive_DMA_NO_IT       DR16初始化中DMA部分的初始化函数
								 DR16_USART1_IT_Init 					 DR16中断初始化函数
						 
	* @value(share)   DR16                          遥控器解码信息储存结构体
										DR16Buffer[DR16BufferNumber]  遥控器数据流储存数组
										DR16_Export_data              遥控器转换为姿态信息的储存结构体
  ******************************************************************************
  */

#include "DR16_Remote.h"

/* =========================== FuntionsPulk Begin=========================== */
void DR16_ReInit(void);
void DR16_Process(uint8_t *pData);
void DR16_Handler(UART_HandleTypeDef *huart);
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size);
void DR16_USART1_IT_Init(void);
/* =========================== FuntionsPulk End=========================== */

/* =========================== Value Begin=========================== */
/*遥控器解码内容储存结构体初始化*/
DR16_t DR16 = DR16_GroundInit;

/*遥控器数据储存数组初始化*/
uint8_t DR16Buffer[DR16BufferNumber];

/*遥控器数据转换为运动姿态输出结构体*/
DR16_Export_Data_t DR16_Export_data;

/* =========================== Value End=========================== */

/* =========================== Funtions Begin=========================== */

/**
  * @Data    2020-12-19 
  * @brief   DR16数据初始化（错误时复位用）
  * @param   void
  * @retval  void
	* @fallback None
  */
void DR16_ReInit(void)
{
	DR16.rc.ch0 = 0;
	DR16.rc.ch1 = 0;
	DR16.rc.ch2 = 0;
	DR16.rc.ch3 = 0;
	DR16.rc.s_left = 0;
	DR16.rc.s_right = 0;
	DR16.mouse.x = 0;
	DR16.mouse.y = 0;
	DR16.mouse.z = 0;
	DR16.mouse.keyLeft = 0;
	DR16.mouse.keyRight = 0;
	DR16.keyBoard.key_code = 0;
	DR16.rc.ch4_DW = 0;
}



/**
  * @Data    2020-12-19 
  * @brief   DR16遥控器解码函数
  * @param   uint8_t *pData 输入进去解码的对应的数组
  * @retval  void
	* @fallback 改变了DR16_Export_data的值
  */
void DR16_Process(uint8_t *pData)
{
	if (pData == NULL)
	{
		return;
	}
	DR16.rc.ch0 = (pData[0] | (pData[1] << 8)) & 0x07FF;
	DR16.rc.ch1 = ((pData[1] >> 3) | (pData[2] << 5)) & 0x07FF;
	DR16.rc.ch2 = ((pData[2] >> 6) | (pData[3] << 2) | (pData[4] << 10)) & 0x07FF;
	DR16.rc.ch3 = ((pData[4] >> 1) | (pData[5] << 7)) & 0x07FF;
	DR16.rc.s_left = ((pData[5] >> 4) & 0x000C) >> 2;
	DR16.rc.s_right = ((pData[5] >> 4) & 0x0003);
	DR16.mouse.x = (pData[6]) | (pData[7] << 8);
	DR16.mouse.y = (pData[8]) | (pData[9] << 8);
	DR16.mouse.z = (pData[10]) | (pData[11] << 8);
	DR16.mouse.keyLeft = pData[12];
	DR16.mouse.keyRight = pData[13];
	DR16.keyBoard.key_code = pData[14] | (pData[15] << 8);

	//your control code ….
	DR16.rc.ch4_DW = (pData[16] | (pData[17] << 8)) & 0x07FF;
	#ifdef DR16FrameFolk
	DR16.DR16_Frame++;
	#endif

	DR16.rc.ch0 -= 1024;
	DR16.rc.ch1 -= 1024;
	DR16.rc.ch2 -= 1024;
	DR16.rc.ch3 -= 1024;
	DR16.rc.ch4_DW -= 1024;

	/* prevent remote control zero deviation */
	if (DR16.rc.ch0 <= 20 && DR16.rc.ch0 >= -20)
		DR16.rc.ch0 = 0;
	if (DR16.rc.ch1 <= 20 && DR16.rc.ch1 >= -20)
		DR16.rc.ch1 = 0;
	if (DR16.rc.ch2 <= 20 && DR16.rc.ch2 >= -20)
		DR16.rc.ch2 = 0;
	if (DR16.rc.ch3 <= 20 && DR16.rc.ch3 >= -20)
		DR16.rc.ch3 = 0;
	if (DR16.rc.ch4_DW <= 20 && DR16.rc.ch4_DW >= -20)
		DR16.rc.ch4_DW = 0;
	
	/*Backup the value..*/
	/*直接运动基础二通道（解构值）VDx VDy*/
	DR16_Export_data.DR16_Direct_Y_Value = DR16.rc.ch3 * 10.0f;
	DR16_Export_data.DR16_Direct_X_Value = DR16.rc.ch2 *10.0f;
	
	/*直接运动通道角度值AD*/
	DR16_Export_data.DR16_Direct_Angle_Value = atan2(DR16_Export_data.DR16_Direct_X_Value, \
	DR16_Export_data.DR16_Direct_Y_Value)* 180.0f / PI;
	
	/*自旋通道VO，因为AO在底盘绝对层更好操作，所以选择放到下层去做变换*/
	DR16_Export_data.DR16_Omega_Value = DR16.rc.ch0 * 10.0f;
	
	/*OMEGA = +右 : LF:45 RF:135 RB:-135 LB:-45*/
	/*OMEGA = -左 : LF:-135 RF:-45 RB:45 LB:135*/
	
//	DR16_Export_data.DR16_Velocity_Value = sqrt( \
//	((DR16_Export_data.DR16_Direct_X_Value + DR16_Export_data.DR16_Omega_X_Value) * \
//	(DR16_Export_data.DR16_Direct_X_Value + DR16_Export_data.DR16_Omega_X_Value)) + \
//	((DR16_Export_data.DR16_Direct_Y_Value + DR16_Export_data.DR16_Omega_Y_Value) * \
//	(DR16_Export_data.DR16_Direct_Y_Value + DR16_Export_data.DR16_Omega_Y_Value)));
	
	DR16_Export_data.Switch_Left = (RemotePole_e)DR16.rc.s_left;
	DR16_Export_data.Switch_Right = (RemotePole_e)DR16.rc.s_right;
}



/**
  * @Data    2020-12-19 
  * @brief   DR16处理函数
  * @param   UART_HandleTypeDef *huart 输入对应的产生中断的外设
  * @retval  void
	* @fallback None
  */
void DR16_Handler(UART_HandleTypeDef *huart)
{

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) &&
		__HAL_UART_GET_IT_SOURCE(huart, UART_IT_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		__HAL_DMA_DISABLE(huart->hdmarx);

		//if(DR16BufferNumber - DMA_GET_COUNTER(huart->hdmarx->Instance) == DR16BufferTruthNumber)
		if (__HAL_DMA_GET_COUNTER(huart->hdmarx) == DR16BufferLastNumber)
		{
			DR16.DR16_Process(DR16Buffer);
		}

		__HAL_DMA_SET_COUNTER(huart->hdmarx, DR16BufferNumber);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}



/**
  * @Data    2020-12-19 
  * @brief   USART_DMA传输（接收）开启和重定向
  * @param   UART_HandleTypeDef* huart 输入对应的需要初始化dma的外设
						 uint8_t* pData            定向dma目标的结构体
						 uint32_t Size             定向dma结构的传输数据多少
  * @retval  void
	* @fallback None
  */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{

		/*检测当前huart状态*/
		if(huart->RxState == HAL_UART_STATE_READY)
		{
			/*输入的地址或者数据有问题的话*/
			if((pData == NULL) || (Size == 0))
			{
					return HAL_ERROR;
			}
			
			/*huart里面对应的Rx变量重定向*/
			huart->pRxBuffPtr = pData;
			huart->RxXferSize = Size;
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			
			/*开启huart1上的RX_DMA*/
			HAL_DMA_Start(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)pData,Size);
			
			/*只开启对应DMA上面的Rx功能（如果是开启Tx的话就是USART_CR3_DMAT）*/
			SET_BIT(huart->Instance->CR3, USART_CR3_DMAR);
			
			
		}
		else
		{
			return HAL_BUSY;
		}

		return HAL_OK;
}


/**
  * @Data    2020-07-13 
  * @brief   DR16中断初始化
  * @param   void
  * @retval  void
  */
void DR16_USART1_IT_Init(void)
{
	/*清空标志位然后使能USART 和 使能USART的空闲中断*/
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE(&huart1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	/*开启DMA传输（但是不开启DMA中断）*/
	USART_Receive_DMA_NO_IT(&huart1,DR16Buffer,DR16BufferNumber);

}
/* =========================== Funtions End=========================== */
