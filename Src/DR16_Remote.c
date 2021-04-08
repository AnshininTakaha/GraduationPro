/**
  ******************************************************************************
  * @file    DR16_Remote.c
  * @author  ����
  * @version V3.21
  * @date		 2020-12-03 �����ε�����ɣ��Ż���������ܺͽṹ�����Ż�������㸴�Ӷ�
						 2020-12-27 �Ż��������룬�޸�FreeRTOS����ṹ��ܣ����ǵ��������ؽ�FreeRTOS�汾��CV2����ΪCV1
						 2021-03-02 �޸�CAN1ͨ�����⣬����ʹ���ڴ��
						 2021-03-08 ��ȫC extern��װ������������ӿ�
						 
  * @brief   DR16 DJIң��������ṹ
						 
	* @funtion(in) DR16_ReInit                   DR16����λ����
								 DR16_Process                  DR16���봦����
								 DR16_Handler 								 DR16�жϴ�����
								 USART_Receive_DMA_NO_IT       DR16��ʼ����DMA���ֵĳ�ʼ������
								 DR16_USART1_IT_Init 					 DR16�жϳ�ʼ������
						 
	* @value(share)   DR16                          ң����������Ϣ����ṹ��
										DR16Buffer[DR16BufferNumber]  ң������������������
										DR16_Export_data              ң����ת��Ϊ��̬��Ϣ�Ĵ���ṹ��
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
/*ң�����������ݴ���ṹ���ʼ��*/
DR16_t DR16 = DR16_GroundInit;

/*ң�������ݴ��������ʼ��*/
uint8_t DR16Buffer[DR16BufferNumber];

/*ң��������ת��Ϊ�˶���̬����ṹ��*/
DR16_Export_Data_t DR16_Export_data;

/* =========================== Value End=========================== */

/* =========================== Funtions Begin=========================== */

/**
  * @Data    2020-12-19 
  * @brief   DR16���ݳ�ʼ��������ʱ��λ�ã�
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
  * @brief   DR16ң�������뺯��
  * @param   uint8_t *pData �����ȥ����Ķ�Ӧ������
  * @retval  void
	* @fallback �ı���DR16_Export_data��ֵ
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

	//your control code ��.
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
	/*ֱ���˶�������ͨ�����⹹ֵ��VDx VDy*/
	DR16_Export_data.DR16_Direct_Y_Value = DR16.rc.ch3 * 10.0f;
	DR16_Export_data.DR16_Direct_X_Value = DR16.rc.ch2 *10.0f;
	
	/*ֱ���˶�ͨ���Ƕ�ֵAD*/
	DR16_Export_data.DR16_Direct_Angle_Value = atan2(DR16_Export_data.DR16_Direct_X_Value, \
	DR16_Export_data.DR16_Direct_Y_Value)* 180.0f / PI;
	
	/*����ͨ��VO����ΪAO�ڵ��̾��Բ���ò���������ѡ��ŵ��²�ȥ���任*/
	DR16_Export_data.DR16_Omega_Value = DR16.rc.ch0 * 10.0f;
	
	/*OMEGA = +�� : LF:45 RF:135 RB:-135 LB:-45*/
	/*OMEGA = -�� : LF:-135 RF:-45 RB:45 LB:135*/
	
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
  * @brief   DR16������
  * @param   UART_HandleTypeDef *huart �����Ӧ�Ĳ����жϵ�����
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
  * @brief   USART_DMA���䣨���գ��������ض���
  * @param   UART_HandleTypeDef* huart �����Ӧ����Ҫ��ʼ��dma������
						 uint8_t* pData            ����dmaĿ��Ľṹ��
						 uint32_t Size             ����dma�ṹ�Ĵ������ݶ���
  * @retval  void
	* @fallback None
  */
static int USART_Receive_DMA_NO_IT(UART_HandleTypeDef* huart, uint8_t* pData, uint32_t Size)
{

		/*��⵱ǰhuart״̬*/
		if(huart->RxState == HAL_UART_STATE_READY)
		{
			/*����ĵ�ַ��������������Ļ�*/
			if((pData == NULL) || (Size == 0))
			{
					return HAL_ERROR;
			}
			
			/*huart�����Ӧ��Rx�����ض���*/
			huart->pRxBuffPtr = pData;
			huart->RxXferSize = Size;
			huart->ErrorCode = HAL_UART_ERROR_NONE;
			
			/*����huart1�ϵ�RX_DMA*/
			HAL_DMA_Start(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)pData,Size);
			
			/*ֻ������ӦDMA�����Rx���ܣ�����ǿ���Tx�Ļ�����USART_CR3_DMAT��*/
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
  * @brief   DR16�жϳ�ʼ��
  * @param   void
  * @retval  void
  */
void DR16_USART1_IT_Init(void)
{
	/*��ձ�־λȻ��ʹ��USART �� ʹ��USART�Ŀ����ж�*/
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE(&huart1);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	/*����DMA���䣨���ǲ�����DMA�жϣ�*/
	USART_Receive_DMA_NO_IT(&huart1,DR16Buffer,DR16BufferNumber);

}
/* =========================== Funtions End=========================== */
