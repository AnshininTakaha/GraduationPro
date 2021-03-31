/**
  ******************************************************************************
  * @file    Enconder.c
  * @author  ����
  * @version V3.21
  * @date		 2020-12-21 ���ο��Ǽ�����ʽ������������ѡ��
						 2021-02-01 ��ʧ������ͨ�����ԣ�����Э�鲿���д��Ż���������շ���������
						 2021-03-13 ȷ����ʧ������Э�飬��ʼ���ľ�Э��
						 
  * @brief   �������������Ͻṹ
						 
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
/*���������������Ժ���*/
EnconderFunction_t EnconderFunction = EnconderFunction_GroundInit;

/*���������ݴ���ṹ��*/
Enconder_Folk_t Enconder[4];
/* =========================== Value End=========================== */


/* =========================== Funtions Begin=========================== */
/**
	* @Data   2020-03-13 
  * @brief  ��ȡ��������ֵ(������CMD����)
  * @param  Encoder_id ��������ID����Χ��2-127��
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
  * @brief  ���ñ�������ID(������CMD����)
  * @param  old_id ���������ڵ�ID����Χ��2-127��
            new_id �������µ�ID����Χ��2-127��
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
  * @brief  ���ñ������Ĳ����ʣ�����ı�����Ĭ�ϲ�����500KHZ�� (������CMD����)
  * @param  Encoder_id ��������ID����Χ��2-127��
						Baud Ҫ���õı������²����ʣ�0x00:500KHZ,0x01:1MHZ,0X02:250KHZ,...��
            ��ע�⣺��Ҫ�Ҹģ�CAN�汾����λ�����Ĵ�ͱ����ˣ�
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
  * @brief  �����������Զ��ش�ģʽ (������CMD����)
  * @param  Encoder_id ��������ID����Χ��2-127��
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
  * @brief  �����Զ��ش����ڣ�ע�⣺�ĵ�ֵ̫С�������ͷ��ˣ�(������CMD����)
  * @param  Encoder_id ��������ID����Χ��2-127��
            time �Զ��ش���ʱ�䣨��ֵ��Χ��50-65535,��λ����s���Ƽ���0x03E8��
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
  * @brief  ���ñ�������ǰλ��Ϊ���(������CMD����)
  * @param  Encoder_id ��������ID����Χ��2-127��
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
  * @brief  ��ȡ��������ֵ��0~4096*50���ֱ���4096����(������CMD����)
  * @param  RxMessage ����IDĬ��Ϊ0x01
  * @retval None
	* @fallback None
  */
void Encoder_Process(CAN_RxTypedef RxMessage)
{
	int list_id = 0;
	/*ȷ��ָ�����Ƿ���ȷ��ָ����Ϊ0x01����Ϊ�����������Ƕȵı���*/
	if(RxMessage.CAN_RxHeader.StdId==0x01 && RxMessage.CAN_RxMessage[2]==0x01)
	{
		/*�����ǵڼ���������*/
		list_id = RxMessage.CAN_RxMessage[1] - Encoder_READ_ID_START;
		
		/*���������븳ֵ*/
		Enconder[list_id].Enconder_ID = RxMessage.CAN_RxMessage[1];
    Enconder[list_id].Enconder_ReadBackAngle = RxMessage.CAN_RxMessage[6]<<24| \
	  RxMessage.CAN_RxMessage[5]<<16 | RxMessage.CAN_RxMessage[4]<<8| \
		RxMessage.CAN_RxMessage[3];
		
	}
	else return;
}



/**
  * @brief  CAN1�жϴ�����
  * @param  CAN_HandleTypeDef *hcan CAN�жϴ�����
  * @retval None
	* @fallback None
  */
void CAN1_EncHandler(CAN_HandleTypeDef *hcan)
{
	if (__HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING))
	{
		CAN_RxTypedef CAN_RxMessage;
		
		/*��ǵ�ǰ�ź�ΪCAN2�ź�,�����ն�Ӧ��CAN2���ź�*/
		CAN_RxMessage.CAN_Switch = 1;
		HAL_CAN_GetRxMessage(&hcan1,
												CAN_RX_FIFO0,
												&CAN_RxMessage.CAN_RxHeader,
												CAN_RxMessage.CAN_RxMessage);
		
		/*�����ݷ��͵���������*/
		xQueueSendToBackFromISR(xQueueCanReceiveHandle,&CAN_RxMessage,0);
		
		
		/*�����жϱ�־λ*/
		__HAL_CAN_CLEAR_FLAG(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
}


/**
  * @brief  ��ʼ��CAN1�жϺ��� 
  * @param  CAN_HandleTypeDef *hcan CAN�жϴ�����
  * @retval None
	* @fallback None
  */
void Enc_CAN1_IT_Init(void)
{
	/*ʹ��CAN1Filter�˲���*/
	CAN1_FILTER_Init(CAN1_Filter);
	
	/*����CAN1*/
	HAL_CAN_Start(&hcan1);
	
	/*ʹ��CAN1��IT�ж�*/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		
}
/* =========================== Funtions End=========================== */
