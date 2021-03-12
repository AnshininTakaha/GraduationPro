/**
  ******************************************************************************
  * @file    DJIMotor.c
  * @author  ����
  * @version V3.21
  * @date		 2020-12-07 ����3508ѡ�͵��
						 2020-12-29 ����M2006ѡ�͵��������3508�������
						 2020-12-30 ����������������ݣ���װ��typedef
						 2021-02-12 ����ֲ�id�ж����ƣ��޸�1.26.0�̼�������CAN��������
						 2021-03-06 ���ڸ������ظ�����Ϊ1.25.0�̼�����ʹ��CAN���ּ����Լ�ǿ
						 2021-03-08 C extern�����Է�װ������������ӿڼ�����
						 2021-03-10 �޸��°汾�̼���CAN1��CAN2ͨ�Ų�������֡���󣬸���1.26.0�̼���
						 
						 
  * @brief   DJI���ֵ�����Ͻṹ
						 
	* @funtion(in) DJI_Motor3508Process             3508�������
								 DJI_Motor_CAN1_IT_Init           CAN1��ʼ��
								 DJI_Motor_CAN2_IT_Init           CAN2��ʼ��
								 CAN1_Handler                     CAN1�жϴ�����
	* @value(share)DJIMotorFunction                 DJI��ع��ܺ���
								 M3508_MoonWheel[x]               ���3508�����ֵľ�����Ϣ
  ******************************************************************************
  */
	
#include "DJIMotor.h"
#include "handle.h"

/* =========================== FuntionsPulk Begin=========================== */
void DJI_Motor3508Process(CAN_RxTypedef RxMessage);
void DJI_Motor_CAN1_IT_Init(void);
void CAN1_Handler(CAN_HandleTypeDef *hcan);
/* =========================== FuntionsPulk End=========================== */

/* =========================== Value Begin=========================== */
DJI_MotorInit_t DJIMotorFunction = DJIMotorGroundInit;
/*3508��������˶����ݴ���ṹ�壬�������ĸ�*/
DJI_Motor3508Folk_t M3508_MoonWheel[4] = {DJI_Motor3508GroundInit,
																					DJI_Motor3508GroundInit,
																					DJI_Motor3508GroundInit,
																					DJI_Motor3508GroundInit};

/* =========================== Value End=========================== */

/* =========================== Funtions Begin=========================== */

/**
	* @Data   2020-12-07 
  * @brief  ����3508�������
  * @param  CAN_RxTypedef RxMessage CAN���뺯��
  * @retval void
	* @fallback None
  */
void DJI_Motor3508Process(CAN_RxTypedef RxMessage) 
{
	/*ȷ���Ƿ���ID��Χ����ȡ��Ӧ����������ǵڼ������ж����ĸ�ID*/
	uint32_t StdId = RxMessage.CAN_RxHeader.StdId;
	uint8_t list_id = (int32_t)(RxMessage.CAN_RxHeader.StdId - M3508_READID_START);
	if(!(StdId >= M3508_READID_START && StdId <= M3508_READID_END))
	{
		return;
	}
			
	/*����*/
	M3508_MoonWheel[list_id].Feedback.ReadAngle = (uint16_t)(RxMessage.CAN_RxMessage[0] << 8 | RxMessage.CAN_RxMessage[1]);
	M3508_MoonWheel[list_id].Feedback.ReadSpeed = (int16_t)(RxMessage.CAN_RxMessage[2] << 8 | RxMessage.CAN_RxMessage[3]);
	M3508_MoonWheel[list_id].Feedback.ReadCurrent = (int16_t)(RxMessage.CAN_RxMessage[4] << 8 | RxMessage.CAN_RxMessage[5]);
	M3508_MoonWheel[list_id].Feedback.temperture = RxMessage.CAN_RxMessage[6];
			
	/*Ȧ��ͳ�ƣ����ﲻһ��Ҫ�ã�*/
	#ifdef Roll_CountingM3508
	if(M3508_MoonWheel[list_id].Feedback.ReadAngle - M3508_MoonWheel[list_id].Value.lastAngle < -6000)
	{
			M3508_MoonWheel[list_id].Value.turnCount++;
	}
	else if(M3508_MoonWheel[list_id].Feedback.ReadAngle - M3508_MoonWheel[list_id].Value.lastAngle > 6000)
	{
			M3508_MoonWheel[list_id].Value.turnCount--;
	}
			
	/*��Ȧ������&ֵ����*/
	M3508_MoonWheel[list_id].Value.totalAngle = M3508_MoonWheel[list_id].Feedback.ReadAngle + (M3508PerR * M3508_MoonWheel[list_id].Value.turnCount);
	M3508_MoonWheel[list_id].Value.lastAngle = M3508_MoonWheel[list_id].Feedback.ReadAngle;
			
	/*totalAngle fulltank process..*/
	#endif
			
	#ifdef USE_3508StepGaining
	/*֡��ͳ��*/
	M3508_MoonWheel[list_id].Frame.Frame++;
			
	#endif
}

/**
	* @Data   2020-12-07 
  * @brief  CAN1��������ʹ���жϣ�
  * @param  void
  * @retval void
	* @fallback None
  */
void CAN1_Handler(CAN_HandleTypeDef *hcan)
{
	if (__HAL_CAN_GET_IT_SOURCE(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING))
	{
		CAN_RxTypedef CAN_RxMessage;
		
		/*��ǵ�ǰ�ź�ΪCAN1�ź�,�����ն�Ӧ��CAN1���ź�*/
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
	* @Data   2020-12-07 
  * @brief  ��ʼ��CAN1�жϺ��������ڽ���Motor�ı���
  * @param  void
  * @retval void
	* @fallback None
  */
void DJI_Motor_CAN1_IT_Init(void)
{
	/*ʹ��CAN1Filter�˲���*/
	CAN1_FILTER_Init(CAN1_Filter);
	
	/*����CAN1*/
	HAL_CAN_Start(&hcan1);
	
	/*ʹ��CAN1��IT�ж�*/
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
		
}




/* =========================== Funtions End=========================== */
