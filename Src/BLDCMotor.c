/**
  ******************************************************************************
  * @file    BLDCMotor.c
  * @author  ����
  * @version V3.21
  * @date		 2021-03-12 ���DJIMoter��BLDCMotor�ĺϲ������ṹ����дBLDCMotor����
						 2021-03-13 ���ο���ʹ��define������bufferapp���ͺ������Ӷ����ٴ���������ջ����������û��ʲô˼·
						 2021-03-13 ���Ľṹ�帳ֵΪ��ָ�븳ֵ���ͣ���ԭ����ָ�븳ֵ��Ϊ������������鿴
						 
						 
  * @brief   BLDC���ֵ�����Ͻṹ
						 
	* @funtion(in) 
						 
	* @value(share)   
  ******************************************************************************
  */
#include "BLDCMotor.h"
#include "handle.h"
/* =========================== FuntionsPulk Begin=========================== */
void BLDC_Motor_CAN2_IT_Init(void);
void BLDCMotor_Process(CAN_RxTypedef RxMessage);
void CAN2_Handler(CAN_HandleTypeDef *hcan);

/*ֵ������*/
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
/*BLDC��������ṹ��*/
BLDC_BJMMotors_t BLDC_Motors;

/*BLDC��������Բ���*/
BLDC_BJMFunction_t BLDCMotorFunction = BLDC_BJMFunctionGroundInit;

/*BLDC����ָ��궨����Ϊ�˷���鿴��*/
CAN_PACKET_ID BLDC_Cmd;

/*��������ָ�룬���㵱ǰ֡���ݣ�ֱ����ָ����е�ַ�����Ƚϱ��*/
can_status_msg *stat_tmp;
can_status_msg_2 *stat_tmp_2;
can_status_msg_3 *stat_tmp_3;
can_status_msg_4 *stat_tmp_4;
can_status_msg_5 *stat_tmp_5;
/* =========================== Value End=========================== */

/* =========================== Funtions Begin=========================== */
/**
	* @Data   2020-03-13 
  * @brief  bufferת����Ϊ�ߵ�λ��ֵ����������ת����
  * @param  uint8_t* buffer   ��Ҫת������ֵ
						int16_t number    ��Ҫת��������
						int32_t *index    ����λ�ã�Ŀ¼
  * @retval void
	* @fallback None
  */
	
void BLDCMotor_Process(CAN_RxTypedef RxMessage)
{
	/*�������ݻ��һ�����*/
	uint8_t BLDC_id = 0;
	int32_t list_id = 0;
	
	/*����Ŀ¼��������*/
	int index = 0;
	
	/*��ȡ��Ӧ����չid���룬������Ҫ�洢����һ���ṹ��*/
	BLDC_id = RxMessage.CAN_RxHeader.ExtId;
	list_id = BLDC_id - BLDC_FisMotor_ID;
	
	
	/*������������д������������IDֻ�ܵ�200*/
	if(list_id < 200)
	{
		/*ȡ��չID��8λ����Ӧ������������*/	
		BLDC_Cmd = (RxMessage.CAN_RxHeader.ExtId >>8);
		switch(BLDC_Cmd)///����
		{
			case CAN_PACKET_STATUS:
				/*��ָ��ָ���Ӧ�ĵ�ַ*/
				stat_tmp = &BLDC_Motors.status_msg[list_id];
			
				/*������ֵ*/
				BLDC_Motors.status_msg[list_id].id = BLDC_id;
				BLDC_Motors.status_msg[list_id].rx_time = 0;
				BLDC_Motors.status_msg[list_id].rpm = (float)buffer_get_int32(RxMessage.CAN_RxMessage, &index);
				BLDC_Motors.status_msg[list_id].current = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 10.0f;
				BLDC_Motors.status_msg[list_id].duty = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 1000.0f;
				break;
			
			case CAN_PACKET_STATUS_2:
				/*��ָ��ָ���Ӧ�ĵ�ַ*/
				stat_tmp_2 = &BLDC_Motors.status_msg_2[list_id];
			
				/*������ֵ*/
				BLDC_Motors.status_msg_2[list_id].id = BLDC_id;
				BLDC_Motors.status_msg_2[list_id].rx_time = 0;
				BLDC_Motors.status_msg_2[list_id].amp_hours = (float)buffer_get_int32(RxMessage.CAN_RxMessage, &index) / 1e4f;
				BLDC_Motors.status_msg_2[list_id].amp_hours_charged = (float)buffer_get_int32(RxMessage.CAN_RxMessage, &index) / 1e4f;
				break;
			
			case CAN_PACKET_STATUS_3:
				/*��ָ��ָ���Ӧ�ĵ�ַ*/
				stat_tmp_3 = &BLDC_Motors.status_msg_3[list_id];
				
				/*������ֵ*/
				BLDC_Motors.status_msg_3[list_id].id = BLDC_id;
				BLDC_Motors.status_msg_3[list_id].rx_time = 0;
				BLDC_Motors.status_msg_3[list_id].watt_hours = (float)buffer_get_int32(RxMessage.CAN_RxMessage, &index) / 1e4f;
				BLDC_Motors.status_msg_3[list_id].watt_hours_charged = (float)buffer_get_int32(RxMessage.CAN_RxMessage, &index) / 1e4f;
				break;
			
			case CAN_PACKET_STATUS_4:
				/*��ָ��ָ���Ӧ�ĵ�ַ*/
				stat_tmp_4 = &BLDC_Motors.status_msg_4[list_id];
			
				/*������ֵ*/
				BLDC_Motors.status_msg_4[list_id].id = BLDC_id;
				BLDC_Motors.status_msg_4[list_id].rx_time = 0;
				BLDC_Motors.status_msg_4[list_id].temp_fet = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 10.0f;
				BLDC_Motors.status_msg_4[list_id].temp_motor = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 10.0f;
				BLDC_Motors.status_msg_4[list_id].current_in = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 10.0f;
				BLDC_Motors.status_msg_4[list_id].pid_pos_now = (float)buffer_get_int16(RxMessage.CAN_RxMessage, &index) / 50.0f;
				break;
			
			case CAN_PACKET_STATUS_5:
				/*��ָ��ָ���Ӧ�ĵ�ַ*/
				stat_tmp_5 = &BLDC_Motors.status_msg_5[list_id];
			
				/*������ֵ*/
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
  * @brief  CAN2��������ʹ���жϣ�
  * @param  void
  * @retval void
	* @fallback None
  */
void CAN2_Handler(CAN_HandleTypeDef *hcan)
{
	if (__HAL_CAN_GET_IT_SOURCE(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING))
	{
		CAN_RxTypedef CAN_RxMessage;
		
		/*��ǵ�ǰ�ź�ΪCAN1�ź�,�����ն�Ӧ��CAN1���ź�*/
		CAN_RxMessage.CAN_Switch = 2;
		HAL_CAN_GetRxMessage(&hcan2,
												CAN_RX_FIFO0,
												&CAN_RxMessage.CAN_RxHeader,
												CAN_RxMessage.CAN_RxMessage);
		
		/*�����ݷ��͵���������*/
		xQueueSendToBackFromISR(xQueueCanReceiveHandle,&CAN_RxMessage,0);
		
		/*�����жϱ�־λ*/
		__HAL_CAN_CLEAR_FLAG(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
}




/**
  * @Data    2020-12-07
  * @brief   ��ʼ��CAN2�жϺ��������ڽ���BLDCMotor�ı���
  * @param   void
  * @retval  void
  */
void BLDC_Motor_CAN2_IT_Init(void)
{
	/*ʹ���˲���*/
	CAN2_FILTER_Init(CAN2_Filter);
	
	/*����CAN*/
	HAL_CAN_Start(&hcan2);
	
	/*ʹ��CAN��IT�ж�*/
	__HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);         //  CAN_IT_FMP0

}

/**
	* @Data   2020-03-13 
  * @brief  bufferת����Ϊ�ߵ�λ��ֵ����������ת����
  * @param  uint8_t* buffer   ��Ҫת������ֵ
						int16_t number    ��Ҫת��������
						int32_t *index    ����λ�ã�Ŀ¼
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

