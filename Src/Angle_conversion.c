/**
  ******************************************************************************
  * @file    Angle_conversation.c
  * @author  ����
  * @version V1.0
  * @date		 2021-04-05 �Ż��㷨���
						 
  * @brief   �Ƕ�ת�������㷨
						 
	* @funtion(share) 
										
	* @funtion(in)
  ******************************************************************************
  */
#include "Angle_conversion.h"
/* =========================== FuntionsPulk Begin=========================== */
float Angle_into_Rad(float Angle);
float Rad_into_Angle(float Rad);
/* =========================== FuntionsPulk End=========================== */

/* =========================== Value Begin=========================== */
Translation_t Translation = GroundInit_Translation;
/* =========================== Value End=========================== */

/* =========================== Funtions Begin=========================== */
/**
	* @Data   2021-04-05 
  * @brief  �Ƕ�ת����Ϊ����
  * @param  float Angle �Ƕ�
  * @retval void
	* @fallback None
  */
float Angle_into_Rad(float Angle)
{
	return Angle*(PI/180.0f);
}

/**
	* @Data   2021-04-05 
  * @brief  ����ת����Ϊ�Ƕ�
  * @param  float Rad ����
  * @retval void
	* @fallback None
  */
float Rad_into_Angle(float Rad)
{
		return Rad*(180.0f/PI);
}
/* =========================== Funtions End=========================== */