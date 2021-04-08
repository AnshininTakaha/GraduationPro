/**
  ******************************************************************************
  * @file    Angle_conversation.c
  * @author  口无
  * @version V1.0
  * @date		 2021-04-05 优化算法框架
						 
  * @brief   角度转换基础算法
						 
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
  * @brief  角度转换成为弧度
  * @param  float Angle 角度
  * @retval void
	* @fallback None
  */
float Angle_into_Rad(float Angle)
{
	return Angle*(PI/180.0f);
}

/**
	* @Data   2021-04-05 
  * @brief  弧度转换成为角度
  * @param  float Rad 弧度
  * @retval void
	* @fallback None
  */
float Rad_into_Angle(float Rad)
{
		return Rad*(180.0f/PI);
}
/* =========================== Funtions End=========================== */