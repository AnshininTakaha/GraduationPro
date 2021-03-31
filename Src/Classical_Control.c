/**
  ******************************************************************************
  * @file    Classical_Control.c
  * @author  口无
  * @version V1.0
  * @date		 
						 
  * @brief   底盘控制模型
						 
	* @funtion(in) 
						 
	* @value(share) 
  ******************************************************************************
  */
#include "Classical_Control.h"
/* =========================== FuntionsPulk Begin=========================== */

/* =========================== FuntionsPulk End=========================== */

/* =========================== Value Begin=========================== */

/* =========================== Value End=========================== */

/* =========================== Funtions Begin=========================== */
/**
  * @Data    2020-12-19 
  * @brief   底盘控制函数
  * @param   ExportData ExpData 指向DR16_Export_Data_t结构体的指针
  * @retval  void
	* @fallback None
  */
void Classical_Control(ExportData ExpData)
{
	float Direct_Velocity, Direct_Angle, Omega_Velocity;
	float	Omega_Angle;
	
	Direct_Velocity  = sqrt((ExpData->DR16_Direct_X_Value * ExpData->DR16_Direct_X_Value) + \
	(ExpData->DR16_Direct_Y_Value * ExpData->DR16_Direct_Y_Value));
	
	Direct_Angle = ExpData->DR16_Direct_Angle_Value;
	
	Omega_Velocity = ExpData->DR16_Omega_Value;
	
	/*OMEGA = +右 : LF:45 RF:135 RB:-135 LB:-45*/
	/*OMEGA = -左 : LF:-135 RF:-45 RB:45 LB:135*/
	Omega_Angle = 0;
	
//	StreeingWheelCalculation();
}

/**
  * @Data    2020-12-19 
  * @brief   舵轮底盘解算函数
  * @param   void
  * @retval  void
	* @fallback None
  */
void StreeingWheelCalculation(float Direct_V, float Direct_A, float Omega_V, float Omega_A)
{
	
}

/* =========================== Funtions End=========================== */


