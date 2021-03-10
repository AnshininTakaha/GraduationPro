/**
  ******************************************************************************
  * @file    PID.c
  * @author  口无
  * @version V3.21
  * @date		 2021-03-08 C extern对象性封装化，调整兼容性
						 
  * @brief   PID基础算法
						 
	* @funtion(share) Incremental_PID     速度PID计算
										Position_PID        位置PID计算
										
	* @funtion(in)    ABS_Limit           值域限值
  ******************************************************************************
  */
	
	
#include "PID.h"


/* =========================== FuntionsPulk Begin=========================== */
void ABS_Limit(float *a, float ABS_MAX);
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
float Position_PID(positionpid_t *pid_t, float target, float measured);
void IncrementalPID_paraReset(incrementalpid_t *pid_t);
void PositionPID_setPara(positionpid_t *pid_t);
/* =========================== FuntionsPulk End=========================== */


/* =========================== Value Begin=========================== */

/* =========================== Value End=========================== */

/* =========================== Funtions Begin=========================== */
/**
  * @Data    2021-03-01 
  * @brief   值域限值
  * @param   float *a        需要限值的值
						 float ABS_MAX   限值的值阀
  * @retval  void
	* @fallback None
  */
void ABS_Limit(float *a, float ABS_MAX) 
{
	if (*a > ABS_MAX)
		*a = ABS_MAX;
	if (*a < -ABS_MAX)
		*a = -ABS_MAX;
}



/**
  * @Data    2021-03-01  
  * @brief   速度型PID计算函数
  * @param   incrementalpid_t *pid_t      速度型PID结构体
						 float target                 输入计算的目标值
						 float measured               输入计算的测量值
  * @retval  float                        计算出来的PWM值
	* @fallback None
  */
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured) 
{
	/*值域传递*/
	pid_t->Target = target;
	pid_t->Measured = measured;
	pid_t->error = pid_t->Target - pid_t->Measured;
	
	/*出现过圈现象*/
	if(pid_t->error < -6000)
	{
		pid_t->error = (pid_t->Target + M3508PerR) - pid_t->Measured;
	}
	else if(pid_t->error > 6000)
	{
		pid_t->error = (pid_t->Target - M3508PerR) - pid_t->Measured;
	}
	else
	{
		pid_t->error = pid_t->error;
	}

	/*PID计算*/
	pid_t->p_out = pid_t->Kp*(pid_t->error - pid_t->Last_error);
	pid_t->i_out = pid_t->Ki*pid_t->error;
	pid_t->d_out = pid_t->Kd*(pid_t->error - 2.0f*pid_t->Last_error + pid_t->BeFLast_error);

	/*积分限幅*/
	ABS_Limit(&pid_t->i_out, pid_t->IntegralLimit);

	/*PWM累加*/
	pid_t->PWM += (pid_t->p_out + pid_t->i_out + pid_t->d_out);

	/*输出限幅*/
	ABS_Limit(&pid_t->PWM, pid_t->MaxOutputLimit);

	/*值域传递*/
	pid_t->BeFLast_error = pid_t->Last_error;
	pid_t->Last_error = pid_t->error;

	/*输出值返回*/
	return pid_t->PWM;
}



/**
  * @Data    2021-03-01  
  * @brief   位置型PID计算函数
  * @param   positionpid_t *pid_t         位置型PID结构体
						 float target                 输入计算的目标值
						 float measured               输入计算的测量值
  * @retval  float                        计算出来的PWM值
	* @fallback None
  */
float Position_PID(positionpid_t *pid_t, float target, float measured)
{
	/*值域传递*/
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	pid_t->error = pid_t->Target - pid_t->Measured;
	
	/*Error变化速率计算*/
	pid_t->Change_error = pid_t->error - pid_t->Last_error;
	
	/*PID计算*/
	pid_t->p_out = pid_t->Kp * pid_t->error;
	pid_t->i_out += pid_t->Ki * pid_t->error;
	pid_t->d_out = pid_t->Kd * (pid_t->error - pid_t->Last_error);
	
	
	/*积分限幅*/
	ABS_Limit(&pid_t->i_out, pid_t->IntegralLimit);//取消积分输出的限幅。

	/*PWM赋值*/
	pid_t->PWM = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

	/*输出限幅*/
	ABS_Limit(&pid_t->PWM, pid_t->MaxOutputLimit);

	/*值域传递*/
	pid_t->Last_error = pid_t->error;
	
	/*输出值返回*/
	return pid_t->PWM;
}

/**
  * @Data    2021-03-01  
  * @brief   速度PID刷新（重新初始化）
  * @param   incrementalpid_t *pid_t      速度型PID结构体
						 kp,ki,kd                     三个对应的PID值
						 MaxOutput                    最大输出
						 IntegralLimit                积分限值
  * @retval  void
	* @fallback None
  */
void IncrementalPID_paraReset(incrementalpid_t *pid_t) 
{
	pid_t->Target = 0;
	pid_t->Measured = 0;
	pid_t->error = 0;
	pid_t->Last_error = 0;
	pid_t->BeFLast_error = 0;
	pid_t->p_out = 0;
	pid_t->i_out = 0;
	pid_t->d_out = 0;
	pid_t->PWM = 0;
}



/**
  * @Data    2021-03-01  
  * @brief   位置PID刷新（重新初始化）
  * @param   incrementalpid_t *pid_t      速度型PID结构体
						 kp,ki,kd                     三个对应的PID值
						 MaxOutput                    最大输出
						 IntegralLimit                积分限值
  * @retval  void
	* @fallback None
  */
void PositionPID_setPara(positionpid_t *pid_t) 
{
	pid_t->Target = 0;
	pid_t->Measured = 0;
	pid_t->error = 0;
	pid_t->Last_error = 0;
	pid_t->Change_error = 0;
	pid_t->p_out = 0;
	pid_t->i_out = 0;
	pid_t->d_out = 0;
	pid_t->PWM = 0;
}
/* =========================== Funtions End=========================== */
