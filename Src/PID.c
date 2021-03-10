/**
  ******************************************************************************
  * @file    PID.c
  * @author  ����
  * @version V3.21
  * @date		 2021-03-08 C extern�����Է�װ��������������
						 
  * @brief   PID�����㷨
						 
	* @funtion(share) Incremental_PID     �ٶ�PID����
										Position_PID        λ��PID����
										
	* @funtion(in)    ABS_Limit           ֵ����ֵ
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
  * @brief   ֵ����ֵ
  * @param   float *a        ��Ҫ��ֵ��ֵ
						 float ABS_MAX   ��ֵ��ֵ��
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
  * @brief   �ٶ���PID���㺯��
  * @param   incrementalpid_t *pid_t      �ٶ���PID�ṹ��
						 float target                 ��������Ŀ��ֵ
						 float measured               �������Ĳ���ֵ
  * @retval  float                        ���������PWMֵ
	* @fallback None
  */
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured) 
{
	/*ֵ�򴫵�*/
	pid_t->Target = target;
	pid_t->Measured = measured;
	pid_t->error = pid_t->Target - pid_t->Measured;
	
	/*���ֹ�Ȧ����*/
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

	/*PID����*/
	pid_t->p_out = pid_t->Kp*(pid_t->error - pid_t->Last_error);
	pid_t->i_out = pid_t->Ki*pid_t->error;
	pid_t->d_out = pid_t->Kd*(pid_t->error - 2.0f*pid_t->Last_error + pid_t->BeFLast_error);

	/*�����޷�*/
	ABS_Limit(&pid_t->i_out, pid_t->IntegralLimit);

	/*PWM�ۼ�*/
	pid_t->PWM += (pid_t->p_out + pid_t->i_out + pid_t->d_out);

	/*����޷�*/
	ABS_Limit(&pid_t->PWM, pid_t->MaxOutputLimit);

	/*ֵ�򴫵�*/
	pid_t->BeFLast_error = pid_t->Last_error;
	pid_t->Last_error = pid_t->error;

	/*���ֵ����*/
	return pid_t->PWM;
}



/**
  * @Data    2021-03-01  
  * @brief   λ����PID���㺯��
  * @param   positionpid_t *pid_t         λ����PID�ṹ��
						 float target                 ��������Ŀ��ֵ
						 float measured               �������Ĳ���ֵ
  * @retval  float                        ���������PWMֵ
	* @fallback None
  */
float Position_PID(positionpid_t *pid_t, float target, float measured)
{
	/*ֵ�򴫵�*/
	pid_t->Target = (float)target;
	pid_t->Measured = (float)measured;
	pid_t->error = pid_t->Target - pid_t->Measured;
	
	/*Error�仯���ʼ���*/
	pid_t->Change_error = pid_t->error - pid_t->Last_error;
	
	/*PID����*/
	pid_t->p_out = pid_t->Kp * pid_t->error;
	pid_t->i_out += pid_t->Ki * pid_t->error;
	pid_t->d_out = pid_t->Kd * (pid_t->error - pid_t->Last_error);
	
	
	/*�����޷�*/
	ABS_Limit(&pid_t->i_out, pid_t->IntegralLimit);//ȡ������������޷���

	/*PWM��ֵ*/
	pid_t->PWM = (pid_t->p_out + pid_t->i_out + pid_t->d_out);

	/*����޷�*/
	ABS_Limit(&pid_t->PWM, pid_t->MaxOutputLimit);

	/*ֵ�򴫵�*/
	pid_t->Last_error = pid_t->error;
	
	/*���ֵ����*/
	return pid_t->PWM;
}

/**
  * @Data    2021-03-01  
  * @brief   �ٶ�PIDˢ�£����³�ʼ����
  * @param   incrementalpid_t *pid_t      �ٶ���PID�ṹ��
						 kp,ki,kd                     ������Ӧ��PIDֵ
						 MaxOutput                    ������
						 IntegralLimit                ������ֵ
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
  * @brief   λ��PIDˢ�£����³�ʼ����
  * @param   incrementalpid_t *pid_t      �ٶ���PID�ṹ��
						 kp,ki,kd                     ������Ӧ��PIDֵ
						 MaxOutput                    ������
						 IntegralLimit                ������ֵ
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
