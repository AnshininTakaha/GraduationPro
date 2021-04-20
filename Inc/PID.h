#ifndef __PID__H
#define __PID__H


#include <stdint.h>

/* =========================== PriviteDefine Begin=========================== */
#define M3508PerR 8192

/* =========================== PriviteDefine End=========================== */

/* =========================== GroundInit Begin=========================== */
#define PID_GroundSpendInit \
{ \
	0, \
	0, \
	0, \
	0, \
	0, \
	0,0,0, \
	0,0,0, \
	0, \
	0, \
	0, \
} \

#define PMode_GroundInitInRB \
{ \
	0, \
	0, \
	0, \
	0, \
	0, \
	1.2f,0,0.0f,\
	0,0,0, \
	0, \
	16000, \
	8000, \
} \

#define PMode_GroundInitOutRB \
{ \
	0, \
	0, \
	0, \
	0, \
	0, \
	-1.3f,0,0.0f, \
	0,0,0, \
	0, \
	6000, \
	16000, \
} \

#define PMode_GroundInitInLB \
{ \
	0, \
	0, \
	0, \
	0, \
	0, \
	0,0,0, \
	0,0,0, \
	0, \
	16000, \
	8000, \
} \

#define PMode_GroundInitOutLB \
{ \
	0, \
	0, \
	0, \
	0, \
	0, \
	0,0,0, \
	0,0,0, \
	0, \
	6000, \
	16000, \
} \

#define PMode_GroundInitInLF \
{ \
	0, \
	0, \
	0, \
	0, \
	0, \
	0,0,0, \
	0,0,0, \
	0, \
	16000, \
	8000, \
} \

#define PMode_GroundInitOutLF \
{ \
	0, \
	0, \
	0, \
	0, \
	0, \
	0,0,0, \
	0,0,0, \
	0, \
	6000, \
	16000, \
} \

#define PMode_GroundInitInRF \
{ \
	0, \
	0, \
	0, \
	0, \
	0, \
	0,0,0, \
	0,0,0, \
	0, \
	16000, \
	8000, \
} \

#define PMode_GroundInitOutRF \
{ \
	0, \
	0, \
	0, \
	0, \
	0, \
	0,0,0, \
	0,0,0, \
	0, \
	6000, \
	16000, \
} \

/* =========================== GroundInit End=========================== */

/* =========================== Structure Begin=========================== */

typedef struct///�ٶ���PID���ݽṹ��
{
	float Target;
	float Measured;
	float error;
	float Last_error;
	float BeFLast_error;
	float Kp, Ki, Kd;
	float p_out, i_out, d_out;
	float PWM;
	uint32_t MaxOutputLimit;				  //����޷�
	uint32_t IntegralLimit;			//�����޷�
	
}incrementalpid_t;


typedef struct//λ����PID���ݽṹ��
{
	float Target;
	float Measured;
	float error; 						      //����ƫ��ֵ
	float Last_error; 				    //��һ��ƫ��
	float Change_error; 				    //���仯��
	float Kp, Ki, Kd;				    //Kp, Ki, Kd����ϵ��
	float p_out, i_out, d_out;//���������ֵ
	float PWM; 						      //pwm���
	uint32_t MaxOutputLimit;				  //����޷�
	uint32_t IntegralLimit;			//�����޷�
	
}positionpid_t;

/* =========================== Structure End=========================== */

/* =========================== ShareValue&funtions Begin=========================== */
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
float Position_PID(positionpid_t *pid_t, float target, float measured);
void IncrementalPID_paraReset(incrementalpid_t *pid_t);
void PositionPID_setPara(positionpid_t *pid_t);


/* =========================== ShareValue&funtions End=========================== */


#endif
