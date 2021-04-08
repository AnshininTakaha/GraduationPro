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

#define PMode_GroundInitIn \
{ \
	0, \
	0, \
	0, \
	0, \
	0, \
	0.8,0,1, \
	0,0,0, \
	0, \
	5000, \
	0, \
} \

#define PMode_GroundInitOut \
{ \
	0, \
	0, \
	0, \
	0, \
	0, \
	-2,0,-2, \
	0,0,0, \
	0, \
	5000, \
	0, \
} \

/* =========================== GroundInit End=========================== */

/* =========================== Structure Begin=========================== */

typedef struct///速度型PID数据结构体
{
	float Target;
	float Measured;
	float error;
	float Last_error;
	float BeFLast_error;
	float Kp, Ki, Kd;
	float p_out, i_out, d_out;
	float PWM;
	uint32_t MaxOutputLimit;				  //输出限幅
	uint32_t IntegralLimit;			//积分限幅
	
}incrementalpid_t;


typedef struct//位置型PID数据结构体
{
	float Target;
	float Measured;
	float error; 						      //本次偏差值
	float Last_error; 				    //上一次偏差
	float Change_error; 				    //误差变化率
	float Kp, Ki, Kd;				    //Kp, Ki, Kd控制系数
	float p_out, i_out, d_out;//各部分输出值
	float PWM; 						      //pwm输出
	uint32_t MaxOutputLimit;				  //输出限幅
	uint32_t IntegralLimit;			//积分限幅
	
}positionpid_t;

/* =========================== Structure End=========================== */

/* =========================== ShareValue&funtions Begin=========================== */
float Incremental_PID(incrementalpid_t *pid_t, float target, float measured);
float Position_PID(positionpid_t *pid_t, float target, float measured);
void IncrementalPID_paraReset(incrementalpid_t *pid_t);
void PositionPID_setPara(positionpid_t *pid_t);


/* =========================== ShareValue&funtions End=========================== */


#endif
