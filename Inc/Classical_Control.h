#ifndef CLASSICAL_CONTROL_H
#define CLASSICAL_CONTROL_H

#include "DR16_Remote.h"
#include "BLDCMotor.h"
#include "Enconder.h"
#include "DJIMotor.h"


/* =========================== PriviteDefine Begin=========================== */
//#define Using_PMode
#define Using_RMode
/* =========================== PriviteDefine End=========================== */

/* =========================== GroundInit Begin=========================== */
#define GroundInit_Classical_Functions \
{ \
	&Classical_Control, \
	&StreeingWheelCalculation_P, \
	&StreeingWheelCalculation_S, \
	&StreeingWheelPush, \
	&MoonWheelPush, \
}
/* =========================== GroundInit End=========================== */

/* =========================== Structure Begin=========================== */
typedef struct{
	void(*Classical_Control)(ExportData ExpData);
	void(*StreeingWheelCalculation_P)(ExportData ExpData);
	void(*StreeingWheelCalculation_S)(ExportData ExpData);
	void(*StreeingWheelPush)(ExportData ExpData);
	void(*MoonWheelPush)(float MW_PIDOutput0,float MW_PIDOutput1,float MW_PIDOutput2,float MW_PIDOutput3);
}Classical_Functions;
/* =========================== Structure End=========================== */

/* =========================== ShareValue&funtions Begin=========================== */
extern Classical_Functions Classical;
/* =========================== ShareValue&funtions End=========================== */

#endif
