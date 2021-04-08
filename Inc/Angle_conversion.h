#ifndef ANGLE_CONVERSION_H
#define ANGLE_CONVERSION_H


/* =========================== PriviteDefine Begin=========================== */
/*‘≤÷‹¬ ºÚªØ*/
#define PI 3.141
/* =========================== PriviteDefine End=========================== */

/* =========================== GroundInit Begin=========================== */
#define GroundInit_Translation \
{ \
	&Angle_into_Rad, \
	&Rad_into_Angle, \
} 
/* =========================== GroundInit End=========================== */

/* =========================== Structure Begin=========================== */
typedef struct{
	float(*Angle_into_Rad)(float Angle);
	float(*Rad_into_Angle)(float Rad);
}Translation_t;
/* =========================== Structure End=========================== */

/* =========================== ShareValue&funtions Begin=========================== */
extern Translation_t Translation;
/* =========================== ShareValue&funtions End=========================== */


#endif
