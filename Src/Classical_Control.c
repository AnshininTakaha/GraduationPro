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
#include "Angle_conversion.h"
#include "PID.h"
/* =========================== FuntionsPulk Begin=========================== */
void Classical_Control(ExportData ExpData);
void StreeingWheelCalculation_P(ExportData ExpData);
void StreeingWheelCalculation_S(ExportData ExpData);
void StreeingWheelPush(ExportData ExpData);
void MoonWheelPush(float MW_PIDOutput0,float MW_PIDOutput1,float MW_PIDOutput2,float MW_PIDOutput3);
/* =========================== FuntionsPulk End=========================== */

/* =========================== Value Begin=========================== */
Classical_Functions Classical = GroundInit_Classical_Functions;
/* =========================== Value End=========================== */

/* =========================== Funtions Begin=========================== */
/**
  * @Data    2020-12-19 
  * @brief   底盘控制函数
  * @param   ExportData ExpData 指向DR16_Export_Data_t结构体的指针
  * @retval  void
	* @fallback None
  */
	/*OMEGA = +右 : LF:45 RF:135 RB:-135 LB:-45*/
	/*OMEGA = -左 : LF:-135 RF:-45 RB:45 LB:135*/
/*计算角度累加值，用于相对式角度定位*/
float OmegaChange[4] = {EncoderRB_BaseAbsEnc, \
												EncoderLB_BaseAbsEnc, \
												EncoderLF_BaseAbsEnc, \
												EncoderRF_BaseAbsEnc};
void Classical_Control(ExportData ExpData)
{
	float PIDOutput[4] = {0,0,0,0};
	/*编码器绝对式角度，用于编码器的中间位置定义，方便绝对式定位*/
	int Enconder_BaseAbsEnc[4] = {EncoderRB_BaseAbsEnc, \
																EncoderLB_BaseAbsEnc, \
																EncoderLF_BaseAbsEnc, \
																EncoderRF_BaseAbsEnc};
	
	/*选择模式*/
	switch((int)ExpData->Switch_Left)
	{
		//云台模式
		case RemotePole_UP:
			/*舵轮底盘解算*/
			Classical.StreeingWheelCalculation_S(ExpData);
			
			
			for(int i=0;i<4;i++)
			{
				/*运动角度的变化速度叠加*/
				OmegaChange[i] += ExpData->Output_Angle[i] * 0.1f;
			}
			
			//加是否自旋判断？
			//闭环，记得对应OmegaChange里面Enc所对应的轮子方向
			Position_PID(&M3508_MoonWheel[0].PID.Posit_PID_Out, \
											 OmegaChange[3], \
											 Enconder[0].Enconder_ReadBackAngle);
			Position_PID(&M3508_MoonWheel[0].PID.Posit_PID_In, \
											 M3508_MoonWheel[0].PID.Posit_PID_Out.PWM, \
											 M3508_MoonWheel[0].Feedback.ReadSpeed);
			PIDOutput[0] = M3508_MoonWheel[0].PID.Posit_PID_In.PWM;
			
			Position_PID(&M3508_MoonWheel[1].PID.Posit_PID_Out, \
											 OmegaChange[2], \
											 Enconder[1].Enconder_ReadBackAngle);
			Position_PID(&M3508_MoonWheel[1].PID.Posit_PID_In, \
											 M3508_MoonWheel[1].PID.Posit_PID_Out.PWM, \
											 M3508_MoonWheel[1].Feedback.ReadSpeed);
			PIDOutput[1] = M3508_MoonWheel[1].PID.Posit_PID_In.PWM;
			
			Position_PID(&M3508_MoonWheel[2].PID.Posit_PID_Out, \
											 OmegaChange[0], \
											 Enconder[2].Enconder_ReadBackAngle);
			Position_PID(&M3508_MoonWheel[2].PID.Posit_PID_In, \
											 M3508_MoonWheel[2].PID.Posit_PID_Out.PWM, \
											 M3508_MoonWheel[2].Feedback.ReadSpeed);
			PIDOutput[2] = M3508_MoonWheel[2].PID.Posit_PID_In.PWM;
			
			Position_PID(&M3508_MoonWheel[3].PID.Posit_PID_Out, \
											 OmegaChange[1], \
											 Enconder[3].Enconder_ReadBackAngle);
			Position_PID(&M3508_MoonWheel[3].PID.Posit_PID_In, \
											 M3508_MoonWheel[3].PID.Posit_PID_Out.PWM, \
											 M3508_MoonWheel[3].Feedback.ReadSpeed);
			PIDOutput[3] = M3508_MoonWheel[3].PID.Posit_PID_In.PWM;
			
			/*月球轮运动参数赋值给轮子*/
			Classical.MoonWheelPush(PIDOutput[0],PIDOutput[1],PIDOutput[2],PIDOutput[3]);
			
			
			/*舵轮运动参数赋值给轮子*/
			Classical.StreeingWheelPush(ExpData);
			
			
			break;
		
		//正常模式
		case RemotePole_MID:
			#ifdef Using_PMode
			/*舵轮底盘解算*/
			Classical.StreeingWheelCalculation_P(ExpData);
			
			/*编码器角度闭环(由于ID不同，记得对应OutputAngle的对应ID)*/
			Position_PID(&M3508_MoonWheel[0].PID.Posit_PID_Out, \
											 Enconder_BaseAbsEnc[0] + ANGLE_TO_Enconder(DR16_Export_data.Output_Angle[3]), \
											 Enconder[0].Enconder_ReadBackAngle);
			Position_PID(&M3508_MoonWheel[0].PID.Posit_PID_In, \
											 M3508_MoonWheel[0].PID.Posit_PID_Out.PWM, \
											 M3508_MoonWheel[0].Feedback.ReadSpeed);
			PIDOutput[0] = M3508_MoonWheel[0].PID.Posit_PID_In.PWM;
		
//			PIDOutput[0] = Position_PID(&M3508_MoonWheel[0].PID.Posit_PID_Out, \
//											 Enconder_BaseAbsEnc[0] + ANGLE_TO_Enconder(DR16_Export_data.Output_Angle[3]), \
//											 Enconder[0].Enconder_ReadBackAngle);
//			PIDOutput[1] = Position_PID(&EncPID[1], \
//											 Enconder_BaseAbsEnc[1] + ANGLE_TO_Enconder(DR16_Export_data.Output_Angle[2]), \
//											 Enconder[1].Enconder_ReadBackAngle);
//			PIDOutput[2] = Position_PID(&EncPID[2], \
//											 Enconder_BaseAbsEnc[2] + ANGLE_TO_Enconder(DR16_Export_data.Output_Angle[0]), \
//											 Enconder[2].Enconder_ReadBackAngle);
//			PIDOutput[3] = Position_PID(&EncPID[3], \
//											 Enconder_BaseAbsEnc[3] + ANGLE_TO_Enconder(DR16_Export_data.Output_Angle[1]), \
//											 Enconder[3].Enconder_ReadBackAngle);
			
			
			/*月球轮运动参数赋值给轮子*/
			Classical.MoonWheelPush(PIDOutput[0],PIDOutput[1],PIDOutput[2],PIDOutput[3]);
			
			/*舵轮运动参数赋值给轮子*/
			Classical.StreeingWheelPush(ExpData);
			#endif
			break;
		
		//失能模式
		case RemotePole_DOWM:
			
			/*关闭所有电流，防止发生问题*/
			Classical.MoonWheelPush(0,0,0,0);
			for(int i=0;i<4;i++)
			{
				ExpData->Output_Velocity[i] = 0;
				ExpData->Output_Angle[i] = 0;
			}
			
			break;
		
		
		default:
			break;
	}

	
}

/**
  * @Data    2020-12-19 
  * @brief   舵轮底盘解算函数（位置模式）
  * @param   void
  * @retval  void
	* @fallback None
  */

void StreeingWheelCalculation_P(ExportData ExpData)
{
	/*没有Direct_V是因为之前通过变量算出相对应的东西了*/
	float Direct_VeloccityX = 0.0f ,Direct_VeloccityY = 0.0f;
	float Direct_Angle = 0.0f, Omega_Velocity = 0.0f;
	
	/*定义切向所有电机的角度值*/
	float Omega_Angle[4]= {0.0f,0.0f,0.0f,0.0f};
	
	/*定义叠加值Deltax，DeltaY*/
	float Delta_X[4],Delta_Y[4];
	
	/*计算直接速度值*/
	Direct_VeloccityX = ExpData->DR16_Direct_X_Value;
	Direct_VeloccityY = ExpData->DR16_Direct_Y_Value;
	
	/*计算直接角度值*/
	Direct_Angle = ExpData->DR16_Direct_Angle_Value;
	
	/*计算切向速度值*/
	Omega_Velocity = ExpData->DR16_Omega_Value;
	
	/*计算切向角度值,顺便角度转弧度*/
	if(Omega_Velocity > 0)
	{
		Omega_Angle[0] = Translation.Angle_into_Rad(45);   //0 LF
		Omega_Angle[1] = Translation.Angle_into_Rad(135);   //1 RF
		Omega_Angle[2] = Translation.Angle_into_Rad(-45);   //2 LB
		Omega_Angle[3] = Translation.Angle_into_Rad(-135);   //3 RB
	}
	else if(Omega_Velocity < 0)
	{
		Omega_Angle[0] = Translation.Angle_into_Rad(-135);   //0 LF
		Omega_Angle[1] = Translation.Angle_into_Rad(-45);   //1 RF
		Omega_Angle[2] = Translation.Angle_into_Rad(135);   //2 LB
		Omega_Angle[3] = Translation.Angle_into_Rad(45);   //3 RB
	}
	else
	{
		Omega_Angle[0] = 0;   //0 LF
		Omega_Angle[1] = 0;   //1 RF
		Omega_Angle[2] = 0;   //2 LB
		Omega_Angle[3] = 0;   //3 RB
	}
	
	/*计算叠加值*/
	for(int j=0; j<4; j++)
	{
		Delta_X[j] = Direct_VeloccityX + Omega_Velocity*sin(Omega_Angle[j]);
		Delta_Y[j] = Direct_VeloccityY + Omega_Velocity*cos(Omega_Angle[j]);
	}
	
	/*计算总速度值和总角度值*/
	for(int i=0; i<4; i++)
	{
		ExpData->Output_Velocity[i] =  sqrt((Delta_X[i]*Delta_X[i])+(Delta_Y[i]*Delta_Y[i]));
		
		/*防止由于Omega_Angle为0的时候合角度被削弱一半，由于是合力角在Omega_Angle角度为0的时候不需要削弱*/
		if(Omega_Angle[i] != 0)
		{
			ExpData->Output_Angle[i] = (Direct_Angle + (Translation.Rad_into_Angle(Omega_Angle[i]))) /2;
		}
		else
		{
			ExpData->Output_Angle[i] = Direct_Angle;
		}
		
		if(i%2)//为了线不绕，1.3轮反向
		{
			ExpData->Output_Velocity[i] = -ExpData->Output_Velocity[i];
		}
	}
}



/**
  * @Data    2020-12-19 
  * @brief   舵轮底盘解算函数（速度模式）
  * @param   void
  * @retval  void
	* @fallback None
  */
void StreeingWheelCalculation_S(ExportData ExpData)
{
	/*定义变量,这个模式下直接忽略左边摇杆的x轴，改为用右边摇杆的x轴进行控制*/
	float VeloccityY = 0.0f ,OmegaChange_Velocity = 0.0f;
	
	/*定义切向所有电机的角度值*/
	float Omega_Angle[4]= {0.0f,0.0f,0.0f,0.0f};
	
	/*前进速度和转弯速度*/
	VeloccityY = ExpData->DR16_Direct_Y_Value*0.1;
	OmegaChange_Velocity = ExpData->DR16_Omega_Value*0.1;
	
	/*对应速度赋值到对应的Output中去*/
	for(int i=0; i<4; i++)
	{
		ExpData->Output_Velocity[i] = VeloccityY;
		ExpData->Output_Angle[i] = OmegaChange_Velocity;
	}

}



/**
  * @Data    2020-12-19 
  * @brief   舵轮运动参数赋值给轮子
  * @param   void
  * @retval  void
	* @fallback None
  */
void StreeingWheelPush(ExportData ExpData)
{
	
	BLDCMotorFunction.VSEC_SetRpm(&hcan2,BLDCMotorLF_ID,ExpData->Output_Velocity[0]);
	BLDCMotorFunction.VSEC_SetRpm(&hcan2,BLDCMotorRF_ID,ExpData->Output_Velocity[1]);
	BLDCMotorFunction.VSEC_SetRpm(&hcan2,BLDCMotorLB_ID,ExpData->Output_Velocity[2]);
	BLDCMotorFunction.VSEC_SetRpm(&hcan2,BLDCMotorRB_ID,ExpData->Output_Velocity[3]);
	
	
}



/**
  * @Data    2020-12-19 
  * @brief   月球轮运动参数赋值给轮子
  * @param   void
  * @retval  void
	* @fallback None
  */
void MoonWheelPush(float MW_PIDOutput0,float MW_PIDOutput1,float MW_PIDOutput2,float MW_PIDOutput3)
{
	DJIMotorFunction.DJIMotor_Set3508Current(MW_PIDOutput0, \
																					 MW_PIDOutput1, \
																					 MW_PIDOutput2, \
																					 MW_PIDOutput3);
}
/* =========================== Funtions End=========================== */


