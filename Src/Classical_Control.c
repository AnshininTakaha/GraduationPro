/**
  ******************************************************************************
  * @file    Classical_Control.c
  * @author  ����
  * @version V1.0
  * @date		 
						 
  * @brief   ���̿���ģ��
						 
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
  * @brief   ���̿��ƺ���
  * @param   ExportData ExpData ָ��DR16_Export_Data_t�ṹ���ָ��
  * @retval  void
	* @fallback None
  */
	/*OMEGA = +�� : LF:45 RF:135 RB:-135 LB:-45*/
	/*OMEGA = -�� : LF:-135 RF:-45 RB:45 LB:135*/
/*����Ƕ��ۼ�ֵ���������ʽ�Ƕȶ�λ*/
float OmegaChange[4] = {EncoderRB_BaseAbsEnc, \
												EncoderLB_BaseAbsEnc, \
												EncoderLF_BaseAbsEnc, \
												EncoderRF_BaseAbsEnc};
void Classical_Control(ExportData ExpData)
{
	float PIDOutput[4] = {0,0,0,0};
	/*����������ʽ�Ƕȣ����ڱ��������м�λ�ö��壬�������ʽ��λ*/
	int Enconder_BaseAbsEnc[4] = {EncoderRB_BaseAbsEnc, \
																EncoderLB_BaseAbsEnc, \
																EncoderLF_BaseAbsEnc, \
																EncoderRF_BaseAbsEnc};
	
	/*ѡ��ģʽ*/
	switch((int)ExpData->Switch_Left)
	{
		//��̨ģʽ
		case RemotePole_UP:
			/*���ֵ��̽���*/
			Classical.StreeingWheelCalculation_S(ExpData);
			
			
			for(int i=0;i<4;i++)
			{
				/*�˶��Ƕȵı仯�ٶȵ���*/
				OmegaChange[i] += ExpData->Output_Angle[i] * 0.1f;
			}
			
			//���Ƿ������жϣ�
			//�ջ����ǵö�ӦOmegaChange����Enc����Ӧ�����ӷ���
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
			
			/*�������˶�������ֵ������*/
			Classical.MoonWheelPush(PIDOutput[0],PIDOutput[1],PIDOutput[2],PIDOutput[3]);
			
			
			/*�����˶�������ֵ������*/
			Classical.StreeingWheelPush(ExpData);
			
			
			break;
		
		//����ģʽ
		case RemotePole_MID:
			#ifdef Using_PMode
			/*���ֵ��̽���*/
			Classical.StreeingWheelCalculation_P(ExpData);
			
			/*�������Ƕȱջ�(����ID��ͬ���ǵö�ӦOutputAngle�Ķ�ӦID)*/
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
			
			
			/*�������˶�������ֵ������*/
			Classical.MoonWheelPush(PIDOutput[0],PIDOutput[1],PIDOutput[2],PIDOutput[3]);
			
			/*�����˶�������ֵ������*/
			Classical.StreeingWheelPush(ExpData);
			#endif
			break;
		
		//ʧ��ģʽ
		case RemotePole_DOWM:
			
			/*�ر����е�������ֹ��������*/
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
  * @brief   ���ֵ��̽��㺯����λ��ģʽ��
  * @param   void
  * @retval  void
	* @fallback None
  */

void StreeingWheelCalculation_P(ExportData ExpData)
{
	/*û��Direct_V����Ϊ֮ǰͨ������������Ӧ�Ķ�����*/
	float Direct_VeloccityX = 0.0f ,Direct_VeloccityY = 0.0f;
	float Direct_Angle = 0.0f, Omega_Velocity = 0.0f;
	
	/*�����������е���ĽǶ�ֵ*/
	float Omega_Angle[4]= {0.0f,0.0f,0.0f,0.0f};
	
	/*�������ֵDeltax��DeltaY*/
	float Delta_X[4],Delta_Y[4];
	
	/*����ֱ���ٶ�ֵ*/
	Direct_VeloccityX = ExpData->DR16_Direct_X_Value;
	Direct_VeloccityY = ExpData->DR16_Direct_Y_Value;
	
	/*����ֱ�ӽǶ�ֵ*/
	Direct_Angle = ExpData->DR16_Direct_Angle_Value;
	
	/*���������ٶ�ֵ*/
	Omega_Velocity = ExpData->DR16_Omega_Value;
	
	/*��������Ƕ�ֵ,˳��Ƕ�ת����*/
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
	
	/*�������ֵ*/
	for(int j=0; j<4; j++)
	{
		Delta_X[j] = Direct_VeloccityX + Omega_Velocity*sin(Omega_Angle[j]);
		Delta_Y[j] = Direct_VeloccityY + Omega_Velocity*cos(Omega_Angle[j]);
	}
	
	/*�������ٶ�ֵ���ܽǶ�ֵ*/
	for(int i=0; i<4; i++)
	{
		ExpData->Output_Velocity[i] =  sqrt((Delta_X[i]*Delta_X[i])+(Delta_Y[i]*Delta_Y[i]));
		
		/*��ֹ����Omega_AngleΪ0��ʱ��ϽǶȱ�����һ�룬�����Ǻ�������Omega_Angle�Ƕ�Ϊ0��ʱ����Ҫ����*/
		if(Omega_Angle[i] != 0)
		{
			ExpData->Output_Angle[i] = (Direct_Angle + (Translation.Rad_into_Angle(Omega_Angle[i]))) /2;
		}
		else
		{
			ExpData->Output_Angle[i] = Direct_Angle;
		}
		
		if(i%2)//Ϊ���߲��ƣ�1.3�ַ���
		{
			ExpData->Output_Velocity[i] = -ExpData->Output_Velocity[i];
		}
	}
}



/**
  * @Data    2020-12-19 
  * @brief   ���ֵ��̽��㺯�����ٶ�ģʽ��
  * @param   void
  * @retval  void
	* @fallback None
  */
void StreeingWheelCalculation_S(ExportData ExpData)
{
	/*�������,���ģʽ��ֱ�Ӻ������ҡ�˵�x�ᣬ��Ϊ���ұ�ҡ�˵�x����п���*/
	float VeloccityY = 0.0f ,OmegaChange_Velocity = 0.0f;
	
	/*�����������е���ĽǶ�ֵ*/
	float Omega_Angle[4]= {0.0f,0.0f,0.0f,0.0f};
	
	/*ǰ���ٶȺ�ת���ٶ�*/
	VeloccityY = ExpData->DR16_Direct_Y_Value*0.1;
	OmegaChange_Velocity = ExpData->DR16_Omega_Value*0.1;
	
	/*��Ӧ�ٶȸ�ֵ����Ӧ��Output��ȥ*/
	for(int i=0; i<4; i++)
	{
		ExpData->Output_Velocity[i] = VeloccityY;
		ExpData->Output_Angle[i] = OmegaChange_Velocity;
	}

}



/**
  * @Data    2020-12-19 
  * @brief   �����˶�������ֵ������
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
  * @brief   �������˶�������ֵ������
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


