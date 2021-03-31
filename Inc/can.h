/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
typedef struct///CAN解码接收完整体
{
	uint8_t     CAN_Switch;
	uint8_t 		CAN_RxMessage[8];
	CAN_RxHeaderTypeDef CAN_RxHeader;
}CAN_RxTypedef;
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */
extern CAN_FilterTypeDef CAN1_Filter;
extern CAN_FilterTypeDef CAN2_Filter;
/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN1_FILTER_Init(CAN_FilterTypeDef CAN1_Filter);
void CAN2_FILTER_Init(CAN_FilterTypeDef CAN2_Filter);
void CAN_SendData_F0(CAN_HandleTypeDef* CANx, uint8_t id_type, uint32_t id, uint8_t data[8]);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
