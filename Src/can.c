/**
  ******************************************************************************
  * File Name          : CAN.c
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

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
CAN_FilterTypeDef CAN1_Filter;
CAN_FilterTypeDef CAN2_Filter;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = ENABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 5;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = ENABLE;
  hcan2.Init.AutoWakeUp = ENABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration    
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }
  
    /**CAN2 GPIO Configuration    
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/**
  * @Data   2019-01-07
  * @brief  CAN1????????????
  * @param   ????????
  * @retval  ????????
  */
void CAN1_FILTER_Init(CAN_FilterTypeDef CAN1_Filter)
{
	/*??????????????FIFO0*/
	CAN1_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;         
	CAN1_Filter.FilterBank = 0;                                                              //????????0
	CAN1_Filter.SlaveStartFilterBank = 0;
	
	/*??????ID????????*/
	CAN1_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	
	/*????????????????32??*/
	CAN1_Filter.FilterScale = CAN_FILTERSCALE_32BIT;  

	/*??????????*/
	CAN1_Filter.FilterActivation = CAN_FILTER_ENABLE;   

	/*??????????*/
	CAN1_Filter.FilterIdHigh = 0x0000;
	CAN1_Filter.FilterIdLow = 0x0000;
	CAN1_Filter.FilterMaskIdHigh = 0x0000;
	CAN1_Filter.FilterMaskIdLow = 0x0000;
	
	HAL_CAN_ConfigFilter(&hcan1, &CAN1_Filter);
}



/**
  * @Data   2019-01-07
  * @brief  CAN????????????
  * @param   ????????
  * @retval  ????????
  */
void CAN2_FILTER_Init(CAN_FilterTypeDef CAN2_Filter)
{
	/*??????????????FIFO0*/
	CAN2_Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN2_Filter.FilterBank = 0;                                                              //????????0
	CAN2_Filter.SlaveStartFilterBank = 0;
	
	/*??????ID????????*/
	CAN2_Filter.FilterMode = CAN_FILTERMODE_IDMASK;
	
	/*????????????????32??*/
	CAN2_Filter.FilterScale = CAN_FILTERSCALE_32BIT; 

	/*??????????*/
	CAN2_Filter.FilterActivation = CAN_FILTER_ENABLE;               
	
	/*??????????*/
	CAN2_Filter.FilterIdHigh = 0x0000;
	CAN2_Filter.FilterIdLow = 0x0000;
	CAN2_Filter.FilterMaskIdHigh = 0x0000;
	CAN2_Filter.FilterMaskIdLow = 0x0000;
	HAL_CAN_ConfigFilter(&hcan2, &CAN2_Filter);
}

/**
  * @brief  CAN??????????????FIFO0??
  * @param  CANx 		   CAN????
  * 				id_type ??	 id???? CAN_ID_STD?? CAN_ID_EXT
  *					id			   id??
  * 				data[8]		 8??????
  * @retval None
  */
void CAN_SendData_F0(CAN_HandleTypeDef* CANx, uint8_t id_type, uint32_t id, uint8_t data[8])  
{	  
	CAN_TxHeaderTypeDef TxMessage;
	
	if(id_type == CAN_ID_STD)
	{
		TxMessage.StdId = id;
	}
	else
	{
		TxMessage.ExtId = id;
	}
	
	TxMessage.IDE = id_type;					 //ID????
	TxMessage.RTR = CAN_RTR_DATA;				 //????????????
	TxMessage.DLC = 0x08;						 //??????????8????
	TxMessage.TransmitGlobalTime = DISABLE;
	
	HAL_CAN_AddTxMessage(CANx,&TxMessage,data,(uint32_t*)CAN_TX_MAILBOX0);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
