/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "DR16_Remote.h"
#include "DJIMotor.h"
#include "BLDCMotor.h"
#include "Enconder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId StartTaskHandle;
osThreadId CANTaskHandle;
osMessageQId xQueueCanReceiveHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartTaskEnter(void const * argument);
void CANTaskEnter(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of xQueueCanReceive */
  osMessageQDef(xQueueCanReceive, 16, CAN_RxTypedef);
  xQueueCanReceiveHandle = osMessageCreate(osMessageQ(xQueueCanReceive), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of StartTask */
  osThreadDef(StartTask, StartTaskEnter, osPriorityNormal, 0, 128);
  StartTaskHandle = osThreadCreate(osThread(StartTask), NULL);

  /* definition and creation of CANTask */
  osThreadDef(CANTask, CANTaskEnter, osPriorityHigh, 0, 256);
  CANTaskHandle = osThreadCreate(osThread(CANTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartTaskEnter */
/**
  * @brief  Function implementing the StartTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTaskEnter */
void StartTaskEnter(void const * argument)
{
  /* USER CODE BEGIN StartTaskEnter */
  /* Infinite loop */
	
	DJIMotorFunction.DJI_Motor_CAN1_IT_Init();
	BLDCMotorFunction.BLDC_Motor_CAN2_IT_Init();
	DR16.DR16_USART1_IT_Init();
	EnconderFunction.Enc_CAN1_IT_Init();
	
  taskENTER_CRITICAL();
	
	
	
	
	
	vTaskDelete(NULL);
	taskEXIT_CRITICAL();
  /* USER CODE END StartTaskEnter */
}

/* USER CODE BEGIN Header_CANTaskEnter */
/**
* @brief Function implementing the CANTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CANTaskEnter */
void CANTaskEnter(void const * argument)
{
  /* USER CODE BEGIN CANTaskEnter */
  /* Infinite loop */
	
	CAN_RxTypedef CanReceiveData;
	
  for(;;)
  {
    xQueueReceive(xQueueCanReceiveHandle, &CanReceiveData, portMAX_DELAY);
		
		if(CanReceiveData.CAN_Switch == 1)
		{
			switch(CanReceiveData.CAN_RxHeader.StdId)
			{
				case M3508_READID_START...M3508_READID_END:
					DJIMotorFunction.DJI_Motor3508Process(CanReceiveData);
					break;
				
				case Encoder_HOST_CMD:
					EnconderFunction.Encoder_Process(CanReceiveData);
					break;
				default:
					break;
			}
			
			
		}
		if(CanReceiveData.CAN_Switch == 2)
		{
			BLDCMotorFunction.BLDCMotor_Process(CanReceiveData);
		}
			
			
		
		
    osDelay(1);
  }
  /* USER CODE END CANTaskEnter */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

	
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
