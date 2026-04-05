/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_Usart_Rec */
osThreadId_t Task_Usart_RecHandle;
uint32_t Task_Usart_RecBuffer[ 256 ];
osStaticThreadDef_t Task_Usart_RecControlBlock;
const osThreadAttr_t Task_Usart_Rec_attributes = {
  .name = "Task_Usart_Rec",
  .cb_mem = &Task_Usart_RecControlBlock,
  .cb_size = sizeof(Task_Usart_RecControlBlock),
  .stack_mem = &Task_Usart_RecBuffer[0],
  .stack_size = sizeof(Task_Usart_RecBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_UsartTrans */
osThreadId_t Task_UsartTransHandle;
uint32_t Task_UsartTransBuffer[ 256 ];
osStaticThreadDef_t Task_UsartTransControlBlock;
const osThreadAttr_t Task_UsartTrans_attributes = {
  .name = "Task_UsartTrans",
  .cb_mem = &Task_UsartTransControlBlock,
  .cb_size = sizeof(Task_UsartTransControlBlock),
  .stack_mem = &Task_UsartTransBuffer[0],
  .stack_size = sizeof(Task_UsartTransBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_CAN_Rec */
osThreadId_t Task_CAN_RecHandle;
uint32_t Task_CAN_RecBuffer[ 256 ];
osStaticThreadDef_t Task_CAN_RecControlBlock;
const osThreadAttr_t Task_CAN_Rec_attributes = {
  .name = "Task_CAN_Rec",
  .cb_mem = &Task_CAN_RecControlBlock,
  .cb_size = sizeof(Task_CAN_RecControlBlock),
  .stack_mem = &Task_CAN_RecBuffer[0],
  .stack_size = sizeof(Task_CAN_RecBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_CAN_Trans */
osThreadId_t Task_CAN_TransHandle;
uint32_t Task_CAN_TransBuffer[ 256 ];
osStaticThreadDef_t Task_CAN_TransControlBlock;
const osThreadAttr_t Task_CAN_Trans_attributes = {
  .name = "Task_CAN_Trans",
  .cb_mem = &Task_CAN_TransControlBlock,
  .cb_size = sizeof(Task_CAN_TransControlBlock),
  .stack_mem = &Task_CAN_TransBuffer[0],
  .stack_size = sizeof(Task_CAN_TransBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Queue_CanPack */
osMessageQueueId_t Queue_CanPackHandle;
uint8_t Queue_CanPackBuffer[ 3 * 13 ];
osStaticMessageQDef_t Queue_CanPackControlBlock;
const osMessageQueueAttr_t Queue_CanPack_attributes = {
  .name = "Queue_CanPack",
  .cb_mem = &Queue_CanPackControlBlock,
  .cb_size = sizeof(Queue_CanPackControlBlock),
  .mq_mem = &Queue_CanPackBuffer,
  .mq_size = sizeof(Queue_CanPackBuffer)
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void TaskUsartRec(void *argument);
void TaskUsartTrans(void *argument);
void TaskCANRec(void *argument);
void TaskCANTrans(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of Queue_CanPack */
  Queue_CanPackHandle = osMessageQueueNew (3, 13, &Queue_CanPack_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Task_Usart_Rec */
  Task_Usart_RecHandle = osThreadNew(TaskUsartRec, NULL, &Task_Usart_Rec_attributes);

  /* creation of Task_UsartTrans */
  Task_UsartTransHandle = osThreadNew(TaskUsartTrans, NULL, &Task_UsartTrans_attributes);

  /* creation of Task_CAN_Rec */
  Task_CAN_RecHandle = osThreadNew(TaskCANRec, NULL, &Task_CAN_Rec_attributes);

  /* creation of Task_CAN_Trans */
  Task_CAN_TransHandle = osThreadNew(TaskCANTrans, NULL, &Task_CAN_Trans_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TaskUsartRec */
/**
* @brief Function implementing the Task_Usart_Rec thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskUsartRec */
__weak void TaskUsartRec(void *argument)
{
  /* USER CODE BEGIN TaskUsartRec */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskUsartRec */
}

/* USER CODE BEGIN Header_TaskUsartTrans */
/**
* @brief Function implementing the Task_UsartTrans thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskUsartTrans */
__weak void TaskUsartTrans(void *argument)
{
  /* USER CODE BEGIN TaskUsartTrans */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskUsartTrans */
}

/* USER CODE BEGIN Header_TaskCANRec */
/**
* @brief Function implementing the Task_CAN_Rec thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskCANRec */
__weak void TaskCANRec(void *argument)
{
  /* USER CODE BEGIN TaskCANRec */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskCANRec */
}

/* USER CODE BEGIN Header_TaskCANTrans */
/**
* @brief Function implementing the Task_CAN_Trans thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TaskCANTrans */
__weak void TaskCANTrans(void *argument)
{
  /* USER CODE BEGIN TaskCANTrans */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END TaskCANTrans */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

