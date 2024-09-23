/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "can_send.h"
#include "gimbal.h"
#include "system.h"
#include "task_center.h"
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
osThreadId UisendHandle;
osThreadId defaultTaskHandle;
//add thread
osThreadId CANSendTaskHandle;
osThreadId GimbalTaskHandle;
osThreadId SystemHandle;
osThreadId imuTaskHandle;
osThreadId MainTaskHandle;
osThreadId CommuniTaskHandle;
osThreadId UI_Send;
osThreadId caliHandle;

osThreadId GimbalOrderHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
//	osThreadDef(SYSTEM, SystemTask, osPriorityHigh, 0, 256);
//  SystemHandle = osThreadCreate(osThread(SYSTEM), NULL);

	osThreadDef(CANTask, MessageSendTask, osPriorityHigh, 0, 256);
  CANSendTaskHandle = osThreadCreate(osThread(CANTask), NULL);

	

//	osThreadDef(GimbalOrderTask, GimbalSendTask, osPriorityHigh, 0, 256);
//  GimbalOrderHandle = osThreadCreate(osThread(GimbalOrderTask), NULL);



	osThreadDef(imuTask, INSTask, osPriorityRealtime, 0, 1024);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);
	
	osThreadDef(main_thread, MainTask, osPriorityAboveNormal, 0, 1024);
  MainTaskHandle = osThreadCreate(osThread(main_thread), NULL);
	

	osThreadDef(communicateTask, CommuniTask, osPriorityHigh, 0, 128);
  CommuniTaskHandle = osThreadCreate(osThread(communicateTask), NULL);
	
	osThreadDef(UI_Send, UiTask, osPriorityNormal, 0, 256);	
	UisendHandle = osThreadCreate(osThread(UI_Send), NULL);
	
//	osThreadDef(cali, CalibrateTask, osPriorityAboveNormal, 0, 512);
//  caliHandle = osThreadCreate(osThread(cali), NULL);
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
   osThreadTerminate(NULL); // ±‹√‚ø’÷√∫Õ«–ªª’º”√cpu
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
