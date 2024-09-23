/**
 ******************************************************************************
 * @file    task_center.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 * @date    2023/9/4
 * @brief		FreeRtos所有任务定义处，可更改每个任务的频率
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "task_center.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"

Test_Module_t gimbal_test;

//机器人各模块初始化
void RobotInit()
{
	//关闭中断——防止在初始化时发生中断
	//不要加入中断和延时函数，若必须，则使用DWT_Delay()
	__disable_irq(); 
	/****机器人各外设初始化****/
	
	/**串口一**/
//	L1SInit();  //激光测距初始化
//	HEX_Conti_Meas_Cmd();  //发送连续测量信号
	RemoteControlInit(); //遥控器初始化
	/**串口六**/
#if BOARD_NUM == ONE_BOARD
	
#elif BOARD_NUM == TWO_BOARD
	RefereeInit();   //裁判系统初始化


#endif
	/****机器人各模块初始化****/
	SysInit();
	GimbalInit();
	ChassisInit();
	RevolverInit();
	DataAddressInit();
	UiInit();
	MonitorInit();
	CaliInit();
	
	//初始化完成，开启中断
	__enable_irq();
	/**串口三**/
}

//1Khz
void INSTask(void const *pvParameters)
{
//	vTaskDelay(TASK_INIT_TIME);
	ImuInit();
	while(1)
	{
#if BOARD_TYPE == DJI_CBOARD
		ImuTask();
#elif BOARD_TYPE == DJI_ABOARD
		
#endif
		vTaskDelay(1);
	}
}

//1Khz
void MessageSendTask(void const *pvParameters)
{
//	vTaskDelay(TASK_INIT_TIME);
//	DataAddressInit();
	while(1)
	{
		MotorSendTask();
		vTaskDelay(1);
		//OthersSendTask();
		GimbalToChassisTask();
		vTaskDelay(1);
	}
}




void GimbalSendTask(void const *pvParameters)
{
	while(1)
	{
		YawSendTask();
		vTaskDelay(1);
		GimbalToChassisTask();
		vTaskDelay(1);
	}
}




//1khz
uint8_t if_same;
void MainTask(void const *pvParameters)
{
	DWT_Init(168);
	if_same=CheckSameID();
	uint32_t currentTime = xTaskGetTickCount();
	while(1)
	{
		//系统任务
		SystemTask();
//		TEST_FUNC_TIME(gimbal_test,GimbalTask());
		//云台任务
		GimbalTask();
		//底盘任务
		ChassisTask();
		//拨盘任务
		RevolverTask();
		vTaskDelayUntil(&currentTime,1);
	}
}


void CommuniTask(void const *pvParameters)
{
//	vTaskDelay(TASK_INIT_TIME);
//	MonitorInit();
	while(1)
	{
		MonitorTask();
		Vision_Send_Data(0);
		vTaskDelay(5);
	}
}

//校准陀螺仪时使用
void CalibrateTask(void const *pvParameters)
{
	while(1)
	{
		Calibrate();
		vTaskDelay(1);
	}
}
