/**
 ******************************************************************************
 * @file    task_center.cpp
 * @author  Xushuang
 * @version V1.0.0 �������
 * @date    2023/9/4
 * @brief		FreeRtos���������崦���ɸ���ÿ�������Ƶ��
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "task_center.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"

Test_Module_t gimbal_test;

//�����˸�ģ���ʼ��
void RobotInit()
{
	//�ر��жϡ�����ֹ�ڳ�ʼ��ʱ�����ж�
	//��Ҫ�����жϺ���ʱ�����������룬��ʹ��DWT_Delay()
	__disable_irq(); 
	/****�����˸������ʼ��****/
	
	/**����һ**/
//	L1SInit();  //�������ʼ��
//	HEX_Conti_Meas_Cmd();  //�������������ź�
	RemoteControlInit(); //ң������ʼ��
	/**������**/
#if BOARD_NUM == ONE_BOARD
	
#elif BOARD_NUM == TWO_BOARD
	RefereeInit();   //����ϵͳ��ʼ��


#endif
	/****�����˸�ģ���ʼ��****/
	SysInit();
	GimbalInit();
	ChassisInit();
	RevolverInit();
	DataAddressInit();
	UiInit();
	MonitorInit();
	CaliInit();
	
	//��ʼ����ɣ������ж�
	__enable_irq();
	/**������**/
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
		//ϵͳ����
		SystemTask();
//		TEST_FUNC_TIME(gimbal_test,GimbalTask());
		//��̨����
		GimbalTask();
		//��������
		ChassisTask();
		//��������
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

//У׼������ʱʹ��
void CalibrateTask(void const *pvParameters)
{
	while(1)
	{
		Calibrate();
		vTaskDelay(1);
	}
}
