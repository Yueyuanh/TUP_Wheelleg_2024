/**
 ******************************************************************************
 * @file    monitor.cpp
 * @author  Xushuang
 * @version V1.0.0 �������
 * @date    2023/9/27
 * @brief		�˴�Ϊ������񣨷�������LED��
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "arm_math.h"
#include "monitor.h"

void DisplayMotorState(Motor_Monitor_Type_e monitor_type);
void DisplayCanState(uint8_t type);
void LedMonitor();

extern "C"{
	extern void (*MotorMonitorDisplay)(Motor_Monitor_Type_e monitor_type) = DisplayMotorState; //���״̬���
	extern void (*CANMonitor)(uint8_t type) = DisplayCanState; //CAN��״̬���
}

Monitor_Pub_Msg_t monitor_msg; //����ö��ʵ��

/**
	* @brief          ����ʼ��
  * @param[in]      NULL
  * @retval         NULL
  */
void MonitorInit()
{
	CenterPointer()->PointerInit(&monitor_msg,MONITORPUB);
}

/**
	* @brief          ���״̬��ⷽʽ
  * @param[in]      monitor_type���������
  * @retval         NULL
  */
void DisplayMotorState(Motor_Monitor_Type_e monitor_type)
{
	//ʹ�÷����������-�ɸ���
	if(monitor_type == MOTOR_PARAM){
		buzzer.BuzzerWarn(0,0,MOTOR_PSC,10000);
	}else if(monitor_type == MOTOR_TEM){
		buzzer.BuzzerWarn(0,0,MOTOR_PSC,10000);
	}
}

/**
	* @brief          CAN��״̬���
  * @param[in]      type������ѡ��
  * @retval         NULL
  */
void DisplayCanState(uint8_t type)
{
	if(type == 1){
		buzzer.BuzzerWarn(0,0,CAN_PSC,10000);
	}else if(type == 2){
		buzzer.BuzzerWarn(0,0,CAN_PSC,10000);
	}
}

void LedMonitor();

/**
	* @brief          ���������
  * @param[in]      NULL
  * @retval         NULL
  */
void MonitorTask()
{
	MonitorMotor();
	LedMonitor();
//	BuzzerMonitor(&buzzer); //���������
}

/**
	* @brief          LED�Ƽ��
  * @param[in]      type������ѡ��
  * @retval         NULL
  */
void LedMonitor()
{
//	if(!CheckMessageCenter())
//		MonitorPointer()->state = YELLOW_RAPID_FLASH;
	//LED���
//	MonitorPointer()->state = THREE_FLOW;
//	MonitorPointer()->state = GREEN_SLOW;
	Ligthten(monitor_msg.state);
//	Ligthten(THREE_FLOW);
//	if(!JudgeVisionState()){
//		//�����ƣ��Ӿ�ͨ������
////		BlinkLED(0xFF0000FF,1);
//		Ligthten(CYAN_BLINK_1S);
//	}else if(1){
//		//��ƿ���
//		
//	}else{
//		//���̵�һ������
//		BlinkLED(0xFF00FF00,400);
//	}
	
}
