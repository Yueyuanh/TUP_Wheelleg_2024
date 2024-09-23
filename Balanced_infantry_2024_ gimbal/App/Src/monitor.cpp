/**
 ******************************************************************************
 * @file    monitor.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 * @date    2023/9/27
 * @brief		此处为监测任务（蜂鸣器和LED）
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
	extern void (*MotorMonitorDisplay)(Motor_Monitor_Type_e monitor_type) = DisplayMotorState; //电机状态监测
	extern void (*CANMonitor)(uint8_t type) = DisplayCanState; //CAN线状态监测
}

Monitor_Pub_Msg_t monitor_msg; //建立枚举实例

/**
	* @brief          监测初始化
  * @param[in]      NULL
  * @retval         NULL
  */
void MonitorInit()
{
	CenterPointer()->PointerInit(&monitor_msg,MONITORPUB);
}

/**
	* @brief          电机状态监测方式
  * @param[in]      monitor_type：监测类型
  * @retval         NULL
  */
void DisplayMotorState(Motor_Monitor_Type_e monitor_type)
{
	//使用蜂鸣器监测电机-可更换
	if(monitor_type == MOTOR_PARAM){
		buzzer.BuzzerWarn(0,0,MOTOR_PSC,10000);
	}else if(monitor_type == MOTOR_TEM){
		buzzer.BuzzerWarn(0,0,MOTOR_PSC,10000);
	}
}

/**
	* @brief          CAN线状态监测
  * @param[in]      type：类型选择
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
	* @brief          监测主任务
  * @param[in]      NULL
  * @retval         NULL
  */
void MonitorTask()
{
	MonitorMotor();
	LedMonitor();
//	BuzzerMonitor(&buzzer); //蜂鸣器监测
}

/**
	* @brief          LED灯监测
  * @param[in]      type：类型选择
  * @retval         NULL
  */
void LedMonitor()
{
//	if(!CheckMessageCenter())
//		MonitorPointer()->state = YELLOW_RAPID_FLASH;
	//LED监测
//	MonitorPointer()->state = THREE_FLOW;
//	MonitorPointer()->state = GREEN_SLOW;
	Ligthten(monitor_msg.state);
//	Ligthten(THREE_FLOW);
//	if(!JudgeVisionState()){
//		//闪蓝灯，视觉通信正常
////		BlinkLED(0xFF0000FF,1);
//		Ligthten(CYAN_BLINK_1S);
//	}else if(1){
//		//红灯快闪
//		
//	}else{
//		//闪绿灯一切正常
//		BlinkLED(0xFF00FF00,400);
//	}
	
}
