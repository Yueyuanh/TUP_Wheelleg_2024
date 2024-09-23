#ifndef __TASK_CENTER_H
#define __TASK_CENTER_H

#include "robot_def.h"
#include "imu.h"
#include "gimbal.h"
#include "system.h"
#include "can_send.h"
#include "chassis.h"
#include "revolver.h"
#include "communicate.h"
#include "monitor.h"
#include "draw_ui.h"
#include "MyAntenne_L1S.h"
#include "tjc_usart_hmi.h"
#include "calibrate.h"
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
	
	
#endif
void RobotInit(void);
void INSTask(void const *pvParameters);
void MainTask(void const *pvParameters);
void MessageSendTask(void const *pvParameters);
void GimbalSendTask(void const *pvParameters);

void MoveTask(void const *pvParameters);	
void UiTask(void const *pvParameters);
void CommuniTask(void const *pvParameters);
void CalibrateTask(void const *pvParameters);
#ifdef __cplusplus
}
#endif
#endif
