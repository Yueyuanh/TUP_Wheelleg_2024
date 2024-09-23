#ifndef MONITOR_H
#define MONITOR_H

#include "vision.h"
#include "buzzer.h"
#include "led.h"
#include "motor.h"
#include "message_center.h"
#include "remote_control.h"
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
void MonitorInit(void);
void MonitorTask(void);
#endif
	
#ifdef __cplusplus
}
#endif

#endif
