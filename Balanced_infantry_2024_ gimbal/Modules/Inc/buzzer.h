
#ifndef BUZZER_H
#define BUZZER_H

#include "struct_typedef.h"

#define BUZZER_MAX_OFF_TICK 10	//调用buzzer_warn后蜂鸣器停止的tick数，buzzer_warn未被调用的时间超过这个tick数后蜂鸣器自动停止
#define MOTOR_PSC 50  //电机频率
#define CAN_PSC 40 //CAN频率

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
class Buzzer_t
{
	public:
		uint8_t buzzer_off_tick;	   //蜂鸣器启动持续的tick数
		uint16_t buzzer_tick;		   //蜂鸣器响、停持续的tick计数
		uint8_t buzzer_warn_num;	   //蜂鸣器报警的次数
		uint8_t buzzer_warn_num_set;   //蜂鸣器报警的设定次数
		uint16_t buzzer_warn_interval; //蜂鸣器响的间隔时间
		uint16_t buzzer_psc;		   //蜂鸣器的分频系数
		uint16_t buzzer_pwm;		   //蜂鸣器的重载值
	
		Buzzer_t();
		void BuzzerWarn(uint8_t num_set, uint16_t interval, uint16_t psc, uint16_t pwm);
	
};

void BuzzerMonitor(Buzzer_t *buzzer);

extern Buzzer_t buzzer;
#endif

#ifdef __cplusplus
}
#endif

#endif
