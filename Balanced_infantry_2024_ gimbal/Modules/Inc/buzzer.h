
#ifndef BUZZER_H
#define BUZZER_H

#include "struct_typedef.h"

#define BUZZER_MAX_OFF_TICK 10	//����buzzer_warn�������ֹͣ��tick����buzzer_warnδ�����õ�ʱ�䳬�����tick����������Զ�ֹͣ
#define MOTOR_PSC 50  //���Ƶ��
#define CAN_PSC 40 //CANƵ��

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
class Buzzer_t
{
	public:
		uint8_t buzzer_off_tick;	   //����������������tick��
		uint16_t buzzer_tick;		   //�������졢ͣ������tick����
		uint8_t buzzer_warn_num;	   //�����������Ĵ���
		uint8_t buzzer_warn_num_set;   //�������������趨����
		uint16_t buzzer_warn_interval; //��������ļ��ʱ��
		uint16_t buzzer_psc;		   //�������ķ�Ƶϵ��
		uint16_t buzzer_pwm;		   //������������ֵ
	
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
