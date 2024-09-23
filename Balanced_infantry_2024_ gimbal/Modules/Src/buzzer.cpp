/**
 ******************************************************************************
 * @file    buzzer.cpp
 * @author  YangMaoLin��Xushuang
 * @version V1.0.0 YangMaoLin �������
 *					V2.0.0 Xushuang ��װ��
 * @date    2023/9/21
 * @brief		�˴�Ϊ��������
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "buzzer.h"
#include "tim.h"
#include "bsp_pwm.h"
Buzzer_t buzzer;
//��ʼ��
Buzzer_t::Buzzer_t()
{
	buzzer_off_tick = BUZZER_MAX_OFF_TICK;
	buzzer_tick = 0;
	buzzer_warn_num = 0;
	buzzer_warn_num_set = 0;
	buzzer_warn_interval = 0;
	buzzer_psc = 0;
	buzzer_pwm = 0;
}

/**
  * @brief          ��������������Ҫ����ʱ�������ô˺���������Ҫ����ʱ�����ü���
  * @param[in]      num_set: ���ñ���������0Ϊһֱ������1Ϊ����һ�Σ�2Ϊ�������Σ��Դ�����
  * @param[in]      interval: ���ñ����������λΪms��num_setΪ0ʱ���ò�����Ч
  * @param[in]      psc: ��������Ƶϵ����Խ��Ƶ��Խ�͡�1-10��Ϊ����
  * @param[in]      pwm: ����������ֵ��һ��Ϊ10000
  * @retval         none
  */
void Buzzer_t::BuzzerWarn(uint8_t num_set, uint16_t interval, uint16_t psc, uint16_t pwm)
{
	//��������������ڵ����Ѿ��趨�ı���������������Ϊһֱ���������������
	if (num_set >= buzzer_warn_num_set || num_set == 0){
		buzzer_warn_num_set = num_set;
		buzzer_warn_interval = interval;
		buzzer_psc = psc;
		buzzer_pwm = pwm;
		//����ֹͣ����
		buzzer_off_tick = 0;
	}
}

void BuzzerMonitor(Buzzer_t *buzzer)
{
	//������ֹͣ�ļ���С�����ֹͣ���������������
	if (buzzer->buzzer_off_tick < BUZZER_MAX_OFF_TICK){
		//����������Ϊ0����һֱ����
		if(buzzer->buzzer_warn_num_set == 0){
			TIMPWMParamSet(&htim4,TIM_CHANNEL_3,buzzer->buzzer_psc, buzzer->buzzer_pwm);
		}else if (buzzer->buzzer_warn_num < buzzer->buzzer_warn_num_set){		
			//��������С���趨ֵ�������趨�ı�����������
			if (buzzer->buzzer_tick < buzzer->buzzer_warn_interval){
				TIMPWMParamSet(&htim4,TIM_CHANNEL_3,buzzer->buzzer_psc, buzzer->buzzer_pwm);
				buzzer->buzzer_tick++;
			}else if (buzzer->buzzer_tick < buzzer->buzzer_warn_interval * 2){
				TIMPWMParamReset(&htim4,TIM_CHANNEL_3);
				buzzer->buzzer_tick++;
			}else{
				buzzer->buzzer_tick = 0;
				buzzer->buzzer_tick++;
				buzzer->buzzer_warn_num++;
			}
		}else{
			//���������ﵽ�趨ֵ����ֹͣ2���ļ��ʱ�䣬�ٴα���
			if (buzzer->buzzer_tick < buzzer->buzzer_warn_interval * 2){
				TIMPWMParamReset(&htim4,TIM_CHANNEL_3);
				buzzer->buzzer_tick++;
			}else{
				buzzer->buzzer_tick = 0;
				buzzer->buzzer_warn_num = 0;
			}
		}

		buzzer->buzzer_off_tick++;
	}else{
		//������ֹͣ�ļ����ﵽ���ֹͣ�������������ֹͣ
		TIMPWMParamReset(&htim4,TIM_CHANNEL_3);
		buzzer->buzzer_warn_num = 0;
		buzzer->buzzer_warn_num_set = 0;
		buzzer->buzzer_tick = 0;
	}
}
