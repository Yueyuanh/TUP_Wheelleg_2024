/**
 ******************************************************************************
 * @file    buzzer.cpp
 * @author  YangMaoLin、Xushuang
 * @version V1.0.0 YangMaoLin 基本完成
 *					V2.0.0 Xushuang 封装类
 * @date    2023/9/21
 * @brief		此处为蜂鸣器类
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "buzzer.h"
#include "tim.h"
#include "bsp_pwm.h"
Buzzer_t buzzer;
//初始化
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
  * @brief          蜂鸣器报警。需要报警时持续调用此函数，不需要报警时不调用即可
  * @param[in]      num_set: 设置报警次数，0为一直报警，1为报警一次，2为报警两次，以此类推
  * @param[in]      interval: 设置报警间隔，单位为ms；num_set为0时，该参数无效
  * @param[in]      psc: 蜂鸣器分频系数，越大频率越低。1-10较为合适
  * @param[in]      pwm: 蜂鸣器重载值，一般为10000
  * @retval         none
  */
void Buzzer_t::BuzzerWarn(uint8_t num_set, uint16_t interval, uint16_t psc, uint16_t pwm)
{
	//如果报警次数大于等于已经设定的报警次数或者设置为一直报警，则更新数据
	if (num_set >= buzzer_warn_num_set || num_set == 0){
		buzzer_warn_num_set = num_set;
		buzzer_warn_interval = interval;
		buzzer_psc = psc;
		buzzer_pwm = pwm;
		//重置停止计数
		buzzer_off_tick = 0;
	}
}

void BuzzerMonitor(Buzzer_t *buzzer)
{
	//蜂鸣器停止的计数小于最大停止计数，则蜂鸣器响
	if (buzzer->buzzer_off_tick < BUZZER_MAX_OFF_TICK){
		//报警次数设为0，则一直报警
		if(buzzer->buzzer_warn_num_set == 0){
			TIMPWMParamSet(&htim4,TIM_CHANNEL_3,buzzer->buzzer_psc, buzzer->buzzer_pwm);
		}else if (buzzer->buzzer_warn_num < buzzer->buzzer_warn_num_set){		
			//报警次数小于设定值，则按照设定的报警次数报警
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
			//报警次数达到设定值，则停止2倍的间隔时间，再次报警
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
		//蜂鸣器停止的计数达到最大停止计数，则蜂鸣器停止
		TIMPWMParamReset(&htim4,TIM_CHANNEL_3);
		buzzer->buzzer_warn_num = 0;
		buzzer->buzzer_warn_num_set = 0;
		buzzer->buzzer_tick = 0;
	}
}
