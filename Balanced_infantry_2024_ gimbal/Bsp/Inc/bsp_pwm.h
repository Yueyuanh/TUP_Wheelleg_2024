#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "stdint.h"
#include "tim.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus


#endif
void TIMSetPWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value);
void TIMPWMParamSet(TIM_HandleTypeDef *tim_pwmHandle,uint8_t Channel,uint16_t psc,uint16_t pwm);
void TIMPWMParamReset(TIM_HandleTypeDef *tim_pwmHandle,uint8_t Channel);
void PWMStart(TIM_HandleTypeDef *tim_pwmHandle,uint8_t Channel);
#ifdef __cplusplus
}
#endif

#endif
