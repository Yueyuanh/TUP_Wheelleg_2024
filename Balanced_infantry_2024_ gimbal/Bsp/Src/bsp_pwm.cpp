/**
  ******************************************************************************
  * @file	   bsp_pwm.c
  * @author  XuShuang
  * @version V1.0.0
  * @date    2023/9/24
  * @brief   ��ʱ���ķ�װ������
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "bsp_pwm.h"

void TIMSetPWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    if (value > tim_pwmHandle->Instance->ARR)
        value = tim_pwmHandle->Instance->ARR;

    switch (Channel)
    {
    case TIM_CHANNEL_1:
        tim_pwmHandle->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        tim_pwmHandle->Instance->CCR2 = value;
        break;
    case TIM_CHANNEL_3:
        tim_pwmHandle->Instance->CCR3 = value;
        break;
    case TIM_CHANNEL_4:
        tim_pwmHandle->Instance->CCR4 = value;
        break;
    }
}

//��ʱ����������
void TIMPWMParamSet(TIM_HandleTypeDef *tim_pwmHandle,uint8_t Channel,uint16_t psc,uint16_t pwm)
{
	__HAL_TIM_PRESCALER(tim_pwmHandle, psc);
  __HAL_TIM_SetCompare(tim_pwmHandle, Channel, pwm);
}

//��ʱ����������
void TIMPWMParamReset(TIM_HandleTypeDef *tim_pwmHandle,uint8_t Channel)
{
	__HAL_TIM_SetCompare(tim_pwmHandle, Channel, 0);
}

//PWMͨ������
void PWMStart(TIM_HandleTypeDef *tim_pwmHandle,uint8_t Channel)
{
	HAL_TIM_PWM_Start(tim_pwmHandle,Channel);
}

