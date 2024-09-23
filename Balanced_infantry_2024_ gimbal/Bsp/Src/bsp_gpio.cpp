/**
  ******************************************************************************
  * @file	   bsp_gpio.c
  * @author  XuShuang
  * @version V1.0.0
  * @date    2023/9/24
  * @brief   ���ŵķ�װ������
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "bsp_gpio.h"

/**
  * @brief          GPIO��ƽ��ת
  * @param[in]      GPIO
  * @param[in]      Pin
  * @retval         Null
  */
void GPIOToggle(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin)
{
	HAL_GPIO_TogglePin(GPIOx,GPIO_Pin);
}

/**
  * @brief          ����GPIO��ƽ
  * @param[in]      GPIO
  * @param[in]      Pin
  * @retval         Null
  */
void GPIOSet(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
}

/**
  * @brief          ��λGPIO��ƽ
  * @param[in]      GPIO
  * @param[in]      Pin
  * @retval         Null
  */
void GPIOReset(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
}

/**
  * @brief          ����GPIO��ƽ
  * @param[in]      GPIO
  * @param[in]      Pin
  * @retval         ��ƽ״̬
  */
GPIO_PinState GPIORead(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin)
{
	return HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);
}
