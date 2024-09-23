/**
  ******************************************************************************
  * @file	   bsp_gpio.c
  * @author  XuShuang
  * @version V1.0.0
  * @date    2023/9/24
  * @brief   引脚的封装函数库
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "bsp_gpio.h"

/**
  * @brief          GPIO电平翻转
  * @param[in]      GPIO
  * @param[in]      Pin
  * @retval         Null
  */
void GPIOToggle(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin)
{
	HAL_GPIO_TogglePin(GPIOx,GPIO_Pin);
}

/**
  * @brief          设置GPIO电平
  * @param[in]      GPIO
  * @param[in]      Pin
  * @retval         Null
  */
void GPIOSet(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
}

/**
  * @brief          复位GPIO电平
  * @param[in]      GPIO
  * @param[in]      Pin
  * @retval         Null
  */
void GPIOReset(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin)
{
	HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
}

/**
  * @brief          设置GPIO电平
  * @param[in]      GPIO
  * @param[in]      Pin
  * @retval         电平状态
  */
GPIO_PinState GPIORead(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin)
{
	return HAL_GPIO_ReadPin(GPIOx,GPIO_Pin);
}
