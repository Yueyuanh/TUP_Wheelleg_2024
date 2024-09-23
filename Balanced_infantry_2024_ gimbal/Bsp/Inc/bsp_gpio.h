#ifndef __BSP_GPIO_H
#define __BSP_GPIO_H

#include "gpio.h"
#include "struct_typedef.h"

#ifdef __cplusplus
extern "C"{
#endif

void GPIOToggle(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin);

void GPIOSet(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin);

void GPIOReset(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin);


GPIO_PinState GPIORead(GPIO_TypeDef * GPIOx,uint16_t GPIO_Pin);


#ifdef __cplusplus
}
#endif

#endif
