#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "struct_typedef.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
extern void InitVrefintReciprocal(void);
extern fp32 GetTemprate(void);
extern fp32 GetBatteryVoltage(void);
extern uint8_t GetHardwareVersion(void);

#endif
	
#ifdef __cplusplus
}
#endif

#endif
