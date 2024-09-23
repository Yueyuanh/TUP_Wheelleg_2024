#ifndef __MYANTENNE_L1S_H
#define __MYANTENNE_L1S_H

#include "struct_typedef.h"
#include "device_monitor.h"
#include "bsp_usart.h"

#define L1S_BUFFER_LEN 8
#ifdef __cplusplus
extern "C"{
#endif

#ifdef __cplusplus
class MyAntennaL1S_t
{
	private:
		uint32_t distance;
	public:
		Usart_Instance_t module_usart;
		MyAntennaL1S_t();
		void CalcDistance(volatile uint8_t *info);
};
void L1SInit();
#endif	
void HEX_Conti_Meas_Cmd(void);
void HEX_FastConti_Meas_Cmd(void);
void HEX_Stop_Meas_Cmd(void);
#ifdef __cplusplus
}
#endif

#endif
