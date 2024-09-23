#ifndef __CAN_COMMU_H
#define __CAN_COMMU_H

#include "struct_typedef.h"
#include "bsp_can.h"
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
class CAN_Commu_t
{
	private:
		CAN_Rx_Instance_t msg_bag1;
		enum Commu_Role_e
		{
			NO_COMMU,
			SENDER,
			RESERVER,
		}commu_role;
	public:
		void CANCommuInit();
};
	
#endif
	
#ifdef __cplusplus
}
#endif

#endif	
