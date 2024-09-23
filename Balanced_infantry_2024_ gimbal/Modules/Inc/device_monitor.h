#ifndef __DEVICE_MONITOR_H
#define __DEVICE_MONITOR_H

#include "struct_typedef.h"
#include "led.h"
#include "debug.h"
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//电机错误类型
enum Motor_Error_e 
{
	NO_MOTOR_ERROR = 0x00,  //无电机错误
	ID_RANGE_ERROR = 0x01,  //设置ID范围越界
	MOTOR_TYPE_ERROR = 0x02,//电机类型错误
	INPUT_PARAM_ERROR = 0x04, // 输入参数错误
};

//设备状态
class DeviceStatus_t
{
	private:
		uint32_t now_time_ms,get_msg_time;
		uint32_t timeout_duration_ms;
		bool online;
	
	public:
		DeviceStatus_t();
		bool InitDeviceStatus(uint32_t timeout);
		void GetMsgT();
		bool UpdateDeviceStatus();
};	

//电机状态
class Motor_Watch_t:public DeviceStatus_t,public Test_Module_t
{
	public:
		Motor_Error_e error_code;
		
		void test();
		
};

//模块状态
class ModulesStatus_t
{
	private:
		bool normal_run;
		float last_data;
		uint16_t remain_time;
	public:
		ModulesStatus_t();
		bool NormalRunJudge(float now_data,float max_set,uint16_t timeout);
};
	
class DeviceHeartBag_t
{
	private:
		int heart_cnt;
		
	public:
		DeviceHeartBag_t();
		void SetHeartBeat(void);
		bool GetHeartBeat(void);
};
#endif
	
#ifdef __cplusplus
}
#endif

#endif
