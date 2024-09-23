#ifndef __DEVICE_MONITOR_H
#define __DEVICE_MONITOR_H

#include "struct_typedef.h"
#include "led.h"
#include "debug.h"
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//�����������
enum Motor_Error_e 
{
	NO_MOTOR_ERROR = 0x00,  //�޵������
	ID_RANGE_ERROR = 0x01,  //����ID��ΧԽ��
	MOTOR_TYPE_ERROR = 0x02,//������ʹ���
	INPUT_PARAM_ERROR = 0x04, // �����������
};

//�豸״̬
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

//���״̬
class Motor_Watch_t:public DeviceStatus_t,public Test_Module_t
{
	public:
		Motor_Error_e error_code;
		
		void test();
		
};

//ģ��״̬
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
