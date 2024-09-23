/**
 ******************************************************************************
 * @file    device_monitor.cpp
 * @author  Xushuang
 * @version V1.0.0 基本完成
 * @date    2023/10/17
 * @brief		监测状态类
 *					
 ******************************************************************************
 * @attention
 *			此文件主要通过每次进入函数的时间差来判断设备是否离线，需要监测设备时仅
 *  需在对应的设备类中添加此类，并进行阈值的设置，在循环中进行信息更新
 ******************************************************************************
 */
#include "device_monitor.h"
#include "bsp_dwt.h"

/**
	* @brief          硬件设备构造函数初始化
  * @retval         null
  */
DeviceStatus_t::DeviceStatus_t()
{
	now_time_ms = 0;
	timeout_duration_ms = 0;
	online = false;
}

/**
	* @brief          功能模块构造函数初始化
  * @retval         null
  */
ModulesStatus_t::ModulesStatus_t()
{
	last_data = 0;
	normal_run = 0;
	remain_time = 0;
}

/**
	* @brief          初始化设备状态
  * @param[in]      timeout：阈值时间（单位：ms）
  * @retval         初始化是否成功
  */
bool DeviceStatus_t::InitDeviceStatus(uint32_t timeout_ms)
{
	if(timeout_ms > 0 ){
		timeout_duration_ms = timeout_ms;
		return true;
	}else{
		return false;
	}
}

void DeviceStatus_t::GetMsgT()
{
	get_msg_time = DWT_GetTimeline_ms();
}

/**
	* @brief          更新设备状态
	* @param[in]      execute_time:正常执行时的时间（单位：ms）
  * @retval         NULL
  */
bool DeviceStatus_t::UpdateDeviceStatus()
{
	now_time_ms = DWT_GetTimeline_ms();
	//未执行过的话必然执行，执行过的话超时后可正常判断
	if(now_time_ms - get_msg_time <= timeout_duration_ms)
		online = true;  //设备在线
	else
		online = false; //设备离线
	
	return online;
}

void Motor_Watch_t::test()
{

}

/**
	* @brief          判断模块是否正常运行
	* @param[in]      now_data：传入检测运行是否正常的参数
	* @param[in]      max_set：最大设定值
  * @param[in]      timeout：时间阈值
  * @retval         normal_run：正常运行
  */
bool ModulesStatus_t::NormalRunJudge(float now_data,float max_set,uint16_t timeout)
{
	if(timeout <= 0)
		return false;
	if(now_data < max_set){
		normal_run = true;
		remain_time = 0;
	}else{
		BlinkLEDByCount(0xFFFF0000,500);
		if(remain_time++ > timeout)
			normal_run = false;
	}
	return normal_run;
}

DeviceHeartBag_t::DeviceHeartBag_t()
{
	heart_cnt = 0;
}
void DeviceHeartBag_t::SetHeartBeat(void)
{
	heart_cnt += 4;//心跳次数增加
}

bool DeviceHeartBag_t::GetHeartBeat(void)
{
	if(heart_cnt-- < 0)//心跳次数减少
	{
		return false;
	}else
	{
		return true;
	}
}
