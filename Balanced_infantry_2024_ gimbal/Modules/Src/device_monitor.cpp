/**
 ******************************************************************************
 * @file    device_monitor.cpp
 * @author  Xushuang
 * @version V1.0.0 �������
 * @date    2023/10/17
 * @brief		���״̬��
 *					
 ******************************************************************************
 * @attention
 *			���ļ���Ҫͨ��ÿ�ν��뺯����ʱ������ж��豸�Ƿ����ߣ���Ҫ����豸ʱ��
 *  ���ڶ�Ӧ���豸������Ӵ��࣬��������ֵ�����ã���ѭ���н�����Ϣ����
 ******************************************************************************
 */
#include "device_monitor.h"
#include "bsp_dwt.h"

/**
	* @brief          Ӳ���豸���캯����ʼ��
  * @retval         null
  */
DeviceStatus_t::DeviceStatus_t()
{
	now_time_ms = 0;
	timeout_duration_ms = 0;
	online = false;
}

/**
	* @brief          ����ģ�鹹�캯����ʼ��
  * @retval         null
  */
ModulesStatus_t::ModulesStatus_t()
{
	last_data = 0;
	normal_run = 0;
	remain_time = 0;
}

/**
	* @brief          ��ʼ���豸״̬
  * @param[in]      timeout����ֵʱ�䣨��λ��ms��
  * @retval         ��ʼ���Ƿ�ɹ�
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
	* @brief          �����豸״̬
	* @param[in]      execute_time:����ִ��ʱ��ʱ�䣨��λ��ms��
  * @retval         NULL
  */
bool DeviceStatus_t::UpdateDeviceStatus()
{
	now_time_ms = DWT_GetTimeline_ms();
	//δִ�й��Ļ���Ȼִ�У�ִ�й��Ļ���ʱ��������ж�
	if(now_time_ms - get_msg_time <= timeout_duration_ms)
		online = true;  //�豸����
	else
		online = false; //�豸����
	
	return online;
}

void Motor_Watch_t::test()
{

}

/**
	* @brief          �ж�ģ���Ƿ���������
	* @param[in]      now_data�������������Ƿ������Ĳ���
	* @param[in]      max_set������趨ֵ
  * @param[in]      timeout��ʱ����ֵ
  * @retval         normal_run����������
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
	heart_cnt += 4;//������������
}

bool DeviceHeartBag_t::GetHeartBeat(void)
{
	if(heart_cnt-- < 0)//������������
	{
		return false;
	}else
	{
		return true;
	}
}
