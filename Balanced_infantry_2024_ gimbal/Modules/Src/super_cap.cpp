/**
 ******************************************************************************
 * @file    super_cap.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang ������� 2023/10/30
 * @date    2023/10/28
 * @brief		�˴�Ϊ��������ģ��
 ******************************************************************************
 * @attention
 *	�˴�ע��Ǽ��˳�������ģ����ص�CAN����Ϣ���ɶ�ȡ����
 ******************************************************************************
 */
#include "super_cap.h"

//��ȡ������������
#define	get_wulie_cap_measure(ptr, data)                                                     \
{   uint16_t *buf =    (uint16_t*) data;               \
		(ptr)->Cap_input_vol = ((fp32)buf[0]/100.f);           \
		(ptr)->Cap_voltage = ((fp32)buf[1]/100.f);     \
		(ptr)->Cap_current = ((fp32)buf[2]/100.f); \
		(ptr)->Cap_power = ((fp32)buf[3]/100.f); \
}

//��ĳ���
#define get_robofuture_cap_measure(ptr, data)			\
{																									\
	(ptr)->warning = data[0];           \
	(ptr)->charge_sign = (data)[1];     \
	(ptr)->chassis_power = (uint16_t)((data)[2] << 8 | (data)[3]); \
	(ptr)->ele_quantity = (uint16_t)((data)[4] << 8 | (data)[5]); \
}

//����ʵ��
Cap_t super_cap;

/**
	* @brief          �Ǽǳ�������ģ��CAN����Ϣ
	* @param[in]      can_num������CAN�ߺ�
	* @param[in]      input_id�����õ�ID
  * @retval         Null
  */
void RegisterSuperCap(CAN_NAME_e can_num,uint32_t input_id)
{
	super_cap.SuperCapInit(can_num,input_id);
}

/**
	* @brief          ���س�������ָ�룬�������ⲿ��Ϣ����
	* @param[in]      NULL
  * @retval         &super_cap
  */
Cap_t *GetCapPointer()
{
	return &super_cap;
}

/**
	* @brief          �����������ݽ���
  * @param[in]      *rx_instance�����յ�ʵ��
  * @retval         Null
  */
static void DecodeSuperCap(CAN_Rx_Instance_t *rx_instance)
{
	uint8_t *rxbuff = rx_instance->rx_buff;
	//address��void*��ǿ��ת��
	Cap_t *cap = (Cap_t *)rx_instance->module_address;


#if SUPER_CAP_TYPE == ROBOFUTURE_CAP	
	Robofuture_Cap_Measure_t *measure = &cap->robofuture_cap;
	get_robofuture_cap_measure(measure,rxbuff);


#elif SUPER_CAP_TYPE == WULIE_CAP	
	WuLie_Cap_Measure_t *measure = &cap->wulie_cap;
	get_wulie_cap_measure(measure,rxbuff);
#endif
}

/**
	* @brief          ���������๹�캯��
  * @param[in]      NULL
  * @retval         Null
  */
Cap_t::Cap_t()
{
	
}

/**
	* @brief          �������ݳ�ʼ��
	* @param[in]      can_num������CAN�ߺ�
	* @param[in]      module_rx_id�����õ�ID
  * @retval         Null
  */
void Cap_t::SuperCapInit(CAN_NAME_e can_num,uint32_t input_id)
{
	CANRxInitSet(&super_cap_can,can_num,input_id,this,DecodeSuperCap);
	CANRxRegister(&super_cap_can);
}
