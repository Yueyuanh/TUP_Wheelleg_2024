/**
 ******************************************************************************
 * @file    super_cap.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang 基本完成 2023/10/30
 * @date    2023/10/28
 * @brief		此处为超级电容模块
 ******************************************************************************
 * @attention
 *	此处注册登记了超级电容模块挂载的CAN线信息，可读取数据
 ******************************************************************************
 */
#include "super_cap.h"

//获取超级电容数据
#define	get_wulie_cap_measure(ptr, data)                                                     \
{   uint16_t *buf =    (uint16_t*) data;               \
		(ptr)->Cap_input_vol = ((fp32)buf[0]/100.f);           \
		(ptr)->Cap_voltage = ((fp32)buf[1]/100.f);     \
		(ptr)->Cap_current = ((fp32)buf[2]/100.f); \
		(ptr)->Cap_power = ((fp32)buf[3]/100.f); \
}

//浙纺超电
#define get_robofuture_cap_measure(ptr, data)			\
{																									\
	(ptr)->warning = data[0];           \
	(ptr)->charge_sign = (data)[1];     \
	(ptr)->chassis_power = (uint16_t)((data)[2] << 8 | (data)[3]); \
	(ptr)->ele_quantity = (uint16_t)((data)[4] << 8 | (data)[5]); \
}

//创建实例
Cap_t super_cap;

/**
	* @brief          登记超级电容模块CAN线信息
	* @param[in]      can_num：挂载CAN线号
	* @param[in]      input_id：设置的ID
  * @retval         Null
  */
void RegisterSuperCap(CAN_NAME_e can_num,uint32_t input_id)
{
	super_cap.SuperCapInit(can_num,input_id);
}

/**
	* @brief          返回超级电容指针，用于与外部信息交互
	* @param[in]      NULL
  * @retval         &super_cap
  */
Cap_t *GetCapPointer()
{
	return &super_cap;
}

/**
	* @brief          超级电容数据解码
  * @param[in]      *rx_instance：接收的实例
  * @retval         Null
  */
static void DecodeSuperCap(CAN_Rx_Instance_t *rx_instance)
{
	uint8_t *rxbuff = rx_instance->rx_buff;
	//address是void*需强制转化
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
	* @brief          超级电容类构造函数
  * @param[in]      NULL
  * @retval         Null
  */
Cap_t::Cap_t()
{
	
}

/**
	* @brief          超级电容初始化
	* @param[in]      can_num：挂载CAN线号
	* @param[in]      module_rx_id：设置的ID
  * @retval         Null
  */
void Cap_t::SuperCapInit(CAN_NAME_e can_num,uint32_t input_id)
{
	CANRxInitSet(&super_cap_can,can_num,input_id,this,DecodeSuperCap);
	CANRxRegister(&super_cap_can);
}
