/**
 ******************************************************************************
 * @file    debug.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang 添加模块测试类 2023/11/12
 *					V1.1.0 Xushuang 添加宏函数 2023/11/13
 * @date    2023/11/13
 * @brief		此处为模块测试类的定义处，封装了性能测试所需的模块，可方便使用
 ******************************************************************************
 * @attention
 *	正常使用测试单模块或者函数运行速度时可以之间在所需要测试的类中添加Test_Module_t
 *类，在需要测试的函数内调用两个函数即可实现。也可直接使用TEST_FUNC_TIME这个宏函数测
 *试函数运行时长。
 *	例：需要监测InfoUpdate函数运行时长
 *	void InfoUpdate()
 *	{
 *		Test_Module_t.RecordStartTime();
 *		- - - - 正常运行部分 - - - - 
 *		Test_Module_t.CalcExcutePeriod();
 *	}
 ******************************************************************************
 */
#include "debug.h"
#include "bsp_dwt.h"

//构造函数
Test_Module_t::Test_Module_t()
{
	start_excute_time = finish_excute_time = period = 0;
}
/**
	* @brief          记录模块开始运行时的时间
  * @param[in]      NULL
  * @retval         NULL
  */
void Test_Module_t::RecordStartTime()
{
	start_excute_time = DWT_GetTimeline_us();
}

/**
	* @brief          计算模块运行时间
  * @param[in]      NULL
  * @retval         period：运行周期
  */
uint64_t Test_Module_t::CalcExcutePeriod()
{
	finish_excute_time = DWT_GetTimeline_us();
	//减去自身运行时间（约为5us）
	if((period=finish_excute_time - start_excute_time) >= 5)
		period -= 5;
	return period;
}
