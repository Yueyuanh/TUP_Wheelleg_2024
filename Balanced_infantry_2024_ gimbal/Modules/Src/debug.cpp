/**
 ******************************************************************************
 * @file    debug.cpp
 * @author  Xushuang
 * @version V1.0.0 Xushuang ���ģ������� 2023/11/12
 *					V1.1.0 Xushuang ��Ӻ꺯�� 2023/11/13
 * @date    2023/11/13
 * @brief		�˴�Ϊģ�������Ķ��崦����װ�����ܲ��������ģ�飬�ɷ���ʹ��
 ******************************************************************************
 * @attention
 *	����ʹ�ò��Ե�ģ����ߺ��������ٶ�ʱ����֮��������Ҫ���Ե��������Test_Module_t
 *�࣬����Ҫ���Եĺ����ڵ���������������ʵ�֡�Ҳ��ֱ��ʹ��TEST_FUNC_TIME����꺯����
 *�Ժ�������ʱ����
 *	������Ҫ���InfoUpdate��������ʱ��
 *	void InfoUpdate()
 *	{
 *		Test_Module_t.RecordStartTime();
 *		- - - - �������в��� - - - - 
 *		Test_Module_t.CalcExcutePeriod();
 *	}
 ******************************************************************************
 */
#include "debug.h"
#include "bsp_dwt.h"

//���캯��
Test_Module_t::Test_Module_t()
{
	start_excute_time = finish_excute_time = period = 0;
}
/**
	* @brief          ��¼ģ�鿪ʼ����ʱ��ʱ��
  * @param[in]      NULL
  * @retval         NULL
  */
void Test_Module_t::RecordStartTime()
{
	start_excute_time = DWT_GetTimeline_us();
}

/**
	* @brief          ����ģ������ʱ��
  * @param[in]      NULL
  * @retval         period����������
  */
uint64_t Test_Module_t::CalcExcutePeriod()
{
	finish_excute_time = DWT_GetTimeline_us();
	//��ȥ��������ʱ�䣨ԼΪ5us��
	if((period=finish_excute_time - start_excute_time) >= 5)
		period -= 5;
	return period;
}
