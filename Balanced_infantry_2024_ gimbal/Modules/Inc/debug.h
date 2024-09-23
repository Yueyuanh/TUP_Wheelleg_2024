#ifndef __DEBUG_H
#define __DEBUG_H

#include "struct_typedef.h"

//���Ժ�������ʱ��ĺ���
#define TEST_FUNC_TIME(test,func)   	\
	(test).RecordStartTime();					  \
	func;																\
	(test).CalcExcutePeriod();					\
	
#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus
//������������
class Test_Module_t
{
	private:
		uint64_t start_excute_time,finish_excute_time;
		uint64_t period;
	public:
		Test_Module_t();
		void RecordStartTime();
		uint64_t CalcExcutePeriod();
		
};
	
#endif
	
#ifdef __cplusplus
}
#endif

#endif
