#ifndef __OBSERVE_PATTERN_H
#define __OBSERVE_PATTERN_H

#include "struct_typedef.h"
//#include "iostream"

//�������
#define MAX_SUBJECT 4

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus	
	
//�۲������贴�������ʵ��
class Observe_t
{
	public:
		virtual void update(void *msg);
};

//��Ҫ�´�������ʱ��Ҫ�������
class Subject_t
{
	private:
		Observe_t *observers[3];	//���֧��3���۲���
		int observe_count;
		void 	*send_message;
	
	public:
		void Attach(Observe_t *observer);
//		void SetData(void *message);
//		void Notify();
};

#endif
	
#ifdef __cplusplus
}
#endif

#endif
