#include "observe_pattern.h"
//#include "stdio.h"

//��ӹ۲���
void Subject_t::Attach(Observe_t *observer)
{
	if(observe_count < 3)
	{
		observers[observe_count] = observer;
		++observe_count;
	}
}

////��������
//void Subject_t::SetData(void *message)
//{
//	send_message = message;
////	Notify();
//}

////���ݸ���
//void Subject_t::Notify()
//{
//	for(int i = 0;i < observe_count; ++i)
//		observers[i]->update(send_message);
//}
