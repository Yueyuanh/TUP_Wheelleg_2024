#ifndef __OBSERVE_PATTERN_H
#define __OBSERVE_PATTERN_H

#include "struct_typedef.h"
//#include "iostream"

//最大话题数
#define MAX_SUBJECT 4

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus	
	
//观察者内需创建此类的实例
class Observe_t
{
	public:
		virtual void update(void *msg);
};

//需要新传递数据时需要添加主题
class Subject_t
{
	private:
		Observe_t *observers[3];	//最多支持3个观察者
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
