#ifndef __PUB_SUB_PATTERN_H
#define __PUB_SUB_PATTERN_H

#include "stdio.h"

#ifdef __cplusplus
extern "C"{
#endif
	
#ifdef __cplusplus	

extern "C" {
    typedef struct Publisher Publisher;

    Publisher* Publisher_Create();
    void Publisher_Destroy(Publisher* publisher);
    void Publisher_Publish(Publisher* publisher, void* data, size_t dataSize);
    void Publisher_Subscribe(Publisher* publisher, void (*callback)(void*, size_t));
}

//class PublisherImpl {
//public:
//    void Publish(void* data, size_t dataSize); 
//    void Subscribe(void (*callback)(void*, size_t)) {
//        subscribers.push_back(callback);
//    }

//private:
//    std::vector<void (*)(void*, size_t)> subscribers;
//};


	
	
#endif
	
#ifdef __cplusplus
}
#endif

#endif

