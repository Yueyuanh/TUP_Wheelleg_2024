#include "pub_sub_pattern.h"
#include <vector>
class PublisherImpl {
public:
    void Publish(void* data, size_t dataSize) {
        for (std::vector<void (*)(void*, size_t)>::iterator it = subscribers.begin(); it != subscribers.end(); ++it) {
            (*it)(data, dataSize);
        }
    }

    void Subscribe(void (*callback)(void*, size_t)) {
        subscribers.push_back(callback);
    }

private:
    std::vector<void (*)(void*, size_t)> subscribers;
};

extern "C" {
    Publisher* Publisher_Create() {
        return reinterpret_cast<Publisher*>(new PublisherImpl);
    }

    void Publisher_Destroy(Publisher* publisher) {
        delete reinterpret_cast<PublisherImpl*>(publisher);
    }

    void Publisher_Publish(Publisher* publisher, void* data, size_t dataSize) {
        reinterpret_cast<PublisherImpl*>(publisher)->Publish(data, dataSize);
    }

    void Publisher_Subscribe(Publisher* publisher, void (*callback)(void*, size_t)) {
        reinterpret_cast<PublisherImpl*>(publisher)->Subscribe(callback);
    }
}
