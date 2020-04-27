#ifndef CAMESSAGE_H
#define CAMESSAGE_H

#include <pthread.h>
#include <assert.h>
#include "../masa_protocol/include/messages.hpp"

namespace fog {

class ClassAggregatorMessage {
protected:
    std::vector<MasaMessage> messageList;
    pthread_mutex_t mutex;
    bool deleteOld(MasaMessage m);
public:
    ClassAggregatorMessage();
    ~ClassAggregatorMessage() {};
    void insertMessage(MasaMessage m);
    std::vector<MasaMessage> getMessages();
    
    
};
}

#endif /* CAMESSAGE_H */