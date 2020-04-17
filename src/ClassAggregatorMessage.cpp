#include "ClassAggregatorMessage.h"

ClassAggregatorMessage::ClassAggregatorMessage() {
    pthread_mutexattr_t mutexattr;
    pthread_mutexattr_init(&mutexattr);
    pthread_mutex_init(&mutex, &mutexattr);
    pthread_mutexattr_destroy(&mutexattr);
}

bool ClassAggregatorMessage::deleteOld(MasaMessage m) {
    bool deleted = false;
    auto it = message_list.begin();
    while (it != message_list.end()) {
        if (it->cam_idx == m.cam_idx && it->t_stamp_ms < m.t_stamp_ms) {
            it = message_list.erase(it);
            deleted = true;
        }
        else 
            ++it;
    }
    return deleted;
}

void ClassAggregatorMessage::insertMessage(MasaMessage m) {
    pthread_mutex_lock(&mutex);
    if (deleteOld(m)) 
        std::cout<<"some message are deleted -- you are too slow\n";
    message_list.push_back(m);
    pthread_mutex_unlock(&mutex);
}

std::vector<MasaMessage> ClassAggregatorMessage::getMessages() {
    pthread_mutex_lock(&mutex);
    std::vector<MasaMessage> copy_list = message_list;
    message_list.clear();
    pthread_mutex_unlock(&mutex);
    assert(copy_list.size() != 0);
    return copy_list;
}