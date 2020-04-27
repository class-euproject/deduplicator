#ifndef SENDER_H
#define SENDER_H

#include <iostream>
#include <sys/socket.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include "utils.h"
#include "ClassAggregatorMessage.h"
#include "LogWriter.h"
#include "../masa_protocol/include/communicator.hpp"

namespace fog {

typedef void * (*THREADFUNCPTR)(void *);

class Sender {

private:
    ClassAggregatorMessage *cm;
    std::vector<std::string> ipList;
    std::vector<int> portList;
    std::vector<int> socketDescList;
    int numComm;
    std::vector<Communicator<MasaMessage>> *comm;
    LogWriter *lw;
    pthread_t writerThread;

public:
    Sender(ClassAggregatorMessage &sharedMessage,
            std::vector<std::string> ipList_,
            std::vector<int> portList_);
    ~Sender();
    void start();
    void end();
    void *send(void *n);
};
}

#endif /* SENDER_H */