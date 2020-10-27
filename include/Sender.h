#ifndef SENDER_H
#define SENDER_H

#include <iostream>
#include <sys/socket.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include "utils.h"
#include "Profiler.h"
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
    std::vector<Communicator<MasaMessageOrig>> *comm;
    LogWriter *lw;
    bool lw_flag;
    pthread_t writerThread;
    int myId;

public:
    Sender(ClassAggregatorMessage &sharedMessage,
            std::vector<std::string> ipList_,
            std::vector<int> portList_,
            int myCamIdx, bool logWriterFlag);
    ~Sender();
    void start();
    void end();
    void *send(void *n);
};
}

#endif /* SENDER_H */