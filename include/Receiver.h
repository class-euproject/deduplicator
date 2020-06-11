#ifndef RECEIVER_H
#define RECEIVER_H

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

class Receiver {

private:
    ClassAggregatorMessage *cm;
    LogWriter *lw;
    bool lw_flag;
    pthread_t snifferThread;
    int port;
    int socketDesc;
    Communicator<MasaMessage> *comm;
public:    
    Receiver(ClassAggregatorMessage &sharedMessage,
             int port_, bool logWriterFlag);
    ~Receiver();
    void start();
    void end();
    void *receive(void *n);
};
}

#endif /* RECEIVER_H */