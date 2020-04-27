#ifndef RECEIVER_H
#define RECEIVER_H

#include <iostream>
#include <sys/socket.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include "utils.h"
#include "ClassAggregatorMessage.h"
#include "LogWriter.h"
#include "../masa_protocol/include/communicator.hpp"

typedef void * (*THREADFUNCPTR)(void *);

class Receiver {

private:
    ClassAggregatorMessage *cm;
    LogWriter *lw;
    pthread_t snifferThread;
    int port;
    int socketDesc;
    Communicator<MasaMessage> *comm;
public:    
    Receiver(ClassAggregatorMessage &sharedMessage,
             int port_);
    ~Receiver();
    void start();
    void end();
    void *receive(void *n);
};

#endif /* RECEIVER_H */