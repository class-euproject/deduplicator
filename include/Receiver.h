#ifndef RECEIVER_H
#define RECEIVER_H

#include <iostream>
#include <sys/socket.h>
#include <stdio.h>
#include "ClassAggregatorMessage.h"
#include "LogWriter.h"
#include "../masa_protocol/include/communicator.hpp"

typedef void * (*THREADFUNCPTR)(void *);

class Receiver {

private:
    ClassAggregatorMessage *cm;
    LogWriter *lw;
    pthread_t sniffer_thread;
    int port;
    int socket_desc;
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
