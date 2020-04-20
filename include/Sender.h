#ifndef SENDER_H
#define SENDER_H

#include <iostream>
#include <sys/socket.h>
#include <stdio.h>
#include "ClassAggregatorMessage.h"
#include "LogWriter.h"
#include "../masa_protocol/include/communicator.hpp"

typedef void * (*THREADFUNCPTR)(void *);

class Sender {

private:
    ClassAggregatorMessage *cm;
    std::vector<std::string> ip_list;
    std::vector<int> port_list;
    std::vector<int> socket_desc_list;
    int num_comm;
    std::vector<Communicator<MasaMessage>> *comm;
    LogWriter *lw;
    pthread_t writer_thread;

public:
    Sender(ClassAggregatorMessage &sharedMessage,
            std::vector<std::string> ipList,
            std::vector<int> portList);
    ~Sender();
    void start();
    void end();
    void *send(void *n);
};

#endif /* SENDER_H */