#include <iostream>
#include <pthread.h>
#include "utils.h"
#include "configuration.h"
#include "ClassAggregatorMessage.h"
#include "Receiver.h"
#include "Sender.h"
#include "Deduplicator.h"
#include "Aggregator.h"
#include "Profiler.h"

int main(int argc, char **argv) {
    fog::ClassAggregatorMessage received_messages;       // Receiver fills this message, Deduplicator deduplicates its objects

    // read yaml parameters file
    fog::Parameters_t param;
    if(!readParameters(argc, argv, &param)) {
        exit(EXIT_SUCCESS);         // help
    }

    pthread_mutexattr_t mutexattr;
    pthread_mutexattr_init(&mutexattr);
    pthread_mutex_init(&profiler_mutex, &mutexattr);
    pthread_mutexattr_destroy(&mutexattr);
    
    gRun = true;

    // TODO: iterate on ports:
    if (param.inputPortList.size() > 1)
        std::cerr<<"many input ports: no supported. Use only the first...\n";
    fog::Receiver r(received_messages, param.inputPortList[0], param.bsc_reciving_port, param.edge_log_saving);
    r.start();

    r.end();
    return EXIT_SUCCESS;
}