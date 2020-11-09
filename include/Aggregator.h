#ifndef AGGREGATOR_H
#define AGGREGATOR_H

#include <assert.h>
#include <chrono>
#include <thread>
#include "ClassAggregatorMessage.h"
#include "utils.h"
#include "Profiler.h"

namespace fog {

typedef void * (*THREADFUNCPTR)(void *);

class Aggregator {
private:
    ClassAggregatorMessage *inCm, *outCm;
    pthread_t aggregatorThread;
    bool show;
public:
    Aggregator(ClassAggregatorMessage &inputSharedMessage, 
               ClassAggregatorMessage &outputSharedMessage,
               bool visual);
    ~Aggregator();
    void start();
    void end();
    void *aggregate(void *n);
};
}

#endif /* AGGREGATOR_H */