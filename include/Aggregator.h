#ifndef AGGREGATOR_H
#define AGGREGATOR_H

#include <assert.h>
#include <chrono>
#include <thread>
#include "ClassAggregatorMessage.h"
#include "utils.h"
#include "AggregatorViewer.h"

typedef void * (*THREADFUNCPTR)(void *);

class Aggregator {
private:
    ClassAggregatorMessage *inCm, *outCm;
    pthread_t aggregatorThread;
    AggregatorViewer *viewer;
    bool show;
public:
    Aggregator(ClassAggregatorMessage &inputSharedMessage, 
               ClassAggregatorMessage &outputSharedMessage,
               AggregatorViewer &v,
               bool visual);
    ~Aggregator();
    void start();
    void end();
    void *aggregate(void *n);
};

#endif /* AGGREGATOR_H */