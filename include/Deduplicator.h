#ifndef DEDUPLICATOR_H
#define DEDUPLICATOR_H

#include <assert.h>
#include "ClassAggregatorMessage.h"
#include "utils.h"
#include "../tracker_CLASS/c++/include/Data.h"
#include "../tracker_CLASS/c++/include/Tracking.h"

typedef void * (*THREADFUNCPTR)(void *);

class Deduplicator {
private:
    ClassAggregatorMessage *inCm, *outCm;
    double *adfGeoTransform;
    // Initialize tracker information
    // std::vector<Tracker> trackers;
    Tracking *t;
    int initialAge;
    int ageThreshold;
    int nStates;
    float dt;
    bool trVerbose;
    geodetic_converter::GeodeticConverter gc;

    pthread_t deduplicatorThread;

public:
    Deduplicator(ClassAggregatorMessage &inputSharedMessage, 
                 ClassAggregatorMessage &outputSharedMessage,
                 std::string tifFile);
    ~Deduplicator();
    void start();
    void end();
    void *deduplicate(void *n);
};

#endif /* DEDUPLICATOR_H */