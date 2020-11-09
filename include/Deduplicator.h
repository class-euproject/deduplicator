#ifndef DEDUPLICATOR_H
#define DEDUPLICATOR_H

#include <assert.h>
#include <chrono>
#include <thread>
#include "ClassAggregatorMessage.h"
#include "utils.h"
#include "Profiler.h"
#include "../class-tracker/include/obj.h"
#include "../class-tracker/include/Tracking.h"

namespace fog {

typedef void * (*THREADFUNCPTR)(void *);

class Deduplicator {
private:
    ClassAggregatorMessage *inCm, *outCm;
    double *adfGeoTransform;
    // Initialize tracker information
    // std::vector<Tracker> trackers;
    int initialAge;
    int nStates;
    float dt;
    bool trVerbose;
    geodetic_converter::GeodeticConverter gc;

    pthread_t deduplicatorThread;
    bool show;
public:

    tracking::Tracking *t;

    Deduplicator(ClassAggregatorMessage &inputSharedMessage, 
                 ClassAggregatorMessage &outputSharedMessage,
                 std::string tifFile,
                 bool visual);
    Deduplicator(double* adfGeoTransform, double latitude, double longitude);
    ~Deduplicator();
    void start();
    void end();
    std::vector<MasaMessage> filterOldMessages(std::vector<MasaMessage> input_messages);
    void computeDeduplication(std::vector<MasaMessage> input_messages, 
                              MasaMessage &deduplicate_message);
    void create_message_from_tracker(const std::vector<tracking::Tracker> &trackers, MasaMessage *m);
    void *deduplicate(void *n);
};
}

#endif /* DEDUPLICATOR_H */