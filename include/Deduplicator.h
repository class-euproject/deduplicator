#ifndef DEDUPLICATOR_H
#define DEDUPLICATOR_H

#include <assert.h>
#include <chrono>
#include <thread>
#include <numeric>
#include <set>
#include "ClassAggregatorMessage.h"
#include "utils.h"
#include "Profiler.h"
//#include "AggregatorViewer.h" // TODO: check if needed
#include "../class-tracker/include/obj.h"
#include "../class-tracker/include/Tracking.h"

namespace fog {

typedef void * (*THREADFUNCPTR)(void *);

#ifndef DDSTRUCT_H
#define DDSTRUCT_H
/**
 * Data structure to perform deduplication
 */
struct DDstruct{
    RoadUser rs;
    float distance;
    size_t message_index;
    size_t object_index;
};

inline bool operator==( DDstruct const& lhs, DDstruct const& rhs) { return lhs.rs.latitude == rhs.rs.longitude      and lhs.rs.error == rhs.rs.error and
                                                                           lhs.rs.category == rhs.rs.category       and lhs.rs.speed == rhs.rs.speed and
                                                                           lhs.rs.orientation == rhs.rs.orientation and lhs.distance == rhs.distance and
                                                                           lhs.message_index == rhs.message_index   and lhs.object_index == rhs.object_index;}
#endif
    

class Deduplicator {
private:
    //multiple deduplication threshold for different road users
    const float CAR_THRESHOLD = 2.5; 
    const float AUTOBUS_THRESHOLD = 4.0;
    const float PERSON_THRESHOLD = 0.5;
    ClassAggregatorMessage *inCm, *outCm;
    double *adfGeoTransform;
    // Initialize tracker information
    // std::vector<Tracker> trackers;
    // tracking::Tracking *t;
    tracking::Tracking *edge_tr;
    int initialAge;
    int nStates;
    float dt;
    bool trVerbose;
    // geodetic_converter::GeodeticConverter gc;

    pthread_t deduplicatorThread;
    bool show;

    float distance(const RoadUser object1, const RoadUser object2);
    //bool nearest_of(const MasaMessage message, const DDstruct ref, const float threshold, DDstruct & ris);
    void deduplicationFromMessages(std::vector<MasaMessage> &input_messages);
public:

    geodetic_converter::GeodeticConverter gc;
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
    //void create_message_from_tracker(const std::vector<tracking::Tracker> &trackers, MasaMessage *m, );

    std::vector<MasaMessage> fillTrackerInfo(std::vector<MasaMessage> input_messages);
    void elaborateMessages(std::vector<MasaMessage>& input_messages, 
                              MasaMessage &output_message,
                              std::map<std::pair<uint16_t, uint16_t>, RoadUser>& last_duplicated_objects);
    // void showUpdates();
    void *deduplicate(void *n);
    double uint8_to_speed(const uint8_t speed);
    double uint16_to_yaw(const uint16_t yaw);
};
}

#endif /* DEDUPLICATOR_H */
