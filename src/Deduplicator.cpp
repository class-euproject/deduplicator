#include "Deduplicator.h"

namespace fog {

void Deduplicator::create_message_from_tracker(const std::vector<tracking::Tracker> &trackers, MasaMessage *m) {
    double lat, lon, alt;
    for (auto t : trackers) {
        if (t.predList.size() > 0) {
            Categories cat = (Categories) t.cl;
            this->gc.enu2Geodetic(t.predList.back().x, t.predList.back().y, 0, &lat, &lon, &alt);
            int pix_x, pix_y;
            GPS2pixel(this->adfGeoTransform, lat, lon, pix_x, pix_y);
            uint8_t orientation = orientation_to_uint8(t.predList.back().yaw);
            uint8_t velocity = speed_to_uint8(t.predList.back().vel);
            RoadUser r{static_cast<float>(lat), static_cast<float>(lon), velocity, orientation, cat};
            //std::cout << std::setprecision(10) << r.latitude << " , " << r.longitude << " " << int(r.speed) << " " << int(r.orientation) << " " << r.category << std::endl;
            m->objects.push_back(r);
        }
    }
}

Deduplicator::Deduplicator(ClassAggregatorMessage &inputSharedMessage, 
                            ClassAggregatorMessage &outputSharedMessage,
                            std::string tifFile,
                            bool visual) {
    inCm = &inputSharedMessage;
    outCm = &outputSharedMessage;
    adfGeoTransform = (double *)malloc(6 * sizeof(double));
    readTiff((char *)tifFile.c_str(), adfGeoTransform);
    /* The initial age indicates how many frames the tracker considers to keep an 
    *object alive after it fails to detect it. It may be higher than the normal value. */
    initialAge = 5;//8; //15; //-5;
    nStates = 5;
    dt = 0.03;
    trVerbose = false;
    gc.initialiseReference(44.655540, 10.934315, 0);
    t = new tracking::Tracking(nStates, dt, initialAge);
    show = visual;
}

Deduplicator::Deduplicator(double* adfGeoTransform, const double latitude, const double longitude) {
    this->adfGeoTransform = adfGeoTransform;
    this->gc.initialiseReference(latitude, longitude, 0);
    /* The initial age indicates how many frames the tracker considers to keep an
    *object alive after it fails to detect it. It may be higher than the normal value. */
    initialAge = 5;//8; //15; //-5;
    nStates = 5;
    dt = 0.03;
    trVerbose = false;
    std::cout << "BEFORE T" << std::endl;
    t = new tracking::Tracking(nStates, dt, initialAge);
    std::cout << "AFTER T" << std::endl;
}

Deduplicator::~Deduplicator() {
    free(adfGeoTransform);
    delete t;
}

void Deduplicator::start() {
    if (pthread_create(&deduplicatorThread, NULL, (THREADFUNCPTR) &Deduplicator::deduplicate, this)) 
        perror("could not create thread");
}

void Deduplicator::end() {
    pthread_join(deduplicatorThread, NULL);
}

/**
 * Check if two o more messages from the same cam_idx (road users or traffic lights) are arrived.
 * Remove the old one.
*/
std::vector<MasaMessage> Deduplicator::filterOldMessages(std::vector<MasaMessage> input_messages) {
    // std::cout<<"messages: "<<input_messages.size()<<std::endl;
    std::vector<MasaMessage> copy = input_messages;
    std::vector<int> delete_ids;
    for (int i = 0; i<input_messages.size()-1; i++) {
        for (int j = i+1; j<input_messages.size(); j++) {
            if(input_messages.at(i).cam_idx == input_messages.at(j).cam_idx) {
                if(input_messages.at(i).t_stamp_ms < input_messages.at(j).t_stamp_ms)
                    delete_ids.push_back(i);
                else
                    delete_ids.push_back(j);
            }
        }
    }
    if (delete_ids.size()==0)
        return input_messages;
    if (delete_ids.size() != 1)
        std::sort(delete_ids.begin(), delete_ids.end(), [](int a, int b) {return a > b; });
    delete_ids.erase( std::unique( delete_ids.begin(), delete_ids.end() ), delete_ids.end() );
    // for(int i=0; i<delete_ids.size(); i++) { std::cout<<delete_ids[i]<<" "<<std::endl;}
    // std::cout<<"\n";
    for(auto d : delete_ids)
        copy.erase(copy.begin() + d);
    return copy;
}

/**
 * Compute the deduplication:
 * - for the road users it uses the tracker. The tracker deletes a point if two different points 
 * are close enough together.
 *  - for the traffic lights it includes all information. Simply delete old messages 
 * (filterOldMessages)
*/
void Deduplicator::computeDeduplication(std::vector<MasaMessage> input_messages, MasaMessage &deduplicate_message) {
    std::vector<tracking::obj_m> cur_message;
    double east, north, up;
    // deduplicate with the Tracker: fill only cur_message with the information of all collected MasaMessage. 
    deduplicate_message.lights.clear(); 
    deduplicate_message.objects.clear();
    deduplicate_message.t_stamp_ms = time_in_ms();
    for(auto m : input_messages) {
        for(size_t i = 0; i < m.objects.size(); i++) {
            // skip some special road user 
            if(m.objects.at(i).category == C_marelli1 || 
               m.objects.at(i).category == C_marelli2 || 
               m.objects.at(i).category == C_quattroporte ||
               m.objects.at(i).category == C_levante || 
               m.objects.at(i).category == C_rover) {
                deduplicate_message.objects.push_back(m.objects.at(i));    
            }
            else { // the normal road user pass to tracker
                this->gc.geodetic2Enu(m.objects.at(i).latitude, m.objects.at(i).longitude, 0, &east, &north, &up);
                cur_message.push_back(tracking::obj_m(east, north, 0, m.objects.at(i).category, 1, 1,
                                                      m.objects.at(i).latitude, m.objects.at(i).longitude,
                                                      m.objects.at(i).idx, m.objects.at(i).idy, 0.0)); //use pixel_x and
                                                      // pixel_y as pointers to info_for_dedu
            }
        }
        for(size_t i = 0; i < m.lights.size(); i++)
            deduplicate_message.lights.push_back(m.lights.at(i)); 
    }
    std::cout << "BEFORE TRACK" << std::endl;
    std::cout << "cur_message size is " << cur_message.size() << std::endl;
    this->t->track(cur_message,this->trVerbose);
    std::cout << "AFTER TRACK" << std::endl;

    this->create_message_from_tracker(t->getTrackers(), &deduplicate_message);

    deduplicate_message.num_objects = deduplicate_message.objects.size();
}

/**
 * Thread function: it waits for a message list, it filters old messages from the same cam_idx 
 * comparing the timestamp, it computes the deduplication, then it shows the updates to the viewer.
 * At the end it sends the deduplicated message. 
*/
void * Deduplicator::deduplicate(void *n) {
    std::vector<MasaMessage> input_messages; 
    MasaMessage deduplicate_message;
    // std::vector<cv::Point2f> map_pixels;
    // fog::Profiler prof("Deduplicator");
    while(gRun){
        /* Delay is necessary. If the Deduplicator takes messages too quickly there is a risk 
        of not tracking the road users correctly. each message is read as a frame. 
        See the initialAge variable. */
        // prof.tick("total time");
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        // prof.tick("get messages");
        input_messages = this->inCm->getMessages();
        // prof.tock("get messages");
        std::cout<<"dedup dim reading list: "<<input_messages.size()<<std::endl;
        if(input_messages.size() == 0) {
            // prof.tick("total time");
            continue;        // no received messages
        }
        std::cout<<"get messages\n";
        // prof.tick("filter old");
        // filter old messages from the same id (camera or traffic light)
        input_messages = filterOldMessages(input_messages);
        // prof.tock("filter old");
        // prof.tick("deduplication");
        // takes the input messages and return the deduplicate message
        computeDeduplication(input_messages, deduplicate_message);
        // prof.tock("deduplication");
        // prof.tick("insert message");
        this->outCm->insertMessage(deduplicate_message);
        input_messages.clear();
        // prof.tock("insert message");
        // prof.tock("total time");
        // prof.printStats();
    }
    return (void *)NULL;
}
}