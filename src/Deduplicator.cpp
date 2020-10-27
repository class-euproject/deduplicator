#include "Deduplicator.h"

namespace fog {

void create_message_from_tracker(const std::vector<tracking::Tracker> &trackers, MasaMessage *m, 
                                 geodetic_converter::GeodeticConverter &gc, double *adfGeoTransform) {
    double lat, lon, alt;
    for (auto t : trackers) {
        if (t.predList.size() > 0) {
            Categories cat = (Categories) t.cl;
            gc.enu2Geodetic(t.predList.back().x, t.predList.back().y, 0, &lat, &lon, &alt);
            int pix_x, pix_y;
            GPS2pixel(adfGeoTransform, lat, lon, pix_x, pix_y);
            uint8_t orientation = orientation_to_uint8(t.predList.back().yaw);
            uint8_t velocity = speed_to_uint8(t.predList.back().vel);
            RoadUser r;
            r.latitude = static_cast<float>(lat);
            r.longitude = static_cast<float>(lon); 
            r.precision = 0.0;
            r.speed = velocity;
            r.orientation = orientation;
            r.category = cat;
            //std::cout << std::setprecision(10) << r.latitude << " , " << r.longitude << " " << int(r.speed) << " " << int(r.orientation) << " " << r.category << std::endl;
            m->objects.push_back(r);
        }
    }
}

Deduplicator::Deduplicator(ClassAggregatorMessage &inputSharedMessage, 
                            ClassAggregatorMessage &outputSharedMessage,
                            std::string tifFile,
                            AggregatorViewer &v,
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
    viewer = &v;
    show = visual;
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
 * Compute euclidian distance 
*/
long double Deduplicator::distance(const RoadUser object1, const RoadUser object2) {
    double north1, north2, east1, east2, up;

    this->gc.geodetic2Enu(object1.latitude, object1.longitude, 0, &north1, &east1, &up);
    this->gc.geodetic2Enu(object2.latitude, object2.longitude, 0, &north2, &east2, &up);
    return sqrt(pow(abs(north1 - north2), 2) + pow(abs(east1 - east2), 2));
}

/**
 * Compute the nearest object under a certain threshold (having the same category of) 
 * to the reference in the given MasaMessage, id any.
 * 
*/
bool Deduplicator::nearest_of(const MasaMessage message, const DDstruct ref, const float threshold, DDstruct & ris){
    DDstruct nearest;
    bool empty = true;
    for(size_t i = 0; i < message.objects.size(); i++){

        if(ref.rs.category == message.objects.at(i).category){
            float distance = this->distance(message.objects.at(i), ref.rs);
            if(distance < threshold){
                if(empty == true or nearest.distance > distance){
                    nearest.rs = message.objects.at(i);
                    nearest.distance = distance;
                    nearest.object_index = i;
                    ris = nearest;
                    empty = false;
                } 
            }
        }
    }
    return empty;
}

/**
 * Compute deduplication from input messages
 * 
*/
void Deduplicator::deduplicationFromMessages(std::vector<MasaMessage> &input_messages){

    const float CAR_THRESHOLD = 1.5;  //multiple deduplication threshold for different road users
    const float PERSON_THRESHOLD = 0.5;

    std::vector<DDstruct> to_keep;
    std::vector<MasaMessage> deduplicated_messages;
    deduplicated_messages.resize(input_messages.size());

    for(size_t i = 0; i < input_messages.size(); i++){

        for(auto elem : input_messages.at(i).lights)
            deduplicated_messages.at(i).lights.push_back(elem);

        for(size_t j = 0; j < input_messages.at(i).objects.size(); j++){
            float threshold;
            switch (input_messages.at(i).objects.at(j).category)
            {
            case Categories::C_person:
                threshold = PERSON_THRESHOLD;
                break;
            case Categories::C_car:
                threshold = CAR_THRESHOLD;
                break;
            default:
                threshold = CAR_THRESHOLD;
                break;
            }

            std::vector<DDstruct> nearest;
            DDstruct ref;
            ref.rs = input_messages.at(i).objects.at(j);
            ref.message_index = i;
            ref.object_index = j;
            ref.distance = 0.0;
            nearest.push_back(ref);
            for(size_t k = 0; k < i; k++){ //FORSE SERVE: esempio: l'ultimo messaggio sarebbe automaticamente messo tutto se non ci fosse
                DDstruct nearest_obj;       //mentre potrebbe caitare che ci sia un oggetto che non vada tenuto perchè duplicato di un altro oggetto 
                nearest_obj.message_index = k;  //trovato in precedenza!
                if (this->nearest_of(input_messages.at(k), ref, threshold, nearest_obj) == true){
                    nearest.push_back(nearest_obj);
                }
            }

            for(size_t k = i+1; k < input_messages.size(); k++){
                DDstruct nearest_obj;
                nearest_obj.message_index = k;
                if (this->nearest_of(input_messages.at(i), ref, threshold, nearest_obj) == true){
                    nearest.push_back(nearest_obj);
                }
            }

            if(nearest.size() == 1){
                //then first the inital object (i, j) have no neighboors. Just keep it.
                to_keep.push_back(nearest.at(0));
            } else {
                //near objects found. Keep the most precise (which has less precision) if it's not already in.
                std::sort(nearest.begin(), nearest.end(), [](const DDstruct & a, const DDstruct & b) -> bool
                { 
                    return a.rs.precision < b.rs.precision; 
                } );
                
                if(std::find(to_keep.begin(), to_keep.end(), nearest.at(0)) == to_keep.end()){
                    to_keep.push_back(nearest.at(0));
                    
                    //activate this to switch the duplications into pedestrians. comment also 214-218 and 225-226
                    /*for(size_t i = 0; i < nearest.size(); i++){

                        input_messages.at(nearest.at(i).message_index).objects.at(nearest.at(i).object_index).category = Categories::C_person;
                    }*/
                }
            }
        }
    }

    //finally, finish to create the output vector of MasaMessages and assign it to input_messages in order to return it
    for(size_t i = 0; i < to_keep.size(); i++){
        DDstruct tmp = to_keep.at(i);
        deduplicated_messages.at(tmp.message_index).objects.push_back(tmp.rs);
        deduplicated_messages.at(tmp.message_index).num_objects = deduplicated_messages.at(tmp.message_index).objects.size();
    }

    /*int prev_size = 0;
    for(auto message : input_messages){
        prev_size +=  message.objects.size();
    }*/

    input_messages.clear();
    input_messages = deduplicated_messages;

    /*int later_size = 0;
    for(auto message : input_messages){
        later_size +=  message.objects.size();
    }
    std::cout << "Number of deduplicated objects: " << prev_size - later_size << std::endl;*/
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

    //deduplicationFromMessages(input_messages);

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
                cur_message.push_back(tracking::obj_m(east, north, 0, m.objects.at(i).category, 1, 1));
            }
        }
        for(size_t i = 0; i < m.lights.size(); i++)
            deduplicate_message.lights.push_back(m.lights.at(i)); 
    }

    this->t->track(cur_message,this->trVerbose);

    create_message_from_tracker(t->getTrackers(), &deduplicate_message, this->gc, this->adfGeoTransform);

    deduplicate_message.num_objects = deduplicate_message.objects.size();

    	
}

/**
 * Get the tracker information and set the frame data to the viewer with the updated information
*/
void Deduplicator::showUpdates() {
    double latitude, longitude, altitude;
    //visualize the trackers
    int map_pix_x, map_pix_y; 
    std::vector<tracker_line>  lines;
    for(auto tr : this->t->getTrackers()) {
        if(tr.predList.size()) {
            tracker_line line;
            line.color = tk::gui::Color_t {tr.r, tr.g, tr.b, 255};
            for(auto traj: tr.predList) {
                //convert from meters to GPS
                this->gc.enu2Geodetic(traj.x, traj.y, 0, &latitude, &longitude, &altitude);
                //convert from GPS to map pixels
                GPS2pixel(this->adfGeoTransform, latitude, longitude, map_pix_x, map_pix_y);
                if(V3D)
                    line.points.push_back(this->viewer->convertPosition3D(map_pix_x,  map_pix_y, 2.004));
                else
                    line.points.push_back(this->viewer->convertPosition2D(map_pix_x,  map_pix_y, -0.004));            
            }
            lines.push_back(line);
        }
    }
    if(this->show)
        this->viewer->setFrameData(lines);
}

/**
 * Thread function: it waits for a message list, it filters old messages from the same cam_idx 
 * comparing the timestamp, it computes the deduplication, then it shows the updates to the viewer.
 * At the end it sends the deduplicated message. 
*/
void * Deduplicator::deduplicate(void *n) {
    std::vector<MasaMessage> input_messages;
    std::vector<MasaMessage> tmp;
    MasaMessage deduplicate_message;
    std::vector<cv::Point2f> map_pixels;
    fog::Profiler prof("Deduplicator");
    while(gRun){
        /* Delay is necessary. If the Deduplicator takes messages too quickly there is a risk 
        of not tracking the road users correctly. each message is read as a frame. 
        See the initialAge variable. */
        prof.tick("total time");
        std::this_thread::sleep_for(std::chrono::milliseconds(30));        
        prof.tick("get messages");
        tmp = this->inCm->getMessages();
        for(int i = 0; i < tmp.size(); i++)
            input_messages.push_back(tmp.at(i));
        prof.tock("get messages");
        // std::cout<<"dedup dim reading list: "<<input_messages.size()<<std::endl;
        if(input_messages.size() == 0) {
            prof.tick("total time");
            continue;        // no received messages
        }
        else if(input_messages.size() >= 2){
            // std::cout<<"get messages\n";
            prof.tick("filter old");
            // filter old messages from the same id (camera or traffic light)
            input_messages = filterOldMessages(input_messages);
            prof.tock("filter old");
            prof.tick("deduplication");
            // takes the input messages and return the deduplicate message
            computeDeduplication(input_messages, deduplicate_message);
            prof.tock("deduplication");
            prof.tick("show update");
            showUpdates();
            prof.tock("show update");
            prof.tick("insert message");
            this->outCm->insertMessage(deduplicate_message);
            input_messages.clear();
            tmp.clear();
            prof.tock("insert message");
            prof.tock("total time");
            prof.printStats();
        }
    }
    return (void *)NULL;
}
}
