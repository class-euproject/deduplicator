#include "Deduplicator.h"
#include "geohash.h"

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
float Deduplicator::distance(const RoadUser object1, const RoadUser object2) {
    double north1, north2, east1, east2, up;

    this->gc.geodetic2Enu(object1.latitude, object1.longitude, 0, &north1, &east1, &up);
    this->gc.geodetic2Enu(object2.latitude, object2.longitude, 0, &north2, &east2, &up);
    return sqrt(pow(north1 - north2, 2) + pow(east1 - east2, 2));
}

/**
 * Compute the nearest object under a certain threshold 
 * to the reference, with the same category in the given MasaMessage, if any.
 * 
*/
bool Deduplicator::nearest_of(const MasaMessage message, const DDstruct ref, const float threshold, DDstruct& ris){
    DDstruct nearest;
    bool found_something = false;
    for(size_t i = 0; i < message.objects.size(); i++){

        if(ref.rs.category == message.objects.at(i).category){
            float distance = this->distance(message.objects.at(i), ref.rs);
            if(distance < threshold){
                if(found_something == false or nearest.distance > distance){
                    nearest.rs = message.objects.at(i);
                    nearest.distance = distance;
                    nearest.object_index = i;
                    ris = nearest;
                    found_something = true;
                } 
            }
        }
    }
    return found_something;
}

/**
 * Compute deduplication from input messages
 * 
*/
void Deduplicator::deduplicationFromMessages(std::vector<MasaMessage> &input_messages){

    const float CAR_THRESHOLD = 1.5;  //multiple deduplication threshold for different road users
    const float PERSON_THRESHOLD = 0.5;

    for(size_t i = 0; i < input_messages.size(); i++){

        for(size_t j = 0; j < input_messages.at(i).objects.size(); j++){
            /*since this loop look for nearest objects in all other messages, we can assume that if an object have multiple ids in cam_id 
            (or object_id, it's the same) it's a duplicated from another object already processed. Therefore we can just skip it.*/
            if(input_messages.at(i).objects.at(j).camera_id.size() == 1){
                //set the appropriate threshold geven the category of the object
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

                //find the nearest object in each message
                std::vector<DDstruct> nearest;
                DDstruct ref;
                ref.rs = input_messages.at(i).objects.at(j);
                ref.message_index = i;
                ref.object_index = j;
                ref.distance = 0.0;
                nearest.push_back(ref);
                size_t k = 0;
                for(; k < i; k++){ 
                    DDstruct nearest_obj;
                    if (this->nearest_of(input_messages.at(k), ref, threshold, nearest_obj) == true){
                        nearest_obj.message_index = k;
                        nearest.push_back(nearest_obj);
                    }
                }

                k = i+1;
                for(; k < input_messages.size(); k++){
                    DDstruct nearest_obj;
                    if (this->nearest_of(input_messages.at(k), ref, threshold, nearest_obj) == true){
                        nearest_obj.message_index = k;
                        nearest.push_back(nearest_obj);
                    }
                }

                //if other objects near the reference are found 
                if(nearest.size() != 1){
                    //update the objects info about cam_id and object_id in input_messages.
                    for(size_t x = 0; x < nearest.size(); x++){

                        int cam_id = input_messages.at(nearest.at(x).message_index).objects.at(nearest.at(x).object_index).camera_id.at(0);
                        int object_id = input_messages.at(nearest.at(x).message_index).objects.at(nearest.at(x).object_index).object_id.at(0);
 
                        for(size_t y = 0; y < x; y++){
                            input_messages.at(nearest.at(y).message_index).objects.at(nearest.at(y).object_index).camera_id.push_back(cam_id);
                            input_messages.at(nearest.at(y).message_index).objects.at(nearest.at(y).object_index).object_id.push_back(object_id);
                        }

                        for(size_t y = x+1; y < nearest.size(); y++){
                            input_messages.at(nearest.at(y).message_index).objects.at(nearest.at(y).object_index).camera_id.push_back(cam_id);
                            input_messages.at(nearest.at(y).message_index).objects.at(nearest.at(y).object_index).object_id.push_back(object_id);
                        }
                    }

                    //Uncomment this to print some results:
                    /*std::cout << "Object at " << j << " from camera " << input_messages.at(i).cam_idx << " is also in ";
                    for(size_t k = 0; k < input_messages.at(i).objects.at(j).camera_id.size(); k++){

                        std::cout<< input_messages.at(i).objects.at(j).camera_id.at(k)<< " ";
                    }
                    std::cout<<std::endl;*/
                }
            }
        }
    }
}

void geohashDeduplication(std::vector<MasaMessage> input_messages){

    //resolution of geohash is the number of bit of the integer geohash. 52 bits corresponds to a precision of 0.5971 m
    //while 50 bits to 1.1943m. The "resolution" is half of those values because it is the number of steps the convertion function
    //takes to do the conversion. Each step defines 2 bit. Ref at https://github.com/yinqiwen/ardb/wiki/Spatial-Index
    const int CAR_RESOLUTION = 25;  
    const int PERSON_RESOLUTION = 26;

    //Maybe changing this values we can improve performances/precision if needed
    GeoHashRange lat_range, lon_range;
    lat_range.max = 20037726.37;
    lat_range.min = -20037726.37;
    lon_range.max = 20037726.37;
    lon_range.min = -20037726.37;

    std::map<uint64_t, std::vector<RoadUser>> car_map;
    std::vector<uint64_t> car_keys;
    std::map<uint64_t, std::vector<RoadUser>> person_map;
    std::vector<uint64_t> person_keys;

    for(size_t i = 0; i < input_messages.size(); i++){

        for(size_t j = 0; j < input_messages.at(i).objects.size(); j++){

            RoadUser object = input_messages.at(i).objects.at(j);
            GeoHashBits hash;
            std::vector<RoadUser> to_update;
            switch (object.category)
            {
            case Categories::C_person:
                if( geohash_fast_encode(lat_range, lon_range, object.latitude, object.longitude, PERSON_RESOLUTION, &hash) == 0){

                    person_map[hash.bits].push_back(object);
                    to_update =  person_map[hash.bits];
                    //if it works we can optimize this operation using sets and overriding the required operators
                    if (std::find(person_keys.begin(), person_keys.end(), hash.bits) == person_keys.end()) 
                        person_keys.push_back(hash.bits);

                } else {
                    std::cerr<< "Geohash conversion of person failed"<< std::endl;
                }
                break;

            case Categories::C_car:
                if( geohash_fast_encode(lat_range, lon_range, object.latitude, object.longitude, CAR_RESOLUTION, &hash) == 0){

                    car_map[hash.bits].push_back(object);
                    to_update =  car_map[hash.bits];
                    if (std::find(car_keys.begin(), car_keys.end(), hash.bits) == car_keys.end()) 
                        car_keys.push_back(hash.bits);
                } else {
                    std::cerr<< "Geohash conversion of person failed"<< std::endl;
                }
                break;

            default:
                if( geohash_fast_encode(lat_range, lon_range, object.latitude, object.longitude, CAR_RESOLUTION, &hash) == 0){

                    car_map[hash.bits].push_back(object);
                    to_update =  car_map[hash.bits];
                    if (std::find(car_keys.begin(), car_keys.end(), hash.bits) == car_keys.end()) 
                        car_keys.push_back(hash.bits);
                } else {
                    std::cerr<< "Geohash conversion of person failed"<< std::endl;
                }
                break;
            }
        
            //if multiple objects have the same hash update the corresponding objects
            if(to_update.size() > 1){
                for(size_t x = 0; x < to_update.size(); x++){

                    //controlla se modificare i dati dentro la hash map li modifica anche in input messages-> NO
                    for(size_t y = 0; y < to_update.size()-1; y++){
                        //std::cout << "Geohash ha trovato dei duplicati" << std::endl;

                        input_messages.at(i).objects.at(j).camera_id.push_back(to_update.at(y).camera_id[0]);
                        input_messages.at(i).objects.at(j).object_id.push_back(to_update.at(y).object_id[0]);

                        to_update.at(y).camera_id.push_back(input_messages.at(i).objects.at(j).camera_id[0]);
                        to_update.at(y).object_id.push_back(input_messages.at(i).objects.at(j).object_id[0]);
                    }
                }
            }
        }
    }

    //now check the neighborhood of the objects
    /*std::map<uint64_t, std::vector<RoadUser>> neighbors;
    for(auto key: car_keys){

        GeoHashNeighbors* neighbors;
        if( geohash_get_neighbors(key, neighbors) == 0 ){
            
            neighbors->north.bits
            neighbors->east.bits
            neighbors->west.bits
            neighbors->south.bits
            neighbors->north_east.bits
            neighbors->south_east.bits
            neighbors->north_west.bits
            neighbors->south_west.bits

        } else {
            std::cerr<< "Could not compute neighbors" << std::endl;
        }
        

    }*/




 /*              //if other objects near the reference are found 
            if(nearest.size() != 1){

                //update the objects info about cam_id and object_id in input_messages.
                for(size_t x = 0; x < nearest.size(); x++){

                    int cam_id = input_messages.at(nearest.at(x).message_index).objects.at(nearest.at(x).object_index).camera_id.at(0);
                    int object_id = input_messages.at(nearest.at(x).message_index).objects.at(nearest.at(x).object_index).object_id.at(0);
                    for(size_t y = 0; y < x; y++){
                        input_messages.at(nearest.at(y).message_index).objects.at(nearest.at(y).object_index).camera_id.push_back(cam_id);
                        input_messages.at(nearest.at(y).message_index).objects.at(nearest.at(yobject_index).object_id.push_back(object_id);
                    }

                    for(size_t y = x+1; y < nearest.size(); y++){
                        input_messages.at(nearest.at(y).message_index).objects.at(nearest.at(y).object_index).camera_id.push_back(cam_id);
                        input_messages.at(nearest.at(y).message_index).objects.at(nearest.at(y).object_index).object_id.push_back(object_id);
                    }
                }
            }*/

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
    //deepcopy of every MasaMessage
    std::vector<MasaMessage> copy_input_messages = input_messages;

    std::cout << "Controllo dei messaggi e della loro copia: " << std::endl;

    for(size_t i = 0; i < input_messages.size(); i++){
        std::cout << input_messages.at(i).cam_idx << " :" << std::endl;
        for(size_t j = 0; j < input_messages.at(i).objects.size(); j++){
            std::cout << input_messages.at(i).objects.at(j).latitude << " " << input_messages.at(i).objects.at(j).longitude << std::endl;
        }
    }

    for(size_t i = 0; i < copy_input_messages.size(); i++){
        std::cout << copy_input_messages.at(i).cam_idx << " :" << std::endl;
        for(size_t j = 0; j < copy_input_messages.at(i).objects.size(); j++){
            std::cout << copy_input_messages.at(i).objects.at(j).latitude << " " << copy_input_messages.at(i).objects.at(j).longitude << std::endl;
        }
    }

    //Old deduplication method
    deduplicationFromMessages(input_messages);

    //New deduplication method with geohash
    geohashDeduplication(copy_input_messages);
    int deduplicated_objects = 0;
    for(size_t i = 0; i < input_messages.size(); i++){
        for(size_t j = 0; j < input_messages.at(i).objects.size(); j++){
            if(input_messages.at(i).objects.at(j).camera_id.size() > 1){
                deduplicated_objects++;
            }
        }
    }
    std::cout << "Oggetti deduplicati da deduplicationFromMessages: " << deduplicated_objects << std::endl;

    deduplicated_objects = 0;
    for(size_t i = 0; i < copy_input_messages.size(); i++){
        for(size_t j = 0; j < copy_input_messages.at(i).objects.size(); j++){
            if(copy_input_messages.at(i).objects.at(j).camera_id.size() > 1){
                deduplicated_objects++;
            }
        }
    }
    std::cout << "Oggetti deduplicati da geohashDeduplication: " << deduplicated_objects << std::endl << std::endl;

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
            if(input_messages.size() >= 2){
                prof.tick("deduplication");
                // takes the input messages and return the deduplicate message
                computeDeduplication(input_messages, deduplicate_message);
                prof.tock("deduplication");
            }
        }

        if( !input_messages.empty())
            
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
    return (void *)NULL;
}
}
