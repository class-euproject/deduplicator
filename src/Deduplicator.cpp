#include "Deduplicator.h"
#include "nabo.h"


namespace fog {

void create_message_from_tracker(const std::vector<tracking::Tracker> &trackers, MasaMessage *m, 
                                 geodetic_converter::GeodeticConverter &gc, double *adfGeoTransform,
                                 const std::vector<uint16_t> camera_id) {
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
            r.camera_id = camera_id;
            r.latitude = static_cast<float>(lat);
            r.longitude = static_cast<float>(lon);
            std::vector<uint16_t> obj_id_vector;
            obj_id_vector.push_back(t.id);
            r.object_id = obj_id_vector;
            r.error = t.traj.back().error;
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
    t = new tracking::Tracking(nStates, dt, initialAge, tracking::UKF_t);
    edge_tr = new tracking::Tracking(nStates, dt, initialAge, tracking::UKF_t);
    viewer = &v;
    show = visual;
}

Deduplicator::~Deduplicator() {
    free(adfGeoTransform);
    delete t;
    delete edge_tr;
}

void Deduplicator::start() {
    if (pthread_create(&deduplicatorThread, NULL, (THREADFUNCPTR) &Deduplicator::deduplicate, this)) 
        perror("could not create thread");
}

void Deduplicator::end() {
    pthread_join(deduplicatorThread, NULL);
}

/**
 * Prints a RoadUser
 */
void printRoadUser(const RoadUser ru){

    std::cout << std::setprecision(10) << ru.category << " " << ru.latitude << " " << ru.longitude << " " << (int) ru.speed  
                                       << " " << ru.orientation << " " << ru.error << " " << ru.camera_id.size();

    for( auto elem : ru.camera_id){
        std::cout << " " << elem;
    }

    for( auto elem : ru.object_id){
        std::cout << " " << elem;
    }

    std::cout << std::endl;
}

/**
 * Prints an array of MasaMessages
*/
void printMessages(std::vector<MasaMessage> input_messages){
    for(auto message: input_messages){
        std::cout << "Message source: " << message.cam_idx << " with " << message.num_objects << " objects" << std::endl;
        for(auto object: message.objects){
            printRoadUser(object);
        }
    }
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
 * Check if the messages contain the tracker information (camera_id and object_id vectors).
 * If object_id vector is empty, then fill it with that information.
*/
std::vector<MasaMessage> Deduplicator::fillTrackerInfo(std::vector<MasaMessage> input_messages) {
    // std::cout<<"messages: "<<input_messages.size()<<std::endl;
    std::vector<MasaMessage> copy = input_messages;
    std::vector<int> delete_ids;

    MasaMessage tracked_message;
    std::vector<uint16_t> camera_id;
    int count_no_tracking_messages = 0;
    //copy the deduplicated objects into a single MasaMessage. Check if some objects need to be tracked
    std::vector<tracking::obj_m> objects_to_track;
    for(size_t i = 0; i < input_messages.size(); i++) {

        // the first object could be a special car, so skip it and check on the second one.
        // If it is a connected vehicle or it is a smart vehicle with no other road user, its size is equal to one.
        // If it is a Traffic Light, objects vector is empty.
        if(input_messages.at(i).objects.size() <= 1 || 
            input_messages.at(i).objects.size() > 1 && input_messages.at(i).objects.at(1).object_id.size()!=0) 
            continue;
        
        //TODO: only a message with no tracker information is supported. For each of this kind of message a tracker is needed 
        if(count_no_tracking_messages == 1) {
            std::cout<<"WARNING!!! TOO MANY MESSAGES WITHOUT TRACKING INFORMATION\n";
            continue;
        }

        delete_ids.push_back(i);
        //The smart vehicle does not need to be tracked so it can immediately be pushed in output messages
        tracked_message.cam_idx = input_messages.at(i).cam_idx;
        tracked_message.t_stamp_ms = input_messages.at(i).t_stamp_ms;
        tracked_message.objects.push_back(input_messages.at(i).objects.at(0));
        for(size_t j = 1; j < input_messages.at(i).objects.size(); j++) {
            double north, east, up;
            this->gc.geodetic2Enu(input_messages.at(i).objects.at(j).latitude, input_messages.at(i).objects.at(j).longitude, 0, &east, &north, &up);
            objects_to_track.push_back(tracking::obj_m(east, north, 0, input_messages.at(i).objects.at(j).category, 1, 1, 0));
        }
        count_no_tracking_messages ++;
    }

    //if some object need to be tracked, track it
    if(objects_to_track.size() > 0){
        camera_id.push_back(tracked_message.cam_idx);
        this->edge_tr->track(objects_to_track, this->trVerbose);
        create_message_from_tracker(edge_tr->getTrackers(), &tracked_message, this->gc, this->adfGeoTransform, camera_id);
    }
    tracked_message.num_objects = tracked_message.objects.size();

    if (delete_ids.size()==0)
        return input_messages;
    if (delete_ids.size() != 1)
        std::sort(delete_ids.begin(), delete_ids.end(), [](int a, int b) {return a > b; });
    delete_ids.erase( std::unique( delete_ids.begin(), delete_ids.end() ), delete_ids.end() );
    for(auto d : delete_ids)
        copy.erase(copy.begin() + d);
    copy.push_back(tracked_message);
    return copy;
}


/** from float to uint8 the conversion is uint8_t(std::abs(vel * 3.6 * 2)), so this is the opposite*/
double uint8_to_speed(const uint8_t speed){ return static_cast<double>(speed)/7.2;}

/** This should calculate the opposite of uint8_t((int((yaw * 57.29 + 360)) % 360) * 17 / 24). 
 * The output is in radiants
*/
double uint16_to_yaw(const uint16_t yaw){ 
	double conversion = static_cast<double>(yaw) * (24.0/17.0);
	if(conversion > 360)
		conversion -= 360;
	return conversion/57.29;
} 

/**
 * Compute the weighted average of values vector
*/
double weightedAverage(std::vector<double>& weights, std::vector<double>& values){

    if( weights.size() == values.size() and weights.size() > 0){
        double average = 0, w_sum = 0;
        for(size_t i = 0; i < values.size(); i++){
            average += weights.at(i) * values.at(i);
            w_sum += weights.at(i);
        }
        return average/w_sum;

    } else { return 0.0; }
}

/**
 * Compute the average between angles. Method taken by https://en.wikipedia.org/wiki/Mean_of_circular_quantities#Example
 * and https://rosettacode.org/wiki/Averages/Mean_angle#C.2B.2B
*/
double angleAverage(std::vector<double>& orientation_vector){

    double sin, cos, sum_of_sin = 0.0, sum_of_cos = 0.0;
    for(auto elem : orientation_vector){
        sincos(elem, &sin, &cos);
        sum_of_sin += sin;
        sum_of_cos += cos;
    }

    double meanSin = sum_of_sin/orientation_vector.size();
    double meanCos = sum_of_cos/orientation_vector.size();

    if(meanSin > 0 and meanCos > 0){
        return atan(meanSin/meanCos);
    } else if(meanCos < 0){
        return atan(meanSin/meanCos) + M_PI;
    } else if(meanSin < 0 and meanCos > 0){
        return atan(meanSin/meanCos) + 2*M_PI;
    } else { return 0.0; }
}

/**
 * Print a RoadUser
*/
void printObject(RoadUser& ru){
    std::cout << std::setprecision(10) << "\t" << ru.latitude << "\t"  << ru.longitude << "\t" << (int) ru.speed << "\t" << (int) ru.orientation << "\t" << ru.error << std::endl;
}

/**
 * Compute the mean of each value for each duplicated objects. 
 * - for latitude and longitude a weighted average is computed
 * - for speed and yaw a conversion to double is needed, then the weighted average is computed
 * - if none of the objects have a defined precision, a standard mean is computed
*/
RoadUser createAggregatedObject(std::vector<std::pair<uint16_t, uint16_t>>              &map_keys_of_current_object,
                                std::pair<uint16_t, uint16_t>                           &key_to_keep,
                                std::map<std::pair<uint16_t, uint16_t>, RoadUser>       &current_message_map,
                                const bool                                              is_connected                ){

    std::vector<double> latitude_vector, longitude_vector, error_vector, speed_vector, orientation_vector;
    std::vector<double> weights;
    double avg_latitude, avg_longitude, avg_speed, avg_orientation;
    std::vector<RoadUser> objects;
    RoadUser aggregatedObject;

    if(!is_connected){
        //retrieval of all data for average computation (skipping the 0-error ones)
        for(size_t i = 0; i < map_keys_of_current_object.size(); i++){
            RoadUser object = current_message_map[map_keys_of_current_object.at(i)];
            objects.push_back(object);

            //Take into account only error > 0 because of the function to calculate the error weight (1/error)
            if (object.error > 0){

                weights.push_back(1.0/object.error);
            }
            
            latitude_vector.push_back(object.latitude);
            longitude_vector.push_back(object.longitude);
            speed_vector.push_back(uint8_to_speed(object.speed));
            orientation_vector.push_back(uint16_to_yaw(object.orientation));
        }

        //if 2 or more data are available compute the average of lat/lon/speed
        if(weights.size() > 1){
            avg_latitude = weightedAverage(weights, latitude_vector);
            avg_longitude = weightedAverage(weights, longitude_vector);
            avg_speed = weightedAverage(weights, speed_vector);
        //otherwise the average of each value is the one available
        } else if (latitude_vector.size() == 1){
            avg_latitude = latitude_vector.at(0);
            avg_longitude = longitude_vector.at(0);
            avg_speed = speed_vector.at(0);
            // compute non-weighted average
        } else {
            for(auto elem : objects){
                latitude_vector.push_back(elem.latitude);
                longitude_vector.push_back(elem.longitude);
            }
                            //check if accumulate returns an int
            avg_latitude  = std::accumulate( latitude_vector.begin(), latitude_vector.end(),  0.0) / latitude_vector.size();
            avg_longitude = std::accumulate(longitude_vector.begin(), longitude_vector.end(), 0.0) / longitude_vector.size();
            avg_speed     = std::accumulate(speed_vector.begin(), speed_vector.end(), 0.0) / speed_vector.size();
        }

        avg_orientation = angleAverage(orientation_vector);

        aggregatedObject.latitude = avg_latitude;
        aggregatedObject.longitude = avg_longitude;
        aggregatedObject.speed = speed_to_uint8(avg_speed);
        aggregatedObject.orientation = orientation_to_uint8(avg_orientation);
    } else {
        aggregatedObject = current_message_map[key_to_keep];
    }

    aggregatedObject.camera_id = {key_to_keep.first};
    aggregatedObject.object_id = {key_to_keep.second};
    for(auto key : map_keys_of_current_object){
        if(key != key_to_keep){
            aggregatedObject.camera_id.push_back(key.first);
            aggregatedObject.object_id.push_back(key.second);
        }
    }

    return aggregatedObject;
}

/**
 * Check if the two road users have appropriate categories in order to be deduplicated
 */
bool check_category(const RoadUser reference, const RoadUser candidate){
    return  reference.category == candidate.category || 
            reference.category == Categories::C_car and candidate.category >= Categories::C_marelli1 ||
            reference.category >= Categories::C_marelli1 and candidate.category == Categories::C_car  ;
}

/**
 * Kdtree algorithm for computing deduplication from input messages
*/
void Deduplicator::deduplicationFromMessages(std::vector<MasaMessage> &input_messages){
    
    double north, east, up;
    Eigen::MatrixXf M;
    Nabo::NNSearchF* nns;

    int nObjects = 0;
    for(size_t i = 0; i < input_messages.size(); i++)
        nObjects+= input_messages.at(i).objects.size();

    const int K = 2*input_messages.size()-1 > nObjects ? nObjects : 2*input_messages.size()-1;
    Eigen::MatrixXi indices;
    Eigen::MatrixXf dists2;

    std::vector<int> toMerge;
    std::vector<std::pair<size_t, size_t>> objectIndexes;

    M.resize(2, nObjects);
    indices.resize(K, nObjects);
    dists2.resize(K, nObjects);

    int offset = 0;
    for(size_t i = 0; i < input_messages.size(); i++){
        for(size_t j = 0; j < input_messages[i].objects.size(); j++, offset++){

            this->gc.geodetic2Enu(input_messages.at(i).objects.at(j).latitude, input_messages.at(i).objects.at(j).longitude, 0, &north, &east, &up);
            M(0, offset) = north;
            M(1, offset) = east;
            objectIndexes.push_back(std::pair<size_t, size_t>(i, j));
        }
    }

    if(nObjects > 30){
        nns = Nabo::NNSearchF::createKDTreeTreeHeap(M, 2);
    } else {
        nns = Nabo::NNSearchF::createKDTreeLinearHeap(M, 2);
    }

    //Perform the queries all in once: be careful, maximum radius must be the maximum between thresholds of objects (defined in Deduplicator.h)
    nns->knn(M, indices, dists2, K, 0.0, Nabo::NNSearchF::SORT_RESULTS|Nabo::NNSearchF::ALLOW_SELF_MATCH, 4.0);
    delete nns;

    for(size_t i = 0, offset = 0; i < input_messages.size(); i++){

        for(size_t j = 0; j < input_messages.at(i).objects.size(); j++, offset++){

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
            case Categories::C_bus:
                threshold = AUTOBUS_THRESHOLD;
                break;
            default:
                threshold = CAR_THRESHOLD;
                break;
            }

            toMerge.clear();
            std::set<int> messages_cam_idx;

            //analyze results to find the actual road users to merge
            for(int k = 0; k < indices.col(offset).size() and indices.col(offset)[k] != -1 and dists2.col(offset)[k] <= threshold*threshold; k++){   
                size_t message_index = objectIndexes[indices.col(offset)[k]].first;
                size_t object_index = objectIndexes[indices.col(offset)[k]].second;

                if( check_category(input_messages[message_index].objects[object_index], input_messages[i].objects[j]) ){

                    std::set<int> ids;
                    for(auto idx : input_messages[message_index].objects[object_index].camera_id)
                        ids.insert((int)idx);

                    std::set<int> intersection;
                    std::set_intersection(messages_cam_idx.begin(), messages_cam_idx.end(), ids.begin(), ids.end(),
                                                                std::inserter(intersection, intersection.begin()));

                    if(intersection.size() == 0 ){

                        toMerge.push_back(indices.col(offset)[k]);
                        std::set_union(messages_cam_idx.begin(), messages_cam_idx.end(), ids.begin(), ids.end(), 
                                                std::inserter(messages_cam_idx, messages_cam_idx.begin()));
                    }
                }
            }

            //if other objects near the reference are found 
            if(toMerge.size() > 1){
                //update cam_id and object_id in input_messages
                for(size_t x = 0; x < toMerge.size(); x++){

                    for(size_t y = 0; y < input_messages.at(objectIndexes[toMerge[x]].first).objects.at(objectIndexes[toMerge[x]].second).camera_id.size(); y++){

                        uint16_t cam_id_to_push = input_messages.at(objectIndexes[toMerge[x]].first).objects.at(objectIndexes[toMerge[x]].second).camera_id.at(y);
                        uint16_t object_id_to_push = input_messages.at(objectIndexes[toMerge[x]].first).objects.at(objectIndexes[toMerge[x]].second).object_id.at(y);

                        for(size_t z = 0; z < toMerge.size(); z++){
                            if(z != x and std::find(input_messages.at(objectIndexes[toMerge[z]].first).objects.at(objectIndexes[toMerge[z]].second).camera_id.begin(),
                                            input_messages.at(objectIndexes[toMerge[z]].first).objects.at(objectIndexes[toMerge[z]].second).camera_id.end(), cam_id_to_push) == 
                                                            input_messages.at(objectIndexes[toMerge[z]].first).objects.at(objectIndexes[toMerge[z]].second).camera_id.end()){
                                input_messages.at(objectIndexes[toMerge[z]].first).objects.at(objectIndexes[toMerge[z]].second).camera_id.push_back(cam_id_to_push);
                                input_messages.at(objectIndexes[toMerge[z]].first).objects.at(objectIndexes[toMerge[z]].second).object_id.push_back(object_id_to_push);
                            }
                        }
                    }
                }
            }
        }
    }
}

std::map<std::pair<uint16_t, uint16_t>, RoadUser> createMapMessage(std::vector<MasaMessage> &input_messages){
    
    std::map<std::pair<uint16_t, uint16_t>, RoadUser> current_messages_map;
    for(size_t i = 0; i < input_messages.size(); i++){
        for(size_t j = 0; j < input_messages.at(i).objects.size(); j++){

            //update the current message map for fast retrieval of data in other locations (eventually you need to update each pair contained)
            std::pair<uint16_t, uint16_t> object_key = std::pair<uint16_t, uint16_t>(input_messages.at(i).objects.at(j).camera_id[0], input_messages.at(i).objects.at(j).object_id[0]);
            if(current_messages_map.find(object_key) == current_messages_map.end()){
                current_messages_map[object_key] = input_messages.at(i).objects.at(j);
            }
        }
    }
    return current_messages_map;
}

/**
 * Given a RoadUser, create the array of pair needed to access the map to find its duplicated
*/
std::vector<std::pair<uint16_t, uint16_t>> getMapKeysFromObject(const RoadUser obj){
    std::vector<std::pair<uint16_t, uint16_t>> keys;
    std::vector<uint16_t> cam_id_vector = obj.camera_id;
    std::vector<uint16_t> obj_id_vector = obj.object_id;
    for(size_t i = 0; i < cam_id_vector.size(); i++){
        keys.push_back(std::pair<uint16_t, uint16_t>(cam_id_vector[i], obj_id_vector[i]));
    }
    return keys;
}

/**
 * Remove Duplicated Objects
 * remove the duplicated objects according to the last duplicated objects map that tries to mantain the history of
 * the local trackers in the edge. The idea is to mantain the oldest object in the scene.
 * 
 * Algorithm: 
 * 1 when (at least) 2 objects are found to be the same, look in the table if some of the pair (tracking id, object id) are already used.
 * 2 (true) If one of them is already used, keep the first pair in the found object that is still present in the current message and discard the others. 
 * Update the table with the new reference.
 * 2 (false) If none of the current pairs is found, look for the oldest message in the current possibilities and keep it. If there are many "oldest messages"
 * keep the most precise detection. Update the table consequently.
 * 3 All the other objects that are not duplicates, must be recorded in the table for the evetntal filtering of the next set of input messages.
 * 
*/
void removeDuplicatedObjects(std::vector<MasaMessage> &input_messages, std::map<std::pair<uint16_t, uint16_t>, RoadUser>& last_duplicated_objects,
                                                                       std::map<std::pair<uint16_t, uint16_t>, RoadUser>& current_message_map   ){

    std::map<std::pair<uint16_t, uint16_t>, RoadUser> current_table;
    std::vector<MasaMessage> deduplicated_messages = input_messages;

    for(size_t i = 0; i < input_messages.size(); i++) {
        deduplicated_messages[i].objects.clear();
    }
    //Here i and j MUST BE INT, otherwise a size_t (aka unsigned long) would never be negative and so the loop would produce out of range indexes
    for(int i = input_messages.size()-1; i >= 0; --i) {
        for(int j = input_messages.at(i).objects.size()-1; j >= 0; --j){
            std::pair<uint16_t, uint16_t> current_pair = std::pair<uint16_t, uint16_t>(input_messages[i].objects[j].camera_id[0], input_messages[i].objects[j].object_id[0]);

            //if the current object has no entry in the current table, it means that the object is missing from deduplicated_messages  
            if(current_table.find(current_pair) == current_table.end()){

                //if no duplicates of the object were found, just update the map and save the object in deduplicated_messages
                if(input_messages[i].objects[j].camera_id.size() <= 1){
                    current_table[current_pair] = input_messages[i].objects[j];
                    deduplicated_messages[i].objects.push_back(input_messages[i].objects[j]);
                //otherwise you need to choose which object to keep between all the duplicates that were detected
                } else {

                    std::pair<uint16_t, uint16_t> key_to_keep;

                    //so get all the duplicated objects of the current one
                    std::vector<std::pair<uint16_t, uint16_t>> map_keys_of_current_object  = getMapKeysFromObject(input_messages[i].objects[j]);

                    //retrive all the keys for the considered cluster of objects
                    for(size_t x = 0; x < map_keys_of_current_object.size(); x++){
                        std::vector<std::pair<uint16_t, uint16_t>> tmp = getMapKeysFromObject(current_message_map[map_keys_of_current_object[x]]);
                        for(size_t y = 0; y < tmp.size(); y++){
                            auto key = tmp.at(y);
                            if (find(map_keys_of_current_object.begin(), map_keys_of_current_object.end(), key) == map_keys_of_current_object.end()){
                                map_keys_of_current_object.push_back(key);
                            }
                        }
                    }

                    //check if there is a connected vehicle otherwise find a key to keep looking at the old messages
                    bool is_connected = false;
                    bool key_found = false;
                    for(size_t x = 0; x < map_keys_of_current_object.size() && is_connected == false; x++){
                        if(current_message_map[map_keys_of_current_object[x]].category >= C_marelli1){
                            is_connected = true;
                            key_found = true;
                            key_to_keep = map_keys_of_current_object[x];
                        }
                    }

                    if(!is_connected){
                        for(size_t x = 0; x < map_keys_of_current_object.size() and key_found == false; x++){
                            auto map_iterator = last_duplicated_objects.find(map_keys_of_current_object[x]);
                            //Now we hav found an object from previous message with one of the keys of the current message
                            if (last_duplicated_objects.find(map_keys_of_current_object[x]) != last_duplicated_objects.end()){

                                std::vector<std::pair<uint16_t, uint16_t>> map_keys_of_old_object = getMapKeysFromObject(map_iterator->second);

                                //Now we select which pair is the one to keep
                                for(size_t y = 0; y < map_keys_of_old_object.size() and key_found == false; y++){
                                    if(std::find(map_keys_of_current_object.begin(), map_keys_of_current_object.end(), map_keys_of_old_object[y]) != map_keys_of_current_object.end()){
                                        key_found = true;
                                        key_to_keep = map_keys_of_old_object[y];
                                    }
                                }
                            }
                        }
                    }

                    RoadUser object_to_keep;
                    //so if a key has been found we can aggregate the objects and register the new entry on the map
                    if(!key_found){
                        key_to_keep = map_keys_of_current_object[0];
                    }

                    object_to_keep = createAggregatedObject(map_keys_of_current_object, key_to_keep, current_message_map, is_connected);

                    //update the current map
                    for(size_t x = 0; x < map_keys_of_current_object.size(); x++){
                        current_table[map_keys_of_current_object[x]] = object_to_keep;
                    }

                    //update the messages
                    deduplicated_messages[i].objects.push_back(object_to_keep);
                }
            }
        }
    }
    //now the old map becomes the current one and swap the messsage to update the current input message
    input_messages = deduplicated_messages;
    last_duplicated_objects = current_table;
}

/**
 * Count Deduplicated Objects
 * Mostly a debug function. Count how many objects have multiple camera/object id in the given messages. 
*/
void countDeduplicatedObjects(std::vector<MasaMessage> &input_messages){

    int counter = 0;
    for(size_t i = 0; i < input_messages.size(); i++) {
        for(size_t j = 0; j < input_messages.at(i).objects.size(); j++) {
            if(input_messages.at(i).objects.at(j).camera_id.size() > 1 ) 
                counter++; 
        }
    }
    std::cout << "Duplicated objects: " << counter << " with " << input_messages.size() << " input messages" << std::endl;
}

void print_n_objects(std::vector<MasaMessage> &input_messages){

    int counter = 0;
    for(size_t i = 0; i < input_messages.size(); i++) {
        counter += input_messages.at(i).objects.size(); 
    }

    std::cout << "There are " << counter << " objects in the messages" << std::endl;
}

/**
 * Elaborate Message:
 * - compute deduplication of objects. 
 * - compute the (weighted) mean of each duplicated object property
 * - for smart vehicles, track their objects
 * - output: a single MasaMessage with the aggregation of all above procedures
*/
void Deduplicator::elaborateMessages(std::vector<MasaMessage> &input_messages, MasaMessage &output_message, 
                                     std::map<std::pair<uint16_t, uint16_t>, RoadUser>& last_duplicated_objects) {
 
    output_message.lights.clear(); 
    output_message.objects.clear();
    output_message.t_stamp_ms = time_in_ms();

    //Kdtree deduplication method
    if(input_messages.size() > 1){ 
        deduplicationFromMessages(input_messages);
        std::map<std::pair<uint16_t, uint16_t>, RoadUser> current_message_map = createMapMessage(input_messages);
        removeDuplicatedObjects(input_messages, last_duplicated_objects, current_message_map);
    }

    //copy the deduplicated objects into a single MasaMessage.
    std::vector<tracking::obj_m> objects_to_track;
    for(size_t i = 0; i < input_messages.size(); i++) {
        for(size_t j = 0; j < input_messages.at(i).objects.size(); j++){

            output_message.objects.push_back(input_messages.at(i).objects.at(j));
        }
        for(size_t j = 0; j < input_messages.at(i).lights.size(); j++)
            output_message.lights.push_back(input_messages.at(i).lights.at(j));
    }

     output_message.num_objects = output_message.objects.size();
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
    std::map<std::pair<uint16_t, uint16_t>, RoadUser> last_duplicated_objects;
    std::vector<MasaMessage> tmp;
    MasaMessage deduplicate_message;
    std::vector<cv::Point2f> map_pixels;
    fog::Profiler prof("Deduplicator");

   while(gRun){
        /* Delay is necessary. If the Deduplicator takes messages too quickly there is a risk 
        of not tracking the road users correctly. each message is read as a frame. 
        See the initialAge variable. */
        prof.tick("total time");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));        
        prof.tick("get messages");
        input_messages = this->inCm->getMessages();
        prof.tock("get messages");
        // std::cout<<"dedup dim reading list: "<<input_messages.size()<<std::endl;
        if(input_messages.size() == 0) {
            prof.tick("total time");
            continue;        // no received messages
        }
        // std::cout<<"get messages\n";
        prof.tick("filter old");
        // filter old messages from the same id (camera or traffic light)
        input_messages = filterOldMessages(input_messages);
        prof.tock("filter old");
        
        prof.tick("tracker");
        input_messages = fillTrackerInfo(input_messages);
        prof.tock("tracker");
        
        prof.tick("elaboration");
        // takes the input messages and return the deduplicate message
        elaborateMessages(input_messages, deduplicate_message, last_duplicated_objects);
        prof.tock("elaboration");
        
        prof.tick("show update");
        showUpdates();
        prof.tock("show update");
        prof.tick("insert message");
        this->outCm->insertMessage(deduplicate_message);
        input_messages.clear();
        prof.tock("insert message");
        prof.tock("total time");
        prof.printStats();
    }

    return (void *)NULL;
}
} //namespace fog

