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

/** from float to uint8 the conversion is uint8_t(std::abs(vel * 3.6 * 2)), so this is the opposite*/
double uint8_to_speed(const uint8_t speed){ return static_cast<double>(speed)/7.2;}

/** This should calculate the opposite of orientation = uint8_t((int((yaw * 57.29 + 360)) % 360) * 17 / 24). 
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
 * Compute the average between angles. Method taken by https://www.themathdoctors.org/averaging-angles/
 * BE CAREFUL TO THE RANGE OF INPUT ANGLES! THIS SHOULD BE BETWEEN +PI/2 AND -PI/2
 * 
*/
double angleAverage(std::vector<double>& orientation_vector){

    double sin, cos, sum_of_sin = 0.0, sum_of_cos = 0.0, average = 0.0;
    for(auto elem : orientation_vector){
        sincos(elem, &sin, &cos);
        sum_of_sin += sin;
        sum_of_cos += cos;
    }
    return atan2(sum_of_cos, sum_of_sin);
}

/**
 * Compute the mean of each value for each duplicated objects. 
*/
void computeMeanOfDuplicatedObjects(std::vector<fog::DDstruct> &nearest, std::vector<MasaMessage> &input_messages){

    std::vector<double> latitude_vector, longitude_vector, error_vector, speed_vector, orientation_vector;
    double avg_latitude, avg_longitude, avg_speed, avg_orientation;

    //retrieval of all data for average computation (skipping the 0-error ones)
    for(size_t i = 0; i < nearest.size(); i++){
        size_t message_index = nearest.at(i).message_index;
        size_t object_index = nearest.at(i).object_index;
        RoadUser object = input_messages.at(message_index).objects.at(object_index);

        //Take into account only error > 0 because if the error is exactly 0 the object is too far away from the camera
        if (object.precision > 0){

            latitude_vector.push_back(object.latitude);
            longitude_vector.push_back(object.longitude);
            error_vector.push_back(object.precision);
            speed_vector.push_back(uint8_to_speed(object.speed));
            orientation_vector.push_back(uint16_to_yaw(object.orientation));
        }
    }

    //if 2 or more data are available compute the average
    if(latitude_vector.size() > 1){
        std::vector<double> weights;
        //hyperbolic weights. Other conversion can be considered (linear, quadratic...)
        for(auto elem: error_vector){
            weights.push_back(1.0/elem);
        }

        avg_latitude = weightedAverage(weights, latitude_vector);
        avg_longitude = weightedAverage(weights, longitude_vector);
        avg_speed = weightedAverage(weights, speed_vector);
        avg_orientation = angleAverage(orientation_vector);

    //otherwise the average of each value is the one available
    } else {

        avg_latitude = latitude_vector.at(0);
        avg_longitude = longitude_vector.at(0);
        avg_speed = speed_vector.at(0);
        avg_orientation = orientation_vector.at(0);
    }

    //overwrite the computed values into the input messages
    for(size_t i = 0; i < nearest.size(); i++){
        size_t message_index = nearest.at(i).message_index;
        size_t object_index = nearest.at(i).object_index;
        input_messages.at(message_index).objects.at(object_index).latitude = avg_latitude;
        input_messages.at(message_index).objects.at(object_index).longitude = avg_longitude;
        input_messages.at(message_index).objects.at(object_index).speed = speed_to_uint8(avg_speed);
        input_messages.at(message_index).objects.at(object_index).orientation = orientation_to_uint8(avg_orientation);
    }

    return;
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
 * Standard algorithm for computing deduplication from input messages
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
                    //update objects info with the average between all the duplicated objects.
                    computeMeanOfDuplicatedObjects(nearest, input_messages);

                    //update cam_id and object_id in input_messages
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
                }
            }
        }
    }
}

/**
 * Elaborate Message:
 * - compute deduplication of objects. 
 * - compute the (weighted) mean of each duplicated object property
 * - for smart vehicles, track their objects
 * - output: a single MasaMessage with the aggregation of all above procedures
*/
void Deduplicator::elaborateMessage(std::vector<MasaMessage> input_messages, MasaMessage &output_message) {

    //i veicoli smart non devono essere "deduplicati". gli vanno aggiunti gli id nel caso sia detectata da una telecamera.
    //id quattroporte/levante anche come id 
    output_message.lights.clear(); 
    output_message.objects.clear();
    output_message.t_stamp_ms = time_in_ms();

    //Standard deduplication method
    if(input_messages.size() > 1)
        deduplicationFromMessages(input_messages);

    //copy the deduplicated objects into a single MasaMessage. Check if some objects need to be tracked
    std::vector<tracking::obj_m> objects_to_track;
    for(size_t i = 0; i < input_messages.size(); i++) {
        MasaMessage& m = input_messages.at(i);
        //if the current message is coming from a special vehicle that only does detection, its objects must be tracked
        if( m.objects.at(0).category == C_marelli1 || 
            m.objects.at(0).category == C_marelli2 || 
            m.objects.at(0).category == C_quattroporte ||
            m.objects.at(0).category == C_levante || 
            m.objects.at(0).category == C_rover) {
            for(size_t j = 1; j < m.objects.size(); j++) {
                //if the size of the object_id is <= 1, it means that it is not a duplicated of another tracked object, so we need to track it
                if(m.objects.at(j).object_id.size() <= 1){
                    double north, east, up;
                    this->gc.geodetic2Enu(m.objects.at(i).latitude, m.objects.at(i).longitude, 0, &east, &north, &up);
                    objects_to_track.push_back(tracking::obj_m(east, north, 0, m.objects.at(i).category, 1, 1));
                }
            }
        } else {
            output_message.objects.push_back(m.objects.at(i));  
        }

        for(size_t j = 0; j < m.lights.size(); j++)
            output_message.lights.push_back(m.lights.at(j)); 
        
    }

    //if some object need to be tracked, track it
    if(objects_to_track.size() > 0){
        this->t->track(objects_to_track, this->trVerbose);
        create_message_from_tracker(t->getTrackers(), &output_message, this->gc, this->adfGeoTransform);
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
    std::vector<MasaMessage> tmp;
    MasaMessage deduplicate_message;
    std::vector<cv::Point2f> map_pixels;
    fog::Profiler prof("Deduplicator");
   while(gRun){
        /* Delay is necessary. If the Deduplicator takes messages too quickly there is a risk 
        of not tracking the road users correctly. each message is read as a frame. 
        See the initialAge variable. */
        prof.tick("total time");
        // std::this_thread::sleep_for(std::chrono::milliseconds(30));        
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
        prof.tick("elaboration");
        // takes the input messages and return the deduplicate message
        elaborateMessage(input_messages, deduplicate_message);
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
}
