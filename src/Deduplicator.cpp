#include "Deduplicator.h"

void create_message_from_tracker(const std::vector<Tracker> &trackers, MasaMessage *m, 
                                 geodetic_converter::GeodeticConverter &gc, double *adfGeoTransform) {
    m->t_stamp_ms = time_in_ms();
    m->num_objects = trackers.size();
    m->objects.clear();
    double lat, lon, alt;
    for (auto t : trackers) {
        if (t.pred_list_.size() > 0) {
            Categories cat = (Categories) t.class_;
            gc.enu2Geodetic(t.pred_list_.back().x_, t.pred_list_.back().y_, 0, &lat, &lon, &alt);
            int pix_x, pix_y;
            coord2pixel(lat, lon, pix_x, pix_y, adfGeoTransform);
            uint8_t orientation = orientation_to_uint8(t.pred_list_.back().yaw_);
            uint8_t velocity = speed_to_uint8(t.pred_list_.back().vel_);
            RoadUser r{static_cast<float>(lat), static_cast<float>(lon), velocity, orientation, cat};
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
    initialAge = 15; //-5;
    ageThreshold = 0;//-17; //-8;  // 3 frames, 4 cameras --> 12
    nStates = 5;
    dt = 0.03;
    trVerbose = false;
    gc.initialiseReference(44.655540, 10.934315, 0);
    t = new Tracking(nStates, dt, initialAge, ageThreshold);
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

void * Deduplicator::deduplicate(void *n) {
    std::vector<Data> cur_message;
    std::vector<MasaMessage> input_messages; 
    MasaMessage deduplicate_message;
    std::vector<cv::Point2f> map_pixels;
    double east, north, up;
    double latitude, longitude, altitude;
    int map_pix_x, map_pix_y; 
    std::vector<tracker_line>  lines;
    while(gRun){
        // TODO: introduce some delay? (active wait) 
        cur_message.clear();
        
        std::cout<<"get messages\n";
        input_messages = this->inCm->getMessages();
        if(input_messages.size() == 0)
            continue;        // no received messages

        // deduplicate with the Tracker: fill only cur_message with the information of all collected MasaMessage. 
        for(auto m : input_messages) {
            for(size_t i = 0; i < m.objects.size(); i++) {
                    this->gc.geodetic2Enu(m.objects.at(i).latitude, m.objects.at(i).longitude, 0, &east, &north, &up);
                    cur_message.push_back(Data(east, north, 0, m.objects.at(i).category));
            }
        }
        this->t->Track(cur_message,this->trVerbose);

        //visualize the trackers
        lines.clear();
        for(auto tr : this->t->getTrackers()) {
            if(tr.pred_list_.size()) {
                tracker_line line;
                line.color = tk::gui::Color_t {tr.r_, tr.g_, tr.b_, 255};
                for(auto traj: tr.pred_list_) {
                    //convert from meters to GPS
                    this->gc.enu2Geodetic(traj.x_, traj.y_, 0, &latitude, &longitude, &altitude);
                    //convert from GPS to map pixels
                    GPS2pixel(this->adfGeoTransform, latitude, longitude, map_pix_x, map_pix_y);

                    line.points.push_back(this->viewer->convertPosition(map_pix_x,  map_pix_y, -0.004));
                }
                lines.push_back(line);
            }
        }
        if(this->show)
            this->viewer->setFrameData(lines);

        create_message_from_tracker(t->getTrackers(), &deduplicate_message, this->gc, this->adfGeoTransform);
        std::cout<<"insert m\n";
        this->outCm->insertMessage(deduplicate_message);
    }
    return (void *)NULL;
}