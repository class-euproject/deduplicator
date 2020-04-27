#include "Deduplicator.h"

namespace fog {

void create_message_from_tracker(const std::vector<tracking::Tracker> &trackers, MasaMessage *m, 
                                 geodetic_converter::GeodeticConverter &gc, double *adfGeoTransform) {
    m->t_stamp_ms = time_in_ms();
    m->num_objects = trackers.size();
    m->objects.clear();
    double lat, lon, alt;
    for (auto t : trackers) {
        if (t.predList.size() > 0) {
            Categories cat = (Categories) t.cl;
            gc.enu2Geodetic(t.predList.back().x, t.predList.back().y, 0, &lat, &lon, &alt);
            int pix_x, pix_y;
            GPS2pixel(adfGeoTransform, lat, lon, pix_x, pix_y);
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
                            AggregatorViewer &v,
                            bool visual) {
    inCm = &inputSharedMessage;
    outCm = &outputSharedMessage;
    adfGeoTransform = (double *)malloc(6 * sizeof(double));
    readTiff((char *)tifFile.c_str(), adfGeoTransform);
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

void Deduplicator::show_updates(double latitude, double longitude, double altitude) {
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

                line.points.push_back(this->viewer->convertPosition(map_pix_x,  map_pix_y, -0.004));
                // std::cout<<"point: "<<line.points[line.points.size()-1].x<<" ---- "<<line.points[line.points.size()-1].y<<" ---- "<<line.points[line.points.size()-1].z<<std::endl;
            }
            lines.push_back(line);
        }
    }
    // std::cout<<"lines: "<<lines.size()<<std::endl;
    if(this->show)
        this->viewer->setFrameData(lines);
}

void * Deduplicator::deduplicate(void *n) {
    std::vector<tracking::obj_m> cur_message;
    std::vector<MasaMessage> input_messages; 
    MasaMessage deduplicate_message;
    std::vector<cv::Point2f> map_pixels;
    double east, north, up;
    double latitude, longitude, altitude;
    while(gRun){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        cur_message.clear();
        
        
        input_messages = this->inCm->getMessages();
        std::cout<<"dedup dim reading list: "<<input_messages.size()<<std::endl;
        if(input_messages.size() == 0)
            continue;        // no received messages
        std::cout<<"get messages\n";
        // deduplicate with the Tracker: fill only cur_message with the information of all collected MasaMessage. 
        for(auto m : input_messages) {
            for(size_t i = 0; i < m.objects.size(); i++) {
                    this->gc.geodetic2Enu(m.objects.at(i).latitude, m.objects.at(i).longitude, 0, &east, &north, &up);
                    cur_message.push_back(tracking::obj_m(east, north, 0, m.objects.at(i).category));
            }
        }
        this->t->track(cur_message,this->trVerbose);

        show_updates(latitude, longitude, altitude);

        create_message_from_tracker(t->getTrackers(), &deduplicate_message, this->gc, this->adfGeoTransform);
        std::cout<<"ded insert m\n";
        this->outCm->insertMessage(deduplicate_message);
        input_messages.clear();
    }
    return (void *)NULL;
}
}