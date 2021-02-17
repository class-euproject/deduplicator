#include <stdio.h>
#include <string.h> //strlen
#include <stdlib.h> //strlen
#include <time.h>
#include <sys/socket.h>
#include <arpa/inet.h> //inet_addr
#include <unistd.h>    //write
#include <pthread.h>   //for threading , link with lpthread
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <string>
#include <map>
#include <sstream>
#include <fstream>
#include <iterator>
#include <limits>
#include <iomanip>
#include <chrono>
#include <thread>

#include "../masa_protocol/include/communicator.hpp"
#include "../masa_protocol/include/messages.hpp"


inline char separator()
{
#ifdef _WIN32
       return '\\';
#else
       return '/';
#endif
}

uint32_t parseFile(std::map<std::string, std::vector<RoadUser>> *mapTimeStamps,
               const std::string filename) {

    char buffer[512];
    std::string s_cwd(getcwd(buffer, sizeof(buffer)));
    std::stringstream ss;
    ss << s_cwd << separator() << filename;
    std::string inputfile = ss.str();
    std::cout << "Opening " << inputfile << std::endl;

    std::ifstream fStream(inputfile);
    int a; std::string b; int detectedClass; float x; float y;
    float speed, orientation;
    unsigned int countLines = 0;

    std::cout << std::setprecision(10);

    RoadUser data;

    std::map<std::string, std::vector<RoadUser>>::iterator it;

    std::string line;
    unsigned long long lastInsertedTS = 0;
    std::string lastInsertedTString = "";

    while (std::getline(fStream, line)) {
        //std::cout << line << std::endl;
        std::stringstream  lineStream(line);

        lineStream >> a; //cidx ignored
        lineStream >> b; //timestamp string
        lineStream >> detectedClass;
        lineStream >> x; //lat
        lineStream >> y; //long
        lineStream >> speed;
        lineStream >> orientation;

        std::cout << "READ " << a << " " << b << " " << detectedClass << " "
                  <<std::fixed << std::setprecision(9) << x << " " << y << " " 
                  <<std::fixed << std::setprecision(2) << speed << " " << orientation << std::endl;

        //getch();

        //be sure to ignore unrecognized roadusers
        //Comment this condition for consistency checks!
        /*if (detectedClass != 1 &&
                detectedClass != 14 &&
                detectedClass != 13 &&
                detectedClass != 6 &&
                detectedClass != 5) {
                //std::cout << "Discarding class " << detectedClass << std::endl;
                continue;
        }*/

        data.category = static_cast<Categories>(detectedClass);
        data.latitude = x;
        data.longitude = y;
        data.speed = static_cast<uint8_t>(speed);
        data.orientation = static_cast<uint8_t>(orientation);

        /*std::cout << "Lat " << data.latitude << " long " << data.longitude << " speed " << (int)speed << " " 
        << "orient " << (int) orientation << std::endl;
        std::cout << "Saved class is " << (int)data.category << std::endl;*/

        it = mapTimeStamps->find(b);

        if (it == mapTimeStamps->end()) {  //potentially new timestamp

                unsigned long long timestamp = stoll(b);

                if (lastInsertedTS == 0 || (timestamp - lastInsertedTS) > 100) { //brand new timestamp
                        std::vector<RoadUser> datasetT;
                        datasetT.push_back(data);
                        mapTimeStamps->insert(std::pair<std::string, std::vector<RoadUser>>(b, datasetT));
                        lastInsertedTS = timestamp;
                        lastInsertedTString = b;
                }
                else { //timestamps that have been previously merged
                        it = mapTimeStamps->find(lastInsertedTString);
                        it->second.push_back(data);

                }
        }
        else { //exact previous timestamp
                it->second.push_back(data);
        }


        countLines++;
        //if (countLines > 10) break;
    }

    fStream.close();
    return countLines;

}

void prepare_message_from_dataset(MasaMessage *m, uint64_t ts,std::vector<RoadUser> *v, uint32_t idx) {
    m->cam_idx = idx;
    m->t_stamp_ms = ts;
    m->num_objects = v->size();
    m->objects.clear();
    std::cout<<"m -- "<< m->cam_idx<<" - "<< m->t_stamp_ms<<" - "<<m->num_objects<<" - \n";
    for(size_t i=0; i<m->num_objects;i++) {
        m->objects.push_back(v->at(i));
        std::cout<<m->objects[i].category<<" : "
        <<std::fixed << std::setprecision(9)<<m->objects[i].latitude<<" : "<<m->objects[i].longitude<<" : "
        <<std::fixed << std::setprecision(2)<<static_cast<float>(m->objects[i].speed)<<" : "<<static_cast<float>(m->objects[i].orientation)<<std::endl;
    }
    // m->lights.clear();
    // TrafficLight l1{.1f,.2f,0,L_green,15};
    // TrafficLight l2{.2f,.3f,2,L_red,15};
    // m->lights.push_back(l1);
    // m->lights.push_back(l2);
}

unsigned long long time_in_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned long long t_stamp_ms = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
    return t_stamp_ms;
}

void prepare_message(MasaMessage *m, int idx)
{
    m->cam_idx = idx;
    m->t_stamp_ms = time_in_ms();
    m->num_objects = 5;

    m->objects.clear();

    RoadUser r1{std::vector<int> {idx}, .3f,.4f,0,std::vector<int>{1},C_car};
    RoadUser r2{std::vector<int> {idx}, .5f,.6f,1,std::vector<int>{2},C_bus};
    RoadUser r3{std::vector<int> {idx}, .7f,.8f,2,std::vector<int>{3},C_bycicle};
    RoadUser r4{std::vector<int> {idx}, .9f,.10f,3,std::vector<int>{4},C_motorbike};
    RoadUser r5{std::vector<int> {idx}, .11f,.12f,4,std::vector<int>{5},C_person};

    m->objects.push_back(r1);
    m->objects.push_back(r2);
    m->objects.push_back(r3);
    m->objects.push_back(r4);
    m->objects.push_back(r5);

    // m->lights.clear();
    // TrafficLight l1{.1f,.2f,0,L_green,15};
    // TrafficLight l2{.2f,.3f,2,L_red,15};
    // m->lights.push_back(l1);
    // m->lights.push_back(l2);

}

void manualDeserializing(Communicator<MasaMessage> &Comm, MasaMessage *m) {

    prepare_message(m,200);
    std::stringbuf s;
    Comm.serialize_coords(m,&s);

    //here we simply reverse engineer the
    //"serialization" procedure of the cereal library.
    //so the following code is not necessary
    unsigned char buffer[s.str().length()];
    memcpy(buffer, s.str().data(), s.str().length());

    std::cout << "HEADER " << std::endl;
    size_t size_ru = sizeof(float) + sizeof(float) + sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint8_t);
    //std::cout << "RU " << size_ru << std::endl;
    uint32_t cidx;
    uint64_t ts;
    uint16_t objs;
    uint64_t svru;
    uint64_t svtl;
    memcpy(&cidx,buffer+1,sizeof(uint32_t));
    memcpy(&ts,buffer+1+sizeof(uint32_t),sizeof(uint64_t));
    memcpy(&objs,buffer+1+sizeof(uint32_t)+sizeof(uint64_t),sizeof(uint16_t));
    size_t first_header_size = 1+sizeof(uint32_t)+sizeof(uint64_t)+sizeof(uint16_t);
    memcpy(&svru,buffer+first_header_size,sizeof(uint64_t));
    size_t st_ru = first_header_size+sizeof(uint64_t);
    memcpy(&svtl,buffer+st_ru+(svru*size_ru),sizeof(uint64_t));

    std::cout << "cidx " << cidx << std::endl;
    std::cout << "ts " << ts << std::endl;
    std::cout << "objs " << objs << std::endl;
    std::cout << "svru " << svru << std::endl;
    std::cout << "svtl " << svtl << std::endl;

    std::cout << "ROAD USERS " << std::endl;

    float lati, longi;
    uint8_t speed, orientation;
    uint8_t cat;
    
    for(size_t i=0; i<svru; i++) {
            std::cout << "Road user " << i << ":" << std::endl;
            memcpy(&lati, buffer+st_ru+(i)*size_ru,sizeof(float));
            memcpy(&longi, buffer+st_ru+(i)*size_ru+sizeof(float),sizeof(float));
            memcpy(&speed, buffer+st_ru+(i)*size_ru+2*sizeof(float),sizeof(uint8_t));
            memcpy(&orientation, buffer+st_ru+(i)*size_ru+2*sizeof(float)+sizeof(uint8_t),sizeof(uint8_t));
            memcpy(&cat, buffer+st_ru+(i)*size_ru+2*sizeof(float)+2*sizeof(uint8_t),sizeof(uint8_t));
            std::cout << "Lat " << lati  << " Long " << longi << " speed " << (int)speed << " orient " <<
            (int)orientation << " class " << (int)cat << std::endl;
    }


    // std::cout << "========= " << std::endl;
    // std::cout << "TRAFFIC LIGHTS " << std::endl;
    // size_t size_tl = 2*sizeof(float) + 3*sizeof(uint8_t);
    // size_t st_tl = st_ru+svru*size_ru+sizeof(uint64_t);

    // for(size_t i=0; i<svtl; i++) {
    //         std::cout << "Traffic light " << i << ":" << std::endl;
    //         memcpy(&lati, buffer+st_tl+i*size_tl, sizeof(float));
    //         memcpy(&longi, buffer+st_tl+i*size_tl+sizeof(float), sizeof(float));
    //         memcpy(&orientation, buffer+st_tl+size_tl*i+2*sizeof(float),sizeof(uint8_t));
    //         memcpy(&cat, buffer+st_tl+size_tl*i+2*sizeof(float)+sizeof(uint8_t),sizeof(uint8_t));
    //         memcpy(&speed, buffer+st_tl+i*size_tl+2*sizeof(float)+2*sizeof(uint8_t),sizeof(uint8_t));
    //         std::cout << "Lat " << lati << " Long " << longi << " orient " << (int)orientation << " status " <<
    //         (int)cat << " ttc " << (int)speed << std::endl;
    // }
    std::cout << "============== " << std::endl;
    //finished reverse engineering serialization//////////////////////////////////////
}

int main(int argc, char *argv[]) {
    int protocol = SOCK_DGRAM;
    char *ipString = (char*)"127.0.0.1";
    int port = 8888;

    if(argc>1) {
        if(strcmp("--help",argv[1])==0) {
            std::cout << "Command line interface: " << std::endl;
            std::cout << argv[0] << " IPADDRESS PORT UDP" << std::endl;
            std::cout << "Default values are:" << std::endl;
            std::cout << "protocol UDP on port " << port << std::endl;
            std::cout << "IP address: " << ipString << std::endl;
            exit(0);
        }
    }
    if(argc>=2)
        ipString = argv[1];
    if(argc>3)
        port = atoi(argv[2]);

    Communicator<MasaMessage> Comm(protocol);
    MasaMessage *m = new MasaMessage;

    //optional for reverse engineering:
    manualDeserializing(Comm, m);

    //////////////////////Dataset Parsing for tests//////////////////////////////////
    std::map<std::string, std::vector<RoadUser>> mapTimeStamps;

    unsigned int count = parseFile(&mapTimeStamps, "../demo/data/testdata.txt");
    // return 0;
    std::cout << "Done. Parsed " << count << " elements" << std::endl;

    //consistency check:
    unsigned int ccheck = 0;
    unsigned int maxChunk = 0;
    unsigned int numBigChunks = 0;

    const size_t MAX_ROAD_USERS = 500;

    for (auto const& value : mapTimeStamps) {
        //std::cout << value.first << " " << value.second.size() << std::endl;
        if (maxChunk < value.second.size())
                maxChunk = value.second.size();

        if (value.second.size() > MAX_ROAD_USERS)
                numBigChunks++;
        ccheck += value.second.size();
    }
    
    std::cout << "CCHECK " << ccheck << "/" << count << ". Keys in map " << mapTimeStamps.size() <<
            " max chunk size: " << maxChunk << " Number of big chunks: " << numBigChunks << std::endl;
    ///////////////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////Actual netcode starts here///////////////////

    Comm.open_client_socket(ipString, port);
    std::cout << "Waiting for estabilishing connection " << std::endl;

    auto it = mapTimeStamps.begin();
    size_t countts=0;
    while(true) {
        std::cout << "Sending msg... " << it->second.size() << std::endl;
        //timestamp and cam_idx are present as message field, but are
        //not properly parsed from the file.
        prepare_message_from_dataset(m,time_in_ms(),&(it->second),0);
        Comm.send_message(m, port);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        countts++;
        it++;

        if(it==mapTimeStamps.end())
                it=mapTimeStamps.begin();
    }
    std::cout << "Execution Terminated" << std::endl;
    return 0;
}