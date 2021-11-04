#include "Deduplicator.h"
#include <ctime>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sys/socket.h> //socket
#include <arpa/inet.h>  //inet_addr
#include <communicator.hpp>
#include <yaml-cpp/yaml.h>
#include <sys/stat.h>
#include <chrono>

namespace py = pybind11;

void deserialize_coords(const std::string& s, MasaMessage *m)
{
    std::istringstream is(s);
    cereal::PortableBinaryInputArchive retrieve(is);
    try
    {
        retrieve(*m);
    }
    catch (std::bad_alloc& ba)
    {
        std::cout << "Packet drop"<<std::endl;
    }
}

void prepare_message(MasaMessage *m)
{
    m->cam_idx = 4;
    m->t_stamp_ms = time_in_ms();
    m->num_objects = 0;
    m->objects.clear();
}


/***************       MASA MESSAGE GENERATION AND TX TO CARS       ***************/
struct Parameters_t {
    std::vector<int> outputPortList;
    std::vector<std::string> outputIpList;
};

/**
 * From the Yaml file we read an unique string. So we split it by the delimiter.
*/
std::vector<std::string> getParamList(std::string s) {
    size_t pos;
    std::vector<std::string> tokens;
    std::string delimiter = ", ";
    while ((pos = s.find(delimiter)) != std::string::npos) {
        tokens.push_back(s.substr(0, pos));
        s.erase(0, pos + delimiter.length());
    }
    tokens.push_back(s);
    return tokens;
}

/**
 * Convert the input strings vector to ints vector.
*/
std::vector<int> strListToIntList(const std::vector<std::string>& s) {
    std::vector<int> converted;
    for(const auto& elem : s) {
        converted.push_back(std::stoi(elem));
    }
    return converted;
}

/*
 * Fill the Parameters_t struct with Yaml file info.
 */
void readParametersYaml(const std::string &camerasParams, Parameters_t *par) {
    std::string cam, tmp;
    YAML::Node config = YAML::LoadFile(camerasParams);
    par->outputPortList = strListToIntList(getParamList(config["output_ports"].as<std::string>()));
    par->outputIpList = getParamList(config["output_ips"].as<std::string>());
}

void initialize_communicators(std::vector<Communicator<MasaMessage>>& comms, Parameters_t *params) {
    int i = 0;
    for (int port : params->outputPortList) {
        Communicator<MasaMessage> p;
        comms.push_back(p);
        std::string ip = params->outputIpList.at(i);
        comms.at(i).open_client_socket((char *)ip.c_str(), port);
        i++;
    }
}

inline bool file_exists(const std::string& filename) {
    struct stat buffer;
    return (stat (filename.c_str(), &buffer) == 0);
}

void send_message_to_car(MasaMessage& m) {
    std::string param_file ="/root/COMPSs-obstacle-detection/class_aggregator_configuration_file.yaml";
    if (file_exists(param_file)) {
        Parameters_t params;
        readParametersYaml(param_file, &params);
        std::vector<Communicator<MasaMessage>> *comms = new std::vector<Communicator<MasaMessage>>(SOCK_DGRAM);;
        initialize_communicators(*comms, &params);
        std::vector<int> objects_to_remove; // TODO: REMOVE LINE
        for (int i = 0; i < comms->size(); i++) {
            for (int j = 0; j < m.objects.size(); ++j) { // TODO: REMOVE ALL FOR AND EXTRA FOR IN THE END
                RoadUser ru = m.objects.at(j);
                if (ru.category == 20 || ru.category == 21 || ru.category == 30 || ru.category == 31 || ru.category == 32 ||
                    ru.category == 40) {
                    if (ru.camera_id.at(0) != 20 && ru.camera_id.at(0) != 21 && ru.camera_id.at(0) != 30 &&
                        ru.camera_id.at(0) != 31
                        && ru.camera_id.at(0) != 32 && ru.camera_id.at(0) != 40) {
                        std::cout << "SKIPPING DUE TO ERROR IN DEDUPLICATOR WITH CONNECTED CAR DETECTED BY CAMERA"
                                  << std::endl;
                        objects_to_remove.push_back(j);
                    }
                }
                std::cout << m.cam_idx << " " << m.objects.at(j).camera_id.at(0) << " " << m.objects.at(j).category << " "
                          << m.objects.at(j).object_id.at(0) << std::endl;
            }
            for (int object_to_remove : objects_to_remove)
                m.objects.erase(m.objects.begin() + object_to_remove);

            comms->at(i).send_message(&m, params.outputPortList[i]);
        }
    }
}
/***************              ***************/



/*void printRoadUser(const RoadUser& ru){

    std::cout << std::setprecision(10) << ru.category << " " << ru.latitude << " " << ru.longitude << " " << (int) ru.speed
              << " " << ru.orientation << " " << ru.error << " " << ru.camera_id.size();
    for( auto elem : ru.camera_id){
        std::cout << " " << elem;
    }
    for( auto elem : ru.object_id){
        std::cout << " " << elem;
    }
    std::cout << std::endl;
}*/

/***************       UDP SOCKET CONNECTION       ***************/
// udp socket with timeout of 5 ms
int receive_messages(Communicator<MasaMessage> &comm, std::vector<MasaMessage> *input_messages)
{
    int message_size = 50000;
    char client_message[message_size];
    memset(client_message, 0, message_size);
    int len;
    struct sockaddr_in cliaddr{};
    int socket_desc = comm.get_socket();

    struct timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 5000; // 10000;
    bool receive = true;
    while (receive) {
        if (setsockopt(socket_desc, SOL_SOCKET, SO_RCVTIMEO, &tv,sizeof(tv)) >= 0) {
            // auto start = std::chrono::system_clock::now();
            int res = recvfrom(socket_desc, client_message, message_size, 0,
                     (struct sockaddr *) &cliaddr, (socklen_t *) &len);
            /*auto end = std::chrono::system_clock::now();
            std::chrono::duration<double> elapsed_seconds = end-start;
            std::time_t end_time = std::chrono::system_clock::to_time_t(end);
            std::cout << "Finished blocked socket receiving at " << std::ctime(&end_time)
                      << "elapsed time: " << elapsed_seconds.count() << std::endl;
            std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
            std::cout << "Timestamp after receiving " << ms.count() << std::endl;*/
            if (res > 0) {
                // we have read information coming from the car
                std::string s((char *) client_message, message_size);
                MasaMessage m;
                deserialize_coords(s, &m);
                if (m.objects.size() > 0) {
                    input_messages->push_back(m);
                }
                else {
                    // std::cout << "Message does not have any object inside, dummy message. Exiting loop..." << std::endl;
                    break;
                }
            } else {
                // std::cout << "No data received from recvfrom" << std::endl;
                receive = false;
            }
        } else {
            std::cout << "Error in setsockopt" << std::endl;
        }
    }
    return 0;
}
/***************              ***************/


int convert_category_from_berkeley_to_dedu(int category_berkeley) {
    int category_dedu;
    switch (category_berkeley) {
        case 0:
            category_dedu = C_person;
            break;
        case 1:
        case 2:
            category_dedu = C_car;
            break;
        case 3:
            category_dedu = C_bus;
            break;
        case 4:
            category_dedu = C_motorbike;
            break;
        case 5:
        case 6:
            category_dedu = C_bycicle;
            break;
        default:
            std::cout << "REACH HERE WHEN IT SHOULD NOT with category " << category_berkeley << std::endl;
            category_dedu = C_rover; // TODO: No Categories for unknown or default class
    }
    return category_dedu;
}

int convert_category_from_dedu_to_berkeley(Categories category_dedu) {
    int category_berkeley;
    switch (category_dedu) {
        case C_person:
            category_berkeley = 0;
            break;
        case C_car:
            category_berkeley = 1;
            break;
        case C_bus:
            category_berkeley = 3;
            break;
        case C_motorbike:
            category_berkeley = 4;
            break;
        case C_bycicle:
            category_berkeley = 5;
            break;
        case C_marelli1:
            category_berkeley = 20;
            break;
        case C_marelli2:
            category_berkeley = 21;
            break;
        case C_quattroporte:
            category_berkeley = 30;
            break;
        case C_levante:
            category_berkeley = 31;
            break;
        default:
            std::cout << "SHOULDNT REACH HERE with category " << category_dedu << std::endl;
            category_berkeley = C_rover; // TODO: No Categories for unknown or default class
    }
    return category_berkeley;
}


/***
 *
 * input_deduplicator: vector of vectors in which each inner vector contains the information of a detected object (lat,
 *                     lon, cat, speed, yaw, tracker_id) by a given camera
 * cam_ids: vector with the ids of the cameras correlating the cam_id to the vector<vector<...>> input_deduplicator
 * frames: Used when displaying the output of several videos to correlate the objects of a given video to its corresponding frame
 *
 * ***/
std::tuple<uint64_t, std::vector<std::tuple<int, int, int, double, double, double, double, int, int, int, int, int>>>
compute_deduplicator(std::vector<std::vector<std::tuple<double, double, int, uint8_t, uint8_t, int, int, int,
        int, int>>> &input_deduplicator, std::vector<uint32_t> cam_ids, std::vector<uint32_t> frames) { // std::vector<uint64_t> timestamps
    double latitude = 44.655540;
    double longitude = 10.934315;
    double *adfGeoTransform = (double *) calloc(6, sizeof(double));
    adfGeoTransform[0] = 10.9258;
    adfGeoTransform[1] = 1.29294e-06;
    adfGeoTransform[2] = 0;
    adfGeoTransform[3] = 44.6595;
    adfGeoTransform[4] = 0;
    adfGeoTransform[5] = -8.79695e-07;
    static fog::Deduplicator deduplicator(adfGeoTransform, latitude, longitude);
    std::vector<MasaMessage> input_messages;
    int idx, idy;
    idx = 0;
    // retrieving and preparing data coming from the cameras
    for (const auto& vec_info : input_deduplicator) {
        std::cout << "Info from the camera " << cam_ids[idx] << std::endl;
        MasaMessage message;
        idy = 0;
        for (auto info : vec_info) {
            // info is a tuple of <lat (float), lon (float), category (int), velocity (uint8_t), yaw (uint8_t)>
            // RoadUser receives as params: <lat, lon, vel, yaw, cat>
            RoadUser r;
            r.latitude = static_cast<float>(std::get<0>(info));
            r.longitude = static_cast<float>(std::get<1>(info));
            r.error = 1.0; //0.0; // TODO: not set this here but in tracker/class-edge
            r.speed = std::get<3>(info);
            r.orientation = std::get<4>(info);
            r.category = static_cast<Categories>(convert_category_from_berkeley_to_dedu(std::get<2>(info)));
            r.camera_id.push_back(cam_ids[idx]); // camera id
            r.object_id.push_back(std::get<5>(info)); // tracker id
            r.idx = idx;
            r.idy = idy;
            message.objects.push_back(r);
            idy++;
            std::cout << std::get<5>(info) << " " << r.latitude << " " << r.longitude << " " << r.category << std::endl;
        }
        message.num_objects = message.objects.size();
        message.cam_idx = cam_ids[idx];
        input_messages.push_back(message);
        idx++;
    }

    // retrieving and preparing data from the cars by using a client udp socket at 18888
    std::string ip = "192.168.12.4"; // TODO: deduplicator needs to be fixed to be executed in fog node 4
    int port_receiving = 18888;
    int port_sending = 18889;

    Communicator<MasaMessage> *comm_send = new Communicator<MasaMessage>(SOCK_DGRAM);
    MasaMessage *dummy = new MasaMessage;
    prepare_message(dummy);
    Communicator<MasaMessage> *comm_recv = new Communicator<MasaMessage>(SOCK_DGRAM);
    // comm_recv->open_client_socket(ip, port_receiving);
    comm_recv->open_server_socket(port_receiving);
    std::vector<char> cstr(ip.c_str(), ip.c_str() + ip.size() + 1);
    comm_send->open_client_socket(cstr.data(), port_sending);

    std::vector<MasaMessage> input_messages_car;
    int socketDescClient = comm_send->get_socket();
    int socketDesc = comm_recv->get_socket();
    if (socketDesc != -1) {
        input_messages_car.clear();
        // std::cout << "Dummy message sent" << std::endl;
        comm_send->send_message(dummy, port_sending);
        std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(std::chrono::system_clock::now().time_since_epoch());
        // std::cout << "Timestamp after sending " << ms.count() << std::endl;
        receive_messages(*comm_recv, &input_messages_car);
        // input_messages_car = deduplicator.fillTrackerInfo(input_messages_car); // not needed any more as tracker is performed in forwarder
        for (const MasaMessage& m : input_messages_car) {
            /* std::cout << "Receiving data from the cars with num objects: " << m.num_objects << " and source id is "
                    << m.cam_idx<< std::endl; */
            if (m.num_objects > 0)
                input_messages.push_back(m);
        }
        close(socketDesc);
        close(socketDescClient);
    } else {
        std::cout << "Could not open server socket in port_receiving " << port_receiving << std::endl;
    }

    MasaMessage return_message;
    std::map<std::pair<uint16_t, uint16_t>, RoadUser> last_duplicated_objects;
    deduplicator.elaborateMessages(input_messages, return_message, last_duplicated_objects);

    double lat, lon;
    float vel, yaw;
    int pixel_x, pixel_y, pixel_w, pixel_h;
    std::vector<std::tuple<int, int, int, double, double, double, double, int, int, int, int, int>> info;
    int category_berkeley;
    int frame;
    for (const RoadUser& ru : return_message.objects) {
        category_berkeley = convert_category_from_dedu_to_berkeley(ru.category);
        if (ru.camera_id.at(0) == 0) { // TODO: REMOVE FILTER
            std::cout << "IT SHOULD NOT BE HERE" << std::endl;
            continue;
        }
        if (ru.category == 20 || ru.category == 21 || ru.category == 30 || ru.category == 31 || ru.category == 32 || ru.category == 40) { // TODO: REMOVE FILTER
            if (ru.camera_id.at(0) != 20 && ru.camera_id.at(0) != 21 && ru.camera_id.at(0) != 30 && ru.camera_id.at(0) != 31
                && ru.camera_id.at(0) != 32 && ru.camera_id.at(0) != 40) {
                std::cout << "SKIPPING DUE TO ERROR IN DEDUPLICATOR WITH CONNECTED CAR DETECTED BY CAMERA" << std::endl;
                continue;
            }
        }
        if (ru.camera_id.at(0) == 20 || ru.camera_id.at(0) == 21 || ru.camera_id.at(0) == 30 || ru.camera_id.at(0) == 31
                || ru.camera_id.at(0) == 32 || ru.camera_id.at(0) == 40) {
            // data coming from the cars have no bounding box nor frame
            pixel_x = pixel_y = pixel_w = pixel_h = 0;
            lat = ru.latitude;
            lon = ru.longitude;
            vel = deduplicator.uint8_to_speed(ru.speed);
            yaw = deduplicator.uint16_to_yaw(ru.orientation);
            frame = -1; // TODO: objects coming from car do not belong to a specific frame. How to select in which frame are they displayed?
        } else {
            // data coming from the cameras have bounding box
            lat = std::get<0>(input_deduplicator[ru.idx][ru.idy]);
            lon = std::get<1>(input_deduplicator[ru.idx][ru.idy]);
            vel = std::get<3>(input_deduplicator[ru.idx][ru.idy]);
            yaw = std::get<4>(input_deduplicator[ru.idx][ru.idy]);
            pixel_x = std::get<6>(input_deduplicator[ru.idx][ru.idy]);
            pixel_y = std::get<7>(input_deduplicator[ru.idx][ru.idy]);
            pixel_w = std::get<8>(input_deduplicator[ru.idx][ru.idy]);
            pixel_h = std::get<9>(input_deduplicator[ru.idx][ru.idy]);
            frame = frames[ru.idx];
        }
        info.emplace_back(ru.camera_id.at(0), ru.object_id.at(0), category_berkeley, vel, yaw, lat, lon,
                                    pixel_x, pixel_y, pixel_w, pixel_h, frame);
    }
    send_message_to_car(return_message);
    return std::make_tuple(return_message.t_stamp_ms, info);
}

PYBIND11_MODULE(deduplicator, m) {

    py::enum_<Categories>(m, "Categories")
            .value("C_person", Categories::C_person)
            .value("C_bycicle", Categories::C_bycicle)
            .value("C_car", Categories::C_car)
            .value("C_motorbike", Categories::C_motorbike)
            .value("C_bus", Categories::C_bus)
            .export_values();

    py::enum_<LightStatus>(m, "LightStatus")
            .value("L_green", LightStatus::L_green)
            .value("L_yellow", LightStatus::L_yellow)
            .value("L_red", LightStatus::L_red)
            .export_values();

    py::class_<TrafficLight>(m, "TrafficLight")
            .def(py::init<>())
            .def_readwrite("latitude", &TrafficLight::latitude)
            .def_readwrite("longitude", &TrafficLight::longitude)
            .def_readwrite("orientation", &TrafficLight::orientation)
            .def_readwrite("time_to_change", &TrafficLight::time_to_change)
            .def_readwrite("status", &TrafficLight::status)
            .def(py::pickle(
                    [](const py::object& self) {
                        return py::make_tuple(self.attr("latitude"), self.attr("longitude"), self.attr("orientation"),
                                              self.attr("time_to_change"), self.attr("status"));
                    },
                    [](const py::tuple &t) {
                        if (t.size() != 5)
                            throw std::runtime_error("Invalid state");
                        TrafficLight trafficLight{};
                        trafficLight.latitude = t[0].cast<float>();
                        trafficLight.longitude = t[1].cast<float>();
                        trafficLight.orientation = t[2].cast<uint8_t>();
                        trafficLight.time_to_change = t[3].cast<uint8_t>();
                        trafficLight.status = t[4].cast<LightStatus>();
                        return trafficLight;
                    }
            ));

    py::class_<RoadUser>(m, "RoadUser")
            .def(py::init<>())
            .def_readwrite("latitude", &RoadUser::latitude)
            .def_readwrite("longitude", &RoadUser::longitude)
            .def_readwrite("speed", &RoadUser::speed)
            .def_readwrite("orientation", &RoadUser::orientation)
            .def_readwrite("category", &RoadUser::category)
            .def(py::pickle(
                    [](const py::object& self) {
                        return py::make_tuple(self.attr("latitude"), self.attr("longitude"), self.attr("speed"),
                                              self.attr("orientation"), self.attr("category"));
                    },
                    [](const py::tuple &t) {
                        if (t.size() != 5)
                            throw std::runtime_error("Invalid state");
                        RoadUser roadUser;
                        roadUser.latitude = t[0].cast<float>();
                        roadUser.longitude = t[1].cast<float>();
                        roadUser.speed = t[2].cast<uint8_t>();
                        roadUser.orientation = t[3].cast<uint16_t>();
                        roadUser.category = t[4].cast<Categories>();
                        return roadUser;
                    }
            ));

    py::class_<MasaMessage>(m, "MasaMessage")
            .def(py::init<>())
            .def_readwrite("cam_idx", &MasaMessage::cam_idx)
            .def_readwrite("t_stamp_ms", &MasaMessage::t_stamp_ms)
            .def_readwrite("num_objects", &MasaMessage::num_objects)
            .def_readwrite("objects", &MasaMessage::objects)
            .def_readwrite("lights", &MasaMessage::lights)
            .def(py::pickle(
                    [](const py::object& self) {
                        return py::make_tuple(self.attr("cam_idx"), self.attr("t_stamp_ms"), self.attr("num_objects"),
                                              self.attr("objects"), self.attr("lights"));
                    },
                    [](const py::tuple &t) {
                        if (t.size() != 5)
                            throw std::runtime_error("Invalid state!");
                        MasaMessage msg;
                        msg.cam_idx = t[0].cast<uint32_t>();
                        msg.t_stamp_ms = t[1].cast<uint64_t>();
                        msg.num_objects = t[2].cast<uint16_t>();
                        msg.objects = t[3].cast<std::vector<RoadUser>>();
                        msg.lights = t[4].cast<std::vector<TrafficLight>>();
                        return msg;
                    }
            ));

    /*
    py::class_<fog::Deduplicator>(m, "Deduplicator")
            .def(py::init<double*, double, double>())
            .def(py::pickle(
                [](py::object self) {
                    self.
                    return py::make_tuple(self.attr("adfGeoTransform"), self.attr("t"), self.attr("initialAge"),
                                          self.attr("nStates"), self.attr("dt"), self.attr("trVerbose"),
                                          self.attr("gc"));
                },
                [](const py::tuple &t) {
                    if (t.size() != 7)
                        throw std::runtime_error("Invalid state!");
                    fog::Deduplicator deduplicator;
                    msg.cam_idx = t[0].cast<uint32_t>();
                    msg.t_stamp_ms = t[1].cast<uint64_t>();
                    msg.num_objects = t[2].cast<uint16_t>();
                    msg.objects = t[3].cast<std::vector<RoadUser>>();
                    msg.lights = t[4].cast<std::vector<TrafficLight>>();
                    return msg;
                }
            ));
            */

    m.def("compute_deduplicator", &compute_deduplicator, py::return_value_policy::automatic);
    //m.def("compute_deduplicator", &compute_deduplicator2, py::return_value_policy::automatic);
}