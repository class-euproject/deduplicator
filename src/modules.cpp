#include "Deduplicator.h"
#include <ctime>
#include <tuple>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <sys/socket.h> //socket
#include <arpa/inet.h>  //inet_addr
#include <sys/types.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <communicator.hpp>


namespace py = pybind11;

Categories category_parse(int class_number) {
    switch (class_number) {
        case 0:
            return C_person;
        case 1:
            return C_car;
        case 3:
            return C_bus;
        case 4:
            return C_motorbike;
        case 5:
            return C_bycicle;
        default:
            return C_rover; // TODO: No Categories for unknown or default class
    }
}

void deserialize_coords(std::string s, MasaMessage *m)
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

/*int receive_message(int socket_desc, MasaMessage *m)
{
    int message_size = 50000;
    char client_message[message_size];
    memset(client_message, 0, message_size);

    int len;
    struct sockaddr_in cliaddr;
    recvfrom(socket_desc, client_message, message_size, MSG_DONTWAIT,
                         ( struct sockaddr *) &cliaddr, (socklen_t *)&len);
    std::string s((char *)client_message, message_size);
    deserialize_coords(s, m);
    return 0;
}*/


// udp socket with timeout of 10 ms
int receive_message(Communicator<MasaMessage> &comm, MasaMessage *m)
{
    int message_size = 50000;
    char client_message[message_size];
    memset(client_message, 0, message_size);

    int len;
    struct sockaddr_in cliaddr;
    int socket_desc = comm.get_socket();

    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 50000;
    if (setsockopt(socket_desc, SOL_SOCKET, SO_RCVTIMEO, &tv,sizeof(tv)) >= 0) {
        if (recvfrom(socket_desc, client_message, message_size, 0,
                 ( struct sockaddr *) &cliaddr, (socklen_t *)&len) < 0) {
            std::cout << "Timeout reached, keep on going without data from the car" << std:: endl;
        } else {
            // we have read information coming from the car
            std::string s((char *)client_message, message_size);
            deserialize_coords(s, m);
        }
    } else {
        std::cout << "Error in setsockopt" << std::endl;
    }
    return 0;
}

/*std::tuple<uint64_t, std::vector<std::tuple<uint32_t, int, int, float, float, double, double, int, int, int, int>>>
compute_deduplicator(std::vector<std::vector<std::tuple<double, double, int, uint8_t, uint8_t, int, int, int,
        int, int>>> &input_deduplicator) {*/
// TODO: should we return one camera_id or the whole vector? Same for object id?
std::tuple<uint64_t, std::vector<std::tuple<int, int, int, double, double, double, double, int, int, int, int>>>
                compute_deduplicator(std::vector<std::vector<std::tuple<double, double, int, uint8_t, uint8_t, int, int, int,
                             int, int>>> &input_deduplicator, std::vector<uint32_t> cam_ids) { // std::vector<uint64_t> timestamps
    double latitude = 44.655540;
    double longitude = 10.934315;
    double *adfGeoTransform = (double *) calloc(6, sizeof(double));
    adfGeoTransform[0] = 10.9258;
    adfGeoTransform[1] = 1.29294e-06;
    adfGeoTransform[2] = 0;
    adfGeoTransform[3] = 44.6595;
    adfGeoTransform[4] = 0;
    adfGeoTransform[5] = -8.79695e-07;
    /* std::string tifFile;
     * adfGeoTransform = (double *)malloc(6 * sizeof(double));
     * readTiff((char *)tifFile.c_str(), adfGeoTransform);*/
    static fog::Deduplicator deduplicator(adfGeoTransform, latitude, longitude);
    std::vector<MasaMessage> input_messages;
    int idx, idy;
    idx = 0;
    // retrieving and preparing data coming from the cameras
    for (const auto& vec_info : input_deduplicator) {
        MasaMessage message;
        idy = 0;
        for (auto info : vec_info) {
            // info is a tuple of <lat (float), lon (float), category (int), velocity (uint8_t), yaw (uint8_t)>
            // RoadUser receives as params: <lat, lon, vel, yaw, cat>
            RoadUser r;
            r.latitude = static_cast<float>(std::get<0>(info));
            r.longitude = static_cast<float>(std::get<1>(info));
            r.error = 1.0; //0.0;
            r.speed = std::get<3>(info);
            r.orientation = std::get<4>(info);
            r.category = static_cast<Categories>(std::get<2>(info));
            r.camera_id.push_back(cam_ids[idx]); // camera id
            r.object_id.push_back(std::get<5>(info)); // tracker id
            r.idx = idx;
            r.idy = idy;

            message.objects.push_back(r);
            idy++;
        }
        input_messages.push_back(message);
        idx++;
    }

    // retrieving and preparing data from the cars by using a client udp socket at 18888
    char *ip = "192.168.12.4"; // TODO: deduplicator needs to be fixed to be executed in fog node 4
    int port = 18888;
    /*int socketDesc = socket(AF_INET, SOCK_DGRAM, 0);
    if (socketDesc != -1) { // created correctly
        printf("Created socket to %s:%d\n\n", ip, port);
        struct sockaddr_in server;
        server.sin_addr.s_addr = inet_addr(ip);
        server.sin_family = AF_INET;
        server.sin_port = htons(port);
        // connect to remote server
        if (connect(socketDesc, (struct sockaddr *)&server, sizeof(server)) >= 0) {
            MasaMessage *m = new MasaMessage();
            std::cout << "Receiving data from the cars with num objects: " << m->num_objects << std::endl;
            receive_message(socketDesc, m);
            if (m->num_objects > 0)
                input_messages.push_back(*m);
        }
    } else {
        std::cout << "Could not open udp socket for the car connection" << std::endl;
        // perror("error in socket");
    }*/

    Communicator<MasaMessage> *comm = new Communicator<MasaMessage>(SOCK_DGRAM);
    // comm->open_client_socket(ip, port);
    std::cout << "Opening server socket" << std::endl;
    comm->open_server_socket(port);

    int socketDesc = comm->get_socket();
    if (socketDesc != -1) {
        std::cout << "Server socket created" << std::endl;
        MasaMessage *m = new MasaMessage();
        std::cout << "Before receive_messages" << std::endl;
        // comm->receive_message(socketDesc, m);
        receive_message(*comm, m);
        std::cout << "Receiving data from the cars with num objects: " << m->num_objects << std::endl;
        if (m->num_objects > 0)
            input_messages.push_back(*m);
        close(socketDesc);
    } else {
        std::cout << "Could not open server socket in port " << port << std::endl;
    }

    std::cout << "Before elaborate messages" << std::endl;
    MasaMessage return_message;
    std::map<std::pair<uint16_t, uint16_t>, RoadUser> last_duplicated_objects;
    deduplicator.elaborateMessages(input_messages, return_message, last_duplicated_objects);
    std::cout << "After elaborate messages" << std::endl;

    // std::cout << "After deduplication" << std::endl;
    // camera_id (uint32_t), timestamp (uint64_t), tracker.id (int), tracker.cl (int), tracker.predList[-1].vel (float),
    // tracker.predList[-1].yaw (float), tracker.traj[-1].x (double), tracker.traj[-1].y (double) (which can be directly
    // converted to lat, lon)
    // std::vector<std::tuple<uint32_t, int, int, float, float, double, double, int, int, int, int>>
    //    info(deduplicator.t->getTrackers().size());
    int i = 0;
    double lat, lon, alt;
    float vel, yaw;
    int pixel_x, pixel_y, pixel_w, pixel_h;
    /*for (const tracking::Tracker& tracker : deduplicator.t->getTrackers()) {
        vel = yaw = 0.0f;
        lat = std::get<0>(input_deduplicator[tracker.idx_masa][tracker.idy_masa]);
        lon = std::get<1>(input_deduplicator[tracker.idx_masa][tracker.idy_masa]);
        pixel_x = std::get<6>(input_deduplicator[tracker.idx_masa][tracker.idy_masa]);
        pixel_y = std::get<7>(input_deduplicator[tracker.idx_masa][tracker.idy_masa]);
        pixel_w = std::get<8>(input_deduplicator[tracker.idx_masa][tracker.idy_masa]);
        pixel_h = std::get<9>(input_deduplicator[tracker.idx_masa][tracker.idy_masa]);
        // deduplicator.gc.enu2Geodetic(tracker.traj.back().x, tracker.traj.back().y, 0, &lat, &lon, &alt);
        if (tracker.predList.size() > 0) {
            vel = tracker.predList.back().vel;
            yaw = tracker.predList.back().yaw;
        }
        info[i++] = std::make_tuple(20939, tracker.id, tracker.cl, vel, yaw, lat, lon, pixel_x, pixel_y, pixel_w,
                                    pixel_h); // TODO: remove hardcoded CamId
    }*/
    // std::vector<std::tuple<std::vector<int>, std::vector<int>, int, double, double, double, double, int, int, int, int>>
    std::vector<std::tuple<int, int, int, double, double, double, double, int, int, int, int>>
            info(return_message.num_objects);
    // std::cout << "RETURN MESSAGE HAS " << return_message.num_objects << std::endl;
    std::cout << "Before loop" << std::endl;
    for (const RoadUser& ru : return_message.objects) {
        std::cout << "Car id is: " << ru.camera_id.at(0) << ", object type is " << int(ru.category) << std::endl;
        if (ru.camera_id.at(0) == 20 || ru.camera_id.at(0) == 21 || ru.camera_id.at(0) == 30 || ru.camera_id.at(0) == 31
                || ru.camera_id.at(0) == 32 || ru.camera_id.at(0) == 40) {
            // data coming from the cars have no bounding box
            std::cout << "Connected car tracked" << std::endl;
            pixel_x = pixel_y = pixel_w = pixel_h = 0;
        } else {
            // data coming from the cameras have bounding box
            pixel_x = std::get<6>(input_deduplicator[ru.idx][ru.idy]);
            pixel_y = std::get<7>(input_deduplicator[ru.idx][ru.idy]);
            pixel_w = std::get<8>(input_deduplicator[ru.idx][ru.idy]);
            pixel_h = std::get<9>(input_deduplicator[ru.idx][ru.idy]);
        }
        lat = std::get<0>(input_deduplicator[ru.idx][ru.idy]);
        lon = std::get<1>(input_deduplicator[ru.idx][ru.idy]);
        vel = deduplicator.uint8_to_speed(ru.speed);
        yaw = deduplicator.uint16_to_yaw(ru.orientation);
        // TODO: this -> info[i++] = std::make_tuple(ru.camera_id, ru.object_id, int(ru.category), vel, yaw, lat, lon, pixel_x, pixel_y,
        // TODO:     OR just first camera id and object id (instead of whole vectors)
        info[i++] = std::make_tuple(ru.camera_id.at(0), ru.object_id.at(0), int(ru.category), vel, yaw, lat, lon,
                                    pixel_x, pixel_y, pixel_w, pixel_h); // TODO: or should we return entire vectors
                                    // TODO: camera_id and object_id?

    }
    std::cout << "After loop" << std::endl;
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
                [](py::object self) {
                    return py::make_tuple(self.attr("latitude"), self.attr("longitude"), self.attr("orientation"),
                                          self.attr("time_to_change"), self.attr("status"));
                },
                [](const py::tuple &t) {
                    if (t.size() != 4)
                        throw std::runtime_error("Invalid state");
                    TrafficLight trafficLight;
                    trafficLight.latitude = t[0].cast<float>();
                    trafficLight.longitude = t[1].cast<float>();
                    trafficLight.orientation = t[2].cast<uint8_t>();
                    trafficLight.time_to_change = t[3].cast<uint8_t>();
                    trafficLight.status = t[4].cast<LightStatus>(); // TODO
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
                [](py::object self) {
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
                [](py::object self) {
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