#include "Deduplicator.h"
#include "../include/Deduplicator.h"
#include <ctime>
#include <tuple>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

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

// std::vector<tracking::Tracker>
// compute_deduplicator(std::vector<std::vector<tracking::Tracker>> &input_deduplicator) {
//std::vector<std::tuple<uint32_t, uint64_t, int, int, float, float, double, double>>
std::tuple<uint64_t, std::vector<std::tuple<uint32_t, int, int, float, float, double, double, int, int, int, int>>>
        compute_deduplicator(std::vector<std::vector<std::tuple<double, double, int, uint8_t, uint8_t, int, int, int,
                             int, int>>> &input_deduplicator) {
                                    //, std::vector<uint32_t> cam_ids, std::vector<uint64_t> timestamps) {
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
    for (const auto& vec_info : input_deduplicator) {
        MasaMessage message;
        idy = 0;
        for (auto info : vec_info) {
            // deduplicator.create_message_from_tracker(tracker_list, &message);
            // info is a tuple of <lat (float), lon (float), category (int), velocity (uint8_t), yaw (uint8_t)>
            // RoadUser receives as params: <lat, lon, vel, yaw, cat>
            RoadUser r{static_cast<float>(std::get<0>(info)), static_cast<float>(std::get<1>(info)),
                       static_cast<uint8_t>(std::get<3>(info)), std::get<4>(info),
                       static_cast<Categories>(std::get<2>(info)), static_cast<int>(idx),  static_cast<int>(idy)};

            // TODO: check if correct
            // TODO: message.t_stamp_ms = timestamps[idx];
            // TODO: message.cam_idx = cam_ids[idx];
            message.objects.push_back(r);
            idy++;
        }
        input_messages.push_back(message);
        idx++;
    }
    MasaMessage return_message;
    deduplicator.computeDeduplication(input_messages, return_message);
    std::cout << "After deduplication" << std::endl;
    // camera_id (uint32_t), timestamp (uint64_t), tracker.id (int), tracker.cl (int), tracker.predList[-1].vel (float),
    // tracker.predList[-1].yaw (float), tracker.traj[-1].x (double), tracker.traj[-1].y (double) (which can be directly
    // converted to lat, lon)
    std::vector<std::tuple<uint32_t, int, int, float, float, double, double, int, int, int, int>>
        info(deduplicator.t->getTrackers().size());
    int i = 0;
    double lat, lon, alt;
    float vel, yaw;
    int pixel_x, pixel_y, pixel_w, pixel_h;
    // TODO: add new field idx to Tracker to point to the exact input message object in order to retrieve lat/lon &
    //  avoid unneeded conversions
    for (const tracking::Tracker& tracker : deduplicator.t->getTrackers()) {
        vel = yaw = 0.0f;
        // TODO: store XXX and YYY in RoadUser and tracker structure so it can be matched to initial position (also previous TODO)
        lat = std::get<0>(input_deduplicator[tracker.idx_masa][tracker.idy_masa]);
        lon = std::get<1>(input_deduplicator[tracker.idx_masa][tracker.idy_masa]);
        // TODO: tracker.id from input_deduplicator or the one provided in tracker?
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
    }
    std::cout << "After creating info" << std::endl;
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