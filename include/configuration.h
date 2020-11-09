#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <iostream>
#include <string>
#include <algorithm>
#include <yaml-cpp/yaml.h>
#include <getopt.h>

namespace fog {

struct Parameters_t {
    std::vector<int> inputPortList;
    std::vector<int> outputPortList;
    std::vector<std::string> outputIpList;
    int camIdx;
    bool visualization;
    bool edge_log_saving;
    bool aggr_log_saving;
    std::string tifFile;
    std::string pngFile;
};

std::string delimiter = ", ";       // The delimiter chosen to divide the elements in the Yaml file
bool readParameters(int argc, char **argv, Parameters_t *param);
}

#endif /* CONFIGURATION_H */