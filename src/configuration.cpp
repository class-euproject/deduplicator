#include "configuration.h"

/**
 * From the Yaml file we read an unique string. So we split it by the delimiter. 
*/
std::vector<std::string> getParamList(std::string s) {
    size_t pos = 0;
    std::vector<std::string> tokens; 
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
std::vector<int> strListToIntList(std::vector<std::string> s) {
    std::vector<int> converted;
    for(auto elem : s) {
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
    par->inputPortList = strListToIntList(getParamList(config["input_ports"].as<std::string>()));
    par->outputPortList = strListToIntList(getParamList(config["output_ports"].as<std::string>()));
    par->outputIpList = getParamList(config["output_ips"].as<std::string>());
    par->visualization = config["visualization"].as<bool>();
}

/* 
 * Parse the options from command line. Read the parameters from the configuration Yaml file.
 */
bool readParameters(int argc, char **argv, Parameters_t *param) {
    
    if (argc == 1) { //no parameter
        param->inputPortList.push_back(8888);
        param->outputPortList.push_back(9999);
        param->outputIpList.push_back("127.0.0.1");
        param->visualization = false;
        return true;
    }
    std::string help =  "class-aggregator demo\nCommand:\n"
                        "-i\tparameters file\n"
                        "or none - use default parameters";
    std::string param_file;
    int opt;
    while((opt = getopt(argc, argv, ":i:h")) != -1)
    {
        switch(opt)
        {
            case 'i':
                std::cout<<"input parameters file"<<std::endl;
                param_file = optarg;
                std::cout<<"file: "<<param_file<<std::endl;
                break;
            case 'h':
                std::cout<<"help"<<std::endl;
                std::cout<<help<<std::endl;
                return false;
            case ':':
                std::cerr<<"option needs a value\n";
                return false;
            case '?':
                std::cerr<<"unknown option: "<<optopt<<std::endl;
                return false;
            default:
                return false;
        }
    }
    readParametersYaml(param_file, param);
    return true;
}