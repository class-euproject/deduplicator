#include "LogWriter.h"

LogWriter::LogWriter(std::string fileSavingPath) {
    path = fileSavingPath;
}

std::string LogWriter::createSubDirectories() {
    std::string new_folder = path + std::to_string(tmRealYear);
    if(!mkdir((const char *)new_folder.c_str(), 0777)){
        std::cout<<"create year folder\n";
    }
    new_folder = new_folder + separator() + std::to_string(tmYday);
    if(!mkdir((const char *)new_folder.c_str(), 0777)){
        std::cout<<"create day folder\n";
    }
    new_folder = new_folder + separator() + std::to_string(tmHour);
    if(!mkdir((const char *)new_folder.c_str(), 0777)){
        std::cout<<"create hour folder\n";
    }
    return new_folder + separator();
}

std::string LogWriter::getTimePath() {
    time(&rawtime);
    tmStruct = localtime(&rawtime);
    tmYear = tmStruct->tm_year;
    tmRealYear = tmYear + 1900;
    tmHour = tmStruct->tm_hour;
    tmYday = tmStruct->tm_yday;
    tmMin = tmStruct->tm_min;
    return createSubDirectories() + std::to_string(tmMin);
}

void LogWriter::write(MasaMessage m) {
    std::string filename = getTimePath() + "_" + std::to_string(m.cam_idx) + ".txt";
    std::ofstream of;
    of.open(filename, std::ios_base::app);
    for(auto const& ru: m.objects) {
        of << m.cam_idx << " " << m.t_stamp_ms << " " << ru.category << " "
           << std::fixed << std::setprecision(9) << ru.latitude << " " << ru.longitude << " "
           << std::fixed << std::setprecision(2) << static_cast<float>(ru.speed) << " " << static_cast<float>(ru.orientation) << "\n";
    }
    of.close();
}