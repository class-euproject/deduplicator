#include "LogWriter.h"

LogWriter::LogWriter(std::string fileSavingPath) {
    path = fileSavingPath;
}

std::string LogWriter::createSubDirectories() {
    std::string new_folder = path + std::to_string(tm_real_year);
    if(mkdir((const char *)new_folder.c_str(), 0777)){
        std::cout<<"create year folder\n";
    }
    new_folder = new_folder + '/' + std::to_string(tm_yday);
    if(mkdir((const char *)new_folder.c_str(), 0777)){
        std::cout<<"create day folder\n";
    }
    new_folder = new_folder + '/' + std::to_string(tm_hour);
    if(mkdir((const char *)new_folder.c_str(), 0777)){
        std::cout<<"create hour folder\n";
    }
    new_folder = new_folder + '/' + std::to_string(tm_min);
    if(mkdir((const char *)new_folder.c_str(), 0777)){
        std::cout<<"create min folder\n";
    }
    return new_folder + '/';
}

std::string LogWriter::getTimePath() {
    time(&rawtime);
    tm_struct = localtime(&rawtime);
    tm_year = tm_struct->tm_year;
    tm_real_year = tm_year + 1900;
    tm_hour = tm_struct->tm_hour;
    tm_yday = tm_struct->tm_yday;
    tm_min = tm_struct->tm_min;

    return createSubDirectories();
}

void LogWriter::write(MasaMessage m) {
    std::string filename = getTimePath() + std::to_string(m.cam_idx) + "txt";

    std::ofstream of;
    of.open(filename, std::ios_base::app);
    for(auto const& ru: m.objects) {
        of << m.cam_idx << " " << m.t_stamp_ms << " " << ru.category << " "
           << std::fixed << std::setprecision(9) << ru.latitude << " " << ru.longitude << " "
           << std::fixed << std::setprecision(2) << ru.speed << " " << ru.orientation << " ";
    }
    of.close();
}