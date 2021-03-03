#ifndef LOGWRITER_H
#define LOGWRITER_H

#include <iostream>
#include <assert.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <fstream>
#include <iomanip>
#include "../masa_protocol/include/messages.hpp"

using namespace masa;

namespace fog {
    
inline char separator()
{
#ifdef _WIN32
    return '\\';
#else
    return '/';
#endif
}

class LogWriter {

private:
    std::string path;   //"./demo/data/class_fog_log/"
    struct stat st = {0};
    time_t rawtime;
    struct tm *tmStruct;
    int tmYear, tmRealYear, tmHour, tmYday, tmMin;

    std::string createSubDirectories();
    std::string getTimePath();

public:
    pthread_mutex_t mutex;
    pthread_t writerThread;

    LogWriter(std::string fileSavingPath);
    ~LogWriter() {};
    void write(MasaMessage m);

};
}


#endif /* LOGWRITER_H */