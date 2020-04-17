#ifndef LOGWRITER_H
#define LOGWRITER_H

#include <assert.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <fstream>
#include <iomanip>
#include "../masa_protocol/include/messages.hpp"

class LogWriter {

private:
    std::string path;   //"./demo/data/class_fog_log/"
    struct stat st = {0};
    time_t rawtime;
    struct tm *tm_struct;
    int tm_year, tm_real_year, tm_hour, tm_yday, tm_min;

    std::string createSubDirectories();
    std::string getTimePath();

public:
    // Message *m;
    int message_count;
    pthread_mutex_t mutex;
    pthread_t writer_thread;

    LogWriter(std::string fileSavingPath);
    ~LogWriter() {};
    void write(MasaMessage m);

};


#endif /* LOGWRITER_H */