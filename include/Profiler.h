#ifndef PROFILER_H
#define PROFILER_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <map>
#include <iomanip>
#include <chrono>
#include <pthread.h>

#define MAX_MIN 9999999

extern pthread_mutex_t profiler_mutex;

namespace fog {

struct stats{
    std::chrono::time_point<std::chrono::system_clock> start;
    std::vector<double> diff;
    unsigned long int count = 0; //total count
    double oMin = MAX_MIN;       //overall max
    double oMax = 0;             //overall min
    double oAvg = 0;             //overall average
};

class Profiler{
    std::string name;
    std::map<std::string,stats> timers;
    
public:
    Profiler(std::string profiler_name)  {
        name = profiler_name;
    }
    
    ~Profiler() {}
    void tick(std::string timer_name);

    void tock(std::string timer_name);

    void printStats(int interval = 100);
};

}
#endif /*PROFILER_H*/
