#include <utils.h>


bool gRun;
bool V3D;

void sig_handler(int signal) {
    std::cout << "request gateway stop\n";
    gRun = false;
}

void readTiff(char *filename, double *adfGeoTransform) {
    GDALDataset *poDataset;
    GDALAllRegister();
    poDataset = (GDALDataset *)GDALOpen(filename, GA_ReadOnly);
    if (poDataset != NULL)
    {
        poDataset->GetGeoTransform(adfGeoTransform);
    }
}

unsigned long long time_in_ms() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    unsigned long long t_stamp_ms = (unsigned long long)(tv.tv_sec) * 1000 + (unsigned long long)(tv.tv_usec) / 1000;
    return t_stamp_ms;
}

/**
 *  Convert orientation from radian to quantized degree (from 360 to 255)
*/
uint8_t orientation_to_uint8(float yaw)
{
    // 57.29 is (180/pi) -> conversion in degrees
    // 17 / 24 is (255/360) -> quantization
    // let a yaw in radians, it is converted into degrees (*57.29), 
    // then into positive degrees, then it is quantized into 255. 
    uint8_t orientation = uint8_t((int((yaw * 57.29 + 360)) % 360) * 17 / 24);
    return orientation;
}

/**
 *  Convert speed from m/s to quantized km/h every 1/2 km/h
*/
uint8_t speed_to_uint8(float vel)
{
    // 3.6 -> conversion in km/h
    // *2 -> quantization km/h evrey 1/2
    // let a velocity in m/s, it is converted into km/h (*3.6), then (*2) 
    // we achive a double speed. In a urban track we can consider a maximum 
    // speed of 127 km/h. So we can fit 127 on a byte with a multiplication 
    // by 2. Each increment corresponds to a speed greater than 1/2 km/h.
    uint8_t velocity = uint8_t(std::abs(vel * 3.6 * 2));
    return velocity;
}  

void GPS2pixel(double *adfGeoTransform, double lat, double lon, int &x, int &y){
    //conversion from GPS to pixels, via georeferenced map parameters
    x = int(round( (lon - adfGeoTransform[0]) / adfGeoTransform[1]) );
    y = int(round( (lat - adfGeoTransform[3]) / adfGeoTransform[5]) );
}