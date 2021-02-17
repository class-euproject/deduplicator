#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <sys/time.h>
#include <gdal_priv.h>
#include <gdal/gdal.h>
#include <cmath>

extern bool gRun;
extern bool V3D;

void sig_handler(int signal);
void readTiff(char *filename, double *adfGeoTransform);
unsigned long long time_in_ms();
uint8_t orientation_to_uint8(float yaw);
uint8_t speed_to_uint8(float vel);
void GPS2pixel(double *adfGeoTransform, double lat, double lon, int &x, int &y);

#endif /* UTILS_H */