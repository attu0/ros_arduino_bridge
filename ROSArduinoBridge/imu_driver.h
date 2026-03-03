#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <Arduino.h>

void initIMU();
void readIMU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz);

#endif