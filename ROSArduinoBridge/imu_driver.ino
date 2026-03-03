/***************************************************************
   IMU driver definitions

   MPU6050 support using Adafruit library
***************************************************************/

#ifdef USE_IMU

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

/* Initialize IMU */
void initIMU() {
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("IMU_INIT_FAIL");
    while (1) delay(10);
  }
}

/* Read IMU values */
void readIMU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz) {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;

  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
}

#endif