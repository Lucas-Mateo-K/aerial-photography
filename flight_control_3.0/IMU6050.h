#ifndef IMU6050_H
#define IMU6050_H

#include <Arduino.h>

#define MPU 0x68

#define accScale 2 // 0 to 3
#define gyrScale 2 // 0 to 3

#define calibrationSamples 200

class MPU6050{
  public:

  float *AxP;
  float *AyP;
  float *AzP;
  float *AzNoCorrectP;
  float *GxP;
  float *GyP;
  float *GzP;

  void readMPU();
  void setupMPU();
  void accCalibration();
  void gyrCalibration();

  private:

  uint8_t ACCEL_FS_SEL;
  uint8_t GYRO_FS_SEL;

  float accScaleValue;
  float gyrScaleValue;

  bool accCal = false;
  bool gyrCal = false;

  float AxErr;
  float AyErr;
  float AzErr;
  float AzNoCorrectERR;
  float GxErr;
  float GyErr;
  float GzErr;

};

#endif