#include <PID_v1.h>
#include <Wire.h>

#define MPU 0x68
#define accScale 3 // 0 to 3
#define gyrScale 0 // 0 to 3
#define calibrationSamples 500

uint8_t ACCEL_FS_SEL;
uint8_t GYRO_FS_SEL;

float accScaleValue;
float gyrScaleValue;

float Ax;
float Ay;
float Az;

float Gx;
float Gy;
float Gz;

float AxErr;
float AyErr;
float AzErr;

float GxErr;
float GyErr;
float GzErr;

float AxAngle;
float AyAngle;

float GxAngle;
float GyAngle;
float GzAngle;

float CxAngle;
float CyAngle;
float CzAngle;

float accTotal;

double rollERR;
double pitchERR;
double altERR;

double rollCON;
double pitchCON;
double altCON;

double rollSET;
double pitchSET;
double altSET;

double KpRoll;
double KiRoll;
double KdRoll;

double KpPitch;
double KiPitch;
double KdPitch;

double KpAlt;
double KiAlt;
double KdAlt;

int motor1;
int motor2;
int motor3;
int motor4;

PID pidRoll(&rollERR, &rollCON, &rollSET, KpRoll, KiRoll, KdRoll, DIRECT);
PID pidPitch(&pitchERR, &pitchCON, &pitchSET, KpPitch, KiPitch, KdPitch, DIRECT);
PID pidAlt(&altERR, &altCON, &altSET, KpAlt, KiAlt, KdAlt, DIRECT);

void setup() {
  Serial.begin(115200);

  switch(accScale){
    case 1:
      ACCEL_FS_SEL = 0x08;
      accScaleValue = 32767/4;
    break;
    case 2:
      ACCEL_FS_SEL = 0x10;
      accScaleValue = 32767/8;
    break;
    case 3:
      ACCEL_FS_SEL = 0x18;
      accScaleValue = 32767/16;
    break;
    default:
      ACCEL_FS_SEL = 0x00;
      accScaleValue = 32767/2;
    break;
  };

  switch(gyrScale){
    case 1:
      GYRO_FS_SEL = 0x08;
      gyrScaleValue = 32767/500;
    break;
    case 2:
      GYRO_FS_SEL = 0x10;
      gyrScaleValue = 32767/1000;
    break;
    case 3:
      GYRO_FS_SEL = 0x18;
      gyrScaleValue = 32767/2000;
    break;
    default:
      GYRO_FS_SEL = 0x00;
      gyrScaleValue = 32767/250;
    break;
  };

  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  
  Wire.write(0);    
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);  
  Wire.write(ACCEL_FS_SEL);    
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  
  Wire.write(GYRO_FS_SEL);    
  Wire.endTransmission(true);

  rollSET = 0;
  pitchSET = 0;
  altSET = 0;

  pidRoll.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidAlt.SetMode(AUTOMATIC);

  accCalibration();
}

void loop() {
  pidRoll.Compute();
  pidPitch.Compute();
  pidAlt.Compute();
  readMPU();
  accAngle();
  Serial.print("Variable_1:");
  Serial.println(Ax);
  Serial.print("Variable_2:");
  Serial.println(AxErr);
  //Serial.print("Variable_4:");
  //Serial.println(Az);
  delay(100);
}

void readMPU(){
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  Gx = Wire.read() << 8 | Wire.read();
  Gy = Wire.read() << 8 | Wire.read();
  Gz = Wire.read() << 8 | Wire.read();
  Gx = Gx/gyrScaleValue;
  Gy = Gy/gyrScaleValue;
  Gz = Gz/gyrScaleValue;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  Ax = Wire.read() << 8 | Wire.read();
  Ay = Wire.read() << 8 | Wire.read();
  Az = Wire.read() << 8 | Wire.read();
  Ax = Ax/accScaleValue;
  Ay = Ay/accScaleValue;
  Az = Az/accScaleValue;
}

void filterMPU(){

}

void accAngle(){
  accTotal = sqrt(pow(Ax, 2) + pow(Ay, 2) + pow(Az, 2));
  if (abs(Ay) < accTotal) {
      AyAngle = asin((float)Ay / accTotal) * (180 / PI);
  }
  if (abs(Ax) < accTotal) {
      AxAngle = asin((float)Ax / accTotal) * (180 / PI);
  }
}

void gyrAngle(){

}

void accCalibration(){
  int i = 0;
  AxErr = 0;
  AyErr = 0;
  AzErr = 0;
  while(i < calibrationSamples){
    readMPU();
    AxErr = AxErr + Ax;
    AyErr = AyErr + Ay;
    AzErr = AzErr + Az;
    i ++;
  }
  AxErr = AxErr/calibrationSamples;
  AyErr = AyErr/calibrationSamples;
  AzErr = AzErr/calibrationSamples;
  
}

void gyrCalibration(){
  
}