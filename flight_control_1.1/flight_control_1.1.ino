#include <Wire.h>
#include "DShot.h"
#include <PID_v1.h>

#define MPU 0x68

#define accScale 3 // 0 to 3
#define gyrScale 3 // 0 to 3

#define calibrationSamples 500

#define gyrCorrection 0.1
#define filter 0.8

// pid values for pitch and roll
//12, 5, 0.2
#define p 14
#define i 5
#define d 0.4

#define M1 3
#define M2 5
#define M3 6
#define M4 7

DShot esc(DShot::Mode::DSHOT300);

uint16_t throttle = 0;
uint16_t throttle1 = 0;
uint16_t throttle2 = 0;
uint16_t throttle3 = 0;
uint16_t throttle4 = 0;

uint8_t ACCEL_FS_SEL;
uint8_t GYRO_FS_SEL;

float accScaleValue;
float gyrScaleValue;

float Ax;
float Ay;
float Az;
float AzNoCorrect;

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

bool accCal = false;
bool gyrCal = false;

float elapsedTime;
float currentTime; 
float previousTime;

float accTotal;

float xIIR;
float yIIR;

double rollERR;
double pitchERR;
double altERR;

double rollCON;
double pitchCON;
double altCON;

double rollSET;
double pitchSET;
double altSET;

double KpRoll = p;
double KiRoll = i;
double KdRoll = d;

double KpPitch = p;
double KiPitch = i;
double KdPitch = d;

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

  pinMode(A0, INPUT);

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
  //Configure low pass filter
  //Set Digital Low Pass Filter about ~43Hz
  //Wire.beginTransmission(MPU);
  //Wire.write(0x1A);
  //Wire.write(0x03);
  //Wire.endTransmission();

  rollSET = 0;
  pitchSET = 0;
  altSET = 0;

  pidRoll.SetOutputLimits(-700, 700);
  pidPitch.SetOutputLimits(-700, 700);
  pidAlt.SetOutputLimits(-700, 700);

  pidRoll.SetSampleTime(10);
  pidPitch.SetSampleTime(10);
  pidAlt.SetSampleTime(10);

  pidRoll.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidAlt.SetMode(AUTOMATIC);

  accCalibration();
  gyrCalibration();

  esc.attach(M1);
  esc.setThrottle(M1, throttle1, 0);
  esc.attach(M2);
  esc.setThrottle(M2, throttle2, 0);
  esc.attach(M3);
  esc.setThrottle(M3, throttle3, 0);
  esc.attach(M4);
  esc.setThrottle(M4, throttle4, 0);
  delay(700);

  esc.setThrottle(M1, 100, 0);
  esc.setThrottle(M2, 100, 0);
  esc.setThrottle(M3, 100, 0);
  esc.setThrottle(M4, 100, 0);

  previousTime = millis();
}

void loop() {
  readMPU();
  accAngle();
  gyrAngle();
  filterMPU();
  
  pidRoll.Compute();
  pidPitch.Compute();
  pidAlt.Compute();

  throttle = analogRead(A0);
  if(throttle>2047){
    throttle = 2047;
  }

  throttle1 = max(0, throttle - (-rollCON) + (pitchCON));
  throttle2 = max(0, throttle - (-rollCON) - (pitchCON));
  throttle3 = max(0, throttle + (-rollCON) + (pitchCON));
  throttle4 = max(0, throttle + (-rollCON) - (pitchCON));

  if(throttle<5){
    throttle1 = 0;
    throttle2 = 0;
    throttle3 = 0;
    throttle4 = 0;
  }
/*
  Serial.print("motor1:");
  Serial.println(throttle1);
  Serial.print("motor2:");
  Serial.println(throttle2);
  Serial.print("motor3:");
  Serial.println(throttle3);
  Serial.print("motor4:");
  Serial.println(throttle4);
*/

  Serial.print("rollERR:");
  Serial.println(rollERR);
  //Serial.print("pitchERR:");
  //Serial.println(pitchERR);
  Serial.print("rollCON:");
  Serial.println(rollCON);
  //Serial.print("pitchCON:");
  //Serial.println(pitchCON);
  //Serial.print("AxAngle:");
  //Serial.println(AxAngle);
  //Serial.print("GxAngle:");
  //Serial.println(GxAngle);

/*
  Serial.print("p:");
  Serial.println(pidPitch.GetKp());
  Serial.print("i:");
  Serial.println(pidPitch.GetKi());
  Serial.print("d:");
  Serial.println(pidPitch.GetKd());
*/
  esc.setThrottle(M1, throttle1, 0);
  esc.setThrottle(M2, throttle2, 0);
  esc.setThrottle(M3, throttle3, 0);
  esc.setThrottle(M4, throttle4, 0);

  delay(50);
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
  AzNoCorrect = Az;

  if(accCal == true){
    Ax -= AxErr;
    Ay -= AyErr;
    Az -= AzErr;
  }

  if(gyrCal == true){
    Gx -= GxErr;
    Gy -= GyErr;
    Gz -= GzErr;
  }
}

void accCalibration(){
  int count = 0;
  AxErr = 0;
  AyErr = 0;
  AzErr = 0;
  while(count < calibrationSamples){
    readMPU();
    AxErr = AxErr + Ax;
    AyErr = AyErr + Ay;
    AzErr = AzErr + Az;
    count ++;
  }
  AxErr = AxErr/calibrationSamples;
  AyErr = AyErr/calibrationSamples;
  AzErr = AzErr/calibrationSamples;
  
  accCal = true;
}

void gyrCalibration(){
  int count = 0;
  GxErr = 0;
  GyErr = 0;
  GzErr = 0;
  while(count < calibrationSamples){
    readMPU();
    GxErr = GxErr + Gx;
    GyErr = GyErr + Gy;
    GzErr = GzErr + Gz;
    count ++;
  }
  GxErr = GxErr/calibrationSamples;
  GyErr = GyErr/calibrationSamples;
  GzErr = GzErr/calibrationSamples;
  gyrCal = true;
}

void accAngle(){
  accTotal = sqrt(pow(Ax, 2) + pow(Ay, 2) + pow(AzNoCorrect, 2));
  if (abs(Ay) < accTotal) {
    AxAngle = asin((float)Ay / accTotal) * (180 / PI);
  }
  if (abs(Ax) < accTotal) {
    AyAngle = -1 * (asin((float)Ax / accTotal) * (180 / PI));
  }
}

void gyrAngle(){
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;
  GxAngle = GxAngle + Gx * elapsedTime;
  GyAngle = GyAngle + Gy * elapsedTime;
}

void filterMPU(){
  xIIR = xIIR * (1-gyrCorrection) + (AxAngle - GxAngle) * gyrCorrection;
  yIIR = yIIR * (1-gyrCorrection) + (AyAngle - GyAngle) * gyrCorrection;
  CxAngle = GxAngle;
  CyAngle = GyAngle;
  CxAngle += xIIR;
  CyAngle += yIIR;
  rollERR = rollERR * (1-filter) + (CxAngle) * filter;
  pitchERR = pitchERR * (1-filter) + (CyAngle) * filter;
}