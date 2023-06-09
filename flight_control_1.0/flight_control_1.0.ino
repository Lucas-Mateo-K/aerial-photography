#include <Wire.h>
#include "DShot.h"
#include <PID_v1.h>

#define MPU 0x68

#define accScale 3 // 0 to 3
#define gyrScale 3 // 0 to 3

#define calibrationSamples 500

#define gyrCorrection 0.08

#define M1 7
#define M2 6
#define M3 5
#define M4 3

DShot esc(DShot::Mode::DSHOT300);

uint16_t throttle1 = 0;
uint16_t target1 = 0;
uint16_t throttle2 = 0;
uint16_t target2 = 0;
uint16_t throttle3 = 0;
uint16_t target3 = 0;
uint16_t throttle4 = 0;
uint16_t target4 = 0;

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
    // Configure low pass filter
    // Set Digital Low Pass Filter about ~43Hz
  //Wire.beginTransmission(MPU);
  //Wire.write(0x1A);
  //Wire.write(0x03);
  //Wire.endTransmission();

  rollSET = 0;
  pitchSET = 0;
  altSET = 0;

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

  target1 = 100;

  while(throttle1 < 100){
    if (throttle1 < 48) {  // special commands disabled
    throttle1 = 48;
    }
    if (target1 <= 48) {
      esc.setThrottle(M1, target1, 0);
      esc.setThrottle(M2, target1, 0);
      esc.setThrottle(M3, target1, 0);
      esc.setThrottle(M4, target1, 0);
      
      if (target1 == 0) throttle1 = 48;
    } else {
      if (target1 > throttle1) {
        throttle1++;
        esc.setThrottle(M1, target1, 0);
        esc.setThrottle(M2, target1, 0);
        esc.setThrottle(M3, target1, 0);
        esc.setThrottle(M4, target1, 0);
        
      } else if (target1 < throttle1) {
        throttle1--;
        esc.setThrottle(M1, target1, 0);
        esc.setThrottle(M2, target1, 0);
        esc.setThrottle(M3, target1, 0);
        esc.setThrottle(M4, target1, 0);
        
      }
    }
  }

  previousTime = millis();
}

void loop() {
  readMPU();
  accAngle();
  gyrAngle();
  filterMPU();

  Serial.print("Cx:");
  Serial.println(CxAngle);
  Serial.print("Cy:");
  Serial.println(CyAngle);

  pidRoll.Compute();
  pidPitch.Compute();
  pidAlt.Compute();

  target = analogRead(A0);
  if (target>2047)
    target = 2047;
  if (throttle<48){
    throttle = 48;
  }
  if (target<=48){
        esc.setThrottle(M1, target, 0);
        esc.setThrottle(M2, target, 0);
        esc.setThrottle(M3, target, 0);
        esc.setThrottle(M4, target, 0);
  }else{
    if (target>throttle){
      throttle += 5;
        esc.setThrottle(M1, throttle, 0);
        esc.setThrottle(M2, throttle, 0);
        esc.setThrottle(M3, throttle, 0);
        esc.setThrottle(M4, throttle, 0);
    }else if (target<throttle){
      throttle -= 5;
        esc.setThrottle(M1, throttle, 0);
        esc.setThrottle(M2, throttle, 0);
        esc.setThrottle(M3, throttle, 0);
        esc.setThrottle(M4, throttle, 0);
    }
  }

  delay(5);
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
  
  accCal = true;
}

void gyrCalibration(){
  int i = 0;
  GxErr = 0;
  GyErr = 0;
  GzErr = 0;
  while(i < calibrationSamples){
    readMPU();
    GxErr = GxErr + Gx;
    GyErr = GyErr + Gy;
    GzErr = GzErr + Gz;
    i ++;
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
    AyAngle = asin((float)Ax / accTotal) * (180 / PI);
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
}