#include <Wire.h>
#include "DShot.h"

// pid values for pitch and roll
//8, 0, 22

#define p 20
#define i 0.05
#define d 60

#define MPU 0x68

#define accScale 2 // 0 to 3
#define gyrScale 2 // 0 to 3

#define calibrationSamples 200

#define gyrCorrection 0.006
#define filter 0.8

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

double roll;
double pitch;

bool getPitch = false;

uint8_t ACCEL_FS_SEL;
uint8_t GYRO_FS_SEL;

float accScaleValue;
float gyrScaleValue;

float Ax;
float Ay;
float Az;
float AzNoCorrect;
float accTotal;

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

float xIIR;
float yIIR;

#define pidRoll 0
#define pidPitch 1
#define pidAlt 2
#define constrainPid 500
#define constrainIntegral 600

double previousError[3];
double integral[3];
double derivative[3];

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

void setup() {
  Serial.begin(115200);

  pinMode(8, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

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
  delay(1000);

  previousTime = millis();
}

void loop() {
  readMPU();
  accAngle();
  gyrAngle();
  filterMPU();

/*
  if(Serial.available() > 0){
    int g = KdRoll;
    KdRoll = Serial.parseInt();
    if(KdRoll <= 0){
      KdRoll = g;
    }
  }
*/

  noInterrupts();
  throttle = pulseIn(8, HIGH);
  interrupts();
  throttle = min(1842, throttle);
  throttle = max(1117, throttle);
  throttle = map(throttle, 1117, 1842, 0, 2047);

/*
  if(getPitch == false){
  noInterrupts();
  roll = pulseIn(A0, HIGH);
  interrupts();
  roll = constrain(roll, 1117, 1842);
  roll = map(roll, 1117, 1842, -10, 10);
  getPitch = true;
  }

  else{
  noInterrupts();
  pitch = pulseIn(A1, HIGH);
  interrupts();
  pitch = constrain(pitch, 1117, 1842);
  pitch = map(pitch, 1117, 1842, -10, 10);
  getPitch = false;
  }

  Serial.print("roll:");
  Serial.println(roll);
  */

  if(throttle > 50){
    PID(pidRoll, rollERR, &rollCON, rollSET  - roll, KpRoll, KiRoll, KdRoll);
    PID(pidPitch, pitchERR, &pitchCON, pitchSET + pitch, KpPitch, KiPitch, KdPitch);
  }

  throttle1 = max(0, throttle - (-rollCON) + (pitchCON));
  throttle2 = max(0, throttle - (-rollCON) - (pitchCON));
  throttle3 = max(0, throttle + (-rollCON) + (pitchCON));
  throttle4 = max(0, throttle + (-rollCON) - (pitchCON));

  if(throttle < 50){
    throttle1 = 0;
    throttle2 = 0;
    throttle3 = 0;
    throttle4 = 0;
    integral[0] = 0;
    integral[1] = 0;
    integral[2] = 0;
  }

  //Serial.print("motor1:");
  //Serial.println(throttle1);
  //Serial.print("motor2:");
  //Serial.println(throttle2);
  //Serial.print("motor3:");
  //Serial.println(throttle3);
  //Serial.print("motor4:");
  //Serial.println(throttle4);

  Serial.print("rollERR:");
  Serial.println(rollERR);
  //Serial.print("pitchERR:");
  //Serial.println(pitchERR);
  Serial.print("rollCON:");
  Serial.println(rollCON);
  //Serial.print("pitchCON:");
  //Serial.println(pitchCON);
  //Serial.print("AyAngle:");
  //Serial.println(AyAngle);
  //Serial.print("GyAngle:");
  //Serial.println(GyAngle);

  esc.setThrottle(M1, throttle1, 0);
  esc.setThrottle(M2, throttle2, 0);
  esc.setThrottle(M3, throttle3, 0);
  esc.setThrottle(M4, throttle4, 0);

}

void PID(int pidSelect, double ERR, double *CON, double SET, double Kp, double Ki, double Kd){
  double error = ERR - SET;
  integral[pidSelect] += error;
  integral[pidSelect] = constrain(integral[pidSelect], (-1 * constrainIntegral), constrainIntegral);
  derivative[pidSelect] = error - previousError[pidSelect];
  previousError[pidSelect] = error;
  *CON = -1 * ((Kp * error) + (Ki * integral[pidSelect]) + (Kd * derivative[pidSelect]));
  *CON = constrain(*CON, (-1 * constrainPid), constrainPid);
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
