#include "main.h"
#include "IMU6050.h"
#include "calculateDOF.h"
#include "PID.h"
#include "spectrumReceiver.h"
#include "DShot.h"

// pid values for pitch and roll
// 8, 0, 22

#define gyrCorrection 0.006
#define filter 0.8

#define M1 3
#define M2 5
#define M3 6
#define M4 7

MPU6050 MPU6050;
calculateDOF calculateDOF;

DShot esc(DShot::Mode::DSHOT300);

#define pidRoll 0
#define pidPitch 1
#define pidAlt 2

double roll;
double pitch;
double yaw;

double x;
double y;
double z;

float Ax;
float Ay;
float Az;
float AzNoCorrect;
float Gx;
float Gy;
float Gz;

uint16_t throttle = 0;
uint16_t throttle1 = 0;
uint16_t throttle2 = 0;
uint16_t throttle3 = 0;
uint16_t throttle4 = 0;

double rollCON;
double pitchCON;
double altCON;

double rollSET;
double pitchSET;
double altSET;

#define KpRotation 20
#define KiRotation 0.05
#define KdRotation 60

void setup() {
  Serial.begin(115200);

  MPU6050.setupMPU();

  pinMode(8, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  rollSET = 0;
  pitchSET = 0;
  altSET = 0;

  MPU6050.accCalibration();
  MPU6050.gyrCalibration();

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

  calculateDOF.setPreviousTime();
}

void loop() {
  MPU6050.readMPU();
  calculateDOF.accAngle();
  calculateDOF.gyrAngle();
  calculateDOF.calcDOF();

  //getThrottel();

  if(throttle > 50){
    PID(pidRoll, roll, &rollCON, rollSET, KpRotation, KiRotation, KdRotation);
    PID(pidPitch, pitch, &pitchCON, pitchSET, KpRotation, KiRotation, KdRotation);
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
    resetIntegral(pidRoll);
    resetIntegral(pidPitch);
  }
/**
  Serial.print("roll:");
  Serial.println(roll);
  Serial.print("pitch:");
  Serial.println(pitch);
  Serial.print("yaw:");
  Serial.println(yaw);

  Serial.print("X:");
  Serial.println(x);
  Serial.print("Y:");
  Serial.println(y);
  Serial.print("Z:");
  Serial.println(z);
**/
  //Serial.print("Ax:");
  //Serial.println(Ax);
  //Serial.print("Y:");
  //Serial.println(y);
  //Serial.print("Z:");
  //Serial.println(z);

  //Serial.print("motor1:");
  //Serial.println(throttle1);
  //Serial.print("motor2:");
  //Serial.println(throttle2);
  //Serial.print("motor3:");
  //Serial.println(throttle3);
  //Serial.print("motor4:");
  //Serial.println(throttle4);

  //Serial.print("rollCON:");
  //Serial.println(rollCON);
  //Serial.print("pitchCON:");
  //Serial.println(pitchCON);
  //Serial.print("AyAngle:");
  //Serial.println(AyAngle);
  //Serial.print("GyAngle:");
  //Serial.println(GyAngle);

  //esc.setThrottle(M1, throttle1, 0);
  //esc.setThrottle(M2, throttle2, 0);
  //esc.setThrottle(M3, throttle3, 0);
  //esc.setThrottle(M4, throttle4, 0);
}


