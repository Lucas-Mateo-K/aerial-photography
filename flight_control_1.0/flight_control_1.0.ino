#include <PID_v1.h>
#include<Wire.h>

const int MPU=0x68;

float Ax;
float Ay;
float Az;

float Gx;
float Gy;
float Gz;

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
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  
  Wire.write(0);    
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);  
  Wire.write(0x18);    
  Wire.endTransmission(true);
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);  
  Wire.write(0x18);    
  Wire.endTransmission(true);
  RollSET = 0;
  pitchSET = 0;
  altSET = 0;
  pidRoll.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidAlt.SetMode(AUTOMATIC);
}

void loop() {
  pidRoll.Compute();
  pidPitch.Compute();
  pidAlt.Compute();
}

void readMPU(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0;
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0;
}

