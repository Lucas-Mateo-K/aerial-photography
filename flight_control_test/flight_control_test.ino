#include <Wire.h>

#include <PID_v1.h>

#define MPU 0x68

#define accScale 3 // 0 to 3
#define gyrScale 3 // 0 to 3

#define calibrationSamples 500

#define gyrCorrection 0.08

// pid values for pitch and roll
#define p 0.01
#define i 0.01
#define d 0.01

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
    // Configure low pass filter
    // Set Digital Low Pass Filter about ~43Hz
  //Wire.beginTransmission(MPU);
  //Wire.write(0x1A);
  //Wire.write(0x03);
  //Wire.endTransmission();

  rollSET = 0;
  pitchSET = 0;
  altSET = 0;

  pidRoll.SetOutputLimits(-300, 300);
  pidPitch.SetOutputLimits(-300, 300);
  pidAlt.SetOutputLimits(-300, 300);

  pidRoll.SetSampleTime(1);

  pidRoll.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidAlt.SetMode(AUTOMATIC);

  accCalibration();
  gyrCalibration();


  previousTime = millis();
}

void loop() {
  readMPU();
  accAngle();
  gyrAngle();
  filterMPU();

  rollERR = CxAngle;
  pitchERR = CyAngle;

  pidRoll.Compute();
  pidPitch.Compute();
  pidAlt.Compute();


  Serial.print("output:");
  Serial.println(rollCON);
  Serial.print("input:");
  Serial.println(rollERR);
  Serial.print("setpoint:");
  Serial.println(rollSET);

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