#include "main.h"
#include "calculateDOF.h"

float AxAngle;
float AyAngle;

float GxAngle;
float GyAngle;
float GzAngle;

float CxAngle;
float CyAngle;
float CzAngle;

float elapsedTime;
float currentTime; 
float previousTime;

float xAngleDrift;
float yAngleDrift;

float accTotal;

float thrust;

float xSpeed;
float ySpeed;
float zSpeed;

void calculateDOF::accAngle(){
  accTotal = sqrt(pow(Ax, 2) + pow(Ay, 2) + pow(AzNoCorrect, 2));
  if (abs(Ay) < accTotal) {
    AxAngle = asin((float)Ay / accTotal) * (180 / PI);
  }
  if (abs(Ax) < accTotal) {
    AyAngle = -1 * (asin((float)Ax / accTotal) * (180 / PI));
  }
}

void calculateDOF::gyrAngle(){
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0000;
  GxAngle += Gx * elapsedTime;
  GyAngle += Gy * elapsedTime;
  GzAngle += Gz * elapsedTime;
  //Serial.print("GxAngle:");
  //Serial.println(GxAngle);
  //Serial.print("GyAngle:");
  //Serial.println(GyAngle);
}

void calculateDOF::calcDOF(){
  accAngle();
  gyrAngle();
  xAngleDrift = xAngleDrift * (1-gyrCorrection) + (AxAngle - GxAngle) * gyrCorrection;
  yAngleDrift = yAngleDrift * (1-gyrCorrection) + (AyAngle - GyAngle) * gyrCorrection;
  CxAngle = GxAngle;
  CyAngle = GyAngle;
  CxAngle += xAngleDrift;
  CyAngle += yAngleDrift;
  roll = roll * (1-filter) + (CxAngle) * filter;
  pitch = pitch * (1-filter) + (CyAngle) * filter;
  yaw = GzAngle;
  thrust = accTotal * weight * gravity;
  //xSpeed = elapsedTime * ((thrust * sin(roll / (180 / PI))) / weight);
  //x += pow(elapsedTime, 2) * Ax;
}

void calculateDOF::setPreviousTime(){
  previousTime = millis();
}
