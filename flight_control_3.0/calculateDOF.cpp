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
  /**
  if (abs(Ay) < accTotal) {
    AxAngle = asin((float)Ay / accTotal) * (180 / PI);
  }
  if (abs(Ax) < accTotal) {
    AyAngle = -1 * (asin((float)Ax / accTotal) * (180 / PI));
  }
  **/
  AxAngle = atan2(Ay, AzNoCorrect) * (180 / PI);
  AyAngle = atan2(Ax, AzNoCorrect) * (180 / PI);

  Serial.print("AxAngle:");
  Serial.println(AxAngle);
  Serial.print("AyAngle:");
  Serial.println(AyAngle);
  Serial.print("accTotal:");
  Serial.println(accTotal);
  
}

void calculateDOF::gyrAngle(){
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;
  GxAngle += Gx * elapsedTime;
  GyAngle += Gy * elapsedTime;
  GzAngle += Gz * elapsedTime;
  /**
  Serial.print("Gz:");
  Serial.println(Gz);
  Serial.print("elapsedTime:");
  Serial.println(elapsedTime);
  Serial.print("GzAngle:");
  Serial.println(GzAngle);
  **/
}

void calculateDOF::calcDOF(){
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
