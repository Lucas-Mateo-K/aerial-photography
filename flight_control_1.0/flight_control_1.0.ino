#include <PID_v1.h>

float Ax;
float Ay;
float Az;

float Gx;
float Gy;
float Gz;

float rollERR;
float pitchERR;
float altERR;

float rollCON;
float pitchCON;
float altCON;

float rollSET;
float pitchSET;
float altSET;

int motor1;
int motor2;
int motor3;
int motor4;

void setup() {
PID pidRoll(&rollERR, &rollCON, &rollSET);
PID pidPitch(&pitchERR, &pitchCON, &pitchSET);
PID pidalt(&altERR, &altCON, &altSET);

}

void loop() {

}
