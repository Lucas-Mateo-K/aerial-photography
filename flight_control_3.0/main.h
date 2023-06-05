#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>
#include <Wire.h>

//weight in grams
#define weight 680

//gravity in m/s/s
#define gravity 9.8066

extern double roll;
extern double pitch;
extern double yaw;

extern double x;
extern double y;
extern double z;

extern float Ax;
extern float Ay;
extern float Az;
extern float AzNoCorrect;
extern float Gx;
extern float Gy;
extern float Gz;

extern uint16_t throttle;
extern uint16_t throttle1;
extern uint16_t throttle2;
extern uint16_t throttle3;
extern uint16_t throttle4;

extern double rollCON;
extern double pitchCON;
extern double altCON;

extern double rollSET;
extern double pitchSET;
extern double altSET;

#endif