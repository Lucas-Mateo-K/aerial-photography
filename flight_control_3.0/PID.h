#ifndef PID_H
#define PID_H

#include <Arduino.h>

#define pidLoops 3
#define constrainPid 500
#define constrainIntegral 600

void PID(int pidSelect, double ERR, double *CON, double SET, double Kp, double Ki, double Kd);
void resetIntegral(int pidLoop);

#endif