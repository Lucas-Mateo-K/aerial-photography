#include "main.h"
#include "PID.h"

double previousError[pidLoops];
double integral[pidLoops];
double derivative[pidLoops];

void PID(int pidSelect, double ERR, double *CON, double SET, double Kp, double Ki, double Kd){
  double error = ERR - SET;
  integral[pidSelect] += error;
  integral[pidSelect] = constrain(integral[pidSelect], (-1 * constrainIntegral), constrainIntegral);
  derivative[pidSelect] = error - previousError[pidSelect];
  previousError[pidSelect] = error;
  *CON = -1 * ((Kp * error) + (Ki * integral[pidSelect]) + (Kd * derivative[pidSelect]));
  *CON = constrain(*CON, (-1 * constrainPid), constrainPid);
}

void resetIntegral(int pidLoop){
  integral[pidLoop] = 0;
}