#ifndef CALCULATEDOF_H
#define CALCULATEDOF_H

#include <Arduino.h>

#define gyrCorrection 0.009
#define filter 0.99

class calculateDOF{
  public:
  void accAngle();
  void gyrAngle();
  void calcDOF();
  void setPreviousTime();
};

#endif