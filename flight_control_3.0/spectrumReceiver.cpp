#include "main.h"
#include "spectrumReceiver.h"

bool getPitch = false;

void getThrottel(){
  noInterrupts();
  throttle = pulseIn(8, HIGH);
  interrupts();
  throttle = min(1842, throttle);
  throttle = max(1117, throttle);
  throttle = map(throttle, 1117, 1842, 0, 2047);
}

void SETRollPitchYaw(){
  if(getPitch == false){
  noInterrupts();
  roll = pulseIn(A0, HIGH);
  interrupts();
  roll = constrain(roll, 1117, 1842);
  roll = map(roll, 1117, 1842, -10, 10);
  getPitch = true;
  }

  else{
  noInterrupts();
  pitch = pulseIn(A1, HIGH);
  interrupts();
  pitch = constrain(pitch, 1117, 1842);
  pitch = map(pitch, 1117, 1842, -10, 10);
  getPitch = false;
  }
}