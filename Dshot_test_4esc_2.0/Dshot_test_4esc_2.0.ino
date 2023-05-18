#include "DShot.h"

/*
  redefine DSHOT_PORT if you want to change the default PORT
  Defaults
  UNO: PORTD, available pins 0-7 (D0-D7)
  Leonardo: PORTB, available pins 4-7 (D8-D11)
  e.g.
  #define DSHOT_PORT PORTD
*/

#define DSHOT_PORT PORTD

#define M1 4
#define M2 5
#define M3 6
#define M4 7

DShot esc(DShot::Mode::DSHOT300);

int throttle = 0;
int target = 0;

void setup() {
  Serial.begin(500000);

  // Notice, all pins must be connected to same PORT
  esc.attach(M1);
  esc.setThrottle(throttle);

  while(throttle < 800){
    esc.setThrottle(throttle);
    Serial.println(throttle);
    throttle ++;
    delay(2);
  }
  while(throttle > 0){
    esc.setThrottle(throttle);
    Serial.println(throttle);
    throttle --;
    delay(2);
  }  
}

void loop() {
  esc.setThrottle(throttle);
  Serial.println(throttle);
  if(throttle < 1500){
  throttle ++;
  }
  delay(100);
}