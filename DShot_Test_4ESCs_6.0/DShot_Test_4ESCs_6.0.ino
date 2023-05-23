#include "DShot.h"

/*

  redefine DSHOT_PORT if you want to change the default PORT

  Defaults
  UNO: PORTD, available pins 0-7 (D0-D7)
  Leonardo: PORTB, available pins 4-7 (D8-D11)

  e.g.
  #define DSHOT_PORT PORTD
*/

#define M1 7
#define M2 6
#define M3 5
#define M4 3

DShot esc(DShot::Mode::DSHOT300);

uint16_t throttle = 0;
uint16_t target = 0;

void setup() {
  Serial.begin(115200);
  pinMode(A0, INPUT);

  // Notice, all pins must be connected to same PORT
  esc.attach(M1);
  esc.setThrottle(M1, throttle, 0);
  esc.attach(M2);
  esc.setThrottle(M2, throttle, 0);
  esc.attach(M3);
  esc.setThrottle(M3, throttle, 0);
  esc.attach(M4);
  esc.setThrottle(M4, throttle, 0);
  delay(700);

  esc.setThrottle(M1, 100, 0);
  esc.setThrottle(M2, 100, 0);
  esc.setThrottle(M3, 100, 0);
  esc.setThrottle(M4, 100, 0);

  delay(5000);

}

void loop() {

  target = analogRead(A0);
  if (target>2047){
    target = 2047;
  }
  esc.setThrottle(M1, target, 0);
  esc.setThrottle(M2, target, 0);
  esc.setThrottle(M3, target, 0);
  esc.setThrottle(M4, target, 0);
  
  delay(10);

}
