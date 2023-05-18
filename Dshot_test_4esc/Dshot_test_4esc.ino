#include "DShot.h"

/*
  redefine DSHOT_PORT if you want to change the default PORT
  Defaults
  UNO: PORTD, available pins 0-7 (D0-D7)
  Leonardo: PORTB, available pins 4-7 (D8-D11)
  e.g.
  #define DSHOT_PORT PORTD
*/


#define M1 4
#define M2 5
#define M3 6
#define M4 7

DShot esc(DShot::Mode::DSHOT300);

int throttle = 0;
int target = 0;

void setup() {
  Serial.begin(115200);

  // Notice, all pins must be connected to same PORT
  esc.attach(M1);
  esc.setThrottle(M1, throttle, 0);
  esc.attach(M2);
  esc.setThrottle(M2, throttle, 0);
  esc.attach(M3);
  esc.setThrottle(M3, throttle, 0);
  esc.attach(M4);
  esc.setThrottle(M4, throttle, 0);
}

void loop() {
  target = analogRead(A7);

    if (target > throttle) {
      throttle++;
      esc.setThrottle(M1, throttle, 0);
      Serial.print("throttle increas");
      Serial.println(throttle);
    }
    if (target < throttle) {
      throttle--;
      esc.setThrottle(M1, throttle, 0);
    }

  delay(10);
}