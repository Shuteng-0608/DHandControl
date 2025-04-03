#include "MPD20Control.h"

MPD20Controller servo(Serial);

void setup() {
  servo.InitServo();
  servo.setDeviceID(0, 1);
//  servo.setDeviceID(1, 3);
  
}

void loop() {
  servo.setPosition(1, 500, 90);
  delay(5000);
  servo.setPosition(1, 200, 90);
  delay(5000);
}
