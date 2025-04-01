#include "MicroServoControl.h"

MicroServoController servo(Serial);

void setup() {
  servo.InitServo();
//  servo.setDeviceID(1, 3);
  
}

void loop() {
  servo.setPosition(1, 500);
  delay(5000);
  servo.setPosition(1, 1000);
  delay(5000);
//  servo.ParameterSave(3);
//  delay(5000);
}
