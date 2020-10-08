#include <Servo.h>

#define SERVO_PIN_LR 4
#define SERVO_PIN_FB 5

Servo servo_LR;
Servo servo_FB;

int i = 0;

void setup() {
  // put your setup code here, to run once:
  servo_LR.attach(SERVO_PIN_LR);
  servo_FB.attach(SERVO_PIN_FB);
}

void loop() {
  // put your main code here, to run repeatedly:
  servo_FB.write(40);
  servo_LR.write(90);
  
}
