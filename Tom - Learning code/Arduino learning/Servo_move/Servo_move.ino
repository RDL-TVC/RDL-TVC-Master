#include <Servo.h>

#define SERVO_PIN_LR 4
#define SERVO_PIN_FB 5

Servo servo_LR;
Servo servo_FB;

void setup() {
  // put your setup code here, to run once:
  servo_LR.attach(SERVO_PIN_LR);
  servo_FB.attach(SERVO_PIN_FB);
  servo_FB.write(90);
  servo_LR.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:

  servo_FB.write(60);
  servo_LR.write(50);
  delay(2000);
  servo_FB.write(120);
  servo_LR.write(150);
  delay(2000);
}
