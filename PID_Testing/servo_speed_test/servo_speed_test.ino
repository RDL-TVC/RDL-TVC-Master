#include <Servo.h>


#define servo1_Pin 1
#define led_Pin 2
// The code will start with setting the servo to a neutral angle (0 deg) and wait
// a second. Then moves to the second angle and turns on a LED light on. 

Servo servo1;

const int inSensPin = 0;

const int p1 = 0; // intial angle of the serv
const int p2 = 10*.4825; // final angle of the servos


void setup() {
  pinMode(led_Pin,OUTPUT);
  servo1.attach(servo1_Pin);
  servo1.write(p1);
  delay(1000);
  servo1.write(p2);
  digitalWrite (led_Pin,HIGH);
  
}


void loop() {
  // put your main code here, to run repeatedly:
}
