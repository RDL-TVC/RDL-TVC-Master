#include <Servo.h>


#define servo1_Pin 1
#define servo2_Pin 2
#define servo3_Pin 3
#define servo4_Pin 4

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

const int inSensPin = 0;

int ingSens = HIGH;
const int p1 = -45; // intial angle of the serv
const int p2 = 45; // final angle of the servos
// these create 4 objects that are servos named servo1, servo2,...

void setup() {
  pinMode(inSensPin,INPUT);
  
  servo1.attach(servo1_Pin);
  servo2.attach(servo2_Pin);
  servo3.attach(servo3_Pin);
  servo4.attach(servo4_Pin);

  servo1.write(p1);
  servo2.write(p1);
  servo3.write(p1);
  servo4.write(p1);
}

void loop() {
  // put your main code here, to run repeatedly:
 ingSens = digitalRead(inSensPin); //pin 5 is connected to the trip wire

  if (ingSens == LOW){
    servo1.write(p2);
    servo2.write(p2);
    servo3.write(p2);
    servo4.write(p2);
  }
}
