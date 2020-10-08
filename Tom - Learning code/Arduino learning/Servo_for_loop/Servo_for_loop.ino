#include <Servo.h>

#define SERVO_PIN 4

Servo myservo;



void setup()
{
  // put your setup code here, to run once:
  myservo.attach(SERVO_PIN);
}

void loop() {
  // put your main code here, to run repeatedly:
  myservo.write(0);
  delay(1000);
  myservo.write(180);
  delay(1000);
    
    //for(int pos = 0; pos < 180; pos ++)
   // {
    //  myservo.write(pos);
    //  delay(15);
    //}
  //servo_FB.write(60);
  //servo_LR.write(50);
  //delay(2000);
  //servo_FB.write(120);
 // servo_LR.write(150);
  //delay(2000);
}
