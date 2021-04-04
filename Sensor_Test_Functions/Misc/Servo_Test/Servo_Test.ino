/*
#include <Servo.h>

#define SERVO_PIN_LR 0
#define SERVO_PIN_FB 1

Servo servo_LR;
Servo servo_FB;

int i = 0;

void setup() {
  // put your setup code here, to run once:
  servo_LR.attach(SERVO_PIN_LR);
  servo_FB.attach(SERVO_PIN_FB);
  Serial.begin(9600); // send and receive at 9600 baud rate
}


void loop() 
{
  for(i = 0; i <= 50; i++)
  {
    servo_FB.write(90 - i);
    servo_LR.write(40 + i);
    Serial.print(90 - i);
    Serial.print("     ");
    Serial.println(40 + i);
    delay(10);
  }  
  Serial.println(" ");
  Serial.println("First Phase Done");
  Serial.println(" ");
  for(i = 0; i <= 50; i++)
  {
    servo_FB.write(40 + i);
    servo_LR.write(90 + i);
    Serial.print(40 + i);
    Serial.print("     ");
    Serial.println(90 + i);
    delay(10);
  }
  Serial.println(" ");
  Serial.println("Second Phase Done");
  Serial.println(" ");
  for(i = 0; i <= 50; i++)
  {
    servo_FB.write(90 + i);
    servo_LR.write(140 - i);
    Serial.print(90 + i);
    Serial.print("     ");
    Serial.println(140 - i);
    delay(10);
  }
  Serial.println(" ");
  Serial.println("Third Phase Done");
  Serial.println(" ");
  for(i = 0; i <= 50; i++)
  {
    servo_FB.write(140 - i);
    servo_LR.write(90 - i);
    Serial.print(140 - i);
    Serial.print("     ");
    Serial.println(90 - i);
    delay(10);
  }
  Serial.println(" ");
  Serial.println("Fourth Phase Done");
  Serial.println(" ");
  delay(100);

//https://www.google.com/search?q=arduino+print+to+console&rlz=1C1CHBF_enUS830US830&oq=arduino+print+&aqs=chrome.1.69i57j0l7.5312j0j7&sourceid=chrome&ie=UTF-8
  

}
*/
