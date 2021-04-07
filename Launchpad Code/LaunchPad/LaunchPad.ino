#include <Servo.h>

#define S1_PIN 1 //Defines pins that servos are attached to
#define S2_PIN 2
#define S3_PIN 3
#define S4_PIN 4

#define POS1 0 //Defines the two positions the servos will move between
#define POS2 180

#define IGNITIONSENSE_PIN 0

Servo s1; //Servos 1 through 4 correspond to the numbers marked on the launch pad
Servo s2;
Servo s3;
Servo s4;

void setup() {

  pinMode(IGNITIONSENSE_PIN, INPUT);

  Serial.begin(115200);

  s1.attach(S1_PIN); //Attach servos
  s2.attach(S2_PIN);
  s3.attach(S3_PIN);
  s4.attach(S4_PIN);
  Serial.println("Attached Servos");

  s1.write(POS1); //Set servos to starting position
  s2.write(POS1);
  s3.write(POS1);
  s4.write(POS1);
  Serial.println("Position 1");

  delay(1000);
  Serial.println("Waiting for Ignition...");
}

void loop() {

//Serial.println(digitalRead(IGNITIONSENSE_PIN));

  if (digitalRead(IGNITIONSENSE_PIN) == HIGH) //Tests if pin has been set to high (continuity is broken on the ignition sensing wire)
  {
    s1.write(POS2); //Move servos to open launch clamps
    s2.write(POS2);
    s3.write(POS2);
    s4.write(POS2);
    Serial.println("Position 2");
  }
  
//  else
//  {
//    s1.write(POS1);
//    s2.write(POS1);
//    s3.write(POS1);
//    s4.write(POS1);
//    Serial.println("Position 1");
//  }

  delay(1);
  
}
