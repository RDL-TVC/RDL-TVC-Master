#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

#define SERVO_PIN_LR 0
#define SERVO_PIN_FB 1

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Servo servo_LR;
Servo servo_FB;

float x;
float y;

// Declare orientation array: [check, degX, degY, degZ] 
float orient[4];
  
void setup(void) 
{
  servo_LR.attach(SERVO_PIN_LR);
  servo_FB.attach(SERVO_PIN_FB);
  
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  orient[0] = 0;
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  } 
  orient[0] = 1;
  delay(1000);   
  bno.setExtCrystalUse(true);
}

void loop(void) 
{
  /* Angle data */
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  
  /* Display the floating point data of degrees*/
  orient[1] = euler.x();
  orient[3] = euler.y(); 
  orient[2] = euler.z(); 
  Serial.printf("degX:%5.2f",orient[1]);
  Serial.printf("   degY:%5.2f",orient[2]);
  Serial.printf("   degZ:%5.2f",orient[3]);
  Serial.print("      ");

  if(orient[1] < 5 || orient[1] > -5) //if x < 5deg magnitude
  {
    x = orient[1];
    servo_LR.write(x);
    Serial.printf("ServoX:%5.2f", x);
    Serial.print("     ");
  }

   if(orient[2] < 5 || orient[2] > -5) //if y < 5deg magnitude
  {
    y = orient[2];
    servo_FB.write(90-y);
    Serial.printf("ServoY:%5.2f", y);
  }

  Serial.printf("\n");
  delay(100);

}
