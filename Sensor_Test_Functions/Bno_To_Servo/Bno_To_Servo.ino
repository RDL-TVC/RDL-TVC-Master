#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

#define SERVO_PIN_PITCH 0
#define SERVO_PIN_YAW 1

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Servo servo_pitch;
Servo servo_yaw;

float pitch;
float yaw;

// Declare orientation array: [check, degX, degY, degZ] 
float orient[] = {0,0,0,0};
float orient_cur[3] = {0,0,0};
float orient_last[3] = {0,0,0};
  
void setup(void) 
{
  servo_pitch.attach(SERVO_PIN_PITCH);
  servo_yaw.attach(SERVO_PIN_YAW);
  servo_pitch.writeMicroseconds(1500);
  servo_yaw.writeMicroseconds(1500);
  
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
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
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  /* Display the floating point data of degrees*/
  orient_last[0] = orient_cur[0];
  orient_last[1] = orient_cur[1];
  orient_last[2] = orient_cur[2];
  
  orient_cur[0] = euler.z(); //Roll 180 >> -180
  orient_cur[1] = euler.y(); //Pitch 90 >> -90 (2 zeros)
  orient_cur[2] = euler.x(); //Yaw 0 >> 360
  
  //Roll -infinity >> infinity
  if (orient_cur[0] <= -160 && orient_last[0] >= 160)
  {
    orient[1] = orient[1] + (orient_cur[0] + 360 - orient_last[0]);
  }
  else if (orient_cur[0] >= 160 && orient_last[0] <= -160) 
  {
    orient[1] = orient[1] + (orient_cur[0] - 360 - orient_last[0]);
  }
  else  
  {
    orient[1] = orient[1] + (orient_cur[0] - orient_last[0]);
  }

  //Pitch -infinity >> infinity

  //Yaw -infinity >> infinity
  if (orient_cur[2] >= 340 && orient_last[2] <= 20)
  {
    orient[3] = orient[3] + (orient_cur[2] - 360 - orient_last[2]);
  }
  else if (orient_cur[2] <= 20 && orient_last[2] >= 340) 
  {
    orient[3] = orient[3] + (orient_cur[2] + 360 - orient_last[2]);
  }
  else  
  {
    orient[3] = orient[3] + (orient_cur[2] - orient_last[2]);
  }
    
  orient[2] = orient[2] + (orient_cur[1] - orient_last[1]);
  
  
Serial.printf("Roll:%5.2f",orient[1]);
  Serial.printf("   Pitch:%5.2f",orient[2]);
  Serial.printf("   Yaw:%5.2f",orient[3]);
  Serial.print("      ");
  Serial.printf("   Pitch Rad/s :%5.2f",gyro.y());
  Serial.print("      ");

//  if(orient[2] < 90 || orient[2] > 270) //if Pitch < 90deg magnitude
//  {
//    pitch = orient[2];
//    servo_pitch.write(pitch);
//    Serial.printf("ServoPitch:%5.2f", pitch);
//    Serial.print("     ");
//  }

  if(orient[2] <= 90 && orient[2] >= -90)
  {
    pitch = map(orient[2], -90, 90, 1200, 1800);
    servo_pitch.writeMicroseconds(pitch);
  }

//  if(orient[3] <= 90 && orient[3] >= -90)
//  {
//    yaw = map(orient[3], , 90, 1200, 1800);
//    servo_pitch.writeMicroseconds(yaw);
//  }

//   if(orient[3] < 90 || orient[3] > 270) //if Yaw < 45deg magnitude
//  {
//    
//    
//    yaw = orient[3];
//    servo_yaw.write(yaw);
//    Serial.printf("ServoYaw:%5.2f", yaw);
//  }

  Serial.printf("\n");

}
