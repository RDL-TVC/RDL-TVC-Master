#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <utility/quaternion.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int buzzerPin = 14;

float yaw;
float pitch;
float roll;

float i; // Rad
float w; // Rad


void setup() {
//Test BNO055
  Serial.begin(9600);
  Serial.println("BNO055 Test");
  Serial.println("");
  if(!bno.begin(bno.OPERATION_MODE_NDOF)) //bno.OPERATION_MODE_IMUPLUS   bno.OPERATION_MODE_NDOF
  {
    Serial.print("BNO055 failed to start");
    while(1);
  }
  
  bno.setExtCrystalUse(true);

//  pinMode(2,OUTPUT); //Pin 2 for BNO reset set as output

  uint8_t cal, gyro, accel, mag = 0;
  bno.getCalibration(&cal, &gyro, &accel, &mag);

  Serial.print("Calibrating BNO055  ");
  Serial.print(cal);
  Serial.print("  ");
  Serial.print(gyro);
  Serial.print("  ");
  Serial.print(accel);
  Serial.print("  ");
  Serial.println(mag);

  tone(buzzerPin,1000,1000);

  while(cal != 3)
  {
    bno.getCalibration(&cal, &gyro, &accel, &mag);
    Serial.print("Calibrating BNO055  ");
    Serial.print(cal);
    Serial.print("  ");
    Serial.print(gyro);
    Serial.print("  ");
    Serial.print(accel);
    Serial.print("  ");
    Serial.println(mag);
    delay(1000);
  }

  tone(buzzerPin,500,500);
  delay(500);
  tone(buzzerPin,1000,1000);

  Serial.println("BNO055 Calibrated and reset");
  
  delay(1000);
  
}

void loop() {
  //imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  //imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion quat = bno.getQuat();
  
  /*
   * 
   * Using quaternion to euler XZX conversion.
   * Get inclination through the 2nd rotation
   * Get orientation of servos from 3rd rotation
   * implementation as seen in MATLAB qparts2feul.m
   * 
   */

  q[0] = quat.w();
  q[1] = quat.x();
  q[2] = quat.y();
  q[3] = quat.z();
  
  // Inclination: angle from upwards x-axis
  // we want this to be 0, Use PID to do so
  i = acos(2 * (q[0]**2 + q[1]**2) - 1);
  
  // angle of Z axis away from inertial ZY plane measured on rotated ZY frame
  // needed to determine how servos will partition wanted angle.
  w = atan2(2 * (q[0]*q[2] + q[3]*q[1]), 2 * (q[0]*q[3] - q[1]*q[2]));
  
 

  yaw = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
  pitch = asin(2*(q[0]*q[2]-q[3]*q[1]));
  roll = atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));  

  Serial.print("yaw: ");
  Serial.print(yaw * RAD_TO_DEG);
  Serial.print("    pitch: ");
  Serial.print(pitch * RAD_TO_DEG);
  Serial.print("    roll: ");
  Serial.print(roll * RAD_TO_DEG);

  Serial.print("    Inclination: ");
  Serial.print(i * RAD_TO_DEG);
  Serial.print("    Omega: ");
  Serial.print(w * RAD_TO_DEG);

  
  Serial.print("  w: ");
  Serial.print(q[0]);
  Serial.print("  x: ");
  Serial.print(q[1]);
  Serial.print("  y: ");
  Serial.print(q[2]);
  Serial.print("  z: ");
  Serial.println(q[3]);
}
