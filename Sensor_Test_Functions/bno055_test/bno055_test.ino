#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <utility/quaternion.h>

#define SERVO_PIN_PITCH 0
#define SERVO_PIN_YAW 1

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int buzzerPin = 14;

//Servo s_pitch
//Servo s_yaw

float yaw;
float pitch;
float roll;
float q[4] = {0,0,0,0};
float yaw_last; //used for detecting yaw jump


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

//  digitalWrite(2,HIGH);
//  delay(1000);
//  digitalWrite(2,LOW);

  Serial.println("BNO055 Calibrated and reset");

//Attach and center servos
//  s_pitch.attach(SERVO_PIN_PITCH);
//  s_yaw.attach(SERVO_PIN_YAW);
//  s_pitch.writeMicroseconds(1500);
//  s_yaw.writeMicroseconds(1500);
//  Serial.println("Servos attached and centered");

  delay(1000);
  
}

void loop() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Quaternion quat = bno.getQuat();
//  imu::Vector<3> eul = quat.toEuler();

  
//Yaw, pitch, roll

//  yaw = eul[0];
//  pitch = eul[1];
//  roll = eul[2];

  q[0] = quat.w();
  q[1] = quat.x();
  q[2] = quat.y();
  q[3] = quat.z();

  yaw = atan2(2*(q[0]*q[3]+q[1]*q[2]),1-2*(q[2]*q[2]+q[3]*q[3]));
  pitch = asin(2*(q[0]*q[2]-q[3]*q[1]));
  roll = atan2(2*(q[0]*q[1]+q[2]*q[3]),1-2*(q[1]*q[1]+q[2]*q[2]));  

  Serial.print("yaw: ");
  Serial.print(yaw);
  Serial.print("    pitch: ");
  Serial.print(pitch);
  Serial.print("    roll: ");
  Serial.print(roll);
  
  Serial.print("  w: ");
  Serial.print(q[0]);
  Serial.print("  x: ");
  Serial.print(q[1]);
  Serial.print("  y: ");
  Serial.print(q[2]);
  Serial.print("  z: ");
  Serial.println(q[3]);

}
