#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <utility/quaternion.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int buzzerPin = 14;
unsigned int loopTime = 500; // Millis per loop

float yaw;
float pitch;
float roll;

float i; // Rad
float w; // Rad

unsigned long lastLoop = 0; //Millis when the last loop was executed

void setup() {
  //Test BNO055
  Serial.begin(9600);
  Serial.println("BNO055 Test");
  Serial.println("");
  if(!bno.begin(bno.OPERATION_MODE_NDOF, B00001001)) //bno.OPERATION_MODE_IMUPLUS
  {
    Serial.print("BNO055 failed to start");
    while(1);
  }
  
  bno.setExtCrystalUse(true);
  
//  pinMode(2,OUTPUT); //Pin 2 for BNO reset set as output

  uint8_t cal, gyro, accel, mag = 0;
  bno.getCalibration(&cal, &gyro, &accel, &mag);

  Serial.print("Calibrating BNO055: Sys = ");
  Serial.print(cal);
  Serial.print(", Gyro = ");
  Serial.print(gyro);
  Serial.print(", Accel = ");
  Serial.print(accel);
  Serial.print(", Mag = ");
  Serial.println(mag);
  tone(buzzerPin,1000,1000);

  while(cal != 3)
  {
    bno.getCalibration(&cal, &gyro, &accel, &mag);
    Serial.print("Calibrating BNO055: Sys = ");
    Serial.print(cal);
    Serial.print(", Gyro = ");
    Serial.print(gyro);
    Serial.print(", Accel = ");
    Serial.print(accel);
    Serial.print(", Mag = ");
    Serial.println(mag);
    delay(1000);
  }

 

  tone(buzzerPin,500,500);
  delay(500);
  tone(buzzerPin,1000,1000);

  Serial.println("BNO055 Calibrated and reset");
  
  delay(1000);
  lastLoop = millis();
  
}

void loop() {
  
  imu::Quaternion quat = bno.getQuat();
  
  /*
   * 
   * Using quaternion to euler ZXZ conversion.
   * Get inclination through the 2nd rotation
   * Get orientation of servos from 3rd rotation (Roll around actual centerline of rocket)
   * implementation as seen in MATLAB qparts2feul.m
   * 
   */

  double q[] = {quat.w(), quat.x(), quat.y(), quat.z()};

  // Inclination: angle from upwards x-axis
  // we want this to be 0, Use PID to do so
  i = acos(2 * (q[0]*q[0] + q[3]*q[3]) - 1); // Rad
  
  // angle of Z axis away from inertial XY plane measured around rotated frame X axis
  // needed to determine how servos will partition wanted angle.
  w = - atan2(2 * (q[0]*q[3] + q[1]*q[2]), 2 * (q[0]*q[2] - q[1]*q[3]));

  // Testing bno.getEvent, bno.get vector and how accurate the gravity vector is.

  sensors_event_t orientationData , angVelocityData , linearAccelData, gravityData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
  bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
  bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

  imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  // double grav[] = {g.x(), g.y(), g.z()};

  // i = acos(grav[0]/sqrt(grav[0] * grav[0] + grav[1] * grav[1] + grav[2] * grav[2]));

  Serial.print("Calculated from Raw: ");
  Serial.print("Inclination=");
  Serial.print(i * RAD_TO_DEG);
  Serial.print(", Roll=");
  Serial.print(w * RAD_TO_DEG);

  Serial.print(". Raw Quat: ");
  Serial.print(", w=");
  Serial.print(q[0]);
  Serial.print(", x=");
  Serial.print(q[1]);
  Serial.print(", y=");
  Serial.print(q[2]);
  Serial.print(", z=");
  Serial.println(q[3]);

  Serial.print("Raw Eul: ");
  Serial.print(", x=");
  Serial.print(eul[0]);
  Serial.print(", y=");
  Serial.print(eul[1]);
  Serial.print(", z=");
  Serial.println(eul[2]);
  
  Serial.print("Raw Gyro: ");
  Serial.print(", x=");
  Serial.print(gyro[0]);
  Serial.print(", y=");
  Serial.print(gyro[1]);
  Serial.print(", z=");
  Serial.println(gyro[2]);
  
  Serial.print("Raw Lin Accel: ");
  Serial.print(", x=");
  Serial.print(linAccel[0]);
  Serial.print(", y=");
  Serial.print(linAccel[1]);
  Serial.print(", z=");
  Serial.println(linAccel[2]);

  Serial.print("Raw G: ");
  Serial.print(", x=");
  Serial.print(g[0]);
  Serial.print(", y=");
  Serial.print(g[1]);
  Serial.print(", z=");
  Serial.println(g[2]);

  printEvent(&orientationData);
  printEvent(&angVelocityData);
  printEvent(&linearAccelData);
  printEvent(&gravityData);
  
  while ((millis() - lastLoop) < loopTime);
  lastLoop = millis();
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print(" x=");
  Serial.print(x);
  Serial.print(", y=");
  Serial.print(y);
  Serial.print(", z=");
  Serial.println(z);
}
