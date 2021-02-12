/*
 * Modified from Arduino BN055 tutorial:
 * https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/arduino-code
 * 
 * File added by Rita Tagarao
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
Adafruit_BNO055 bno = Adafruit_BNO055(55);
/* Declare orientation array 
 * [check, xRad, yRad, zRad, accelX, accelY, accelZ, gyroX, gryoY, gyroZ, magX, magY, magZ] 
 */
float orient[13];
  
void setup(void) 
{
  
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
  Serial.printf("radX:%.2f",euler.x());
  Serial.printf("radY:%.2f",euler.y());
  Serial.printf("radZ:%.2f",euler.z());
  Serial.print("      ");
  orient[1] = euler.x();
  orient[2] = euler.y();
  orient[3] = euler.z();
  delay(100);

  /* Acceleration data */
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  /* Display the floating point data m/s^2*/
  Serial.printf("accelX:%.2f",accel.x());
  Serial.printf("accelY:%.2f",accel.y());
  Serial.printf("accelZ:%.2f",accel.z());
  Serial.print("      ");
  orient[4] = accel.x();
  orient[5] = accel.y();
  orient[6] = accel.z();
  delay(100);
  
  /* Gyro data */
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  /* Display the floating point data rad/sec*/
  Serial.printf("gyroX:%.2f",gyro.x());
  Serial.printf("gyroY:%.2f",gyro.y());
  Serial.printf("gyroZ:%.2f",gyro.y());
  Serial.print("      ");
  orient[7] = gyro.x();
  orient[8] = gyro.y();
  orient[9] = gyro.z();
  delay(100);

  /* Magnetometer data */
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  
  /* Display the floating point data micro Teslas*/
  Serial.printf("magX:%.2f",mag.x());
  Serial.printf("magY:%.2f",mag.y());
  Serial.printf("magZ:%.2f",mag.z());
  Serial.println("      ");
  orient[10] = mag.x();
  orient[11] = mag.y();
  orient[12] = mag.z();
  delay(100);
}
