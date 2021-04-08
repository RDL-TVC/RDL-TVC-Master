#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

#define SERVO_PIN_PITCH 0
#define SERVO_PIN_YAW 1
#define RAD_TO_DEG 57.295779513082320876798154814105

Adafruit_BNO055 bno = Adafruit_BNO055(55);

Servo servo_pitch;
Servo servo_yaw;

float pitch;
float yaw;

// Declare orientation array: [check, degX, degY, degZ]
float orient[] = {0, 0, 0, 0};
float orient_cur[3] = {0, 0, 0};
float orient_last[3] = {0, 0, 0};

double usArray[2];

void setup(void)
{
  servo_pitch.attach(SERVO_PIN_PITCH);
  servo_yaw.attach(SERVO_PIN_YAW);
  servo_pitch.writeMicroseconds(1500);
  servo_yaw.writeMicroseconds(1500);

  Serial.begin(9600);
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  orient[0] = 1;
  delay(1000);
  bno.setExtCrystalUse(true);


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

  while (cal != 3)
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

  int upDirStartTime = millis();
  int upDirCurrTime = millis();
  int remainingTime;
  
  while (upDirCurrTime-upDirStartTime <= 5000){

    remainingTime = 5000-(upDirCurrTime-upDirStartTime);
    Serial.println("picking up direction in:");
    Serial.println(remainingTime);
    upDirCurrTime = millis();
    delay(100);
    
  }
  Serial.println("Direction Chosen!");
}

imu::Quaternion getInverse(imu::Quaternion q) {
  imu::Quaternion conj;
  conj.w() = q.w();
  conj.x() = -q.x();
  conj.y() = -q.y();
  conj.z() = -q.z();

  double norm = sqrt(q.w() * q.w() + q.x() * q.x() + q.y() * q.y() + q.z() * q.z());

  imu::Quaternion inv;

  inv.w() = conj.w() / norm;
  inv.x() = conj.x() / norm;
  inv.y() = conj.y() / norm;
  inv.z() = conj.z() / norm;

  return inv;
}

int firstRun = 1;
double DCM[3][3];
double newRollVec1z;
double newRollVec2z;

void loop(void)
{
  /* Angle data */
  imu::Quaternion quat = bno.getQuat();
  imu::Quaternion qInv = getInverse(quat);
  //Serial.printf("q = [%5f %5f %5f %5f]         ", quat.w(),quat.x(),quat.y(),quat.z());

  //original i vector (towards nosecone due to how bno055 is oriented)
  imu::Quaternion pt1; //vector to this point on a unit sphere from origin
  pt1.w() = 0;
  pt1.x() = 1;
  pt1.y() = 0;
  pt1.z() = 0;

  //original k
  imu::Quaternion pt2;
  pt2.w() = 0;
  pt2.x() = 0;
  pt2.y() = 0;
  pt2.z() = 1;

  //original j
  imu::Quaternion pt3;
  pt3.w() = 0;
  pt3.x() = 0;
  pt3.y() = 1;
  pt3.z() = 0;

  imu::Quaternion dir = quat * pt1 * qInv;
  imu::Quaternion rollVec1 = quat * pt2 * qInv;
  imu::Quaternion rollVec2 = quat * pt3 * qInv;

  if (firstRun == 1){
    firstRun = firstRun - 1;
    
    /*DCM[0][0] = rollVec2.x();
    DCM[1][0] = rollVec2.y();
    DCM[2][0] = rollVec2.z();
    DCM[0][1] = rollVec1.x();
    DCM[1][1] = rollVec1.y();
    DCM[2][1] = rollVec2.z();
    DCM[0][2] = dir.x();
    DCM[1][2] = dir.y();
    DCM[2][2] = dir.z();*/

    /*imu::Quaternion newDir = dir;
    imu::Quaternion newRoll1 = rollVec1;
    imu::Quaternion newRoll2 = rollVec2;*/
    
  }
  
  PIDFunction(rollVec1, rollVec2, dir, usArray, DCM);
  
  Serial.printf("%f   %f   %f   %f   %f   %f\n", dir.x(), dir.y(), dir.z(), rollVec1.x(), rollVec1.y(), rollVec1.z());

  /*double newDirX = DCM[0][0] * dir.x() + DCM[0][1] * dir.y() + DCM[0][2] * dir.z();
  double newDirY = DCM[1][0] * dir.x() + DCM[1][1] * dir.y() + DCM[1][2] * dir.z();
  double newDirZ = DCM[2][0] * dir.x() + DCM[2][1] * dir.y() + DCM[2][2] * dir.z();
  Serial.printf("%f   %f   %f\n", newDirX, newDirY, newDirZ);*/
  
  //Serial.printf("%f   %f\n", usArray[0], usArray[1]);
}

//argument: double DCM[][3]

void PIDFunction(imu::Quaternion rollVec1, imu::Quaternion rollVec2, imu::Quaternion dir, double usArray[], double DCM[][3]){
  
  /*newRollVec1z = DCM[2][0] * rollVec1.x() + DCM[2][1] * rollVec1.y() + DCM[2][2] * rollVec1.z();
  newRollVec2z = DCM[2][0] * rollVec2.x() + DCM[2][1] * rollVec2.y() + DCM[2][2] * rollVec2.z();*/

  /*errorDir = newDir - dir;
  errorRol1 = newRoll1 - rollVec1;
  errorRol2 = newRoll2 - rollVec2;*/
  
  double rolVecServoAngle = asin(rollVec1.z());
  double sidVecServoAngle = asin(rollVec2.z());

  //Serial.printf("%f   %f\n", rolVecServoAngle, sidVecServoAngle);

  double rolDeg = rolVecServoAngle * RAD_TO_DEG;
  double sidDeg = sidVecServoAngle * RAD_TO_DEG;

  double rolVecServoUS = map(rolDeg, -90, 90, 900, 2100);
  double sidVecServoUS = map(sidDeg, -90, 90, 900, 2100);

  servo_pitch.writeMicroseconds(rolVecServoUS);
  servo_yaw.writeMicroseconds(sidVecServoUS);

  usArray[0] = rolVecServoUS;
  usArray[1] = sidVecServoUS;
}
