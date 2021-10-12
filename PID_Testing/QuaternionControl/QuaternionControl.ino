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

double total[2] = {0,0};
double lastErrors[2] = {0,0};

double PIDLastMill;
double PIDNextMill;
double dt;

int armingButton = 5;
int armingButton2 = 6;
int buttonCycles = 0;
int greenPin = 7;
int redPin = 8;

int piezoPin = 4;

void setup(void)
{
  servo_pitch.attach(SERVO_PIN_PITCH);
  servo_yaw.attach(SERVO_PIN_YAW);
  servo_pitch.writeMicroseconds(1500);
  servo_yaw.writeMicroseconds(1500);

  pinMode(greenPin,OUTPUT);
  pinMode(redPin,OUTPUT);
  
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

int fifthRun = 5;
imu:Quaternion upDir;

void loop(void)
{

    /* Angle data */
  imu::Quaternion quat = bno.getQuat();
  imu::Quaternion qInv = getInverse(quat);
  //Serial.printf("q = [%5f %5f %5f %5f]         ", quat.w(),quat.x(),quat.y(),quat.z());

  if (fifthRun > 0){
    fifthRun -= 1;
    upDir = quat;
  }

  quat -= upDir;

  double roll = 


  
  double rolVecServoUS = map(rolDeg, -60, 60, 900, 2100);
  double sidVecServoUS = map(sidDeg, -60, 60, 900, 2100);

  servo_pitch.writeMicroseconds(rolVecServoUS);
  servo_yaw.writeMicroseconds(sidVecServoUS);
  
  usArray[0] = rolVecServoUS;
  usArray[1] = sidVecServoUS;
}

void Proportional(double angleRol, double angleSid, double proComps[]){
  double pCoef = 0.7; // old value 0.6
  double errorRol = -angleRol;
  double errorSid = -angleSid;

  proComps[0] = errorRol * pCoef;
  proComps[1] = errorSid * pCoef;

  Serial.printf("Pitch: %f, Yaw: %f\n",proComps[0],proComps[1]);
  
}

void Integral(double angleRol, double angleSid, double total[], double intComps[]){
  double iCoef = 0.000; //set to 0.005 earlier
  double tConst = 1;
  double errorRol = -angleRol;
  double errorSid = -angleSid;

    //Serial.println((double)dt);
  if (((total[0]+errorRol*dt) < 100) && ((total[0]+errorRol*dt) > -100)){
    total[0] = total[0] + errorRol * dt;
  } else {
    total[0] = 60 * total[0]/abs(total[0]);
  }
  if (((total[1]+errorRol*dt) < 100) && ((total[1]+errorRol*dt) > -100)){
    total[1] = total[1] + errorSid * dt;
  } else {
    total[1] = 60 * total[1]/abs(total[1]);
  }

  intComps[0] = total[0] * iCoef * tConst;
  intComps[1] = total[1] * iCoef * tConst;

  Serial.printf("Total Rol: %f, Total Sid: %f, dt: %i\n", total[0], total[1], dt);
  
}

void Derivative(double angleRol, double angleSid, double lastErrors[], double derComps[]){
  double dCoef = 0; //0.15
  double tConst = 1000;
  double errorRol = -angleRol;
  double errorSid = -angleSid;

  double dErrRol = errorRol-lastErrors[0];
  double dErrSid = errorSid-lastErrors[1];

  derComps[0] = (dErrRol/dt) * dCoef * tConst;
  derComps[1] = (dErrSid/dt) * dCoef * tConst;
  
  Serial.printf("Time: %f, Change Rol: %f, Change Sid: %f, Order Rol: %f, Order Sid: %f\n",(double)dt,dErrRol,dErrSid,derComps[0],derComps[1]);

  lastErrors[0] = errorRol;
  lastErrors[1] = errorSid;
  
}
