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
double proComps[2];
double intComps[2];
double derComps[2];

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

  /*while(true){
    if(digitalRead(armingButton) == HIGH && digitalRead(armingButton2) == HIGH){
      break;
    }
  }*/
  
  /* Initialise the sensor */
  if (!bno.begin(bno.OPERATION_MODE_NDOF, B00001001))
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

  tone(piezoPin,4000,1000);
  digitalWrite(greenPin,HIGH);

  while(true){
      if(digitalRead(armingButton) == HIGH && digitalRead(armingButton2) == HIGH) {
        ++buttonCycles;
        Serial.println(buttonCycles);
        delay(10);
        if (buttonCycles >= 500) {  //hold down both buttons for 5s
          Serial.printf("Rocket armed: Startup-->Groundidle\n");
          tone(piezoPin,4000,1000);
          break;
        }
      }
   }
  
  digitalWrite(greenPin,LOW);

  int upDirStartTime = millis();
  int upDirCurrTime = millis();
  int remainingTime;
  
  /*while (upDirCurrTime-upDirStartTime <= 5000){

    remainingTime = 5000-(upDirCurrTime-upDirStartTime);
    Serial.println("picking up direction in:");
    Serial.println(remainingTime);
    upDirCurrTime = millis();
    delay(100);
    
  }*/
  
  tone(piezoPin,4000,1000);
  digitalWrite(redPin,HIGH);
  delay(2000);
  
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

int fifthRun = 5;
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

  if (fifthRun > 0){
    fifthRun -= 1;
    
    DCM[0][0] = quat.w()*quat.w() + quat.x()*quat.x() - quat.y()*quat.y() - quat.z()*quat.z();
    DCM[0][1] = 2 * (quat.x()*quat.y() + quat.z()*quat.w());
    DCM[0][2] = 2 * (quat.x()*quat.z() - quat.y()*quat.w());
    DCM[1][0] = 2 * (quat.x()*quat.y() - quat.z()*quat.w());
    DCM[1][1] = quat.w()*quat.w() - quat.x()*quat.x() + quat.y()*quat.y() - quat.z()*quat.z();
    DCM[1][2] = 2 * (quat.y()*quat.z() + quat.x()*quat.w());
    DCM[2][0] = 2 * (quat.x()*quat.z() + quat.y()*quat.w());
    DCM[2][1] = 2 * (quat.y()*quat.z() - quat.x()*quat.w());
    DCM[2][2] = quat.w()*quat.w() - quat.x()*quat.x() - quat.y()*quat.y() + quat.z()*quat.z();

    PIDLastMill = millis();
  }

  PIDNextMill = millis();
  dt = PIDNextMill-PIDLastMill;
  PIDLastMill = PIDNextMill;

  delay(20);
  
  PIDFunction(rollVec1, rollVec2, dir, usArray, DCM);
  
}

//argument: double DCM[][3]

void PIDFunction(imu::Quaternion rollVec1, imu::Quaternion rollVec2, imu::Quaternion dir, double usArray[], double DCM[][3]){

  //newRollVec1z = DCM[2][0] * rollVec1.x() + DCM[2][1] * rollVec1.y() + DCM[2][2] * rollVec1.z();
  //newRollVec2z = DCM[2][0] * rollVec2.x() + DCM[2][1] * rollVec2.y() + DCM[2][2] * rollVec2.z();
  
  //double rolVecAngle = -asin(newRollVec1z);
  //double sidVecAngle = asin(newRollVec2z);

  double testValue1 = -1.0 * (double)rollVec1.x();
  double testValue2 = -1.0 * (double)rollVec2.x();
  double testValue3 = dir.x();

  double theta1 = asin(testValue1);
  double theta2 = atan2(testValue2,testValue3);  

  //Serial.printf("%f   %f\n", rolVecServoAngle, sidVecServoAngle);
  Proportional(theta1, theta2, proComps);
  Integral(theta1, theta2, total, intComps);
  Derivative(theta1, theta2,lastErrors, derComps);

  double rolDeg = (proComps[0] + intComps[0] + derComps[0]) * RAD_TO_DEG * (0.5);
  double sidDeg = (proComps[1] + intComps[1] + derComps[1]) * RAD_TO_DEG * (0.5);
  
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
