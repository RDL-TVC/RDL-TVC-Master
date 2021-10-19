#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

#define SERVO_PIN_PITCH 0
#define SERVO_PIN_YAW 1
#define RAD_TO_DEG 57.295779513082320876798154814105
#define BUZZER 4

#define _USE_MATH_DEFINES
#include <cmath>

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

const int armingPin1 = 5;
const int armingPin2 = 6;
int buttonCycles = 0;

int calRun = 100; // amount of cycles for calibration of up direction
imu::Quaternion upDir;
bool armed = false;
float angle[3];  //0 - roll, 1 - pitch, 2 - yaw


void setup(void)
{
  servo_pitch.attach(SERVO_PIN_PITCH);
  servo_yaw.attach(SERVO_PIN_YAW);
  servo_pitch.writeMicroseconds(1500);
  servo_yaw.writeMicroseconds(1500);

  pinMode(BUZZER,OUTPUT); //piezo buzzer pin 4
  
  Serial.begin(9600);
  tone(BUZZER, 4000, 1000);  //Bno calibration start

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

  tone(BUZZER, 3000, 500);  //Bno calibration end
  delay(500);
  tone(BUZZER, 4000, 1000);
}

void loop(void) {
      
  imu::Quaternion quat = bno.getQuat();
  
  //checks for buttons to be pressed before continuing
  if(digitalRead(armingPin1) == HIGH && digitalRead(armingPin2) == HIGH) {
    ++buttonCycles;
    delay(10);
    if (buttonCycles >= 500) {  //hold down both buttons for 5s
      Serial.printf("Rocket armed, release buttons\n");
      armed = true;     
      tone(BUZZER, 4000, 1000);  //armed noise
    }
  } else if(armed) {
      Serial.printf("Setting up direction:\n");
      //get servo angles after up direction is calibrated
      if (calRun >= 0){
        --calRun;
        upDir = quat;
        
       Serial.printf("Calibration run: %d     ", calRun);
       Serial.printf("%.2f   %.2f   %.2f   %.2f\n", upDir.z(), upDir.x(), upDir.y(), upDir.z());
        
        if (calRun == 0) {
          tone(BUZZER, 3000, 500);  //up direction calibration end
          delay(500);
          tone(BUZZER, 4000, 1000);
        }
      } else {
        quat.w() -= upDir.w();
        quat.x() -= upDir.x();
        quat.y() -= upDir.y();
        quat.z() -= upDir.z();
        getAngles(quat,angle);
    
        Serial.printf("Roll = %.4f   Pitch = %.4f   Yaw = %.4f\n", angle[0], angle[1], angle[2]);
        
        angle[1] = map(angle[1], -60, 60, 900, 2100);
        angle[2] = map(angle[2], -60, 60, 900, 2100);
      
        servo_pitch.writeMicroseconds(angle[1]);
        servo_yaw.writeMicroseconds(angle[2]);
      }
    
  } else {
    buttonCycles = 0;
  } 
  
}

int getAngles(imu::Quaternion q, float angle[]) {
  // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    angle[0] = std::atan2(sinr_cosp, cosr_cosp);
    
  // pitch (y-axis rotation)
  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (std::abs(sinp) >= 1) {
        angle[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    } else {
        angle[1] = std::asin(sinp);
    }
    
  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  angle[2] = std::atan2(siny_cosp, cosy_cosp);

  angle[0] = angle[0]*180/PI;
  angle[1] = angle[1]*180/PI;
  angle[2] = angle[2]*180/PI;
  return 1;
}

/*
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
*/
