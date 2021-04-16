//#include "Log_Functions.h"
//#include "Pressure_Altitude.h"

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_INA260.h>
#include <Servo.h>
#include <SD.h>

#define SERVO_PIN_PITCH 0
#define SERVO_PIN_YAW 1

#define BUZZER 14

const int LED1 = 8;
const int LED2 = 9;
const int armingPin1 = 6;
const int armingPin2 = 7;
const int chuteCharge1 = 4; 
const int chuteCharge2 = 5; 

const float accelThreshold = 10; // Placeholder
const float seaLevelPressure = 1013.25; //units of hPa, required for pressure altitude

const int chipSelect = BUILTIN_SDCARD;

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_INA260 ina260 = Adafruit_INA260();
Servo servoPitch;
Servo servoYaw;

int state = 0; // State of the state machine to know which flight function to call. Starts at startup.

float alts[2];
float orient[16];

// PID variables
double sumPitch = 0;
double sumRoll = 0;
double lastErrorPitch = 0;
double lastErrorRoll = 0;
int lastTime;

const double P = 1;
const double I = .1;
const double D = .2;

elapsedMillis buzzerTime = 0;

void setup() {
  Serial.begin(115200);
  Serial.printf("Begin test:\n");

  // Initializing sensors and center equipment
  indicatorSetup();
  SDSetup();
  inaSetup();
  orient[0] = bnoSetup();
  bmpSetup();
  servoSetup();
  miscSetup();
}

void loop() {
  /* 
  Intakes the current state of the state machine and runs the appropriate function for that state.
  Returns the next state of the state machine
  */
  switch(state) {
    case 0 :
      state = startup();
      break;
    case 1 :
      state = groundidle();
      break;
    case 2 :
      state = boost();
      break;
    case 3 :
      state = burnout();
      break;
    case 4 :
      state = freefall();
      break;
    case 5 :
      state = chute();
      break;           
    case 6 :
      state = landing();
      break;
    default : 
      state = failure(); // General failure state might need to specify different failure states.
  }   
}

int startup() {
  int nextState = 0;

  if(1) { //digitalRead(armingPin1) == HIGH && digitalRead(armingPin2) == HIGH) {
    nextState = 1;
    Serial.printf("Rocket armed: Startup-->Groundidle\n");
    delay(1000);
    alts[1] = 0;
    alts[2] = 0;
    getAlt(alts);
  }
  
  return nextState;
}

int groundidle() {
  int nextState = 1;
  if (buzzerTime >= 1000) {
    buzzerTime = 0;
    tone(BUZZER, 4000, 500);
  }

  getAlt(alts);
  orientation(orient);
  
  if (orient[7] >= 12) { //if acceleration in the x direction (towards nosecone) is greater than 12
    nextState = 2;
    tone(BUZZER, 4000, 1000); //Victory Screech: runs buzzer for 1s when liftoff is detected
    Serial.printf("Liftoff detected: Groundidle-->Boost\n");
    delay(1000);
  }

  return nextState;
}

//TODO find acceleration vector compared to direction vector to see which component feels gravity
int boost() {
  int nextState = 2;

  getAlt(alts);
  orientation(orient);
  float* gimbalAngle = findGimbalAngles(orient);
  float* servoAngle = PID(gimbalAngle);

  servoPitch.writeMicroseconds(servoAngle[0]);
  servoYaw.writeMicroseconds(servoAngle[1]);

  float accelForward = orient[7]*orient[1]+orient[8]*orient[2]+orient[9]*orient[3];  //dot product of acceleration and direction vectors
  //Serial.printf("acceleration = %f\n", accelForward);
  if (accelForward <= 5  || alts[2] > 5) {  //if acceleration in the direction of the rocket is less than 5 m/s2, assume burnout or if apogee is detected without burnout (decreasing altitude)
    nextState = 3;
    servoPitch.write(90);
    servoYaw.write(90);
    Serial.printf("Burnout detected: Boost-->Burnout\n");
    delay(1000);
  } 

  return nextState;
}

int burnout(){
  int nextState = 3;

  getAlt(alts);
  orientation(orient);
  
  if (alts[2] < 5){ /*1560 is placeholder*/
    nextState = 4;
    Serial.printf("Apogee detected: Burnout-->Freefall\n");
    delay(1000);
  }

  return nextState;
}

int freefall(){
  int nextState = 4;

  //chuteDeployAltitude = 1; //Determine threshold altitude for deploying parachutes; commented out due to immediate chute deployment upon reaching apogee
  
  getAlt(alts);
  orientation(orient);

  if (alts[0] > 1550){ //alt <= chuteDeployAltitude; 1560 placeholder altitude
    nextState = 5;
    //deployChutes();
    Serial.printf("Chute Deployment Altitude detected: Freefall-->Chute\n");
    delay(1000);
  }

  return nextState;
}

int chute(){
  int nextState = 5;

  getAlt(alts);
  orientation(orient);

  float accelMag = sqrt(orient[7]*orient[7]+orient[8]*orient[8]+orient[9]*orient[9]);

  if (accelMag >= 9){ //9 m/s2 to account for gravity once landed
    nextState = 6; 
    Serial.printf("Landing detected: Chute-->Landing\n");
    delay(1000);
  }
  
  return nextState;
}

int landing(){
  int nextState = 6;
  tone(BUZZER, 4000,1000); //Victory Screech No.2
  while(1);

  //writeToSD(); //Under current conditions, this will run in a loop indefitely. Either the main loop should stop after landing or the write function call should be called at the end of chute().

  return nextState;
}
