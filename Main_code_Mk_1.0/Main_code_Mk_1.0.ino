#include "Log_Functions.h"
#include "Pressure_Altitude.h"

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

int currentState = 0; // State of the state machine to know which flight function to call. Starts at startup.
  float currentAlt;
  float lastAlt;

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

void setup() {
  Serial.begin(115200);

  // Initializing sensors and center equipment
  indicatorSetup();
  SDSetup();
  inaSetup();
  orient[0] = bnoSetup();
  bmpSetup();
  servoSetup();
  miscSetup();

  //TODO Mario powerup sound for "might work"!
  tone(BUZZER, 4000, 1000); //Victory Screech
}

void loop() {

  currentState = callFlightFunc(currentState); // added a function rapper to make more modular, however unlikely to be needed

}

int callFlightFunc(int state) {
  /* 
  Intakes the current state of the state machine and runs the appropriate function for that state.
  Returns the next state of the state machine
  */
  int nextState = state;
  switch(state) {
    case 1 :
      nextState = startup();
      break;
    case 2 :
      nextState = groundidle();
      break;
    case 3 :
      nextState = boost();
      break;
    case 4 :
      nextState = burnout();
      break;
    case 5 :
      nextState = freefall();
      break;
    case 6 :
      nextState = chute();
      break;           
    case 7 :
      nextState = landing();
      break;
    default : 
      nextState = failure(); // General failure state might need to specify different failure states.
  } return nextState;
}

int startup() {
  int nextState = 1;

  if(digitalRead(armingPin1) == HIGH && digitalRead(armingPin2) == HIGH) {
    nextState = 2;
    //Print to event string
  }

  return nextState;
}

int groundidle() {
  int nextState = 2;

  orientation(orient);
  if (orient[7] >= 12) {
    nextState = 3;
  }

  return nextState;
  
  /*
  float* alts = altSensor.getAlt();
  orientation(orient);
  
  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alts,orient);
  
  if (orient[2] >= accelThreshold) { // TODO: placeholder values that need to be changed once data format has been determined.
    nextState = 3;
    //initPID();
    //unlockServos();
  }
  */
}

//TODO find acceleration vector compared to direction vector to see which component feels gravity
int boost() {
  int nextState = 3;
  tone(BUZZER, 4000, 1000); //runs buzzer for 1s when liftoff is detected
  
  orientation(orient);
  float* gimbalAngle = findGimbalAngles(orient);
  float* servoAngle = PID(gimbalAngle);

  servoPitch.writeMicroseconds(servoAngle[0]);
  servoYaw.writeMicroseconds(servoAngle[1]);

  /*
  float accelMag = sqrt(orient[7]*orient[7] + orient[8]*orient[8] + orient[9]*orient[9]);
  if (accelMag <= ) {
    nextState = 4;
  } */
  return nextState;
}

int burnout(){
  int nextState = 4;

  float* alts = altSensor.getAlt();
  orientation(orient);
  
  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alts,orient);

  if (alt < lastAlt){
    nextState = 5;
  }

  return nextState;
}

int freefall(){
  int nextState = 5;

  //chuteDeployAltitude = 1; //TODO: Determine threshold altitude for deploying parachutes
  
  float* alts = altSensor.getAlt();
  orientation(orient);

  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alts,orient);

  if (alt <= chuteDeployAltitude){
    nextState = 6;
    //deployChutes()
  }

  return nextState;
}

int chute(){
  int nextState = 6;

  float* alts = altSensor.getAlt();
  orientation(orient);

  currentAlt = alts[0];
  lastAlt = alts[1];

  float lastAccel = 0; //placeholder
  
  dataLog.logData(alts,orient);

  if (orient[2] >= lastAccel){
    nextState = 0; //Placeholder for failure state if parachutes do not deploy
  }else if (alt >= lastAlt){
    nextState = 7;
  }

  lastAccel = orient[2];

  return nextState;
}

int landing(){
  int nextState = 7;

  writeToSD(); //Under current conditions, this will run in a loop indefitely. Either the main loop should stop after landing or the write function call should be called at the end of chute().

  return nextState;
}

int failure() {
  float* alts = altSensor.getAlt();
  orientation(orient);
  
  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alts,orient);
  return 0;
}
