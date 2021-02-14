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

#define SERVO_PIN_LR 0
#define SERVO_PIN_FB 1

const int armingPin = 2; // Place holder pin for the arming button
const int chuteChargeContOut = 5; // Placeholder Not sure how the continuity of the chute charge will be tested.
const int chuteCharge1 = 3; // Placeholder
const int chuteCharge2 = 4; // Placeholder

const float accelThreshold = 10; // Placeholder
const float seaLevelPressure = 1013.25; //units of hPa, required for pressure altitude

const int chipSelect = BUILTIN_SDCARD;

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_INA260 ina260 = Adafruit_INA260();
Servo servo_LR;
Servo servo_FB;

int currentState = 0; // State of the state machine to know which flight function to call. Starts at startup.

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
  // Initializing all Pins

  pinMode(armingPin, INPUT);
  pinMode(chuteCharge1, INPUT);
  pinMode(chuteCharge2, INPUT);

  pinMode(chuteChargeContOut, OUTPUT);

  Serial.begin(115200);

  // Initializing sensors and center equipment
  SDSetup();
  inaSetup();
  bmpSetup();
  bnoSetup();
  servoSetup();
  miscSetup();
  

}

void loop() {

  currentState = callFLightFunc(currentState); // added a function rapper to make more modular, however unlikely to be needed

}

int callFLightFunc(int state) {
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
      nextState = liftoff();
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
  digitalWrite(chuteChargeContOut, HIGH);
  int charge1 = digitalRead(chuteCharge1);
  int charge2 = digitalRead(chuteCharge2); 
  digitalWrite(chuteChargeContOut, LOW);
  int armed = digitalRead(armingPin);

  if ((not charge1) or (not charge2)){
    nextState - 0; // Placeholder for failure state at least one of the chute charges is not connected
  } else if (armed) {
    nextState = 2;
  }
  return nextState;
}

int groundidle() {
  int nextState = 2;

  float* alts = altSensor.getAlt();
  float* orient = orientation();

  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alt,orient);
  
  if (orient[2] >= accelThreshold) { // TODO: placeholder values that need to be changed once data format has been determined.
    nextState = 3;
    initPID();
    unlockServos();
  }
  return nextState;
}

int liftoff() {
  int nextState = 2;
  
  return nextState;
}

int burnout(){
  int nextState = 4;

  float* alts = altSensor.getAlt();
  float* orient = orientation();
  
  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alt,orient);

  if (alt < lastAlt){
    nextState = 5;
  }

  return nextState;
}

int freefall(){
  int nextState = 5;

  chuteDeployAltitude = 1; //TODO: Determine threshold altitude for deploying parachutes
  
  float* alts = altSensor.getAlt();
  float* orient = orientation();

  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alt,orient);

  if (alt <= chuteDeployAltitude){
    nextState = 6;
    deployChutes()
  }

  return nextState;
}

int chute(){
  int nextState = 6;

  float* alts = altSensor.getAlt();
  float* orient = orientation();

  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alt,orient);

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
  float* orient = orientation();
  
  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alt,orient);
  return 0;
}
