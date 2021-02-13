#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_INA260.h>
#include <Servo.h>
#include <SD.h>

const int armingPin = 2; // Place holder pin for the arming button
const int chuteChargeContOut = 5; // Placeholder Not sure how the continuity of the chute charge will be tested.
const int chuteCharge1 = 3; // Placeholder
const int chuteCharge2 = 4; // Placeholder

const float accelThreshold = 10; // Placeholder
const float seaLevelPressure = 1013.25; //units of hPa, required for pressure altitude

Adafruit_BMP3XX bmp;

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

  // Initializing sensors and center equipment
  initializeSensors();
  initializeServos();
  bmpSetup();

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
