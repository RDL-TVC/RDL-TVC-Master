#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

const int armingPin = 2; // Place holder pin for the arming button
const int chuteChargeContOut = 5; // Placeholder Not sure how the continuity of the chute charge will be tested.
const int chuteCharge1 = 3; // Placeholder
const int chuteCharge2 = 4; // Placeholder

//Defines the pins for I2C for the BMP388 - Placeholder Values
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

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
  if (state == 1) {
    nextState = startup();
  } else if (state == 2) {
    nextState = groundidle();
  } else if (state == 3) {
    nextState = liftoff();
  } else if (state == 4) {
    nextState = burnout();
  } else if (state == 5) {
    nextState = freefall();
   }else if (state == 6) {
    nextState = chute();
  } else if (state == 7) {
    nextState = landing();
  } else {
    nextState = failure(); // General failure state might need to specify different failure states.
  }
  return nextState;
  }
