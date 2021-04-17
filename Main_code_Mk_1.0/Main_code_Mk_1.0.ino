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
int previousStage = 0; //place to store previous flight function

float alts[4];
float orient[16];

bool charge1 = true;
bool charge2 = true;

// PID variables
double sumPitch = 0;
double sumRoll = 0;
double lastErrorPitch = 0;
double lastErrorRoll = 0;
int lastTime;

const double P = 1;
const double I = .1;
const double D = .2;

elapsedMillis timer = 0;
elapsedMillis timer2 = 0;
int cycle = 0;

void setup() {
  Serial.begin(115200);
  Serial.printf("Begin test:\n");

  // Initializing sensors and center equipment
  indicatorSetup();
  SDSetup();
  inaSetup();
  orient[0] = bnoSetup();
  alts[0] = bmpSetup();
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
      state = failure(state); // Failure state wrapper - state determines type of failure (under failures.ino tab)
  }   
}

int startup() {
  int nextState = 0;

  if(1) { //digitalRead(armingPin1) == HIGH && digitalRead(armingPin2) == HIGH) {
    nextState = 1;
    previousStage = 0;
    Serial.printf("Rocket armed: Startup-->Groundidle\n");
    delay(1000);
    
    alts[2] = 0;
    alts[3] = 0;
    getAlt(alts);
    
    if (alts[0] == 0) {
      timer = 0;
      nextState = chute();
    }  
  }
  return nextState;
}

int groundidle() {
  int nextState = 1;
  if (timer >= 1000) {
    timer = 0;
    tone(BUZZER, 4000, 500);
  }

  getAlt(alts);
  orientation(orient);

  /* if acceleration in the x direction (towards nosecone) is greater than 12 */
  if (orient[7] >= 12) { 
    nextState = 2;
    previousStage = 1;
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

  /*if acceleration in the direction of the rocket is less than 5 m/s2, assume burnout or if apogee is detected without burnout (decreasing altitude) */
  if (accelForward <= 5  || alts[3] > 5) {  
    nextState = 3;
    previousStage = 2;
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
  
  if (alts[3] < 5){ /*1560 is placeholder*/
    nextState = 4;
    previousStage = 3;
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

  if (alts[1] > 1550){ //alt <= chuteDeployAltitude; 1560 placeholder altitude
    nextState = 5;
    previousStage = 4;
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

  /*if the acceleration is below a certain threshold, above a certain altitude (with the exception of altitude data loss), 
      and the previous stage was not a ground stage, deploy chute charges */
  if (accelMag < 2 && (alts[0] != 1 || alts[1] >= 1600) && (previousStage >= 2 && previousStage != 6)) { 
    //If less than 1 second has passed and the first charge has not deployed, send a charge through the MOSFET1
    if (timer <= 1000 && charge1) {
      digitalWrite(chuteCharge1, HIGH);
      charge1 = false;
    } else if (timer >= 1000 && charge2) {  //If 1 second has passed and the second charge has not deployed, send a charge through the MOSFET2
      digitalWrite(chuteCharge2, HIGH);
      charge2 = false;
    } else if (!charge1 && !charge2){  //if both charges have been deployed and acceleration still reads freefall (still going through first if-statement), start a second timer
      ++cycle;
      if (cycle == 1) {  //so that timer2 only resets once
        timer2 = 0;
      }
      if (timer2 >= 1000) {  //if 1 second has passed with no change in acceleration, go to noChute failure
        timer = 0;
        chute1 = true;
        chute2 = true;
        nextState = 5;
      }
    } else {}
  }

  if (accelMag >= 9){ //9 m/s2 to account for gravity once landed
    nextState = 6; 
    previousStage = 5;
    Serial.printf("Landing detected: Chute-->Landing\n");
    delay(1000);
  }
  return nextState;
}

int landing(){
  int nextState = 6;
  tone(BUZZER, 4000,1000); //Victory Screech No.2

//Under current conditions, this will run in a loop indefitely. Either the main loop should stop after landing or the write function call should be called at the end of chute().
  while(1);

  //writeToSD(); 

  return nextState;
}

int failure() {
  orientation(orient);
  Serial.printf("Failure\n");
  while(1);
  int nextStage = 7;
  return nextStage;
}
