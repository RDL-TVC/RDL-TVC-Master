#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_INA260.h>
#include <Servo.h>
#include <SD.h>

// Pins for external componets and associated constants
const int SERVO_PIN_PITCH = 0;
const int SERVO_PIN_YAW = 1;

const int BUZZER = 4;

const int TONE_SUCCESS = 523; // In Hz, Currently C5
const int TONE_FAILURE = 261; // In Hz, currently C4
const int TONE_VICTORY = 1046; // In Hz, Currently C6

const int CHUTE_1_PIN = 2;
const int CHUTE_2_PIN = 3;

const int CHUTE_DELAY = 1000; // Time between chute charge activations in millis. 2nd charge only used if 1st fails

const int TIME_FREEFALL_THRESHOLD = 100; // How long in Mills Altitude needs to be decreasing to declare freefall
const int ACCEL_TAKEOFF_THRESHOLD = 12; // The Threshold for acceleration that triggers boost phase m/s^2
const int CHUTE_DEPLOY_ALT = 1000000; // Altitude under which chute is deployed, currently large for immediate chute deployment

const int ARM_B1_PIN = 5;
const int ARM_B2_PIN = 6;

const int SECONDS_TO_ARM = 10; // Number of seconds needed to hold down both buttons to arm

const int LED_GREEN = 7;
const int LED_RED = 8;

const int DUTY_CYCLE = 1000; // Length of LED blinking Duty cycle in millis
const float LED_TIME_ON = .05; // fraction of duty cycle that LED is on

const float SL_PRESSURE = 1013.25; //units of hPa, required for pressure altitude

const int MAX_ERROR_CYCLES = 10; // Maximum number of error cycles in a row that are tolerated before proceding to failure.

// Objects used
const int SD_CARD = BUILTIN_SDCARD;

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_INA260 ina260 = Adafruit_INA260();

Servo servoPitch;
Servo servoYaw;

// Global Variables that carry from cycle to cycle

float avgVoltage = 0;

int state = 0; // State of the state machine to know which flight function to call. Starts at startup.

int BNO055Status = 0;
double quaternion[4]; // current Raw Quaternion Vector from BNO055
double accelVector[3]; // current Acceleration Vector
double avelVector[3]; // current Angular Velocity Vector
double gravVector[3]; // current gravitational vector relative to absolute orientation

int BMP388Status = 0;
float groundAltitude = 0;
float alt = 0; // current altitude in m
float apogee = 0; // maximum altitued reached in m

bool charge1 = true; // Is chute charge 1 availible?
bool charge2 = true; // Is chute charge 2 availible?

// Cycle counters
int errorCycle = 0;

// Various timers
elapsedMillis PROGRAM_TIME = 0; // Time since start of program do not reset
elapsedMillis armTimer = 0; // Elapsed time while arming buttons are held.

elapsedMillis chuteChargeTimer = 0; // Time since last chute charge was activated
elapsedMillis freefallTimer = 0; // time since free fall started

/********************************************************************************
 *  Setup : void setup()
 *  Runs once all one time functions to set up program for the statemachine.
 *  Initializes all components.
 ********************************************************************************/
void setup() 
{
  PROGRAM_TIME = 0;
  
  Serial.begin(115200);
  Serial.println("Begin Initialization:");

  bool i = true;
  
  // Initialize Indicators (LEDs, Buzzer)
  i = i & indicatorInit();

  digitalWrite(LED_GREEN,HIGH);
  
  // Intilize storage
  i = i & SDInit();
  
  // Initialize Sensors
  i = i & inaInit();
  BNO055Status = bnoInit();
  i = i & BNO055Status;
  BMP388Status = bmpInit();
  i = i & int(BMP388Status);

  // Intialize and Test Servos
  i = i & servoInit();
  i = i & servoTest();

  // Initilize Charge pins and Arming buttons
  i = i & chuteInit();
  i = i & armingInit();

  if (!i)
  { // prevent program from continuing if any component failed to initialize
    Serial.println("Initialization failed");
    tone(BUZZER,TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    while(1){}
  }

  // Sensors Initialized. Celebrate!
  digitalWrite(LED_GREEN, HIGH);
  tone(BUZZER, TONE_SUCCESS, 500);
  delay(500);
  tone(BUZZER, TONE_VICTORY, 250);
  delay(250);
  
  // Proceed to startup to wait for arming
  armTimer = 0;
  
}

/********************************************************************************
 *  Loop(main) : void loop()
 *  Loops through statemachine states and checks for errors.
 ********************************************************************************/
void loop() 
{ 
  
  if (BMP388Status == 0 || BNO055Status == 0 || ina260.readBusVoltage() <= (avgVoltage - 6000) ) 
  { // Check know statuses of sensors to check for sensor or battery failure
      ++errorCycle;
      if (errorCycle >= MAX_ERROR_CYCLES) 
      { //if the problem continues for specified number of cycles in a row, proceed to correction
        tone(BUZZER, TONE_FAILURE);
        
        //TODO: lock gimbal
  
        //TODO: save to SD card

        // Rocket no longer controlable, deploy chute to minimize damage
        state = chute();
      }
  } else 
  { 
    errorCycle = 0; 
  }
  
  // Intakes the current state of the state machine and runs the appropriate function for that state.
  // State function returns next state to run (itself or next state)
  switch(state) 
  {
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
      state = failure();
  }
  delay(10);
}

/********************************************************************************
 *  Startup State Function     : int startup() 
 *      returns                : nextState, Next state function to run
 *  
 *  Rocket starting state - may be placed in setup().
 *  Waits for button push to arm the rocket.
 *  returns the nextState of groundidle() if armed. 
 ********************************************************************************/
int startup() 
{
  int nextState = 0;
  
  if(digitalRead(ARM_B1_PIN) == HIGH && digitalRead(ARM_B2_PIN) == HIGH) 
  { // If both arming buttons are pressed, start counting down.
    Serial.print("Arming, Seconds till armed: ");
    Serial.println(SECONDS_TO_ARM - (armTimer / 1000));
    if (armTimer > (SECONDS_TO_ARM * 1000)) 
    { // Countdown finished. Rocket has been armed
      Serial.printf("Rocket armed: Startup-->Groundidle\n");
      tone(BUZZER,TONE_SUCCESS,1000);
      delay(1000);
      nextState = 1;
    }
  } else 
  { // Reset countdown if arming buttons are not active
    armTimer = 0;
    Serial.println("Idle");
  }

  //Blink LED rapidly to indicate waiting for arming.
  LEDBlink(LED_GREEN, DUTY_CYCLE / 2, LED_TIME_ON);

  // TODO: Implement check to see if sensors are still runing
  
  return nextState;
}

/********************************************************************************
 *  Ground Idle State Function : int groundidle() 
 *      returns                : nextState, Next state function to run
 *  
 *  Rocket armed, waiting for ignition and liftoff.
 *  Starts orentation and altitude data collection.
 *  Detects lifoff through significant upwards acceleration.
 ********************************************************************************/
int groundidle() 
{
  int nextState = 1;

  // Blink both status LEDs to indicate armed.
  LEDBlink(LED_GREEN, DUTY_CYCLE, LED_TIME_ON);
  LEDBlink(LED_RED, DUTY_CYCLE, LED_TIME_ON);

  // Add periodic buzzer beep

  // Update sensor data
  BMP388Status = getAlt(&alt, &apogee);
  BNO055Status = getOrient(quaternion, accelVector, avelVector, gravVector);

  if (accelVector[0] >= ACCEL_TAKEOFF_THRESHOLD) 
  { // if acceleration in the x direction (towards nosecone) is greater than set threshold:
    // Launched!
    nextState = 2;
    digitalWrite(LED_GREEN,LOW);
    digitalWrite(LED_RED,LOW);
    tone(BUZZER, TONE_VICTORY, 1000); //Victory Screech: runs buzzer for 1s when liftoff is detected; MAX f needed
    Serial.printf("Liftoff detected: Groundidle-->Boost\n");
  }
  return nextState;
}

/********************************************************************************
 *  Powerflight State Function : int bost() 
 *      returns                : nextState, Next state function to run
 *  
 *  Rocket is now in powered flight.
 *  Collects orientation and altitude data.
 *  Runs PID Loop to adjust gimbal angle and remain upright.
 *  Detects burnout through loss of acceleration -> head to burnout state.
 *  Detects freefall through decreasing altitude -> head to freefall state.
 ********************************************************************************/
int boost() 
{
  int nextState = 2;

  // Update sensor data
  BMP388Status = getAlt(&alt, &apogee);
  BNO055Status = getOrient(quaternion, accelVector, avelVector, gravVector);
  
  // TODO: Run PID and adjust servos accordingly

  // If acceleration is in roughly same direction as gravity, motor has burned out. grav dot accel > 0
  double accelDown = gravVector[0] * accelVector[0] + gravVector[1] * accelVector[1] + gravVector[2] * accelVector[2];

  if (accelDown > 0)
  { // No longer acelerating upwards head to burnout
    Serial.printf("Burnout detected: Boost-->Burnout\n");
    nextState = 3;
    
    // Center Servos
  } else if (BMP388Status == 2)
  { // Rocket Started Falling, missed burnout head to freefall.
    Serial.printf("Apogee detected: Boost-->Freefall\n");
    nextState = 4;

    // Center Servos
  }
  
  return nextState;
}

/********************************************************************************
 *  Burnout State Function    : int burnout() 
 *      returns               : nextState, Next state function to run
 *  
 *  Rocket is no longer in powerd flight but still moving upwards.
 *  Collects orientation and altitude data.
 *  Detects freefall through decreasing altitude
 ********************************************************************************/
int burnout(){
  int nextState = 3;

  // Update sensor data
  BMP388Status = getAlt(&alt, &apogee);
  BNO055Status = getOrient(quaternion, accelVector, avelVector, gravVector);
  
  if (BMP388Status == 2)
  { // Rocket started falling, head to freefall.
    nextState = 4;
    Serial.printf("Apogee detected: Burnout-->Freefall\n");
  }
  return nextState;
}

/********************************************************************************
 *  Freefall State Function   : int freefall() 
 *      returns               : nextState, Next state function to run
 *  
 *  State before parachute deploy.
 *  Collects orientation and altitude data.
 *  Detects time for parachute deploy when threshold altitude is crossed.
 ********************************************************************************/
int freefall()
{
  int nextState = 4;

  // Update sensor data
  BMP388Status = getAlt(&alt, &apogee);
  BNO055Status = getOrient(quaternion, accelVector, avelVector, gravVector);
  
  if (alt <= CHUTE_DEPLOY_ALT)
  { // Under parachute deploying altitude head to chute deploy.
    nextState = 5;
    Serial.printf("Chute Deployment Altitude detected: Freefall-->Chute\n");
  }
  return nextState;
}

/********************************************************************************
 *  Chute Deploy State Function : int chute() 
 *      returns                 : nextState, Next state function to run
 *  
 *  Rocket attempts to deploy parachute.
 *  Collects orientation and altitude data.
 *  If first chute charge does not work it uses backup charge
 *  If sucessfully deployed waits till accel ~ 0 to go to landed state
 *  If chute does not successfully deploy head to error state.
 ********************************************************************************/
int chute()
{
  int nextState = 5;

  // Update sensor data
  BMP388Status = getAlt(&alt, &apogee);
  BNO055Status = getOrient(quaternion, accelVector, avelVector, gravVector);

  // Calculate accurate acceleration magnitude that is towards the ground.
  double gmag = sqrt(gravVector[0]*gravVector[0] + gravVector[1]*gravVector[1] + gravVector[2]*gravVector[2]);
  double accelDown = (gravVector[0] * accelVector[0] + gravVector[1] * accelVector[1] + gravVector[2] * accelVector[2]) / gmag;

  // TODO: Need to figure out how much downwards acceleration is acceptable
  if ((accelDown > gmag / 2) && alt > 20 && BMP388Status != 0) 
  { // If falling with too much acceleration, and above a certain altitude, deploy all chutes charges possible (with delay).
    // Note: only if BMP388 is confirmed to be working
    if (charge1)
    {
      digitalWrite(CHUTE_1_PIN, HIGH);
      charge1 = false;
      Serial.println("Chute charge 1 active");
      chuteChargeTimer = 0;
    } else if (charge2 && chuteChargeTimer > CHUTE_DELAY)
    { // Chute Charge 1 used and still not slower than falling, use charge 2.
      digitalWrite(CHUTE_1_PIN, LOW);
      Serial.println("Chute charge 1 deactivated");
      digitalWrite(CHUTE_2_PIN, HIGH);
      Serial.println("Chute charge 2 active");
      charge2 = false;
      chuteChargeTimer = 0;
    } else if (!charge1 && !charge2 && chuteChargeTimer > CHUTE_DELAY)
    { // both chute charges used and still in freefall, chute failure, head to failure state. Scram!
      digitalWrite(CHUTE_2_PIN, LOW);
      Serial.println("Chute charge 2 deactivated");
      Serial.println("Chute failure");
      nextState = -1; // TODO: Add a failure state for chute failure.
    }
  }
  
  if ((-0.1 < accelDown) && (accelDown < 0.1))
  { // Downwards acceleration near 0, on ground, head to landing state.
    nextState = 6; 
    Serial.printf("Landing detected: Chute-->Landing\n");
  }
  
  return nextState;
}

/********************************************************************************
 *  Landed State Function    : int landing() 
 *      returns               : nextState, Next state function to run
 *  
 *  Rocket has landed. Other functionality to be implemented.
 ********************************************************************************/
int landing(){
  int nextState = 6;
  
  // TODO: Add check chute failure state.
  // TODO: Add victory buzz, adds to ability to find rocket.
  
  //Under current conditions, this will run in a loop indefitely. Either the main loop should stop after landing or the write function call should be called at the end of chute().
  while(1);
  //writeToSD(); 
  return nextState;
}

/********************************************************************************
 *  Failure State Function    : int failure() 
 *      returns               : nextState, Next state function to run
 *  
 *  Oh no! Something went wrong what do we do now?
 ********************************************************************************/
int failure() {
  // Update sensor data
  BMP388Status = getAlt(&alt, &apogee);
  BNO055Status = getOrient(quaternion, accelVector, avelVector, gravVector);
  
  Serial.printf("Failure\n");
  
  while(1); // Gets stuck here, TODO: Implement course of action for complete failure.
  
  int nextStage = -1;
  return nextStage;
}
