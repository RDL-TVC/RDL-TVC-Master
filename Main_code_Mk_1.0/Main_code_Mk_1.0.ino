#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_INA260.h>
#include <Servo.h>
#include <SD.h>

const int SERVO_PIN_PITCH = 0;
const int SERVO_PIN_YAW = 1;

const int BUZZER = 4;

const int CHUTE_1_PIN = 2;
const int CHUTE_2_PIN = 3;

const int ARM_B1_PIN = 5;
const int ARM_B2_PIN = 6;

const int LED_GREEN = 7;
const int LED_RED = 8;

const float SL_PRESSURE = 1013.25; //units of hPa, required for pressure altitude

const int SD_CARD = BUILTIN_SDCARD;

float groundAltitude = 0;
float avgVoltage = 0;

Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_INA260 ina260 = Adafruit_INA260();

Servo servoPitch;
Servo servoYaw;

int state = 0; // State of the state machine to know which flight function to call. Starts at startup.
int previousState = 0; //Place to store index of the previous flight function

float alts[4];
float orient[20];

bool charge1 = true;
bool charge2 = true;
bool timer2Start = true;

elapsedMillis timer = 0;
elapsedMillis timer2 = 0;

int errorCycle = 0;
int buttonCycles = 0;

/* Initializes all the sensors, the required variables, and calibrates data */
void setup() {
  Serial.begin(115200);
  Serial.printf("Begin test:\n");

  //Ensure indicators are off
  digitalWrite(LED_GREEN,LOW);
  digitalWrite(LED_RED,LOW);
  noTone(BUZZER);

  // Initialize sensors and center equipment
  indicatorSetup();
    //indicators validated to be working, battery plugged in
    digitalWrite(LED_RED,HIGH); 
    tone(BUZZER,3000,1000);
    delay(1000);
    tone(BUZZER,5000,1500);
  SDSetup();
  inaSetup();
  orient[0] = bnoSetup();
  alts[0] = bmpSetup();
  servoSetup();
  miscSetup();
    //All sensors and components calibrated
    digitalWrite(LED_RED,LOW);
    digitalWrite(LED_GREEN,HIGH);
    tone(BUZZER,4000,1000);
    delay(1000);

}

void loop() {
  /* check first for sensor errors
   * BMP388 stops working
   * BNO055 stops working
   * INA260 reads an average voltage drop of 6V */
  if (alts[0] == 0 || orient[0] == 0 || ina260.readBusVoltage() <= (avgVoltage - 6000) ) { 
      ++errorCycle;
      //if the problem continues for 10 cycles in a row, proceed to correction
      if (errorCycle >= 10) {
        tone(BUZZER, 3900);
        
        //lock gimbal
        servoPitch.write(90);
        servoYaw.write(90);
  
        //TODO: save to SD card
  
        //send to chute or landed state
        timer = 0;
        previousState = state;
        state = chute();
      }
  } else { 
    errorCycle = 0; 
  }
  
  /* Intakes the current state of the state machine and runs the appropriate function for that state.
  Returns the next state of the state machine */
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
      state = failure(); // failure state - for unknown errors that can't go directly to chute or landed state
  }   
}

/* 
 *  int startup(): 
 *  Beginning state - may be placed in setup()
 *  Waits for button push to arm the rocket and to reset altitude measurements 
 *  returns the nextState of groundidle() if successful 
 */
int startup() {
  int nextState = 0;
  Serial.print(digitalRead(ARM_B1_PIN));
  Serial.print("   ");
  Serial.print(digitalRead(ARM_B2_PIN));

  if(digitalRead(ARM_B1_PIN) == HIGH && digitalRead(ARM_B2_PIN) == HIGH) {
    ++buttonCycles;
    delay(10);
    if (buttonCycles >= 500) {  //hold down both buttons for 5s
      nextState = 1;
      previousState = 0;
      Serial.printf("Rocket armed: Startup-->Groundidle\n");
  
      //calls once to see if bmp is still working, sets all values to 0
      getAlt(alts);
      alts[1] = 0;
      alts[2] = 0;
      alts[3] = 0;
    }
  } else {
    buttonCycles = 0;
  }
  Serial.print("   ");
  Serial.println(buttonCycles);
  return nextState;
}

/* 
 *  int groundidle(): 
 *  State in which the rocket is armed but ignition has not yet been set off
 *  Plays warning sound and collects altitude + orientation data but otherwise stays inactive
 *  Detects liftoff or boost() through an upwards acceleration measurement 
 */
int groundidle() {
  int nextState = 1;
  if (timer >= 1000) {
    digitalWrite(LED_GREEN,HIGH);
    digitalWrite(LED_RED,HIGH);
    if (timer >= 500){
      digitalWrite(LED_GREEN,LOW);
      digitalWrite(LED_RED,LOW);
    }
    timer = 0;
    tone(BUZZER, 4000, 500);
  }

  getAlt(alts);
  orientation(orient);

  /* if acceleration in the x direction (towards nosecone) is greater than 12 */
  if (orient[7] >= 12) { 
    nextState = 2;
    previousState = 1;
    digitalWrite(LED_GREEN,LOW);
    digitalWrite(LED_RED,LOW);
    tone(BUZZER, 6000, 1000); //Victory Screech: runs buzzer for 1s when liftoff is detected; MAX f needed
    Serial.printf("Liftoff detected: Groundidle-->Boost\n");
  }
  return nextState;
}

/* 
 *  int boost():
 *  State in which the rocket is under powered flight
 *  Collects and stores altitude and orientation data
 *  Determines gimbal and servo angle through a DCM applied to the orientation data
 *  Uses a PID loop to move the servo to the desired angle
 *  Detects both Burnout and Freefall - Burnout through decreasing acceleration forward and Freefall through decreasing altitude
 *  May bypass the Burnout stage completely if the acceleration data forward fails
 */
int boost() {

  int nextState = 2;
  tone(BUZZER, 4000, 1000); //runs buzzer for 1s when liftoff is detected

  getAlt(alts);
  orientation(orient);
  //float* gimbalAngle = findGimbalAngles(orient);
  float servoAngle[] = {90, 90}; //PID(gimbalAngle);

  servoPitch.writeMicroseconds(servoAngle[0]);
  servoYaw.writeMicroseconds(servoAngle[1]);

  /*if acceleration in the direction of the rocket is less than 5 m/s2, assume burnout or if apogee is detected without burnout (decreasing altitude) */
  float accelForward = orient[7]*orient[1]+orient[8]*orient[2]+orient[9]*orient[3];  //dot product of acceleration and direction vectors
  if (accelForward <= 5  || alts[3] > 5) {  
    if (accelForward <= 5) {
      Serial.printf("Burnout detected: Boost-->Burnout\n");
      nextState = 3;
    }
    else {
      Serial.printf("Apogee detected: Boost-->Freefall\n");
      nextState = 4;
    }
    previousState = 2;
    servoPitch.write(90);
    servoYaw.write(90);
    alts[3] = 0;
  } 
  return nextState;
}

/* 
 *  int burnout():
 *  State in which the rocket exits powered flight until apogee is reached
 *  Collects and stores altitude and orientation data
 *  Detects decreasing altitude to signal that apogee has been reached, and that the rocket is in freefall()
 */
int burnout(){
  int nextState = 3;

  getAlt(alts);
  orientation(orient);
  
  if (alts[3] > 5){ 
    nextState = 4;
    previousState = 3;
    Serial.printf("Apogee detected: Burnout-->Freefall\n");
  }
  return nextState;
}

/*
 * int freefall():
 * State before reaching the alititude required to safely deploy the parachute
 * Collects and stores altitude and orientation data
 * Parachute currently set to deploy at apogee
 * Skeleton for future projects that may need this state
 */
int freefall(){
  int nextState = 4;

  int chuteDeployAltitude = 1000000; //Determine threshold altitude for deploying parachutes; set to 1000000 due to immediate chute deployment upon reaching apogee
  
  getAlt(alts);
  orientation(orient);

  if (alts[1] <= chuteDeployAltitude){ 
    nextState = 5;
    previousState = 4;
    Serial.printf("Chute Deployment Altitude detected: Freefall-->Chute\n");
  }
  return nextState;
}

/* 
 *  int chute():
 *  State for parachute deployment
 *  Collects and stores altitude and orientation data
 *  Doubles as an emergency state for errors - checks previous state to ensure that it does not go off while on the ground
 *  Checks for freefall acceleration and altitude for a safe time to deploy chute
 *  Releases two chute charges as a redundancy
 *  If both charges fail to deploy the chute, program tries again 1 second after charge 2 until lowest altitude (20m) is reached
 *  Detects landed gravity (similar to hovering, 9m/s2 up) for landing()
 */
int chute(){
  int nextState = 5;

  getAlt(alts);
  orientation(orient);

  /*if the acceleration is below a certain threshold, above a certain altitude (with the exception of altitude data loss), 
      and the previous stage was not a ground stage, deploy chute charges */
  float accelMag = sqrt(orient[7]*orient[7]+orient[8]*orient[8]+orient[9]*orient[9]);
  if (accelMag < 2 && (alts[0] != 1 || alts[1] >= 20) && (previousState >= 2 && previousState != 6)) { 
    //If less than 1 second has passed and the first charge has not deployed, send a charge through the MOSFET1
    if (timer <= 1000 && charge1) {
      digitalWrite(CHUTE_1_PIN, HIGH);
      charge1 = false;
      Serial.println("Chute charge 1 active");
    } else if (timer >= 2000 && timer < 4000 && charge2) {  //If 1 second has passed and the second charge has not deployed, send a charge through the MOSFET2
      digitalWrite(CHUTE_1_PIN, LOW);
      Serial.println("Chute charge 1 deactivated");
      digitalWrite(CHUTE_2_PIN, HIGH);
      Serial.println("Chute charge 2 active");
      charge2 = false;
    } else if (!charge1 && !charge2){  //if both charges have been deployed and acceleration still reads freefall (still going through first if-statement), start a second timer
      digitalWrite(CHUTE_2_PIN, LOW);
      Serial.println("Chute charge 2 deactivated");
      if (timer2Start) {  //so that timer2 only resets once
        timer2 = 0;
        timer2Start = false;
      }
      if (timer2 >= 1000) {  //if 1 second has passed with acceleration still saying freefall, go to noChute failure
        timer = 0;
        charge1 = true;
        charge2 = true;
        timer2Start = true;
        nextState = 5;
      }
    } else {}
  }

  if (accelMag >= 9){ //9 m/s2 to account for gravity once landed
    nextState = 6; 
    previousState = 5;
    Serial.printf("Landing detected: Chute-->Landing\n");
  }
  return nextState;
}

/*
 * int landing():
 * Last state where rocket is grounded after flight
 * Plays beeping noise to be located and to signal that the last stage has been reached
 * Saves all data to the SD card
 */
int landing(){
  int nextState = 6;
  if (timer >= 1000) {  //Victory Beeeps :D or over failure tone :(
    timer = 0;
    tone(BUZZER, 4000, 500);
  } 

//Under current conditions, this will run in a loop indefitely. Either the main loop should stop after landing or the write function call should be called at the end of chute().
  while(1);
  //writeToSD(); 
  return nextState;
}

/*
 * int failure():
 * Placeholder state for failures that do not go directly to chute() or landing()
 */
int failure() {
  orientation(orient);
  Serial.printf("Failure\n");
  while(1);
  int nextStage = 7;
  return nextStage;
}
