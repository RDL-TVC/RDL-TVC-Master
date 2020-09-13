#include <SFE_BMP180.h>
#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

////////////////////TVC VARIABLES////////////////////

float P_pitch = 0.5;
float I_pitch = 0.2;
float D_pitch = 0.1;
float P_yaw = 0.5;
float I_yaw = 0.2;
float D_yaw = 0.1;
float error_pitch = 0;
float error_yaw = 0;

Servo servo_pitch;
double pitch_trim = 80; // positive trim is positive x
double pitch_center = 1500 + pitch_trim;
double pitch_pos = pitch_center
double pitch_max = 1730;
double pitch_min = 1245;
 
Servo servo_yaw;
double yaw_trim = 85; // positive trim is positive y
double yaw_center = 1500 + yaw_trim;
double yaw_pos = yaw_center
double yaw_max = 1770;
double yaw_min = 1285;

unsigned long time,time_prev,dT,time_liftoff; // current time, time last cycle, time since last cycle, time at liftoff

int16_t aX,aY,aZ,temp,gX,gY,gZ;
float gX_cal,gY_cal,gZ_cal; 
float pitch,roll,yaw; //  x, y, and z respectively
float pitch_prev,roll_prev,yaw_prev;

float desired_pitch = 0;
float desired_yaw = 0;

////////////////////CHUTE DEPLOYMENT VARIABLES////////////////////

float A,A_last,dA,A_max = 0; // current alt for cycle, alt last cycle, delta alt since last cycle, max alt overall
float P,P_prelaunch; // current pressure for cycle, pressure on ground before launch
float V,V_max = 0; // velocity during cycle, max velocity overall
int descent_count = 0; // number of times alt has been below highest alt in a row

boolean ignition_detected = false;
boolean liftoff_detected = false;
boolean drogue_deployed = false;
boolean main_deployed = false;
boolean error_occured = false;

int deploy_drogue_count = 8; // number at which A_count triggers drogue deployment
int deploy_main_alt = 8; // alt at which main chute deploys
float liftoff_threshold = 1; // alt at which a liftoff is detected
float landing_threshold = 0.1; // delta alt at which a landing is detected (alt is not changing much)

////////////////////DATA COLLECTION VARIABLES////////////////////

int file_number = 0; // starting number tag added to text file name
String file_name = "data_"; // name of file, minus the number tag
String file_full_name = file_name + file_number + ".txt"; 

////////////////////SETUP AND LOOP////////////////////

void setup()
{
  Serial.begin(9600);
  delay(500);
  Serial.println("Serial start")
  setupSD();
  setupPressure();
  setupTVC();  
  setupIMU();
  
  time_prev = millis();
  data.print(Pr_prelaunch);
  data.print(" mb at 0 m: ");
  data.print(millis() / 1000.0);
  data.println(" s");

  ignitionDetected = true;
}

void loop()
{
  time_prev = time;
  time = millis();
  dT = time - time_prev;
  
  P = (getPressure() + getPressure()) / 2;
  A_last = A;
  A = pressure.altitude(P,P_prelaunch);
  dA = A - A_last;
  
  readIMU();
  gX -= gXcal;                                                
  gY -= gYcal;                                                
  gZ -= gZcal;

  pitchPrev = pitch;
  rollPrev = roll;
  yawPrev = yaw;
  
  pitch += gX / 65.5 * (dT) / 1000; 
  roll += gY / 65.5 * (dT) / 1000; 
  yaw += gZ / 65.5 * (dT) / 1000;  

  if(ignition_detected)
  {
    recordData();
  }

  if(liftoff_detected)  // max tracking
  {
    V = dA / (dT / 1000.0);
    
    if (V > V_max)
    { 
      V_max = V;
    }

    if(A > A_max)
    {
      A_max = A;
    }

    descentDetection();
  }

  liftoffDetection();

  if ((A_count >= deployDrogue_count) and (not drogue_deployed)) // deploy drogue if descending for long enough
  {
    deployDrogue();
  }

  if (drogue_deployed and (not main_deployed) and (A <= deployMain_alt)) // deploy main at or below set altitude
  {
    deployMain();
  }

  landingDetection();

  noTone(0);
}
