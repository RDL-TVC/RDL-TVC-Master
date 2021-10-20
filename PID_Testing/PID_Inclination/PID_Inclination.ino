
/*
 * Modified from PID_Test_Main from same repo retrieved on Oct. 13, 2021.
 * 
 * Manual Change Log (Complementary to git)
 * 10/13/2021 : v0.1.0 : First Test version created, needs to be tested
 * 10/19/2021 : v0.2.0 : Still Needs to be tested, however based on data recieved from BNO_Test_Inclination, code has been modified.
 */


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

#define RAD_TO_DEG 57.295779513082320876798154814105
#define SERVO_TO_GIMBAL 0.43; // Rad/Rad  (or deg/deg) to convert wanted gimble angle to servo angle with inverse of this.

// Defining Pins for external devices
const int SERVO_PIN_Y = 0; // pin for servo that rotates gimble about y axis
const int SERVO_PIN_Z = 1; // pin for servo that rotates gimble about z axis

const int MAX_GIMBAL_ANGLE = 10; // Degrees

const int ARM_B1 = 5;
const int ARM_B2 = 6;

const int SECONDS_TO_ARM = 10; // Seconds

const int LED_GREEN = 7;
const int LED_RED = 8;

const int DUTY_CYCLE = 1000; // Length of total LED duty cycle in millis
const float LED_TIME_ON = .05; // LED will be on for this fraction of duty cycle

const int PIEZO = 4;

const int TONE_SUCCESS = 523; // In Hz, Currently C5
const int TONE_FAILURE = 261; // In Hz, currently C4
const int TONE_VICTORY = 1046; // In Hz, Currently C6

const double P = 0.1;
const double I = 0; // given that inclination is always positive, this term will only accumulate (should not use for this controller)
const double D = 0;

// Defining Variables that carry from cycle to cycle
double errorLast = 0; // Rad
double errorSum = 0; // Rad * Sec

double targetInclination = 0; // Target Inclination for PID loop in RAD
// double targetHeading = 0; // Target absolute heading for PID loop (not implemented)

// Used in Loop to be updated
double a;

bool dutyCyclePrintFlag = false;

elapsedMillis PROGRAM_TIME = 0;
elapsedMillis armTimer = 0;
elapsedMillis PIDTimer = 0;

// Defining Sensor and servo objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);

Servo servoY;
Servo servoZ;

void setup(void)
{
  // Initialise Servos
  servoY.attach(SERVO_PIN_Y);
  servoZ.attach(SERVO_PIN_Z);
  
  // Set Pin mode for status LED
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_RED,OUTPUT);

  // Connect to Serial
  Serial.begin(9600);

  Serial.print("Initializing BNO055...");
  
  if (!bno.begin())
  { // BNO055 was not able to initialize
    Serial.println("BNO055 failed to initialize.");
    tone(PIEZO,TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    while(1);
  }

  Serial.println("BNO055 Initialized. Now Calibrating.");

  delay(1000);
  bno.setExtCrystalUse(true);

  // Calibrating BNO055. Do not Disturb!
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
  
  // Sensors Initialize Test Gimbal
  tone(PIEZO,TONE_SUCCESS);
  digitalWrite(LED_GREEN,HIGH);

  testGimbal();

  // Ready to Arm
  noTone(PIEZO);
  delay (500);
  tone(PIEZO,TONE_SUCCESS, 500);
  delay (1000);

  armTimer = 0;
  bool armTimerPrintFlag = false;
  
  while(1)
  {
    if(digitalRead(ARM_B1) == HIGH && digitalRead(ARM_B2) == HIGH) 
    { // If both arming buttons are pressed, start counting down.
      if ((armTimer % 1000) < 500)
      {
        if (!armTimerPrintFlag)
        {
          Serial.print("Arming, Seconds till armed: ");
          Serial.println(SECONDS_TO_ARM - (armTimer / 1000));
          armTimerPrintFlag = true;
        }
      } else
      {
        armTimerPrintFlag = false;
      }
      if (armTimer > (SECONDS_TO_ARM * 1000)) 
      { // Countdown finished. Rocket has been armed
        Serial.println("Rocket armed");
        tone(PIEZO,TONE_SUCCESS,1000);
        delay(1000);
        break;
      }
    } else 
    { // Reset countdown if arming buttons are not active
      if (LEDBlink(LED_GREEN, DUTY_CYCLE, LED_TIME_ON))
      {
        if (!dutyCyclePrintFlag)
        {
          Serial.println("Idle");
          dutyCyclePrintFlag = true;
        }
      } else 
      {
        dutyCyclePrintFlag = false;
      }
      armTimer = 0;
    }
    
  }

  // Rocket is now armed. Stay away!
  Serial.println("Rocket armed, why are you still looking at serial!?");
  digitalWrite(LED_GREEN,LOW);
  digitalWrite(LED_RED,HIGH);
  
  tone(PIEZO, TONE_SUCCESS, 500);
  delay(500);
  tone(PIEZO, TONE_VICTORY, 250);
  delay(250);

  PIDTimer = 0;
  
}

void loop(void)
{

  double i;
  double yz[2];
  
  getOrient(&i, yz);

  double a = PID(i - targetInclination);

  angle2Servo(a, yz);

  if (LEDBlink(LED_GREEN, DUTY_CYCLE, LED_TIME_ON))
  {
    if (!dutyCyclePrintFlag)
    {
      Serial.printf("Inclination: %d, PID Angle Out: %d, Y Comp: %f, Z Comp: %f", i, a, yz[0], yz[1]);
      dutyCyclePrintFlag = true;
    }
  } else 
  {
    dutyCyclePrintFlag = false;
  }
  
  LEDBlink(LED_RED, DUTY_CYCLE, LED_TIME_ON);

}

/* Using quaternion to euler XZX conversion.
* Get inclination through the 2nd rotation
* Get orientation of servos from 3rd rotation
* implementation as seen in MATLAB qparts2feul.m
*/
void getOrient(double *i, double yz[2])
{
  imu::Vector<3> g = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  double grav[3] = {g.x(), g.y(), g.z()};

  // Inclination: angle from upwards x-axis
  // we want this to be 0, Use PID to do so
  *i = acos(grav[0]/sqrt(grav[0] * grav[0] + grav[1] * grav[1] + grav[2] * grav[2]));

  // other code relies on yz being a unit vector.

  double gyz = sqrt(grav[1]*grav[1] + grav[2]*grav[2]);

  yz[0] = grav[1]/gyz;
  yz[1] = grav[2]/gyz;
}

/*
 * Returns Adjusted absolute angle for servo.
 * Input i is current inclination (Rad)
 */

double PID(double e)
{
  unsigned long dt = PIDTimer;
  
  errorSum += e * dt / 1000;
  
  double out = P * e + I * errorSum + ((e - errorLast) * 1000 / dt);
  
  errorLast = e;
  PIDTimer = 0;

  return out;
}

/*
 * Given wanted absolute angle (Rad) and roll (Rad), 
 * calculates needed angle for servos and transmit it to them
 */

void angle2Servo(double a, double yz[2])
{
  if (a == 0)
  { // To protect against div by 0 errors
    // Center Servos
    servoY.writeMicroseconds(1500);
    servoZ.writeMicroseconds(1500);
    return;
  }
  
  double angle = min(a, MAX_GIMBAL_ANGLE / RAD_TO_DEG);

  double x = 1/tan(angle);
  
  double servoYRad = atan2(yz[2], x) / SERVO_TO_GIMBAL;
  double servoZRad = atan2(yz[1], x) / SERVO_TO_GIMBAL;

  double servoYMicro = map(servoYRad, -PI/3, PI/3, 900, 2100);
  double servoZMicro = map(servoZRad, -PI/3, PI/3, 900, 2100);

  servoY.writeMicroseconds(servoYMicro);
  servoZ.writeMicroseconds(servoZMicro);

}
/*
 * Test routine to make sure gimbal is functioning within range
 */
void testGimbal()
{
  // center gimbal
  double yz[2] = {0, 0};
  
  angle2Servo(0, yz);

  double wMax = PI * 2;
  double aMax = MAX_GIMBAL_ANGLE / RAD_TO_DEG;
  double w_to_a = aMax / wMax;

  // Spiral Outwards
  for (float w = 0; w < wMax; w += .01)
  {
    yz[0] = cos(w);
    yz[1] = sin(w);
    angle2Servo(w * w_to_a, yz);
    delay(10);
  }

  // One full rotation at max angle
  for (float w = 0; w < wMax; w += .01)
  {
    yz[0] = cos(w);
    yz[1] = sin(w);
    angle2Servo(aMax, yz);
    delay(10);
  }

  // Spiral Inwards
  for (float w = 0; w < wMax; w += .01)
  {
    yz[0] = cos(w);
    yz[1] = sin(w);
    angle2Servo(aMax - (w * w_to_a), yz);
    delay(10);
  }

  // Re-Center gimbal
  angle2Servo(0, yz);
}

int LEDBlink(int LED, unsigned int dutyCycle, float ratio)
{
  
  if (PROGRAM_TIME % (dutyCycle) <= (dutyCycle) * ratio)
  { // If at time during cycle where LED should be on.
    digitalWrite(LED, HIGH);
    return 1;
  } else
  {
    digitalWrite(LED, LOW);
    return 0;
  }
  
}
