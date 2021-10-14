
/*
 * Modified from PID_Test_Main from same repo retrieved on Oct. 13, 2021.
 * 
 * Manual Change Log (Complementary to git)
 * 10/13/2021 : v0.1.0 : First Test version created, needs to be tested
 */


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>

// Defining Pins for external devices
#define SERVO_PIN_Y 0 // pin for servo that rotates gimble about y axis
#define SERVO_PIN_Z 1 // pin for servo that rotates gimble about z axis

#define ARM_B1 5
#define ARM_B2 6

#define LED_GREEN 7
#define LED_RED 8

#define PIEZO 4

#define DUTY_CYCLE 1000 // Length of total LED duty cycle in millis
#define LED_DUTY_CYCLE .05 // LED will be on for this fraction of duty cycle

#define SECONDS_TO_ARM 10 // Seconds

#define MAX_GIMBAL_ANGLE 10 // Degrees

#define RAD_TO_DEG 57.295779513082320876798154814105
#define SERVO_TO_GIMBAL 0.43; // Rad/Rad  (or deg/deg) to convert wanted gimble angle to servo angle with inverse of this.

const double P = 0.1;
const double I = 0; // given that inclination is always positive, this term will only accumulate (should not use for this controller)
const double D = 0;

// Defining Variables that carry from cycle to cycle
double errorLast = 0; // Rad
double errorSum = 0; // Rad * Sec

unsigned long timeStart; // Time in Millis at begining of ARM Sequence
unsigned long timeLast; // Time in millis at last PID loop

double targetInclination = 0; // Target Inclination for PID loop in RAD
// double targetHeading = 0; // Target absolute heading for PID loop (not implemented)

// Used in Loop to be updated
double i;
double w;
double a;

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

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
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
  tone(PIEZO,4000);
  digitalWrite(LED_GREEN,HIGH);

  testGimbal();

  // Ready to Arm
  noTone(PIEZO);
  delay (500);
  tone(PIEZO,4000, 500);
  delay (1000);
  tone(PIEZO,4000, 500);
  
  timeStart = millis();
  
  while(true){
      if(digitalRead(ARM_B1) == HIGH && digitalRead(ARM_B2) == HIGH) {
        Serial.print("Seconds till armed: ");
        Serial.println(SECONDS_TO_ARM - ((millis() - timeStart)/1000));
        if (millis() - timeStart > (SECONDS_TO_ARM * 1000)) {  //hold down both buttons for specified time
          Serial.printf("Rocket armed: Startup-->Groundidle\n");
          tone(PIEZO,4000,1000);
          break;
        }
      } else {
        timeStart = millis();
        Serial.print("Seconds till armed: ");
        Serial.println(SECONDS_TO_ARM);
      }

      // Half duty cycle to let user know that it is waiting to be armed
      if ((millis() - timeStart) % (DUTY_CYCLE / 2) <= (DUTY_CYCLE / 2) * LED_DUTY_CYCLE)
      {
      digitalWrite(LED_GREEN,HIGH);
      } else
      {
        digitalWrite(LED_GREEN,LOW);  
      }
      
      delay(100);
   }

  // Rocket is now armed. Stay away!
  Serial.println("Rocket armed, why are you still looking at serial!?");
  digitalWrite(LED_GREEN,LOW);
  tone(PIEZO,4000,1000);
  digitalWrite(LED_RED,HIGH);
  delay(2000);
  
}

void loop(void)
{
  getOrient(&i, &w);

  a = PID(i - targetInclination);

  angle2Servo(a, w);

  if ((millis() - timeStart) % DUTY_CYCLE <= DUTY_CYCLE * LED_DUTY_CYCLE)
  {
    digitalWrite(LED_GREEN,HIGH);
    digitalWrite(LED_RED,HIGH);
  } else
  {
    digitalWrite(LED_GREEN,LOW);
    digitalWrite(LED_RED,LOW);
  }

  delay(10);
}

//argument: double DCM[][3]

/* Using quaternion to euler XZX conversion.
* Get inclination through the 2nd rotation
* Get orientation of servos from 3rd rotation
* implementation as seen in MATLAB qparts2feul.m
*/
void getOrient(double *i, double *w)
{
  imu::Quaternion quat = bno.getQuat();

  double q[4] = {quat.w(), quat.x(), quat.y(), quat.z()};

  // Inclination: angle from upwards x-axis
  // we want this to be 0, Use PID to do so
  *i = acos(2 * (q[0]*q[0] + q[1]*q[1]) - 1); // Rad
  
  // angle of Z axis away from inertial ZY plane measured on rotated ZY frame
  // needed to determine how servos will partition wanted angle.
  *w = atan2(2 * (q[0]*q[2] + q[3]*q[1]), 2 * (q[0]*q[3] - q[1]*q[2])); // Rad
}

/*
 * Returns Adjusted absolute angle for servo.
 * Input i is current inclination (Rad)
 */

double PID(double e)
{
  unsigned long timeNow = millis();
  unsigned long dt = timeNow - timeLast;
  
  errorSum += e * dt / 1000;
  
  double out = P * e + I * errorSum + ((e - errorLast) * 1000 / dt);
  
  errorLast = e;
  timeLast = timeNow;

  return out;
}

/*
 * Given wanted absolute angle (Rad) and roll (Rad), 
 * calculates needed angle for servos and transmit it to them
 */

void angle2Servo(double a, double w)
{
  double angle = min(a, MAX_GIMBAL_ANGLE / RAD_TO_DEG);
  
  double servoYRad = - tan(sin(w) * atan(angle)) / SERVO_TO_GIMBAL;
  double servoZRad = tan(cos(w) * atan(angle)) / SERVO_TO_GIMBAL;

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
  angle2Servo(0, 0);

  double wMax = PI * 2;
  double aMax = MAX_GIMBAL_ANGLE / RAD_TO_DEG;
  double w_to_a = aMax / wMax;

  // Spiral Outwards
  for (w = 0; w < wMax; w += .01)
  {
      angle2Servo(w * w_to_a, w);
      delay(10);
  }

  // One full rotation at max angle
  for (w = 0; w < wMax; w += .01)
  {
      angle2Servo(aMax, w);
      delay(10);
  }

  // Spiral Inwards
  for (w = 0; w < wMax; w += .01)
  {
      angle2Servo(aMax - (w * w_to_a), w);
      delay(10);
  }

  // Re-Center gimbal
  angle2Servo(0, 0);
  
}
