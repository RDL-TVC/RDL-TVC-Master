
/*
 * Modified from PID_Test_Main from same repo retrieved on Oct. 13, 2021.
 * 
 * Manual Change Log (Complementary to git)
 * 10/13/2021 : v0.1.0 : First Test version created, needs to be tested
 * 10/19/2021 : v0.2.0 : Still Needs to be tested, however based on data recieved from BNO_Test_Inclination, code has been modified.
 * 11/09/2021 : v0.3.0 : Tested dry run. Gives expected result, however needs to be tested with thrust applied.
 * 11/13/2021 : v0.3.1 : Reformated into seperate functions. Ready to test combined Interrupt based PID and datalogging.
 */


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <PWMServo.h>
#include <SD.h>
#include <SdFat.h>
#include <RingBuf.h>
#include <TeensyThreads.h>

#define RAD_TO_DEG 57.295779513082320876798154814105
#define SERVO_TO_GIMBAL 0.43; // Rad/Rad  (or deg/deg) to convert wanted gimble angle to servo angle with inverse of this.

// Use Teensy SDIO
#define SD_CONFIG  SdioConfig(FIFO_SDIO)


ThreadWrap(Serial, SerialXtra);
#define Serial ThreadClone(SerialXtra)
// Defining Pins for external devices
const int SERVO_PIN_X = 1; // pin for servo that rotates gimble about x axis
const int SERVO_PIN_Y = 0; // pin for servo that rotates gimble about y axis
const int SERVO_LIMITING = 20;
const int SERVO_FREQUENCY = 250; // Hz
const int SERVO_RESOLUTION = 10; // Bits

const int MAX_GIMBAL_ANGLE = 10; // Degrees

const int ARM_B1 = 5;
const int ARM_B2 = 6;

const int SECONDS_TO_ARM = 10; // Seconds

const int LED_GREEN = 7;
const int LED_RED = 8;

const int DUTY_CYCLE = 100; // Length of total LED duty cycle in millis
const float LED_TIME_ON = .05; // LED will be on for this fraction of duty cycle

const double P = 0.3; // 0.1;
const double I = 0; // given that inclination is always positive, this term will only accumulate (should not use for this controller)
const double D = 0;

const char LOG_FILENAME[14] = "threadlog.csv";
const unsigned long MAX_TIME = 1000 * 3600;
const unsigned long LOG_FILE_SIZE = 72*MAX_TIME / 10 + 153 + 20;
const int RING_BUF_CAPACITY = 72*200;

// Defining Variables that carry from cycle to cycle
volatile double errorLast = 0; // Rad
volatile unsigned long timeLast = 0;
volatile int lastServoWrite[2] = {1500, 1500};

volatile unsigned long lastPID = 0; // ms
volatile unsigned long lastWrite = 0; // ms

const double targetInclination = 0; // Target Inclination for PID loop in RAD
// double targetHeading = 0; // Target absolute heading for PID loop (not implemented)

volatile size_t maxUsed; // used to understand buffer overrun

// Used in Loop to be updated

bool dutyCyclePrintFlag = false;

elapsedMillis armTimer = 0;

// Defining Sensor and servo objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);

SdFs sd;
FsFile file;

RingBuf<FsFile, RING_BUF_CAPACITY> rb;

const int SD_CARD = BUILTIN_SDCARD;

void setup(void)
{

  if (!(IOInit() & logInit() & BNOInit() & servoInit()))
  { // Something failed to intialize
    while(1);
  }
  /*
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
  */
  // Rocket is now armed. Stay away!
  Serial.println("Rocket armed, why are you still looking at serial!?");
  digitalWrite(LED_GREEN,LOW);
  digitalWrite(LED_RED,HIGH);

  //pulse.begin(tick, dt * 1000);
  //threads.setDefaultTimeSlice(10);
  threads.addThread(PIDThread);
  //threads.addThread(logThread);
  //threads.setSliceMillis(10);
  
}

void loop(void)
{
  long unsigned timeNow = millis();
  size_t n = rb.bytesUsed();
  if ((n + file.curPosition()) > (LOG_FILE_SIZE - 20)) 
  {
    Serial.println("File full - quiting.");
    file.truncate();
    file.close();
    return;
  } if (n > maxUsed) 
  {
    maxUsed = n;
  }
  if (n >= 512 && !file.isBusy()) 
  {
    // Not busy only allows one sector before possible busy wait.
    // Write one sector from RingBuf to file.
    if (512 != rb.writeOut(512)) 
    {
      Serial.println("writeOut failed"); 
      // Probably not the best call to give up whole program if writeout fails
      file.truncate();
      file.close();
      return;
    } else{
      file.sync();
      /*
      Serial.print("Last File Write: ");
      Serial.println(millis() - timeNow);
      Serial.print("Bytes Used In Buffer: ");
      Serial.println(n);
      */
    }
  }
  if (millis() > MAX_TIME) 
  {
    Serial.println("Times Up - quitting.");
    file.truncate();
    file.close();
    return;
  }
  
  threads.yield();
  
}
