#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>
#include <SdFat.h>
#include <RingBuf.h>
#include <TeensyThreads.h>

// Use Teensy SDIO
#define SD_CONFIG  SdioConfig(FIFO_SDIO)

const int BUZZER = 4;

const int TONE_SUCCESS = 523; // In Hz, Currently C5
const int TONE_FAILURE = 261; // In Hz, currently C4
const int TONE_VICTORY = 1046; // In Hz, Currently C6

const int LED_GREEN = 7;
const int LED_RED = 8;

const int PRECISION = 6; // Number of Decimal places recorded
const int MAGNITUDE = 4; // Order of magnitude of maximum number to be recorded.
const int INT_BUFF_LEN = MAGNITUDE + PRECISION + 3; // + 3 for decimal point, - sign, & null delimiter
const int MAX_DATA_VALUE = pow(10, MAGNITUDE) - 1;
const int MAX_TIME_VALUE = 10000; // hour of runtime in millis

const int LOG_FILE_SIZE = 100 * 100 * MAX_TIME_VALUE; // Enough bytes for hour of run time at 100Hz
const char LOG_FILENAME[12] = "datalog.csv";
const int RING_BUF_CAPACITY = (INT_BUFF_LEN * 14 + 2)*512; // Enough bytes to hold over 1s of data at 100Hz

int logTime;

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;

size_t maxUsed; // used to understand buffer overrun

IntervalTimer pulse;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int SD_CARD = BUILTIN_SDCARD;

Threads::Mutex rbLock;

void setup() {
  
  // Set Pin mode for status LED
  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_RED,OUTPUT);

  // Connect to Serial
  Serial.begin(9600);

  Serial.print("Initializing SD card...");

  // Initialize the SD.
  if (!sd.begin(SD_CONFIG)) {
    sd.initErrorHalt(&Serial);
    tone(BUZZER, TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    while(1);
  }

  Serial.println("Card initialized.");

  Serial.print("Creating and Opening Logfile...");
  
  // Open or create file - truncate existing file.
  if (!file.open(LOG_FILENAME, O_RDWR | O_CREAT | O_TRUNC)) 
  {
    Serial.println("File opening failed\n");
    tone(BUZZER, TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    while(1);
  }
  // File must be pre-allocated to avoid huge
  // delays searching for free clusters.
  if (!file.preAllocate(LOG_FILE_SIZE)) 
  {
    Serial.println("Preallocate failed\n");
    file.close();
    tone(BUZZER, TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    while(1);
  }
  
  // initialize the RingBuf.
  rb.begin(&file);

  Serial.println("Ringbuffer and Logfile initialized.");
  
  // Add Header to file
  // datalogInit();

  // Sensors Initialized. Celebrate!
  digitalWrite(LED_GREEN, HIGH);
  tone(BUZZER, TONE_SUCCESS, 500);
  delay(500);
  tone(BUZZER, TONE_VICTORY, 250);
  delay(250);

  Serial.println("Starting BNO055 Thread: ");
  // threads.addThread(BNO055Thread);
  pulse.begin(BNO055Thread, 10000);
}

void loop() 
{
  // put your main code here, to run repeatedly:
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
      return;
    }
  }
  if (millis() > MAX_TIME_VALUE) 
  {
    Serial.println("Times Up - quitting.");
    file.truncate();
    file.close();
    while (1){}
  }
  

}

void BNO055Thread()
{

  //while(1)
  //{
    char str[100];
    sprintf(str, "0001,0002,0003,0004,0005,0006,0007,0008,0009,0010,0011,0012,00%d\n", int(millis() - logTime));
    logTime = millis();
    // Not sure if Lock and unlock are needed but put in just in case. If it works well with locking and unlocking, keep them.
    rb.memcpyIn(str, 65);
    
    //while(millis() - logTime < 1);

  //}
  
}

int datalogInit()
{

  //Since total Header is longer than 512 Bytes (the buffer for SD card writing) it has to be split
  // Split into multiple strcat operations to improve readability.
  char header[512]; 
  strcpy(header, "(double) Quaternion W,(double) Quaternion X,(double) Quaternion Y,(double) Quaternion Z,");
  strcat(header, "(double) Linear Acceleration X [m/s^2],(double) Linear Acceleration Y [m/s^2],(double) Linear Acceleration Z [m/s^2],");
  strcat(header, "(double) Angular Velocity X [rad/s],(double) Angular Velocity Y [rad/s],(double) Angular Velocity Z [rad/s],");
  strcat(header, "(double) Gravitational Vector X [m/s^2],(double) Gravitational Vector Y [m/s^2],(double) Gravitational Vector Z [m/s^2];");

  rb.memcpyIn(header, 512);
  
  return 1;
}
