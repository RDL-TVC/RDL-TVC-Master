#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>
#include <SdFat.h>
#include <RingBuf.h>

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
const int MAX_TIME_VALUE = 3600000; // hour of runtime in millis

const int LOG_FILE_SIZE = (INT_BUFF_LEN * 14 + 2) * MAX_TIME_VALUE / 10; // Enough bytes for hour of run time at 100Hz
const char LOG_FILENAME[12] = "datalog.csv";
const int RING_BUF_CAPACITY = (INT_BUFF_LEN * 14 + 2)*512; // Enough bytes to hold over 1s of data at 100Hz

SdFs sd;
FsFile file;

// RingBuf for File type FsFile.
RingBuf<FsFile, RING_BUF_CAPACITY> rb;

size_t maxUsed; // used to understand buffer overrun

IntervalTimer pulse;

Adafruit_BNO055 bno = Adafruit_BNO055(55);

const int SD_CARD = BUILTIN_SDCARD;

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
  
  Serial.print("Initializing BNO055...");
  
  if (!bno.begin())
  { // BNO055 was not able to initialize
    Serial.println("BNO055 failed to initialize.");
    tone(BUZZER,TONE_FAILURE);
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

  Serial.println("Calibration of BNO055 finished.");

  // Add Header to file
  datalogInit();

  // Sensors Initialized. Celebrate!
  digitalWrite(LED_GREEN, HIGH);
  tone(BUZZER, TONE_SUCCESS, 500);
  delay(500);
  tone(BUZZER, TONE_VICTORY, 250);
  delay(250);

  Serial.println("Starting Interrupt Pulse.");
  pulse.begin(BNO055ISR,10000);
  
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
  }
  

}

void BNO055ISR()
{

  uint32_t logTime = millis();
  
  // Get Quaternion data from BNO055.
  imu::Quaternion quat = bno.getQuat();
  // Get Linear Acceleration (No Gravity) data [m/s^2].
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // Get Gyroscope data [rad/s].
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Get Diretion of Gravitational Vector [m/s^2].
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  char databuf[14 * INT_BUFF_LEN + 2]; // Space for 12 * 14 characters + 1 "," and "\n\0" at end
  char dtostrbuf[INT_BUFF_LEN]; // space for "-0000.000000\0"

  sprintf(databuf, "%d", min((int)logTime, MAX_TIME_VALUE));
  
  dtostrf(min(quat.w(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcpy(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(quat.x(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(quat.y(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(quat.z(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");

  dtostrf(min(accel.x(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(accel.y(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(accel.z(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");

  dtostrf(min(gyro.x(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(gyro.y(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(gyro.z(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  
  dtostrf(min(grav.x(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(grav.y(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(grav.z(), MAX_DATA_VALUE), 0, PRECISION, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, "\n");

  rb.memcpyIn(databuf, 14 * INT_BUFF_LEN + 2);
}

int datalogInit()
{

  //Since total Header is longer than 512 Bytes (the buffer for SD card writing) it has to be split
  // Split into multiple strcat operations to improve readability.
  char header[512]; 
  strcpy(header, "(double) Quaternion W,(double) Quaternion X,(double) Quaternion Y,(double) Quaternion Z,");
  strcat(header, "(double) Linear Acceleration X [m/s^2],(double) Linear Acceleration Y [m/s^2],(double) Linear Acceleration Z [m/s^2],");
  strcat(header, "(double) Angular Velocity X [rad/s],(double) Angular Velocity Y [rad/s],(double) Angular Velocity Z [rad/s],");
  strcat(header, "(double) Gravitational Vector X [m/s^2],(double) Gravitational Vector Y [m/s^2],(double) Gravitational Vector Z [m/s^2]\n");

  rb.memcpyIn(header, 512);
  
  return 1;
}

void BNO055IntEnable()
{
    
    // Need to be on page 0 to get into config mode
    adafruit_bno055_page_t lastPage = _page;
    if (lastPage != PAGE_0) setPage(PAGE_0);

    // Must be in config mode, so force it
    adafruit_bno055_opmode_t lastMode = _mode;
    setMode(OPERATION_MODE_CONFIG);

    // Change to page 1 for interrupt settings
    setPage(PAGE_1);

    // Set duration (bits 1-6)
    int8_t intSettings = (int8_t)(read8(ACC_INT_Settings_ADDR));
    intSettings = sliceValueIntoRegister(duration, intSettings, ACC_INT_Settings_AM_DUR_MSK, ACC_INT_Settings_AM_DUR_POS);
    write8(ACC_INT_Settings_ADDR, intSettings);

    // Set the threshold (full byte)
    int8_t threshSettings = (int8_t)(read8(ACC_AM_THRES_ADDR));
    threshSettings = sliceValueIntoRegister(threshold, threshSettings, ACC_AM_THRES_MSK, ACC_AM_THRES_POS);
    write8(ACC_AM_THRES_ADDR, threshSettings);

    // Enable the interrupt
    setInterruptEnableAccelAM(ENABLE);

    // Fire on the pin
    setInterruptMaskAccelAM(ENABLE);

    delay(30);

    // Return the mode to the last mode
    setPage(PAGE_0);
    setMode(lastMode);

    // Change the page back to whichever it was initially
    if (lastPage != PAGE_0) setPage(lastPage);
  
}
