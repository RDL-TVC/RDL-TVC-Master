#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SD.h>

const int BUZZER = 4;

const int TONE_SUCCESS = 523; // In Hz, Currently C5
const int TONE_FAILURE = 261; // In Hz, currently C4
const int TONE_VICTORY = 1046; // In Hz, Currently C6

const int LED_GREEN = 7;
const int LED_RED = 8;

const int dataPrecision = 6; // Number of Decimal places recorded
const int dataMag = 4; // Order of magnitude of maximum number to be recorded.
const int dbuflen = dataMag + dataPrecision + 3; // + 3 for decimal point, - sign, & null delimiter
const int dataMax = pow(10, dataMag) - 1;
const int timeMax = 3600000; // hour of runtime

elapsedMillis PROGRAM_TIME = 0;

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
                                                                                                                                              
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CARD))
  { // SD Card Failed to initialize, indicate and return failure
    Serial.println("Card failed to initialize.");
    tone(BUZZER, TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    while(1);
  }
  
  Serial.println("Card initialized.");

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

void loop() {
  // put your main code here, to run repeatedly:

}

void BNO055ISR()
{

  
  // Get Quaternion data from BNO055.
  imu::Quaternion quat = bno.getQuat();
  // Get Linear Acceleration (No Gravity) data [m/s^2].
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  // Get Gyroscope data [rad/s].
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // Get Diretion of Gravitational Vector [m/s^2].
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  char databuf[14 * dbuflen + 1]; // Space for 12 * 14 characters + 1 "," and "\n\0" at end
  char dtostrbuf[dbuflen]; // space for "-0000.000000\0"

  sprintf(databuf, "%d", min((int)PROGRAM_TIME, timeMax));
  
  dtostrf(min(quat.w(), dataMax), 0, dataPrecision, dtostrbuf);
  strcpy(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(quat.x(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(quat.y(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(quat.z(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");

  dtostrf(min(accel.x(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(accel.y(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(accel.z(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");

  dtostrf(min(gyro.x(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(gyro.y(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(gyro.z(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  
  dtostrf(min(grav.x(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(grav.y(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, ",");
  dtostrf(min(grav.z(), dataMax), 0, dataPrecision, dtostrbuf);
  strcat(databuf, dtostrbuf);
  strcat(databuf, "\n");

  
}

int datalogInit()
{
  char fileName[25];
  int fileNum = 1;
  
  sprintf(fileName, "Test %d datalog.csv", fileNum);
  
  while (SD.exists(fileName))
  {
    fileNum++;
    sprintf(fileName, "Test %d datalog.csv", fileNum);
  }

  // Using O_CREAT | O_WRITE Instead of FILE_WRITE and manually flushing after each print is faster by a factor of 100,
  // Note make sure to flush buffer after each print and do not over run buffer.
  File datalog = SD.open(fileName, O_CREAT | O_WRITE | O_APPEND);

  //Since total Header is longer than 512 Bytes (the buffer for SD card writing) it has to be split
  // Split into multiple strcat operations to improve readability.
  char header[512]; 
  strcpy(header, "(double) Quaternion W,(double) Quaternion X,(double) Quaternion Y,(double) Quaternion Z,");
  strcat(header, "(double) Linear Acceleration X [m/s^2],(double) Linear Acceleration Y [m/s^2],(double) Linear Acceleration Z [m/s^2],");
  strcat(header, "(double) Angular Velocity X [rad/s],(double) Angular Velocity Y [rad/s],(double) Angular Velocity Z [rad/s],");
  strcat(header, "(double) Gravitational Vector X [m/s^2],(double) Gravitational Vector Y [m/s^2],(double) Gravitational Vector Z [m/s^2],");

  if (datalog) 
  { // File is open, Write initial header.
    datalog.print(header);
    datalog.close();
  } else 
  { // If the file isn't open, pop up an error:
    Serial.print("Error opening ");
    Serial.println(fileName);
    tone(BUZZER,TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    while(1);
  }

  return 1;
}

int getOrient(double q[4], double aV[3], double avV[3], double gV[3]) {

  // TODO: check if bno still works

  // Get Quaternion data from BNO055.
  imu::Quaternion quat = bno.getQuat();
  q[0] = quat.w();
  q[1] = quat.x();
  q[2] = quat.y();
  q[3] = quat.z();
  
  // Get Linear Acceleration (No Gravity) data [m/s^2].
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  aV[0] = accel.x();
  aV[1] = accel.z();
  aV[2] = accel.y();  

  // Get Gyroscope data [rad/s].
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  avV[0] = gyro.x();
  avV[1] = gyro.z();
  avV[2] = gyro.y();

  // Get Diretion of Gravitational Vector [m/s^2].
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  gV[0] = grav.x();
  gV[1] = grav.z();
  gV[2] = grav.y();

  return 1;
}
