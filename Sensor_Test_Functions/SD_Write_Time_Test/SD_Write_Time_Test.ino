#include <SD.h>
#include <SPI.h>

const int SD_CARD = BUILTIN_SDCARD;

elapsedMillis writeTimer = 0;

void setup() {

  SDInit();
  writeTimer = 0;
  datalogInit();
  Serial.printf("It took %d milliseconds to write header./n",writeTimer);

}

void loop() {
  // put your main code here, to run repeatedly:

}

int SDInit() 
{
  Serial.print("Initializing SD card...");
                                                                                                                                              
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CARD)) 
  { // SD Card Failed to initialize, indicate and return failure
    Serial.println("Card failed to initialize.");
    tone(BUZZER, TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    return 0;
  }
  
  Serial.println("Card initialized.");

  return 1;
}

int datalogInit()
{
  char fileName[25];
  int fileNum = 1;
  
  sprintf(fileName, "Launch %d datalog.csv", fileNum);
  
  while (SD.exists(fileName))
  {
    fileNum++;
    sprintf(fileName, "Launch %d datalog.csv", fileNum);
  }

  // Using O_CREAT | O_WRITE Instead of FILE_WRITE and manually flushing after each print is faster by a factor of 100,
  // Note make sure to flush buffer after each print and do not over run buffer.
  File datalog = SD.open(fileName, O_CREAT | O_WRITE | O_APPEND);

  //Since total Header is longer than 512 Bytes (the buffer for SD card writing) it has to be split
  // Split into multiple strcat operations to improve readability.
  char headerP1[512]; 
  strcpy(headerP1, "(int) Frame Number,(unsigned long) Program Time [ms],(int) State,(int) BNO055 State,");
  strcat(headerP1, "(double) Quaternion W,(double) Quaternion X,(double) Quaternion Y,(double) Quaternion Z,");
  strcat(headerP1, "(double) Linear Acceleration X [m/s^2],(double) Linear Acceleration Y [m/s^2],(double) Linear Acceleration Z [m/s^2],");
  strcat(headerP1, "(double) Angular Velocity X [rad/s],(double) Angular Velocity Y [rad/s],(double) Angular Velocity Z [rad/s],");
  
  char headerP2[512];
  strcpy(headerP2, "(double) Gravitational Vector X [m/s^2],(double) Gravitational Vector Y [m/s^2],(double) Gravitational Vector Z [m/s^2],");
  strcat(headerP2, "(int) BMP388 State,(float) Altitude at Ground Level,(float) Relative Altitude [m],(float) Relative Maximum Altitude [m],");
  strcat(headerP2, "(int) INA260 State,(double) Starting Voltage [V],(double) Current Voltage [V],");
  strcat(headerP2, "(double) Inclination [rad],(double) PID Error In [rad],(double) PID Out [rad],(double) PID Error Sum [rad],");
  strcat(headerP2, "(double) Servo Y Position [microseconds],(double) Servo Z Position [microseconds]");

  if (datalog) 
  { // File is open, Write initial header.
    datalog.print(headerP1);
    datalog.flush();
    datalog.println(headerP2);
    datalog.close();
  } else 
  { // If the file isn't open, pop up an error:
    Serial.print("Error opening ");
    Serial.println(fileName);
    return 0;
  }

  return 1;
}
