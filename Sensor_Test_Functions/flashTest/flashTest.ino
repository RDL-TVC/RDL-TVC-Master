#include <SerialFlash.h>
#include <SD.h>
#include <SPI.h>

const int SDchipSelect = BUILTIN_SDCARD;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  unsigned long startMillis = millis();
  while (!Serial && (millis() - startMillis < 10000)) ;
  delay(100);
  Serial.println("Copy all files from SD Card to SPI Flash");

  if (!SD.begin(SDchipSelect)) {
    Serial.println("Unable to access SD card");
  }
  if (!SerialFlash.begin(51)) {
    Serial.println("Unable to access SPI Flash chip");
  }

  SerialFlash.create("NewTestFile", 256);
  SerialFlashFile flashFile = SerialFlash.open("NewTestFile");
  char message[256];
  message[0] = 'n';
  message[1] = 'a';
  flashFile.write(message, 256);

  char buff[256];
  flashFile.read(buff, 256);

  Serial.println(buff[0]);

  flashFile.close();
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
