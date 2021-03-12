#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
float lastAlt;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
  //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  lastAlt = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  float* alts = getAlt();
  Serial.print("Current Altitude: ");
  Serial.println(alts[0]);
  Serial.print("Change: ");
  Serial.println(alts[1]);
  delay(2000);
}

float* getAlt(){
  if (! bmp.performReading()) {
        //error - could not perform reading
        Serial.println("Error: bmp388 could not perform reading");
        return 0;
      }
      float currentAlt = bmp.readAltitude(1013.25);
      float alts[] = {currentAlt,lastAlt};
      lastAlt = currentAlt;
      
      return alts;
}
