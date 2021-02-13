/* Collection of all the startups in the Sensor_Test_Functions:
 * returns 1 if runs/successful, 0 if sensor not found (currently excempting mosfets, buzzer, and servos)
 * TODO: Organize libraries and put sensor variables/names to proper places
 */

 //Bmp388
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

Adafruit_BMP3XX bmp;

int bmp388Setup(){
  if (!bmp.begin_I2C()) {
    //error - could not find sensor
    Serial.println("Error: could not find bmp388 sensor");
    return 0;  
  }

  // Set up oversampling and filder initialization - Placeholder values
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  return 1;
}
  //Bno055
//#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int bnoSetup() {
  
  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
    return 0;
  } 

  delay(1000);
  bno.setExtCrystalUse(true);
  return 1;
}

//Ina260
#include <Adafruit_INA260.h>
 
Adafruit_INA260 ina260 = Adafruit_INA260();
 
int inaSetup() {
  Serial.begin(115200);
  // Wait until serial port is opened
  while (!Serial) { delay(10); 
  }
 
  Serial.println("Adafruit INA260 Test");
 
  if (!ina260.begin()) {
    Serial.println("Couldn't find INA260 chip");
    while (1);
    return 0;
  }
  Serial.println("Found INA260 chip");
  return 1;
}

//Mosfet & buzzer
int mosfetAndBuzzerSetup(){
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  //int piezoBuzzerPin = 14;
  return 1;
}

//Servos
#include <Servo.h>

#define SERVO_PIN_LR 0
#define SERVO_PIN_FB 1

Servo servo_LR;
Servo servo_FB;

int i = 0;

int servosSetup() {
  servo_LR.attach(SERVO_PIN_LR);
  servo_FB.attach(SERVO_PIN_FB);
}

//SD card
#include <SD.h>
#include <SPI.h>

const int chipSelect = BUILTIN_SDCARD;

int SDSetup(){

 // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect.
  }

  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return 0;
  }
  Serial.println("card initialized.");
  return 1;
}
