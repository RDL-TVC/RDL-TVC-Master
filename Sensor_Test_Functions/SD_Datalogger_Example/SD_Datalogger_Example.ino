/*
The following code is taken directly from the Teensyduino SD library example: Datalogger.
It should be utilized as example code in the implemenation of SD card related functions.
It may also be used as a test program for the SD card given 3 analog inputs into the Teensy.

File added by: Evan Grilley
12-28-2020

*/

/*
  SD card datalogger
 
 This example shows how to log data from three analog sensors 
 to an SD card using the SD library.
 	
 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11, pin 7 on Teensy with audio board
 ** MISO - pin 12
 ** CLK - pin 13, pin 14 on Teensy with audio board
 ** CS - pin 4,  pin 10 on Teensy with audio board
 
 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe
 
 This example code is in the public domain.
 	 
 */

#include <SD.h>
#include <SPI.h>

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.

// change this to match your SD shield or module;
// Arduino Ethernet shield: pin 4
// Adafruit SD shields and modules: pin 10
// Sparkfun SD shield: pin 8
// Teensy audio board: pin 10
// Teensy 3.5 & 3.6 & 4.1 on-board: BUILTIN_SDCARD
// Wiz820+SD board: pin 4
// Teensy 2.0: pin 0
// Teensy++ 2.0: pin 20
//const int chipSelect = 11;
const int chipSelect = BUILTIN_SDCARD;
int currentMillis;
int oldMillis;
int loops;

File dataFile;
String dataString;

void setup()
{
  //UNCOMMENT THESE TWO LINES FOR TEENSY AUDIO BOARD:
  //SPI.setMOSI(7);  // Audio shield has MOSI on pin 7
  //SPI.setSCK(14);  // Audio shield has SCK on pin 14
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
    return;
  }
  Serial.println("card initialized.");

  dataFile = SD.open("datalog.txt", O_RDWR | O_CREAT | O_TRUNC);

  // make a string for assembling the data to log:
  dataString = "";

  // read three sensors and append to the string:
  for (int analogPin = 0; analogPin < 50; analogPin++) {
    //int sensor = analogRead(analogPin);
    dataString += String(analogPin);
    if (analogPin < 50) {
      dataString += ","; 
    }
  }
}

void loop()
{
  loops++;

  oldMillis = millis();
  // open the file.
  
  // if the file is available, write to it:
  if (SD.exists("datalog.txt")) {
    dataFile.println(dataString);
    // print to the serial port too:
    //Serial.println(dataString);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  } 
  currentMillis = millis();
  Serial.println(currentMillis-oldMillis);

  if (loops > 500) {
    dataFile.close();
    Serial.println("done!");
    while(true);
  }
}
