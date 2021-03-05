//Function runs during the setup function on the main file. Initiliazes the BMP388
//Evan Grilley - 12/28/2020


#include "Arduino.h"
#include "TVC_Alt.h"

#include "Adafruit_BMP3XX.h"

//Modified such that altitude is now a class, and the previous altitude function is contained and renamed to getAlt().
//This is so the method can access the previous altitude without needing to create a global variable
 
public:
//contructor for the class
TVC_Alt::TVC_Alt()
{
  lastAlt = 0;
  Adafruit_BMP3XX bmp;
}

TVC_Alt::getAlt(){
  if (! bmp.performReading()) {
	//error - could not perform reading
	Serial.println("Error: bmp388 could not perform reading");
	return 0;
  }
  alts[] = {bmp.readAltitude(1013.25),lastAlt};
  lastAlt = alts[0];
  return alts;
  
}

//creating the altitudeSensor to be used in the code
//reference using altSensor.getAlt()
//altitudeSensor altSensor;
