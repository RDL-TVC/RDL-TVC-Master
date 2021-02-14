//Function runs during the setup function on the main file. Initiliazes the BMP388
//Evan Grilley - 12/28/2020

//Modified such that altitude is now a class, and the previous altitude function is contained and renamed to getAlt().
//This is so the method can access the previous altitude without needing to create a global variable
class altitudeSensor{
  float lastAlt;

  public:
    //contructor for the class
    altitudeSensor()
    {
      lastAlt = 0;
    }

    float* getAlt(){
      if (! bmp.performReading()) {
        //error - could not perform reading
        strLog.logString("Error: bmp388 could not perform reading");
        return 0;
      }
      float alts[] = {bmp.readAltitude(seaLevelPressure),lastAlt};
      return alts;
      
    }
};

//creating the altitudeSensor to be used in the code
//reference using altSensor.getAlt()
altitudeSensor altSensor;
