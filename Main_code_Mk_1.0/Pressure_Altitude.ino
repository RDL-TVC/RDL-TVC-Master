//Function runs during the setup function on the main file. Initiliazes the BMP388
//Evan Grilley - 12/28/2020
void bmpSetup(){
  if (!bmp.begin_I2C()) {
    //error - could not find sensor
    strLog.logString("Error: could not find bmp388 sensor")  
  }

  // Set up oversampling and filder initialization - Placeholder values
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

//Modified such that altitude is now a class, and the previous altitude function is contained and renamed to getAlt().
//This is so the method can access the previous altitude without needing to create a global variable
class altitudeSensor{
  float lastAlt;

  public:
    //contructor for the class
    void altitude()
    {
      lastAlt = 0;
    }

    float getAlt(){
      if (! bmp.performReading()) {
        //error - could not perform reading
        strLog.logString("Error: bmp388 could not perform reading");
        return 0;
      }
      return bmp.readAltitude(seaLevelPressure);
      
    }
}

//creating the altitudeSensor to be used in the code
//reference using altSensor.getAlt()
altitudeSensor altSensor();
