/* Collection of all the startups in the Sensor_Test_Functions:
 * returns 1 if runs/successful, 0 if sensor not found (currently excempting mosfets, buzzer, and servos)
 * TODO: Organize libraries and put sensor variables/names to proper places
 */

 //Bmp388
void bmpSetup(){
  if (!bmp.begin_I2C()) {
    //error - could not find sensor
    strLog.logString("Error: could not find bmp388 sensor");  
  }

  // Set up oversampling and filder initialization - Placeholder values
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  //return 1;
}
  
//Bno055
void bnoSetup() {
 
    /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    strLog.logString("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
    //return 0;
  } 

  delay(1000);
  bno.setExtCrystalUse(true);
  //return 1;
}

//Ina260
void inaSetup() {
  // Wait until serial port is opened
  while (!Serial) { delay(10); 
  }
 
  Serial.println("Adafruit INA260 Test");
 
  if (!ina260.begin()) {
    strLog.logString("Couldn't find INA260 chip");
    while (1);
    //return 0;
  }
  strLog.logString("Found INA260 chip");
  //return 1;
}

//Mosfet & buzzer
void miscSetup() {
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  //int piezoBuzzerPin = 14;
  //return 1;
}

//Servos
int servoSetup() {
  servo_LR.attach(SERVO_PIN_LR);
  servo_FB.attach(SERVO_PIN_FB);
}

//SD card
void SDSetup()
{
 // Open serial communications and wait for port to open:
   while (!Serial) {
    ; // wait for serial port to connect.
  }

  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    //return 0;
  }
  Serial.println("card initialized.");
  //return 1;
}
