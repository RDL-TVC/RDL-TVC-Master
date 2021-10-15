 
/********************************************************************************
 *  Initialize Indicators      : int indicatorInit() 
 *      returns                : Whether indicators were initialized
 *  Inizializes and tests the pins for all external indicators (LEDs and Buzzer).
 ********************************************************************************/
int indicatorInit() 
{
  // Set pinmodes for status LEDs and Piezo Buzzer
  pinMode(LED_GREEN, OUTPUT); 
  pinMode(LED_RED, OUTPUT);
  pinMode(BUZZER,OUTPUT);

  //external confirmation that they work
  // LED_GREEN - lights for 1 second
  digitalWrite(LED_GREEN,HIGH);
  delay(1000);
  digitalWrite(LED_GREEN,LOW);
  
  // LED_RED - lights for 1 seconds
  digitalWrite(LED_RED,HIGH);
  delay(1000);
  digitalWrite(LED_RED,LOW);

  // Piezo Buzzer - plays for 1 second
  tone(BUZZER, TONE_SUCCESS, 1000);
  delay(2000);

  // TODO: Test for initialization failure internaly
  
  return 1;
}

/********************************************************************************
 *  Initialize SD Card         : int SDInit() 
 *      returns                : Whether SD Card was initialized
 *  Initializes SD Card to be used for data logging.
 ********************************************************************************/
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

/********************************************************************************
 *  Initialize INA260          : int inaInit() 
 *      returns                : Whether INA260 was initialized
 *  Initializes INA260 voltage sensor to read battery voltage.
 *  Sets a refrence voltage based on current readings.
 ********************************************************************************/
int inaInit() 
{
  Serial.print("Initializing INA260...");
  
  if (!ina260.begin()) 
  { // Sensor failed to initialize, indicate and return failure
    Serial.println("INA260 failed to initialize.");
    tone(BUZZER,TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    return 0;
  }
  
  Serial.println("INA260 Initialized.");

  float total = 0;
  for (int i = 0; i < 20; ++i) 
  { // Take 20 samples and avarage to get a baseline voltage
    total += ina260.readBusVoltage();
    delay(10);
  }
  
  avgVoltage = total/20;
  Serial.printf("Average Voltage: %f\n", avgVoltage);

  return 1;
}

/********************************************************************************
 *  Initialize BNO055          : int bnoInit() 
 *      returns                : Whether BNO055 was initialized
 *  Initializes BNO055 imu and calibrates it to current orientation.
 ********************************************************************************/
int bnoInit() 
{
  Serial.print("Initializing BNO055...");
  
  if(!bno.begin())
  { // Sensor failed to initialize, indicate and return failure
    Serial.println("BNO055 failed to initialize.");
    tone(BUZZER,TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    return 0;
  }

  Serial.println("BNO055 Initialized. Now Calibrating.");
  
  delay(1000);
  bno.setExtCrystalUse(true);
  
  uint8_t cal, gyro, accel, mag = 0;
  bno.getCalibration(&cal, &gyro, &accel, &mag);

  Serial.print("Calibrating BNO055: Sys=");
  Serial.print(cal);
  Serial.print(", Gyro=");
  Serial.print(gyro);
  Serial.print(", Accel=");
  Serial.print(accel);
  Serial.print(", Mag=");
  Serial.println(mag);

  while(cal != 3)
  {
    bno.getCalibration(&cal, &gyro, &accel, &mag);
    Serial.print("Calibrating BNO055: Sys=");
    Serial.print(cal);
    Serial.print(", Gyro=");
    Serial.print(gyro);
    Serial.print(", Accel=");
    Serial.print(accel);
    Serial.print(", Mag=");
    Serial.println(mag);
    delay(1000);
  }

  Serial.println("Calibration of BNO055 finished.");
  
  return 1;
}

/********************************************************************************
 *  Initialize BMP388          : int bmpInit() 
 *      returns                : Whether BMP388 was initialized
 *  Initializes BMP388 barometer.
 *  Sets a refrence altitude based on current readings.
 ********************************************************************************/
int bmpInit()
{
  Serial.print("Initializing BMP388...");
  
  if (!bmp.begin_I2C()) 
  { // Sensor failed to initialize, indicate and return failure
    Serial.println("BMP388 failed to initialize.");  
    tone(BUZZER,TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    return 0;
  }

  // Set up oversampling and filder initialization - Placeholder values
  // TODO: confirm values needed for below
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Serial.println("BMP388 Initialized.");

  double a;
  
  getAlt(&a, &a);  //call once to get rid of garbage values
    
  //take an average of 20 altitude values to find the starting altitude
  double total = 0;
  for (int i = 0; i < 20; ++i) {
    getAlt(&a, &a);
    total += a;
  }
  
  groundAltitude = total/20;
  Serial.printf("Ground Altitude: %f\n",groundAltitude);

  return 1;
}

/********************************************************************************
 *  Initialize Servos          : int servoInit() 
 *      returns                : Whether servos were initialized
 *  Initializes Servos that move gimbal.
 ********************************************************************************/
int servoInit() 
{
  servoPitch.attach(SERVO_PIN_PITCH);
  servoYaw.attach(SERVO_PIN_YAW);

  // TODO: Center Servos

  // TODO: Check if servos are present.

  return 1;
}

/********************************************************************************
 *  Servo Test Routine         : int servoTest() 
 *      returns                : Whether servos were Tested successfully
 *  Runs a test routine on servos to confirm the gimbal has expected range of motion.
 ********************************************************************************/
int servoTest() // TODO Implement better servo test routine and confirm actual values needed
{
  //+ Test
  servoYaw.write(90);
  servoPitch.write(90);
  delay(1000);
  
  servoPitch.write(90+10/.4285);
  delay(250);
  servoPitch.write(90-10/.4285);
  delay(250);
  servoPitch.write(90);
  delay(500);
  
  servoYaw.write(90+10/.4285);
  delay(250);
  servoYaw.write(90-10/.4285);
  delay(250);
  servoYaw.write(90);
  delay(500);

  //X Test
  servoYaw.write(90);
  servoPitch.write(90);
  delay(1000);
  
  servoPitch.write(90+5/.4285);
  servoYaw.write(90+5/.4285);
  delay(250);
  servoPitch.write(90-5/.4285);
  servoYaw.write(90-5/.4285);
  delay(250);
  servoPitch.write(90);
  servoYaw.write(90);
  delay(500);
  
  servoPitch.write(90-5/.4285);
  servoYaw.write(90+5/.4285);
  delay(250);
  servoPitch.write(90+5/.4285);
  servoYaw.write(90-5/.4285);
  delay(250);
  servoYaw.write(90);
  servoPitch.write(90);
  delay(500);

  return 1;
}

/********************************************************************************
 *  Initialize Chute Charges   : int chuteInit() 
 *      returns                : Whether charges were initialized
 *  Initializes Chute Charges, used for Parachute deployment.
 ********************************************************************************/
int chuteInit()
{
  pinMode(CHUTE_1_PIN,OUTPUT);
  pinMode(CHUTE_2_PIN,OUTPUT);

  // TODO: Check if Mosfets are actually there
  
  return 1;
}

/********************************************************************************
 *  Initialize Arming Buttons  : int armingInit() 
 *      returns                : Whether buttons were initialized
 *  Initializes Arming Buttons, used to arm Rocket.
 ********************************************************************************/
int armingInit()
{
  pinMode(ARM_B1_PIN, INPUT);
  pinMode(ARM_B2_PIN,INPUT);

  // TODO: Check if Arming buttons are present

  return 1;
}

/********************************************************************************
 *  LED Blinking Functionality : int LEDBlink() 
 *      returns                : Whether LED was turned on or off
 *  Turns on or off LED to comply with the chosen duty cycle at the point called.
 ********************************************************************************/
int LEDBlink(int LED, unsigned int dutyCycle, float ratio)
{
  
  if (PROGRAM_TIME % (dutyCycle) <= (dutyCycle) * ratio)
  { // If at time during cycle where LED should be on.
    digitalWrite(LED, HIGH);
    return 1;
  } else
  {
    digitalWrite(LED, LOW);
    return 0;
  }
  
}
 
