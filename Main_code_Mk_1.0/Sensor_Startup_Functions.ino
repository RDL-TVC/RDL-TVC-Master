/* 
 * Collection of all the startups in the Sensor_Test_Functions:
 * returns 1 if runs/successful, 0 if sensor not found (currently excempting mosfets, buzzer, and servos)
 */
 
//LEDs & buzzer
int indicatorInit() {
  pinMode(LED_GREEN, OUTPUT); //greenLED  pin 8
  pinMode(LED_RED, OUTPUT); //red LED  pin 9
  pinMode(BUZZER,OUTPUT); //piezo buzzer pin 4

  //external confirmation that they work
  //LED_GREEN - lights for 1 second
  digitalWrite(LED_GREEN,HIGH);
  delay(1000);
  digitalWrite(LED_GREEN,LOW);

  //LED_RED - lights for 1 seconds
  digitalWrite(LED_RED,HIGH);
  delay(1000);
  digitalWrite(LED_RED,LOW);

  //Piezo buzzer - plays for 2 seconds
  tone(BUZZER, TONE_SUCCESS, 1000);
  delay(2000);

  return 1;
}

//SD card
int SDInit() {

  Serial.print("Initializing SD card...");
                                                                                                                                              
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_CARD)) {
    Serial.println("Card failed, or not present");
    
    //indicates SD failed, plays lower note
    tone(BUZZER, TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    return 0;
  }
  
  Serial.println("Card initialized.");

  return 1;
}

//Ina260
int inaInit() {

  //keeps running until ina found
  if (!ina260.begin()) {
    Serial.print("No INA260 detected");
    tone(BUZZER,TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    return 0;
  }
  
  Serial.print("Found INA260 chip\n");

  float total = 0;
  for (int i = 0; i < 20; ++i) {
    total += ina260.readBusVoltage();
    delay(10);
  }
  
  avgVoltage = total/20;
  Serial.printf("Average Voltage: %f\n", avgVoltage);

  return 1;
}

//Bno055
int bnoInit() {
 
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("No BNO055 detected");
    tone(BUZZER,TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    return 0;
  }
  
  delay(1000);
  bno.setExtCrystalUse(true);
  
  uint8_t cal, gyro, accel, mag = 0;
  bno.getCalibration(&cal, &gyro, &accel, &mag);

  Serial.print("Calibrating BNO055  ");
  Serial.print(cal);
  Serial.print("  ");
  Serial.print(gyro);
  Serial.print("  ");
  Serial.print(accel);
  Serial.print("  ");
  Serial.println(mag);

  while(cal != 3)
  {
    bno.getCalibration(&cal, &gyro, &accel, &mag);
    Serial.print("Calibrating BNO055  ");
    Serial.print(cal);
    Serial.print("  ");
    Serial.print(gyro);
    Serial.print("  ");
    Serial.print(accel);
    Serial.print("  ");
    Serial.println(mag);
    delay(1000);
  }

  return 1;
}

//Bmp388
int bmpInit(){
  
  if (!bmp.begin_I2C()) {
    //error - could not find sensor
    Serial.println("Error: could not find bmp388 sensor");  
    tone(BUZZER,TONE_FAILURE);
    digitalWrite(LED_RED, HIGH);
    return 0;
  }

  // Set up oversampling and filder initialization - Placeholder values
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  
  getAlt(alts);  //call once to get rid of garbage values
  alts[1] = 0;
  alts[2] = 0;
  alts[3] = 0;

  Serial.println("BMP388 Initialized!");
    
  //take an average of 20 altitude values to find the groundAltitude
  float total = 0;
  for (int i = 0; i < 20; ++i) {
    getAlt(alts);
    total += alts[1];
  }
  
  groundAltitude= total/20;
  Serial.printf("Ground Altitude: %f\n",groundAltitude);

  return 1;
}

//Servos
int servoInit() 
{
  servoPitch.attach(SERVO_PIN_PITCH);
  servoYaw.attach(SERVO_PIN_YAW);

  servoYaw.write(0);
  servoPitch.write(0);

  return 1;
}

int servoTest()
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

//Mosfet charges
int chuteInit()
{
  pinMode(CHUTE_1_PIN,OUTPUT);
  pinMode(CHUTE_2_PIN,OUTPUT);
  
  return 1;
}

int armingInit()
{
  pinMode(ARM_B1_PIN, INPUT);
  pinMode(ARM_B2_PIN,INPUT);

  return 1;
}

int LEDBlink(int LED, int dutyCycle, int ratio)
{
  
  if (PROGRAM_TIME % (dutyCycle) <= (dutyCycle) * ratio)
  {
    digitalWrite(LED, HIGH);
    return 1;
  } else
  {
    digitalWrite(LED, LOW);
    return 0;
  }
  
}
 
