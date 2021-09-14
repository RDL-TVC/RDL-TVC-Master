/* 
 * Collection of all the startups in the Sensor_Test_Functions:
 * returns 1 if runs/successful, 0 if sensor not found (currently excempting mosfets, buzzer, and servos)
 */
 
//LEDs & buzzer
void indicatorSetup() {
  pinMode(gLED, OUTPUT); //greenLED  pin 8
  pinMode(rLED, OUTPUT); //red LED  pin 9
  pinMode(BUZZER,OUTPUT); //piezo buzzer pin 4

  //external confirmation that they work
    //gLED - lights for 1 second
    digitalWrite(gLED,HIGH);
    delay(1000);
    digitalWrite(gLED,LOW);

    //rLED - lights for 1 seconds
    digitalWrite(rLED,HIGH);
    delay(1000);
    digitalWrite(rLED,LOW);

    //Piezo buzzer - plays for 2 seconds

    tone(BUZZER, 4000, 1000);
    delay(2000);
}

//SD card
void SDSetup() {

  Serial.print("Initializing SD card...");
                                                                                                                                              
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    //don't do anything more
    
    //indicates SD failed, plays lower note
    noTone(BUZZER);
    delay(100);
    tone(BUZZER, 4000);
    while (1); //infinite loop prevents it from reaching other code - no else {} needed 
  }
  
  Serial.println("card initialized.");
}

//Ina260
void inaSetup() {

  //keeps running until ina found
  if (!ina260.begin()) {
    Serial.print("Couldn't find INA260 chip");
    noTone(BUZZER);
    delay(100);
    tone(BUZZER,4000);
    while (1);
  }
  Serial.print("Found INA260 chip\n");

  float total = 0;
  for (int i = 0; i < 20; ++i) {
    total += ina260.readBusVoltage();
  }
  avgVoltage = total/20;
  Serial.printf("Average Voltage: %f\n", avgVoltage);
}

//Bno055
int bnoSetup() {
  int isWorking = 0;
  tone(BUZZER, 3000, 1000);
 
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      noTone(BUZZER);
      delay(100);
      tone(BUZZER,4000);
    while (1);
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

  isWorking = 1;
  
  //indicates bno successfully calibrated
  noTone(BUZZER);
  delay(100);
  
  tone(BUZZER, 3000, 1000);
  delay(1000);
  tone(BUZZER, 4000, 1000);
  delay(1000);

  return isWorking;
}

//Bmp388
int bmpSetup(){
  int isWorking = 0;
  
  if (!bmp.begin_I2C()) {
    //error - could not find sensor
    Serial.println("Error: could not find bmp388 sensor");  
    noTone(BUZZER);
    delay(100);
    tone(BUZZER,4000);
    while (1);
  }

  // Set up oversampling and filder initialization - Placeholder values
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  isWorking = 1;

  Serial.println("bmp found");

  getAlt(alts);  //call once to get rid of garbage values
  alts[1] = 0;
  alts[2] = 0;
  alts[3] = 0;
    
  //take an average of 20 altitude values to find the groundAltitude
  float total = 0;
  for (int i = 0; i < 20; ++i) {
    getAlt(alts);
    total += alts[1];
  }
  groundAltitude= total/20;
  Serial.printf("Ground Altitude: %f\n",groundAltitude);

  return isWorking;
}

//Servos
void servoSetup() {
  servoPitch.attach(SERVO_PIN_PITCH);
  servoYaw.attach(SERVO_PIN_YAW);

  servoYaw.write(0);
  servoPitch.write(0);

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
}

//Mosfet charges
void miscSetup() {
  pinMode(chuteCharge1,OUTPUT);
  pinMode(chuteCharge2,OUTPUT);

  pinMode(armingPin1, INPUT);
  pinMode(armingPin2,INPUT);

}
 
