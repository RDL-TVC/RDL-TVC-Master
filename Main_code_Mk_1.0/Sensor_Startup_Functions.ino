/* Collection of all the startups in the Sensor_Test_Functions:
 * returns 1 if runs/successful, 0 if sensor not found (currently excempting mosfets, buzzer, and servos)
 * TODO: Organize libraries and put sensor variables/names to proper places
 */
 
//LEDs & buzzer
void indicatorSetup() {
  pinMode(LED1, OUTPUT); //LED1  pin 8
  pinMode(LED2, OUTPUT); //LED2  pin 9
  pinMode(BUZZER,OUTPUT); //piezo buzzer pin 14

  //external confirmation that they work
    //LED1 - lights for 1 second
    digitalWrite(LED1,HIGH);
    delay(1000);
    digitalWrite(LED1,LOW);

    //LED2 - lights for 1 seconds
    digitalWrite(LED2,HIGH);
    delay(1000);
    digitalWrite(LED2,LOW);

    //Piezo buzzer - plays for 2 seconds
    int freq = 523; //Placeholder - High C 
    tone(BUZZER, freq, 3000);
    delay(2000);
}

//SD card
void SDSetup() {
  //plays low tone to indicate function start
  tone(BUZZER, 3000);

  Serial.print("Initializing SD card...");
                                                                                                                                              
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more
    
    //indicates SD failed, plays lower note
    noTone(BUZZER);
    delay(100);
    tone(BUZZER, 2000, 3000);
    delay(3000);   
    while(1); //infinite loop prevents it from reaching other code - no else {} needed 
  }
  
  Serial.println("card initialized.");
  //indicates SD initialized
  noTone(BUZZER);
  delay(100);
  tone(BUZZER, 3000, 1000);
  delay(1000);
  tone(BUZZER, 4000, 1000);
  delay(1000);
}

//Ina260
void inaSetup() {
  //plays low tone to indicate function start
  tone(BUZZER, 3000);

  //keeps running until ina found
  if (!ina260.begin()) {
    Serial.print("Couldn't find INA260 chip");
    while (1);
  }
  Serial.print("Found INA260 chip");

  //indicates INA found
  noTone(BUZZER);
  delay(100);
  tone(BUZZER, 3000, 1000);
  delay(1000);
  tone(BUZZER, 4000, 1000);
  delay(1000);
}

//Bno055
int bnoSetup() {
  int isWorking = 0;
  
  //plays low tone to indicate function start
  tone(BUZZER, 3000);
 
  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  } 

  delay(1000);
  bno.setExtCrystalUse(true);

  //indicates bno was successfully found
  noTone(BUZZER);
  delay(100);
  tone(BUZZER, 3000, 1000);
  delay(1000);
  tone(BUZZER, 3500);
  
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
  tone(BUZZER, 3500, 1000);
  delay(1000);
  tone(BUZZER, 4000, 1000);
  delay(1000);

  return isWorking;
}

//Bmp388
int bmpSetup(){
  int isWorking = 0;
  
  //plays low tone to indicate function start
  tone(BUZZER, 3000);
  
  if (!bmp.begin_I2C()) {
    //error - could not find sensor
    Serial.println("Error: could not find bmp388 sensor");  
  }

  // Set up oversampling and filder initialization - Placeholder values
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  isWorking = 1;
  
  //indicates bmp was successfully found
  noTone(BUZZER);
  delay(100);
  tone(BUZZER, 3000, 1000);
  delay(1000);
  tone(BUZZER, 4000,1000);
  delay(1000);
  
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
  //plays low tone to indicate function start
  tone(BUZZER, 3000);
  
  pinMode(chuteCharge1,OUTPUT);
  pinMode(chuteCharge2,OUTPUT);

  pinMode(armingPin1, INPUT);
  pinMode(armingPin2,INPUT);

  //indicates chute charges and arming buttons initialized
  noTone(BUZZER);
  delay(100);
  tone(BUZZER, 3000, 1000);
  delay(1000);
  tone(BUZZER, 4000, 1000);
  delay(1000);
}
 
