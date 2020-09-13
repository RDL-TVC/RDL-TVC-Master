
////////////////////TVC////////////////////

void PID()
{
  error_pitch = desired_pitch + pitch;
  error_yaw = desired_pitch + yaw;
  
  pitch_pos = P_pitch * pitch + I_pitch * error_pitch + D_pitch * ((pitch - pitch_prev) / dt);
  yaw_pos = P_yaw * pitch + I_yaw * error_yaw + D_yaw * ((yaw - yaw_prev) / dt);

  if(pitch_pos < 0)
  {
    servo_pitch.writeMicroseconds(map(pitch_pos));
  }
  else if(pitch_pos > 0)
  {
    servo_pitch.writeMicroseconds(map(pitch_pos));
  }
  else
  {
    servo_pitch.writeMicroseconds(pitch_center);
  }

  if(yaw_pos < 0)
  {
    servo_yaw.writeMicroseconds(map(yaw_pos));
  }
  else if(yaw_pos > 0)
  {
    servo_yaw.writeMicroseconds(map(yaw_pos));
  }
  else
  {
    servo_yaw.writeMicroseconds(yaw_center);
  }
}

void setupIMU()
{
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  Serial.println("Gyro cal start");
  for (int cal = 0; cal < 1000 ; cal ++)  // take 1000 measurements and average them to calibrate gyro start values
  {
    readIMU();
    gXcal += gX;
    gYcal += gY;
    gZcal += gZ;
    delay(3);
  }
  Serial.println("Gyro cal finished");
  
  gXcal /= 1000;                                                 
  gYcal /= 1000;                                                 
  gZcal /= 1000; 

  Serial.print(gXcal);
  Serial.print("\t");
  Serial.print(gYcal);
  Serial.print("\t");
  Serial.println(gZcal);
}

void readIMU()
{
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(0x68,14,true);  // request 14 registers
  while(Wire.available() < 14);   
  aX=Wire.read()<<8|Wire.read();   
  aY=Wire.read()<<8|Wire.read();
  aZ=Wire.read()<<8|Wire.read();
  temp=Wire.read()<<8|Wire.read();
  gX=Wire.read()<<8|Wire.read();
  gY=Wire.read()<<8|Wire.read();
  gZ=Wire.read()<<8|Wire.read();
}

void setupTVC() // servo setup and TVC check movements
{
  servo_pitch.attach(3); // attach servos to pins
  servo_yaw.attach(4);

  servo_pitch.writeMicroseconds(pitch_pos); // center
  servo_yaw.writeMicroseconds(yaw_pos); 
  delay(500);
  servo_yaw.writeMicroseconds(yaw_max); // max y
  delay(500);
  
  for(int deg = 0; deg <= 90; deg++){ // Q1
    float rad = deg * M_PI / 180;
    servo_pitch.writeMicroseconds(pitch_center + (pitch_max - pitch_center) * sin(rad));
    servo_yaw.writeMicroseconds(yaw_center + (yaw_max - yaw_center) * cos(rad));
    delay(3);
  }

  for(int deg = 90; deg <= 180; deg++){ // Q2
    float rad = deg * M_PI / 180;
    servo_pitch.writeMicroseconds(pitch_center + (pitch_max - pitch_center) * sin(rad));
    servo_yaw.writeMicroseconds(yaw_center + (yaw_center - yaw_min) * cos(rad));
    delay(3);
  }

  for(int deg = 180; deg <= 270; deg++){ // Q3
    float rad = deg * M_PI / 180;
    servo_pitch.writeMicroseconds(pitch_center + (pitch_center - pitch_min) * sin(rad));
    servo_yaw.writeMicroseconds(yaw_center + (yaw_center - yaw_min) * cos(rad));
    delay(3);
  }

  for(int deg = 270; deg <= 360; deg++){ // Q4
    float rad = deg * M_PI / 180;
    servo_pitch.writeMicroseconds(pitch_center + (pitch_center - pitch_min) * sin(rad));
    servo_yaw.writeMicroseconds(yaw_center + (yaw_max - yaw_center) * cos(rad));
    delay(3);
  }

  delay(500); 
  servo_pitch.writeMicroseconds(pitch_pos); // center
  servo_yaw.writeMicroseconds(yaw_pos);  
}
