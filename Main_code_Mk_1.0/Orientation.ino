/* TODO
 * Find place to record quaternion data -- appended to end of orient array
 * Insert way to report if the bno disconnects
 * Find alternate way to measure roll angle without the jump
 */
int getOrient(double q[4], double aV[3], double avV[3], double gV[3]) {

  //orient[0] = check if bno still works

  /* Direction and Roll vector data */
  imu::Quaternion quat = bno.getQuat();
  
  //Quaternion data 
  q[0] = quat.w();
  q[1] = quat.x();
  q[2] = quat.y();
  q[3] = quat.z();
  
  /* Acceleration data [m/s^2]*/
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  aV[0] = accel.x();
  aV[1] = accel.z();
  aV[2] = accel.y();  

  /* Gyroscope data [rad/s]*/
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  avV[0] = gyro.x();
  avV[1] = gyro.z();
  avV[2] = gyro.y();

  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  gV[0] = grav.x();
  gV[1] = grav.z();
  gV[2] = grav.y();

  return 1;
}

int getAlt(double *a, double *maxa) {
  
  if (! bmp.performReading()) {
        //error - could not perform reading
        Serial.println("Error: bmp388 could not perform reading");
        return 0;
  }
  
  double altReading = bmp.readAltitude(SL_PRESSURE) - groundAltitude;

  *a = altReading;
  
  if (altReading > *maxa)
  {
    *maxa = altReading;
    freefallTimer = 0;
  } else if (freefallTimer > TIME_FREEFALL_THRESHOLD)
  {
    return 2;
  }

  return 1;
}
