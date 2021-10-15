
/********************************************************************************
 *  Update Oreintation         : int getOrient() 
 *      returns                : Status of BNO055
 *      q[4], aV[3], avV[3], gV[3] : Vectors to update with readings from BNO
 *  Updates given arrays with new readings from BNO055.
 ********************************************************************************/
int getOrient(double q[4], double aV[3], double avV[3], double gV[3]) {

  // TODO: check if bno still works

  // Get Quaternion data from BNO055.
  imu::Quaternion quat = bno.getQuat();
  q[0] = quat.w();
  q[1] = quat.x();
  q[2] = quat.y();
  q[3] = quat.z();
  
  // Get Linear Acceleration (No Gravity) data [m/s^2].
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  aV[0] = accel.x();
  aV[1] = accel.z();
  aV[2] = accel.y();  

  // Get Gyroscope data [rad/s].
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  avV[0] = gyro.x();
  avV[1] = gyro.z();
  avV[2] = gyro.y();

  // Get Diretion of Gravitational Vector [m/s^2].
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  gV[0] = grav.x();
  gV[1] = grav.z();
  gV[2] = grav.y();

  return 1;
}

/********************************************************************************
 *  Update Altitude            : int getAlt() 
 *      returns                : Status of BMP388, Whether freefalling
 *      *a, *maxa              : Pointers to altitued and apogee to be updated
 *  Updates given pointers with new readings from BMP388
 ********************************************************************************/
int getAlt(double *a, double *maxa) {
  
  if (!bmp.performReading()) 
  { //error - could not perform reading.
    Serial.println("Error: bmp388 could not perform reading");
    return 0;
  }

  // Retrieve altitude reading that was just made
  double altReading = bmp.readAltitude(SL_PRESSURE) - groundAltitude;

  // Modify given altitued to new reading.
  *a = altReading;

  
  if (altReading >= *maxa)
  { // New maximum height reached, update given max altitude.
    *maxa = altReading;
    freefallTimer = 0;
  } else if (freefallTimer > TIME_FREEFALL_THRESHOLD)
  { // Read altitude is less than previous for more than threshold, declare freefall.
    return 2;
  }

  return 1;
}
