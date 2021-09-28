/* TODO
 * Find place to record quaternion data -- appended to end of orient array
 * Insert way to report if the bno disconnects
 * Find alternate way to measure roll angle without the jump
 */
void orientation(float orient[]) {

  //orient[0] = check if bno still works

  /* Direction and Roll vector data */
  imu::Quaternion quat = bno.getQuat();
  imu::Quaternion qInv = getInverse(quat);

    //Quaternion data 
    orient[16] = quat.w();
    orient[17] = quat.x();
    orient[18] = quat.y();
    orient[19] = quat.z();
  
    //original Direction (i vector, towards nosecone)
    imu::Quaternion pt;
    pt.w() = 0;
    pt.x() = 1;
    pt.y() = 0;
    pt.z() = 0;

    //original Roll (k vector coincident to 1 servo)
    imu::Quaternion pt2;
    pt.w() = 0;
    pt.x() = 0;
    pt.y() = 0;
    pt.z() = 1;

    //Direction vector
    imu::Quaternion dir = quat * pt * qInv;
    orient[1] = dir.x();
    orient[2] = dir.y();
    orient[3] = dir.z();

    //Serial.printf("dirX = %.4f   dirY = %.4f   dirZ = %.4f\n", orient[1], orient[2], orient[3]);
      
    //Roll vector
    imu::Quaternion roll = quat * pt2 * qInv;
    orient[4] = roll.x();
    orient[5] = roll.y();
    orient[6] = roll.z();

  /* Acceleration data [m/s^2]*/
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  orient[7] = accel.x();
  orient[8] = accel.z();
  orient[9] = accel.y();

  //Serial.printf("accX = %.2f   accY = %.2f   accZ = %.2f\n", orient[7], orient[8], orient[9]);
  
  /* Gyroscope data [rad/s]*/
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  orient[10] = gyro.x();
  orient[11] = gyro.z();
  orient[12] = gyro.y();

  /* Magnetometer data [microTeslas]*/
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  orient[13] = mag.x();
  orient[14] = mag.z();
  orient[15] = mag.y();
}

imu::Quaternion getInverse(imu::Quaternion q) {
  imu::Quaternion conj;
  conj.w() = q.w();
  conj.x() = -q.x();
  conj.y() = -q.y();
  conj.z() = -q.z();

  double norm = sqrt(q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z());

  imu::Quaternion inv;

  inv.w() = conj.w()/norm;
  inv.x() = conj.x()/norm;
  inv.y() = conj.y()/norm;
  inv.z() = conj.z()/norm;

  return inv;
}

/*
//Apply DCM to find relative axis
float* findGimbalAngles(float* orientArr) {
  //[pitchAngle, yawAngle]
  static float angles[2]; 
  
  //uses roll angle measured from the BNO055 
    //WARNING: Measured roll angle is known to jump 
  imu::Vector<3> eul = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float roll = eul.x()*PI/180;

  //Uses DCM to account for roll in the servo angles by altering the original direction vector
  float dir2[3] = {orientArr[1]*cos(roll) + orientArr[2]*sin(roll), -orientArr[1]*sin(roll) + orientArr[2]*cos(roll), orientArr[3]};

  //Uses altered direction vector to compute gimbal angles with roll accounted for
  angles[0] = asin(dir2[0])*180/PI; //Non-simplified Equation: = 90 - acos(dot(dir, i)); then convert to degrees
  angles[1] = asin(dir2[1])*180/PI; 

  return angles;
}
*/

//alts[] = {isWorking, currentAlt, maxAlt, numberOfCycles}
void getAlt(float* alts) {
  if (! bmp.performReading()) {
        //error - could not perform reading
        Serial.println("Error: bmp388 could not perform reading");
        alts[0] = 0;
  }
  
  alts[0] = 1;
  alts[1] = bmp.readAltitude(1013.25) - groundAltitude;

  //if alt is less than max recorded alt for 5 cycles, then is decreasing
  if (alts[1] >= alts[2]) {
    alts[2] = alts[1];
    alts[3] = 0;
  } else {
    ++alts[3];
  }
  /*
  Serial.printf("Numberofcycles = %f      ", alts[3]);
  Serial.printf("Max alt: %f    ", alts[2]);
  Serial.printf("Current alt: %f       ", alts[1]);
  Serial.printf("time taken for a cycle: %d\n", timer2*1);
  */
  timer2 = 0;
}
