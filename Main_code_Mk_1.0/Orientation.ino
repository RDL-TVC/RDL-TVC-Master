void orientation(float orient[]) {

  //orient[0] = check if bno still works

  /* Direction and Roll vector data */
  imu::Quaternion quat = bno.getQuat();
  imu::Quaternion qInv = getInverse(quat);
  
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

float* getPIDError(float* orientArr){
  static float angles[2];
  angles[0] = asin(orientArr[1])*180/PI; //use 90 - acos(dot(dir, i)) and convert to degrees
  angles[1] = asin(orientArr[2])*180/PI; 

  //needs DCM for different correct servo angles
  return angles;
}

//Apply DCM to find relative axis and map the angles to the servos correctly
float* DCM(float* orientArr) {
  static float relativeAngles[2];


  return relativeAnges;
}
