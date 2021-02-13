float* orientation() {

  //[check, xRad, yRad, zRad, accelX, accelY, accelZ, gyroX, gryoY, gyroZ, magX, magY, magZ]
  float orient[13];

  //orient[0] = check if bno still works

  //switch z & y axes based on accelerometer mounting
  /* Angle data [degrees]*/
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  orient[1] = euler.x();
  orient[2] = euler.z();  
  orient[3] = euler.y();

  /* Acceleration data [m/s^2]*/
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  orient[4] = accel.x();
  orient[5] = accel.z();
  orient[6] = accel.y();
  
  /* Gyroscope data [rad/s]*/
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  orient[7] = gyro.x();
  orient[8] = gyro.z();
  orient[9] = gyro.y();

  /* Magnetometer data [microTeslas]*/
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  orient[10] = mag.x();
  orient[11] = mag.z();
  orient[12] = mag.y();

  return orient;
}
