//what if Q1 80 to Q2 85 unanswered
float pitchRate;
float pitchLast;
float pitchCur;
float pitchAcc;
float dPitch;
elapsedMillis dTime;

float getPItch() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  pitchLast = pitchCur;
  pitchRate = gyro.y();
  pitchCur = euler.y();
  dPitch = pitchCur - pitchLast;
  //pitchAcc = pitchAcc + dPitch;

  //Quadrant 1: +pitchRate, +dPitch, +pitchCur OR -pitchRate, -dPitch, +pitchCur
  pitchAcc = pitchAcc + dPitch;
  
  //Quadrant 2: +pitchRate, -dPitch, +pitchCur OR -pitchRate, +dPitch, +pitchCur
  pitchAcc = pitchAcc - dPitch;
  
  //Quadrant 3: +pitchRate, -dPitch, -pitchCur OR -pitchRate, +dPitch, -pitchCur
  pitchAcc = pitchAcc - dPitch;

  //Quadrant 4: +pitchRate, +dPitch, -pitchCur OR -pitchRate, -dPitch, -pitchCur
  pitchAcc = pitchAcc + dPitch;

  return pitchAcc;
}

float gyroPitchInt() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float deltaTime = dTime;
  dTime = 0;
  pitchRate = gyro.y();
  pitchAcc += pitchRate * deltaTime;
  return pitchAcc/PI*180;

}
