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
  if ((pitchRate >= 0 && dPitch >= 0 && pitchCur >= 0) || (pitchRate <= 0 && dPitch <= 0 && pitchCur >= 0)) {
    pitchAcc = pitchAcc + dPitch;
  }
  //Quadrant 2: +pitchRate, -dPitch, +pitchCur OR -pitchRate, +dPitch, +pitchCur
  else if ((pitchRate >= 0 && dPitch <= 0 && pitchCur >= 0) || (pitchRate <= 0 && dPitch >= 0 && pitchCur >= 0)) {
    pitchAcc = pitchAcc - dPitch;
  }
  //Quadrant 3: +pitchRate, -dPitch, -pitchCur OR -pitchRate, +dPitch, -pitchCur
  else if ((pitchRate >= 0 && dPitch <= 0 && pitchCur <= 0) || (pitchRate <= 0 && dPitch >= 0 && pitchCur >= 0)) {
    pitchAcc = pitchAcc - dPitch;
  }
  //Quadrant 4: +pitchRate, +dPitch, -pitchCur OR -pitchRate, -dPitch, -pitchCur
  else if ((pitchRate >= 0 && dPitch >= 0 && pitchCur <= 0) || (pitchRate <= 0 && dPitch <= 0 && pitchCur <= 0)) {
    pitchAcc = pitchAcc + dPitch;
  }
  
  return pitchAcc;
}

float gyroPitchInt() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  float deltaTime = dTime;
  dTime = 0;
  pitchRate = gyro.y();
  pitchAcc += pitchRate * deltaTime/1000;
  pitchAcc = pitchAcc/PI*180;
  
//compare to euler.y() so that when pitchRead == 0, pitchAcc % 180 == 0 or close
  //float rotationNum = pitchAcc/360;
  //float drift = (pitchAcc - rotationNum * 360) % euler.y();
  //pitchAcc -= drift; 
  return pitchAcc;

}
