
int groundidle() {
  int nextState = 2;

  float alts = altSensor.getAlt();
  int orient = orientation();

  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alt,orient);
  
  if (orient[2] >= accelThreshold) { // TODO: placeholder values that need to be changed once data format has been determined.
    nextState = 3;
    initPID();
    unlockServos();
  }
  return nextState;
}
