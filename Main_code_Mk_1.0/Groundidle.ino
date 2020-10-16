
int groundidle() {
  int nextState = 2;

  array alt = altitued();
  array orient = orientation();
  
  datalog(alt, orient, millis());
  
  if (orient[2] >= accelThreshold) { // TODO: placeholder values that need to be changed once data format has been determined.
    nextState = 3;
    initPID();
    unlockServos();
  }
  return nextState;
}
