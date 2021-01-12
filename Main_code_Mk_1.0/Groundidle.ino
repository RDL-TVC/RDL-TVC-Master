
int groundidle() {
  int nextState = 2;

  array alt = altitude();
  array orient = orientation();
  
  datalog(alt, orient);
  
  if (orient[2] >= accelThreshold) { // TODO: placeholder values that need to be changed once data format has been determined.
    nextState = 3;
    initPID();
    unlockServos();
  }
  return nextState;
}
