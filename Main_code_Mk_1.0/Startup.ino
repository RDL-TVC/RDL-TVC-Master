
int startup() {
  int nextState = 1;
  checkContinuity(); // should return a boolean to indicate if a failure state needs to be called
  armed = digitalRead(armingPin);
  if (armed) {
    nextState = 2
  }
  return nextState;
  }
