
int burnout(){
  int nextState = 4;

  float alts = altSensor.getAlt();
  float orient = orientation();

  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alt,orient);

  if (alt < lastAlt){
    nextState = 5;
  }

  return nextState;
}
