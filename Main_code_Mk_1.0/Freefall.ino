
int freefall(){
  int nextState = 5;

  chuteDeployAltitude = 1; //TODO: Determine threshold altitude for deploying parachutes
  
  float alts = altSensor.getAlt();
  int orient = orientation();

  currentAlt = alts[0];
  lastAlt = alts[1];
  
  dataLog.logData(alt,orient);

  if (alt <= chuteDeployAltitude){
    nextState = 6;
    deployChutes()
  }

  return nextState;
}
